import os
import numpy as np
import pandas as pd

from pycorc_io.package_utils.unpack_json import get_subject_params
from pycorc_io.xsens.ub_pckg.ub import ub
from PySide6.QtCore import QObject, Signal,QTimer,Slot,QElapsedTimer,Qt

class sts_logger(QObject):
    time_ready = Signal(dict)
    collect_finish = Signal()
    stopped = Signal()
    def __init__(self,init_args,session_data):
        super().__init__()
        self.init_args = init_args
        self.exp_id = session_data["exp_id"]   #remove?
        self.subject_id = session_data["subject_id"]
        self.take_num = session_data["take_num"]
        self.take_text = "0"
        self.momentum_condition = session_data.get("momentum_condition", "")   
        self.chair_height = session_data.get("chair_height", "")
        self.foot_position = session_data.get("foot_position", "")
        self.subject_path = f"logs/pycorc_recordings/{self.exp_id}/{self.subject_id}/{self.momentum_condition}_{self.chair_height}_{self.foot_position}"
        
        self.save_path = os.path.join(os.path.dirname(__file__), '../../..',f"{self.subject_path}/raw")
        
        self.metadata_path = os.path.join(os.path.dirname(__file__), '../../..',
                                     f"logs/pycorc_recordings/{self.exp_id}/{self.subject_id}/trials_metadata.csv")

        self.sensors_ready_flag = False
        
        # FPS Calculator
        self.logger_frame_count = 0
        self.logger_fps = 0
        self.logger_timer = QElapsedTimer() # use the system clock
        self.logger_timer.start()
        self.logger_cur_time = self.logger_timer.elapsed()
        self.logger_last_time = self.logger_timer.elapsed()
        self.elapsed_time = 0
        
        print("STS Logger Started")
        
    def log_trial_metadata(self, take_text):
        row = {
        "subject_id": self.subject_id,
        "momentum_condition": self.momentum_condition,
        "take": take_text,
        "chair_height": self.chair_height,
        "foot_position": self.foot_position,
        "classification": "",   # based on UI condtional check?
        "notes": "",
        }
        df = pd.DataFrame([row])
        write_header = not os.path.isfile(self.metadata_path)
        df.to_csv(self.metadata_path, mode='a', header=write_header, index=False)

    """
    Logging Callback
    """
    def log_current_data(self):
        # update FPS
        print_text = f"{self.subject_id}\nTake {self.take_text}\n"
        self.logger_frame_count += 1
        self.logger_cur_time = self.logger_timer.elapsed()
        self.elapsed_time = (self.logger_cur_time-self.logger_start_time)/1000
        print_text += f"Time Elapsed:{self.elapsed_time}\n"
        if self.logger_cur_time-self.logger_last_time >= 1000:
            self.logger_fps = self.logger_frame_count * 1000 / (self.logger_cur_time-self.logger_last_time)
            self.logger_last_time = self.logger_cur_time
            self.logger_frame_count = 0

        if self.init_args["corc"]["on"] and hasattr(self, 'corc_response'):
            self.corc_arr.append(self.corc_response["raw_data"])

        if self.init_args["xsens"]["on"] and hasattr(self, 'xsens_response'):
            timecode = self.xsens_response["timecode"]
            right = self.xsens_response["right"]["list"]
            left = self.xsens_response["left"]["list"]
            CoM = self.xsens_response["CoM"][0:6]
            # relative x distance between CoM and foot (CoM x - foot x)
            com_foot_x = [CoM[0]-self.xsens_response["right_foot"][0],
                          CoM[0]-self.xsens_response["left_foot"][0]]
            self.xsens_arr.append([timecode]+right+left+CoM+com_foot_x+[self.elapsed_time])

        if self.init_args.get("forceplate", {}).get("on", False) and hasattr(self, 'fp_response'):
            # log [fp_time, F_0..F_(n-1) (calibrated N per plate), CoPx_0,CoPy_0, ...] each tick
            fp = self.fp_response
            cop_flat = [c for cop in fp["cops"] for c in cop]
            self.fp_arr.append([fp["fp_time"]] + list(fp["summed_forces"]) + cop_flat)
                
        # emit the signal
        data = {
            "print_text":print_text,
            "logger_fps":self.logger_fps,
            "take_num":self.take_num
        }
        self.time_ready.emit(data)
    
    """
    Initialization Callback
    """
    @Slot()
    def start_worker(self):
        if self.sensors_ready_flag:
            self.poll_timer = QTimer()
            self.poll_timer.setTimerType(Qt.PreciseTimer)
            self.poll_timer.timeout.connect(self.log_current_data)
            self.poll_timer.start(int(1/100*1000))
            self.logger_start_time = self.logger_timer.elapsed()
    
            if self.init_args["corc"]["on"]:
                self.corc_arr = []
            if self.init_args["xsens"]["on"]:
                self.xsens_arr = []
            if self.init_args.get("forceplate", {}).get("on", False):
                self.fp_arr = []  # NEW: initialise forceplate buffer alongside xsens/corc
        else:
            data = {
                "print_text":"Bro wait\n",
                "logger_fps":0,
                "take_num":self.take_num
            }
            self.time_ready.emit(data)
        
    """
    External Signals Callbacks
    """
    @Slot()
    def sensors_ready(self): # [self.rft_data_arr,self.rft_pose,self.rft_fps]
        self.sensors_ready_flag = True
        data = {
            "print_text":"Sensors Ready\n",
            "logger_fps":0,
            "take_num":self.take_num
        }
        self.time_ready.emit(data)
    def update_corc(self,corc_response):
        self.corc_response = corc_response
    @Slot(dict)
    def update_xsens(self,xsens_response):
        self.xsens_response = xsens_response
    @Slot(dict)
    def update_forceplate(self, fp_response):
        self.fp_response = fp_response  
    @Slot()
    def stop(self):
        if hasattr(self, 'poll_timer'):
            self.poll_timer.stop()
        self.stopped.emit()
    @Slot()
    def redo_take(self):
        if self.init_args["corc"]["on"]:
            self.corc_arr = []
        if self.init_args["xsens"]["on"]:
            self.xsens_arr = []
        if self.init_args.get("forceplate", {}).get("on", False):
            self.fp_arr = []  # NEW: discard forceplate data for the redone take
        self.logger_start_time = self.logger_timer.elapsed()

        self.take_num -= 1
        self.take_text = f"{self.take_num}"
        print(f"Redo {self.subject_id} Take {self.take_text}")
    @Slot()
    def reset_logger(self):
        print(f"\nTake {self.take_text} Elapsed Time: {self.elapsed_time}")
        
        total_arr = []
        total_col = []
        if hasattr(self, 'corc_response'):
            corc_column = [
                        "corc time",
                        "F1x","F1y","F1z","T1x","T1y","T1z",
                        "F2x","F2y","F2z","T2x","T2y","T2z",
                        "F3x","F3y","F3z","T3x","T3y","T3z",
                       ]
            total_col += corc_column
            total_arr.append(self.corc_arr)
        if hasattr(self, 'xsens_response'):
            joints = ['trunk_ie','trunk_aa','trunk_fe',
                      'hip_fe','hip_aa','hip_ie',
                      'knee_fe','knee_aa','knee_ie',
                      'ankle_fe','ankle_aa','ankle_ie']
            xsens_column = ["timecode"]+[f"{joint}_{side}" for side in ["right","left"] for joint in joints]+["x","y","z","dx","dy","dz","com_foot_x_right","com_foot_x_left","elapsed_time"]
            total_col += xsens_column
            total_arr.append(self.xsens_arr)
        if hasattr(self, 'fp_response'):
            # append per-plate columns: calibrated summed force (N) + CoP (normalized -1..1)
            n = self.init_args.get("forceplate", {}).get("num_plates", 2)
            fp_column = ["fp_time"] + [f"F_{i}" for i in range(n)] \
                        + [c for i in range(n) for c in (f"CoPx_{i}", f"CoPy_{i}")]
            total_col += fp_column
            total_arr.append(self.fp_arr)

        df = pd.DataFrame(np.hstack(total_arr), columns=total_col)
        self.save_file(path=f"{self.save_path}/", df=df, item=f"STSRecord{self.take_text}Log")
        self.log_trial_metadata(self.take_text)  # NEW: record this successfully-saved take in the sidecar

        # reset everything
        if self.init_args["corc"]["on"]:
            self.corc_arr = []
        if self.init_args["xsens"]["on"]:
            self.xsens_arr = []
        if self.init_args.get("forceplate", {}).get("on", False):
            self.fp_arr = []  # NEW: clear buffer ready for the next take
        self.logger_start_time = self.logger_timer.elapsed()

        self.take_num += 1
        self.take_text = f"{self.take_num}"
        
    """
    Helper Function
    """    
    def save_file(self,path,df:pd.DataFrame,item):
        if not os.path.isdir(path):
            os.makedirs(path)
        df.to_csv(f"{path}/{item}.csv",index=False)    

    