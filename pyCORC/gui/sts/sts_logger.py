import os
import json
import numpy as np
import pandas as pd

from pycorc_io.package_utils.unpack_json import get_subject_params
from pycorc_io.xsens.ub_pckg.ub import ub
from pycorc_io.xsens.xsens_tools import SEGMENT_KIN_COLUMNS, JOINT_ANGLE_COLUMNS
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

        # Xsens stall check: excess lag = (logger time - Xsens sample time) above its lowest value
        # this take. Clean takes stay under ~35 ms; MVN/network hiccups push it past 100 ms.
        self.xsens_lag_warn_ms = 50
        self.xsens_warning = ""  # set when a saved take stalled; cleared on the next save/redo
        self.reset_xsens_lag_check()

        # Foot event check: heel (foot segment origin = ankle) and toe (ball of foot) position relative to
        # the start of the take. Planted feet stay within a few mm, so only larger movements are logged.
        self.foot_lift_warn_mm = 15     # vertical rise
        self.foot_shift_warn_mm = 20    # horizontal slide
        self.foot_ref_rows = 50         # first 0.5 s of the take (100 Hz) is the planted reference

        # FPS Calculator
        self.logger_frame_count = 0
        self.logger_fps = 0
        self.logger_timer = QElapsedTimer() # use the system clock
        self.logger_timer.start()
        self.logger_cur_time = self.logger_timer.elapsed()
        self.logger_last_time = self.logger_timer.elapsed()
        self.elapsed_time = 0
        
        print("STS Logger Started")
        
    def log_trial_metadata(self, take_text, trial_info=None):
        trial_info = trial_info or {}
        row = {
        "subject_id": self.subject_id,
        "momentum_condition": self.momentum_condition,
        "take": take_text,
        "chair_height": self.chair_height,
        "foot_position": self.foot_position,
        "xsens_excess_lag_ms": trial_info.get("xsens_excess_lag_ms", ""),
        "hip_init_left_deg": trial_info.get("hip_init_left_deg", ""),
        "hip_init_right_deg": trial_info.get("hip_init_right_deg", ""),
        "knee_init_left_deg": trial_info.get("knee_init_left_deg", ""),
        "knee_init_right_deg": trial_info.get("knee_init_right_deg", ""),
        "ankle_init_left_deg": trial_info.get("ankle_init_left_deg", ""),
        "ankle_init_right_deg": trial_info.get("ankle_init_right_deg", ""),
        # max heel/toe lift and shift this take, only filled in when above the warn thresholds (see foot_events)
        **{col: trial_info.get(col, "") for col in self.foot_event_columns()},
        "classification": "",   # based on UI condtional check?
        "notes": trial_info.get("notes", ""),
        }
        df = pd.DataFrame([row])
        os.makedirs(os.path.dirname(self.metadata_path), exist_ok=True)
        if "fp_position_cm" in trial_info:
            self.save_fp_position(trial_info["fp_position_cm"])
        if os.path.isfile(self.metadata_path):
            existing = pd.read_csv(self.metadata_path, nrows=0)
            if list(existing.columns) != list(df.columns):
                # older file with a different header: merge so columns stay aligned
                old = pd.read_csv(self.metadata_path)
                pd.concat([old, df], ignore_index=True).to_csv(self.metadata_path, index=False)
                return
            df.to_csv(self.metadata_path, mode='a', header=False, index=False)
        else:
            df.to_csv(self.metadata_path, index=False)

    def save_fp_position(self, fp_position_cm):
        # per-subject {foot_position: cm shift} so Natural/Back/Forward each keep their own value
        path = os.path.join(os.path.dirname(self.metadata_path), "fp_positions_cm.json")
        try:
            with open(path) as f:
                positions = json.load(f)
        except (OSError, ValueError):
            positions = {}
        positions[self.foot_position] = fp_position_cm
        with open(path, "w") as f:
            json.dump(positions, f, indent=2)

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
            # all 23 segments x [x,y,z,vx,vy,vz,ax,ay,az] (global, Z-up) for foot-shift / hip-lift detection
            segments = self.xsens_response.get("segments") or [np.nan]*len(SEGMENT_KIN_COLUMNS)
            # all 28 MVN joints (22 body + 6 ergonomic) x [x,y,z] in deg
            joints_all = self.xsens_response.get("joints") or [np.nan]*len(JOINT_ANGLE_COLUMNS)
            self.xsens_arr.append([timecode]+right+left+CoM+com_foot_x+[self.elapsed_time]+joints_all+segments)
            self.update_xsens_lag_check(timecode)
            lag_ms = self.xsens_excess_lag_ms()
            if lag_ms > self.xsens_lag_warn_ms:
                print_text += f"XSENS LAG {lag_ms:.0f}ms!\n"

        if self.init_args.get("forceplate", {}).get("on", False) and hasattr(self, 'fp_response'):
            # log [fp_time, F_0..F_(n-1) (calibrated N per plate), CoPx_0,CoPy_0, ...] each tick
            fp = self.fp_response
            n = self.init_args.get("forceplate", {}).get("num_plates", 2)
            if len(fp["summed_forces"]) == n and len(fp["cops"]) == n:
                cop_flat = [c for cop in fp["cops"] for c in cop]
                self.fp_arr.append([fp["fp_time"]] + list(fp["summed_forces"]) + cop_flat)
            else:
                # no valid packet this tick (client emits empty lists): keep the row width fixed
                self.fp_arr.append([np.nan]*(1+3*n))
                
        if self.xsens_warning:
            print_text += self.xsens_warning
        # emit the signal
        data = {
            "print_text":print_text,
            "xsens_warning":bool(self.xsens_warning) or self.xsens_excess_lag_ms() > self.xsens_lag_warn_ms,
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
                self.reset_xsens_lag_check()
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
            self.reset_xsens_lag_check()
        self.xsens_warning = ""
        if self.init_args.get("forceplate", {}).get("on", False):
            self.fp_arr = []  # NEW: discard forceplate data for the redone take
        self.logger_start_time = self.logger_timer.elapsed()

        self.take_num -= 1
        self.take_text = f"{self.take_num}"
        print(f"Redo {self.subject_id} Take {self.take_text}")
    @Slot(dict)
    def reset_logger(self, trial_info=None):
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
            xsens_column = ["timecode"]+[f"{joint}_{side}" for side in ["right","left"] for joint in joints]+["x","y","z","dx","dy","dz","com_foot_x_right","com_foot_x_left","elapsed_time"]+JOINT_ANGLE_COLUMNS+SEGMENT_KIN_COLUMNS
            total_col += xsens_column
            total_arr.append(self.xsens_arr)
        if hasattr(self, 'fp_response'):
            # append per-plate columns: calibrated summed force (N) + CoP (normalized -1..1)
            n = self.init_args.get("forceplate", {}).get("num_plates", 2)
            fp_column = ["fp_time"] + [f"F_{i}" for i in range(n)] \
                        + [c for i in range(n) for c in (f"CoPx_{i}", f"CoPy_{i}")]
            total_col += fp_column
            total_arr.append(self.fp_arr)

        trial_info = dict(trial_info or {})
        self.xsens_warning = ""
        if hasattr(self, 'xsens_response'):
            lag_ms = self.xsens_excess_lag_ms()
            trial_info["xsens_excess_lag_ms"] = round(lag_ms)
            if lag_ms > self.xsens_lag_warn_ms or self.xsens_error_rows:
                self.xsens_warning = f"T{self.take_text} XSENS BAD: R\n"
                print(f"WARNING: Take {self.take_text} Xsens stalled (excess lag {lag_ms:.0f} ms, "
                      f"{self.xsens_error_rows} error rows) - press R to redo it")
            self.reset_xsens_lag_check()

        df = pd.DataFrame(np.hstack(total_arr), columns=total_col)
        self.save_file(path=f"{self.save_path}/", df=df, item=f"STSRecord{self.take_text}Log")
        if hasattr(self, 'xsens_response'):
            foot_events = self.foot_events(df)
            trial_info.update(foot_events)
            if foot_events:
                print(f"NOTE: Take {self.take_text} foot movement: "
                      + ", ".join(f"{k.replace('_mm','')} {v} mm" for k, v in foot_events.items()))
        self.log_trial_metadata(self.take_text, trial_info)  # record this take + trial info panel in the sidecar

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
    def reset_xsens_lag_check(self):
        self.xsens_min_lag = np.inf
        self.xsens_max_lag = -np.inf
        self.xsens_error_rows = 0
    def update_xsens_lag_check(self, timecode):
        # timecode is the Xsens sample time "HH:MM:SS.mmm"; "ERROR TIME"/"" mean no valid frame
        try:
            h, m, sec = timecode.strip().split(":")
            sample_t = int(h)*3600 + int(m)*60 + float(sec)
        except ValueError:
            self.xsens_error_rows += 1
            return
        lag = self.logger_cur_time/1000 - sample_t
        self.xsens_min_lag = min(self.xsens_min_lag, lag)
        self.xsens_max_lag = max(self.xsens_max_lag, lag)
    def xsens_excess_lag_ms(self):
        if not np.isfinite(self.xsens_min_lag):
            return 0.0
        return (self.xsens_max_lag - self.xsens_min_lag)*1000

    def foot_event_columns(self):
        return [f"{part}_{kind}_{side}_mm" for part in ["heel","toe"] for kind in ["lift","shift"] for side in ["right","left"]]
    def foot_events(self, df):
        # max lift (z rise) and shift (horizontal distance) of each heel/toe from its position at the
        # start of the take, in mm; returns only the ones above the warn thresholds
        timecode = df["timecode"].astype(str)
        valid = ~timecode.str.contains("ERROR") & (timecode.str.strip() != "")  # skip placeholder rows
        if valid.sum() < self.foot_ref_rows:
            return {}
        events = {}
        # heel = foot segment (origin at ankle), toe = toe segment (origin at ball of foot)
        for part, segment in [("heel","Foot"), ("toe","Toe")]:
            for side in ["right","left"]:
                cols = [f"{side.capitalize()}{segment}_{k}" for k in ["x","y","z"]]
                pos = df.loc[valid, cols].astype(float).to_numpy()
                ref = np.nanmedian(pos[:self.foot_ref_rows], axis=0)
                if np.isnan(ref).any():
                    continue
                lift = np.nanmax(pos[:,2] - ref[2])*1000
                shift = np.nanmax(np.linalg.norm(pos[:,0:2] - ref[0:2], axis=1))*1000
                if lift > self.foot_lift_warn_mm:
                    events[f"{part}_lift_{side}_mm"] = round(lift)
                if shift > self.foot_shift_warn_mm:
                    events[f"{part}_shift_{side}_mm"] = round(shift)
        return events

    def save_file(self,path,df:pd.DataFrame,item):
        if not os.path.isdir(path):
            os.makedirs(path)
        df.to_csv(f"{path}/{item}.csv",index=False)    

    