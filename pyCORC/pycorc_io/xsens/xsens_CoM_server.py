import socket
import sys
import pickle as pkl
import os 

sys.path.append(os.path.join(os.path.dirname(__file__)))
from xsens_tools import *

import numpy as np
np.set_printoptions(
    precision=4,
    linewidth=np.inf,   
    formatter={'float_kind': lambda x: f"{x:.4f}"}
)

from PySide6.QtCore import QObject, QThread, Signal,QTimer,Slot,QElapsedTimer, Qt, QMetaObject
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QWidget

joint_keys = [
    'trunk_ie',
    'trunk_aa',
    'trunk_fe',
    'hip_fe',
    'hip_aa',
    'hip_ie',
    'knee_fe',
    'knee_aa',
    'knee_ie',
    'ankle_fe',
    'ankle_aa',
    'ankle_ie'
]
place_holder_angles = {
    'trunk_ie':0,
    'trunk_aa':0,
    'trunk_fe':0,
    'hip_fe':0,
    'hip_aa':0,
    'hip_ie':0,
    'knee_fe':0,
    'knee_aa':0,
    'knee_ie':0,
    'ankle_fe':0,
    'ankle_aa':0,
    'ankle_ie':0
}

class xsens_server(QObject):
    data_ready = Signal(dict)
    stopped = Signal()
    def __init__(self, ip="0.0.0.0", port=9764):
        super().__init__()
        self.ip = ip
        self.port = port
        
        # FPS Calculator
        self.xsens_timer = QElapsedTimer() # use the system clock
        self.xsens_timer.start()
        self.xsens_frame_count = 0
        self.xsens_fps = 0
        self.xsens_cur_time = self.xsens_timer.elapsed()
        self.xsens_last_time = self.xsens_timer.elapsed()
        self.right = {}
        self.left = {}
        self.CoM = []
        self.right_foot = []
        self.left_foot = []
        self.timecode = ""

    """
    CORC Connection Functions
    """
    def reconnect(self):
        print("XSENS Connecting")
        # Create a server_socket
        self.server_socket = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow immediate reuse of the port
        self.server_socket.settimeout(5)
        # Connection
        print('XSENServer: connecting to ('+self.ip+':'+str(self.port)+')...')
        try:
            self.server_socket.bind((self.ip, self.port))
            self.server_socket.listen()
            self.server_conn,addr = self.server_socket.accept()
            print('XSENS: Server connected!')
            self.Connected = True
            self.sampling = 0
            self.server_socket.setblocking(True)
        except Exception as e:
            print('XSENS: Connection failed! (', e, ')')
            self.Connected = False
            self.server_socket.close()
            self.sampling = 10
        return self.Connected

    def get_latest(self):
        try:
            # pose data quaternion (type 02), 23 segments, no props/fingers
            pose = 24+23*32
            # joint angle
            ja = 24+28*20
            # center of mass
            com = 24+3*12
            # timecode 24+16 (always the last 24+16)
            time_bytes = 24+16
            raw_message = self.server_conn.recv(pose+ja+com+time_bytes)

            #timecode
            # timecode = f"{parse_string(raw_message[-40:-40+6])}\n"
            timecode = f"{parse_string(raw_message[-12:])}\n"

            #CoM data
            # timecode += f"{parse_string(raw_message[pose+ja:pose+ja+6])}\n"
            CoM =  parse_COM_kinematics(raw_message[pose+ja+24:pose+ja+com])

            #joints identifier
            # timecode += f"{parse_string(raw_message[pose:pose+6])}\n"
            right,left = parse_LL_joint_angle(raw_message[pose+24:pose+ja])

            #foot pose identifier
            # timecode += f"{parse_string(raw_message[:6])}\n"
            right_foot,left_foot = parse_foot_pose(raw_message[24:pose])

            self.right=right
            self.left=left
            self.right_foot=right_foot
            self.left_foot=left_foot
            self.CoM = CoM
            self.timecode = timecode
            self.Connected=True
            # print("WHERE THE HELL")
        except Exception as e:
            # print(f"Error: {e}")
            
            self.right={"dict":place_holder_angles,"list":list(place_holder_angles.values())}
            self.left={"dict":place_holder_angles,"list":list(place_holder_angles.values())}
            self.CoM = [0,0,0,0,0,0,0,0,0]
            self.right_foot = [0,0,0,1,0,0,0]
            self.left_foot = [0,0,0,1,0,0,0]
            self.timecode = "ERROR TIME"


    def read_xsens_data(self):
        # update FPS
        self.xsens_frame_count += 1
        self.xsens_cur_time = self.xsens_timer.elapsed()
        if self.xsens_cur_time-self.xsens_last_time >= 500:
            self.xsens_fps = self.xsens_frame_count * 1000 / (self.xsens_cur_time-self.xsens_last_time)
            self.xsens_last_time = self.xsens_cur_time
            self.xsens_frame_count = 0

        self.get_latest()
        if(self.right and self.left and self.Connected):
            data = {
                "timecode":self.timecode,
                "xsens_fps":self.xsens_fps,
                "right":self.right,
                "left":self.left,
                "CoM":self.CoM,
                "right_foot":self.right_foot,
                "left_foot":self.left_foot
            }
        else:
            data = {
                "timecode":"",
                "xsens_fps":self.xsens_fps,
                "right":self.right,
                "left":self.left,
                "CoM":self.CoM,
                "right_foot":self.right_foot,
                "left_foot":self.left_foot
            }
        self.data_ready.emit(data)

    """
    Initialization Callback
    """
    def start_worker(self):  
        self.reconnect()
        self.poll_timer = QTimer()
        self.poll_timer.setTimerType(Qt.PreciseTimer)
        self.poll_timer.timeout.connect(self.read_xsens_data)
        self.poll_timer.start(self.sampling)
    
    """
    External Signals Callback
    """
    @Slot()
    def stop(self):
        if self.Connected:
            self.server_conn.close()
            self.server_socket.close()
        self.poll_timer.stop()
        self.stopped.emit()
        
# ----------------------------
# Main application window
# ----------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("QThread Example")
        
        # Label to display data
        self.label = QLabel("Waiting for data...")
        self.button = QPushButton("Stop Thread")

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Setup thread and worker
        self.thread = QThread()
        self.worker = xsens_server()
        self.worker.moveToThread(self.thread)

        # Connect signals
        self.thread.started.connect(self.worker.start_worker)
        self.worker.data_ready.connect(self.update_label)
        self.button.clicked.connect(self.cleanup)

        self.thread.start()

    def update_label(self, text):
        if text:
            txt = text["timecode"] + "XSENS FPS: " + f"{text['xsens_fps']:.2f}\n"
            right = text["right"]["dict"]
            left = text["left"]["dict"]
            CoM = text["CoM"]
            
            txt += f"CoM Position:\t {CoM[0:3]}\n"
            txt += f"CoM Velocity:\t {CoM[3:6]}\n"
            txt += f"R Foot Position:\t {text['right_foot'][0:3]}\n"
            txt += f"L Foot Position:\t {text['left_foot'][0:3]}\n"
            # print(right.keys(),left.keys())
            for key in joint_keys:
                txt += f"{key:15}: {left[key]:8.4f} {right[key]:8.4f}\n"
            self.label.setText(txt)

    def cleanup(self):
        QMetaObject.invokeMethod(self.worker, "stop", Qt.ConnectionType.QueuedConnection)
        self.worker.stopped.connect(self.thread.exit)
        self.label.setText("Thread stopped.")

        import time
        time.sleep(0.3) 
        self.close()

# ----------------------------
# Application entry point
# ----------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())# -*- coding: utf-8 -*-

