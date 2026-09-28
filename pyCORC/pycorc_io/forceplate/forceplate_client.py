import socket
import struct
import sys, os

from PySide6.QtCore import QObject, Signal, QTimer, Slot, QElapsedTimer, Qt

def Checksum(b):
    ck = 0
    for i in range(2, len(b)-1):
        ck = ck ^ b[i]
    return ck


class forceplate_client(QObject):
    data_ready = Signal(dict)
    stopped = Signal()

    def __init__(self, ip="127.0.0.1", port=2048):
        super().__init__()
        self.ip = ip
        self.port = port

        self.fp_timer = QElapsedTimer()
        self.fp_timer.start()
        self.fp_frame_count = 0
        self.fp_fps = 0
        self.fp_cur_time = self.fp_timer.elapsed()
        self.fp_last_time = self.fp_timer.elapsed()

        self.Connected = False
        self.newValsRcv = False
        self.ValsRcv = []

    def reconnect(self):
        # connect to the ForcePlate CORC controller over TCP
        print("ForcePlate Connecting")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(1)
        print(f"ForcePlate: connecting to ({self.ip}:{self.port})...")
        try:
            self.socket.connect((self.ip, self.port))
            self.sampling = 0  # poll as fast as possible once connected
            print("ForcePlate: connected!")
            self.connection = self.socket
            self.Connected = True
        except Exception as e:
            print(f"ForcePlate: connection failed! ({e})")
            self.Connected = False
            self.socket.close()
            self.sampling = 10  # slow poll on failure to avoid busy-looping
        return self.Connected

    def SendCmd(self, cmd, vals=None):
        if self.Connected:
            if vals is None:
                vals = []
            tosend = bytearray(255)
            tosend[0] = ord('C')
            tosend[1] = len(vals)
            cmd_bytes = cmd.encode()
            i = 2
            for byte in cmd_bytes:
                tosend[i] = byte
                i += 1
            for _ in range(4 - len(cmd_bytes)):
                tosend[i] = 0
                i += 1
            for val in vals:
                for byte in bytearray(struct.pack("d", val)):
                    tosend[i] = byte
                    i += 1
            tosend[254] = Checksum(tosend)
            try:
                self.connection.sendall(tosend)
            except Exception as e:
                print(e)
                self.Connected = False
                return False
            return True

    def ProcessRcvValues(self, data, nbvals):
        self.ValsRcv = []
        for i in range(0, 8 * nbvals, 8):
            try:
                self.ValsRcv.append(struct.unpack('d', bytearray(data[i:i+8]))[0])
            except Exception as e:
                print(f"ForcePlate data corrupt: {e}")
                self.ValsRcv = []
                return

    def get_latest(self):
        # receive one 255-byte FLNL packet and unpack the double values
        try:
            data = self.connection.recv(255)
            if data:
                if data[0] == ord('C'):
                    nbvals = data[1]
                    self.ProcessRcvValues(data[6:6+8*nbvals], nbvals)
                if data[0] == ord('V'):
                    nbvals = data[1]
                    self.ProcessRcvValues(data[2:2+8*nbvals], nbvals)
                    if nbvals > 0:
                        self.newValsRcv = True
        except Exception:
            self.ValsRcv = []

    def read_fp_data(self):
        # called by the poll timer; parses packet and emits per-plate calibrated data
        try:
            self.fp_frame_count += 1
            self.fp_cur_time = self.fp_timer.elapsed()
            if self.fp_cur_time - self.fp_last_time >= 500:
                self.fp_fps = self.fp_frame_count * 1000 / (self.fp_cur_time - self.fp_last_time)
                self.fp_last_time = self.fp_cur_time
                self.fp_frame_count = 0

            self.get_latest()
            # layout: [fp_time, F_0..F_(n-1), CoPx_0,CoPy_0, ..., CoPx_(n-1),CoPy_(n-1)]
            # -> 1 + n + 2n = 1 + 3n values total, so n plates is derived from the packet
            # length rather than hardcoded (stays correct if a plate is added/removed).
            if self.newValsRcv and len(self.ValsRcv) >= 4 and (len(self.ValsRcv) - 1) % 3 == 0:
                vals = self.ValsRcv
                fp_time = vals[0]
                n_plates = (len(vals) - 1) // 3
                summed_forces = list(vals[1:1 + n_plates])
                cop_flat = vals[1 + n_plates: 1 + 3 * n_plates]
                cops = [[cop_flat[2 * i], cop_flat[2 * i + 1]] for i in range(n_plates)]
                data = {"fp_time": fp_time, "summed_forces": summed_forces,
                        "cops": cops, "fp_fps": self.fp_fps}
            else:
                data = {"fp_time": 0.0, "summed_forces": [], "cops": [], "fp_fps": self.fp_fps}
            self.data_ready.emit(data)
        except Exception:
            self.data_ready.emit({"fp_time": 0.0, "summed_forces": [], "cops": [], "fp_fps": self.fp_fps})

    def start_worker(self):
        # called when the QThread starts; connects then sends REC to begin streaming
        self.reconnect()
        self.poll_timer = QTimer()
        self.poll_timer.setTimerType(Qt.PreciseTimer)
        self.poll_timer.timeout.connect(self.read_fp_data)
        self.poll_timer.start(self.sampling)
        self.SendCmd("REC")

    @Slot()
    def stop(self):
        # send STP before closing so the microcontroller stops transmitting
        self.SendCmd("STP")
        if self.Connected:
            self.connection.close()
        self.poll_timer.stop()
        self.stopped.emit()
