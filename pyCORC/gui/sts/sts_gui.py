print("Importing Paths")
import os, sys,json
from collections import deque
for mod in [m for m in sys.modules if 'PyQt5' in m]:
    del sys.modules[mod]
    
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
import numpy as np
np.set_printoptions(
    precision=4,
    linewidth=np.inf,   
    formatter={'float_kind': lambda x: f"{x:.4f}"}
)
print("Importing Logger")
from sts_logger import sts_logger
print("Importing GUI")
from base.pycorc_gui import pycorc_gui
print("Importing IOs")
from pycorc_io.corc.corc_FLNL_client import corc_FLNL_client
from pycorc_io.xsens.xsens_CoM_server import xsens_server
from pycorc_io.forceplate.forceplate_client import forceplate_client  # NEW: forceplate TCP client
print("Importing pyqtgraph")
import pyqtgraph as pg
print("Importing PySide6")
from PySide6 import QtWidgets
from PySide6.QtGui import QShortcut, QPainter, QLinearGradient, QColor, QFont, QPen
from PySide6.QtCore import QThread,Qt,QMetaObject,Slot,Signal,QRectF,QPointF
from spatialmath import SO3, SE3

class speed_bar(QtWidgets.QWidget):
    """Horizontal bar that fills with CoM speed; colour runs green (low) -> yellow (medium) -> red (high)."""
    def __init__(self, v_max=0.7, parent=None):
        super().__init__(parent)
        self.v_max = v_max                                                   # speed at which the bar is full (m/s)
        self.colour_stops = [(0.2, "#2ecc40"), (0.35, "#ffdc00"), (0.6, "#ff4136")]
        self.tick_lines = [0.2, 0.3, 0.4, 0.6]                               # zone boundaries
        self.tick_labels = [(0.2, "Low"), (0.35, "Medium"), (0.6, "High")]
        self.value = 0.0
        self.setMinimumSize(400, 120)

    def set_value(self, v):
        self.value = abs(float(v))
        self.update()  # schedule repaint

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        margin, label_h = 10, 30
        bar = QRectF(margin, margin, self.width() - 2*margin, self.height() - 2*margin - label_h)
        x_of = lambda v: bar.left() + bar.width() * min(v / self.v_max, 1.0)

        # empty track
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#dddddd"))
        p.drawRect(bar)

        # gradient spans the full track, only the filled part is drawn -> colour reflects absolute speed
        grad = QLinearGradient(bar.left(), 0, bar.right(), 0)
        for v, c in self.colour_stops:
            grad.setColorAt(v / self.v_max, QColor(c))
        p.setBrush(grad)
        p.drawRect(QRectF(bar.left(), bar.top(), x_of(self.value) - bar.left(), bar.height()))

        # zone boundaries + labels
        font = QFont(); font.setPointSize(12)
        p.setFont(font)
        p.setPen(QPen(Qt.black, 2, Qt.DashLine))
        for v in self.tick_lines:
            p.drawLine(QPointF(x_of(v), bar.top()), QPointF(x_of(v), bar.bottom()))
        p.setPen(QPen(Qt.black))
        for v, name in self.tick_labels:
            p.drawText(QRectF(x_of(v) - 60, bar.bottom() + 4, 120, label_h), Qt.AlignHCenter | Qt.AlignTop, name)

        # outline + numeric readout
        p.setPen(QPen(Qt.black, 2))
        p.setBrush(Qt.NoBrush)
        p.drawRect(bar)
        font.setPointSize(20); font.setBold(True)
        p.setFont(font)
        p.drawText(bar, Qt.AlignCenter, f"{self.value:.2f} m/s")
        p.end()

class sts_gui(pycorc_gui):
    save_take = Signal(dict)  # carries the trial info panel contents to the logger on 'C'
    def __init__(self,init_args):
        self.init_args = init_args
        self.corc_args = self.init_args["init_flags"]["corc"]
        self.xsens_args = self.init_args["init_flags"]["xsens"]
        self.gui_args = self.init_args["init_flags"]["gui"]
        self.log_args = self.init_args["init_flags"]["log"]
        self.fp_args = self.init_args["init_flags"].get("forceplate", {"on": False})  # NEW: forceplate config; default off if key absent
        self.session_data = self.init_args["session_data"]

        self.num_closed_threads = 0
        self.num_opened_threads = 0
        # NEW: count active threads dynamically so sensors_ready fires at the right time
        self.requested_threads = sum([
            self.corc_args["on"],
            self.xsens_args["on"],
            self.log_args["on"],
            self.fp_args["on"],
        ])

        super().__init__(freq=self.gui_args["freq"],gui_3d=self.gui_args["3d"])

       
        self.gui_label.setFixedSize(260, 130)

      
        self.meta_label = self.init_response_label(size=[200, 100])
        self.init_trial_info_panel()

        # initialize CoM speed bar
        if self.gui_args["vel"]:
            self.com_speed_bar = speed_bar(v_max=0.7)
            self.main_layout.addWidget(self.com_speed_bar)
          
        if self.fp_args["on"]:
            self.num_fp_plates = self.fp_args.get("num_plates", 3)  # must match forceplatemaster_params.yaml's plateIDs count (change here)
            
            
            
        """
        Initilization
        """
        self.init_IOs()
        self.init_shortcuts()

    """
    Init Sensor Worker / Thread / GUI Helper Functions
    """
    def add_opened_threads(self):
        self.num_opened_threads += 1
        print(f"Threads Opened:{self.num_opened_threads}")
        
        if self.num_opened_threads == self.requested_threads:
            QMetaObject.invokeMethod(self.logger_worker , "sensors_ready", Qt.ConnectionType.QueuedConnection)
    def init_corc(self):
        # init response label
        self.corc_label = self.init_response_label(size=[525,200])
        # init corc thread and worker
        self.corc_thread = QThread()
        self.corc_worker = corc_FLNL_client(ip=self.corc_args["ip"],port=self.corc_args["port"])
    def init_xsens(self):
        # init response label
        self.xsens_label = self.init_response_label(size=[300,300])
        # large readout of knee angle and CoM-foot x distance
        self.knee_com_label = self.init_response_label(size=[320,300], fontsize=20)

        self.xsens_thread = QThread()
        self.xsens_worker = xsens_server(ip=self.xsens_args["ip"],port=self.xsens_args["port"])
        
        # init CoM point and frame
        if self.gui_args["on"] and self.gui_args["3d"]:
            self.com_frame = self.init_frame(pos=np.array([[0,0,1]]),rot=SO3().R*0.1)
            
    def init_forceplate(self):
        # mirrors init_corc / init_xsens — creates thread, worker.
        
        self.fp_thread = QThread()
        self.fp_worker = forceplate_client(
            ip=self.fp_args["ip"],
            port=self.fp_args["port"],
        )
        fp_row = QtWidgets.QHBoxLayout()
        fp_row.setContentsMargins(0, 0, 0, 0)
        fp_row.setSpacing(0)
        self.fp_force_only_idx = 2  # third plate: force trace only, no CoP
        self.fp_cop_plots = []
        self.fp_force_texts = []
        for i in range(self.num_fp_plates):
            if i == self.fp_force_only_idx:
                widget, trace, buf = self.init_force_trace_plot(i)
                self.fp_force_widget, self.fp_force_trace, self.fp_force_buf = widget, trace, buf
                self.fp_cop_plots.append(None)   # keep list indices aligned with plate index
                self.fp_force_texts.append(None)
            else:
                widget, cop_marker, force_text = self.init_cop_plot(i)
                self.fp_cop_plots.append(cop_marker)
                self.fp_force_texts.append(force_text)
            fp_row.addWidget(widget)
        self.main_layout.addLayout(fp_row)  # added last -> ends up at the bottom of the window

    def init_cop_plot(self, plate_idx):
        
        widget = pg.PlotWidget()
        widget.setMinimumSize(220, 220)  
        widget.setBackground('w')
        widget.setAspectLocked(True)
        widget.hideAxis('left')    
        widget.hideAxis('bottom')  
        widget.setXRange(-1.1, 1.1, padding=0)  # small margin so the outline's own stroke width isn't clipped at the edge
        widget.setYRange(-1.1, 1.1, padding=0)

        widget.setTitle(f"Plate {plate_idx} CoP")
        widget.getPlotItem().layout.setContentsMargins(0, 0, 0, 0)  # trim internal padding - tight layout
        widget.plot([-1, 1, 1, -1, -1], [-1, -1, 1, 1, -1], pen=pg.mkPen('k', width=1))  # plate outline
        cop_marker = widget.plot([0], [0], pen=None, symbol='o', symbolBrush='r', symbolPen='r', symbolSize=12)
        force_text = pg.TextItem(text="F=0.0 N\nCoP=(0.0, 0.0)", color='k', anchor=(1, 0))  # anchor=top-right of text box
        force_text.setPos(1.0, 1.0)  # plate's own top-right corner
        widget.addItem(force_text)
        return widget, cop_marker, force_text
        return cop_marker

    def init_force_trace_plot(self, plate_idx, window_s=5.0, band_n=10.0):
        # scrolling force-vs-time plot (used for the chair plate instead of a CoP plot)
        widget = pg.PlotWidget()
        widget.setMinimumSize(220, 220)  # same as init_cop_plot
        widget.setBackground('w')
        widget.setTitle(f"Plate {plate_idx} F=0.0 N")
        widget.getPlotItem().layout.setContentsMargins(0, 0, 0, 0)
        widget.setLabel('left', 'F (N)')
        widget.setLabel('bottom', 't (s)')
        # seat-off band: 0 N line with +-band_n shaded region
        band = pg.LinearRegionItem(values=(-band_n, band_n), orientation='horizontal',
                                   movable=False, brush=pg.mkBrush(46, 204, 64, 60), pen=pg.mkPen(None))
        widget.addItem(band)
        widget.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('g', width=2, style=Qt.DashLine)))
        trace = widget.plot([], [], pen=pg.mkPen('b', width=2))
        n = int(window_s * self.gui_args["freq"])  # number of GUI ticks kept in the scroll window
        buf = {"t": deque(maxlen=n), "f": deque(maxlen=n)}
        return widget, trace, buf
    def init_trial_info_panel(self):
        # per-trial inputs written to trials_metadata.csv when a take is saved ('C')
        box = QtWidgets.QGroupBox("Trial Info")
        form = QtWidgets.QFormLayout(box)

        def spin(lo, hi, suffix, val=0.0):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(1)
            s.setSuffix(suffix)
            s.setValue(val)
            s.editingFinished.connect(self.return_focus)
            return s

        # forceplate cm shift for this run's foot position, prefilled from earlier runs of this subject
        self.foot_position = self.session_data.get("foot_position", "")
        fp_positions = self.load_fp_positions()
        self.fp_pos_spin = spin(-100, 100, " cm", float(fp_positions.get(self.foot_position, 0)))

        self.knee_left_spin = spin(-30, 180, " deg")
        self.knee_right_spin = spin(-30, 180, " deg")
        capture_btn = QtWidgets.QPushButton("Capture knee from Xsens (K)")
        capture_btn.setFocusPolicy(Qt.NoFocus)
        capture_btn.clicked.connect(self.capture_knee_angle)

        self.notes_edit = QtWidgets.QLineEdit()
        self.notes_edit.setPlaceholderText("Notes for this trial (Enter to finish)")
        self.notes_edit.returnPressed.connect(self.return_focus)

        form.addRow(f"FP position ({self.foot_position or '-'})", self.fp_pos_spin)
        form.addRow("Initial knee L", self.knee_left_spin)
        form.addRow("Initial knee R", self.knee_right_spin)
        form.addRow(capture_btn)
        form.addRow("Notes", self.notes_edit)
        box.setFixedWidth(340)
        self.labels_layout.addWidget(box)

        # clicking empty window space takes focus off the input fields so S/C/R/Q work again
        self.container.setFocusPolicy(Qt.ClickFocus)
    def return_focus(self):
        self.container.setFocus()
    def load_fp_positions(self):
        path = os.path.join(os.path.dirname(__file__), '../../..',
                            f"logs/pycorc_recordings/{self.session_data['exp_id']}/{self.session_data['subject_id']}/fp_positions_cm.json")
        try:
            with open(path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}
    def capture_knee_angle(self):
        if not hasattr(self, 'xsens_response'):
            print("No Xsens data yet - cannot capture knee angle")
            return
        self.knee_left_spin.setValue(self.xsens_response["left"]["dict"]["knee_fe"])
        self.knee_right_spin.setValue(self.xsens_response["right"]["dict"]["knee_fe"])
    def get_trial_info(self):
        return {
            "fp_position_cm": self.fp_pos_spin.value(),
            "knee_init_left_deg": self.knee_left_spin.value(),
            "knee_init_right_deg": self.knee_right_spin.value(),
            "notes": self.notes_edit.text(),
        }
    def save_current_take(self):
        self.save_take.emit(self.get_trial_info())
        self.notes_edit.clear()  # notes are per-trial; fp position/knee carry over to the next take
    def init_logger(self):
        # init response label
        self.logger_label = self.init_response_label(size=[600,300],fontsize=50)
        self.logger_thread = QThread()
        self.logger_worker = sts_logger(init_args = self.init_args["init_flags"],
                                        session_data  = self.session_data)
    def init_shortcuts(self):
        # to start / stop logging at button press
        if self.log_args["on"]:
            # to start logging
            start_log = QShortcut("S", self)
            start_log.activated.connect(self.logger_worker.start_worker)
            # to stop logging and close everything
            stop_log = QShortcut("C", self)
            stop_log.activated.connect(self.save_current_take)
            # to redo previous take
            redo_log = QShortcut("R", self)
            redo_log.activated.connect(self.logger_worker.redo_take)
        capture_knee = QShortcut("K", self)
        capture_knee.activated.connect(self.capture_knee_angle)
        close = QShortcut("Q", self)
        close.activated.connect(self.gui_timer.stop)
        close.activated.connect(self.close_workers)
    def init_IOs(self):
        """
        Init IO thread and worker
        """
        if self.corc_args["on"]:
            self.init_corc()
        if self.xsens_args["on"]:
            self.init_xsens()
        if self.fp_args["on"]:
            self.init_forceplate()
        if self.log_args["on"]:
            self.init_logger()
            self.logger_response = {
            "print_text":"Idle\n",
            "logger_fps":0.0
        }
        
        """
        Move sensor worker to thread 
        Connect intersensor signals
        Connect signals to sensor GUI callbacks
        Connect thread start to start sensor worker, update number of open threads
        Connect worker stop to stop thread at closeup
        Connect thread finished to close app
        """
        if self.corc_args["on"]:
            # move worker to thread
            self.corc_worker.moveToThread(self.corc_thread)
            # connect to gui
            self.corc_worker.data_ready.connect(self.update_corc,type=Qt.ConnectionType.QueuedConnection)
            # connect thread start to start worker   
            self.corc_thread.started.connect(self.corc_worker.start_worker)
            self.corc_thread.started.connect(self.add_opened_threads)
            # connect worker stop to stop thread at closeup
            self.corc_worker.stopped.connect(self.corc_thread.exit)
            # connect thread finished to close app
            self.corc_thread.finished.connect(self.close_app)
        if self.xsens_args["on"]:
            # move worker to thread
            self.xsens_worker.moveToThread(self.xsens_thread)
            # connect to gui
            self.xsens_worker.data_ready.connect(self.update_xsens,type=Qt.ConnectionType.QueuedConnection)
            # connect thread start to start worker   
            self.xsens_thread.started.connect(self.xsens_worker.start_worker)
            self.xsens_thread.started.connect(self.add_opened_threads)
            # connect worker stop to stop thread at closeup
            self.xsens_worker.stopped.connect(self.xsens_thread.exit)
            # connect thread finished to close app
            self.xsens_thread.finished.connect(self.close_app)
        if self.fp_args["on"]:
            self.fp_worker.moveToThread(self.fp_thread)
            self.fp_worker.data_ready.connect(self.update_forceplate, type=Qt.ConnectionType.QueuedConnection)
            self.fp_thread.started.connect(self.fp_worker.start_worker)
            self.fp_thread.started.connect(self.add_opened_threads)
            self.fp_worker.stopped.connect(self.fp_thread.exit)
            self.fp_thread.finished.connect(self.close_app)
        if self.log_args["on"]:
            # move worker to thread
            self.logger_worker.moveToThread(self.logger_thread)
            # connect corc, xsens and forceplate to logger
            if self.corc_args["on"]:
                self.corc_worker.data_ready.connect(self.logger_worker.update_corc,type=Qt.ConnectionType.QueuedConnection)
            if self.xsens_args["on"]:
                self.xsens_worker.data_ready.connect(self.logger_worker.update_xsens,type=Qt.ConnectionType.QueuedConnection)
            if self.fp_args["on"]:
                self.fp_worker.data_ready.connect(self.logger_worker.update_forceplate, type=Qt.ConnectionType.QueuedConnection)
            # connect logger to sensor gui
            self.logger_worker.time_ready.connect(self.update_logger,type=Qt.ConnectionType.QueuedConnection)
            # save take ('C') with the trial info panel contents
            self.save_take.connect(self.logger_worker.reset_logger,type=Qt.ConnectionType.QueuedConnection)
            # connect thread start to add opened threads  
            self.logger_thread.started.connect(self.add_opened_threads)
            # connect worker stop to stop thread at closeup
            self.logger_worker.stopped.connect(self.logger_thread.exit)
            # connect worker collect finish to stop app
            self.logger_worker.collect_finish.connect(self.gui_timer.stop)
            self.logger_worker.collect_finish.connect(self.close_workers)
            # connect thread finished to close app
            self.logger_thread.finished.connect(self.close_app)

        """
        Start threads
        """
        if self.corc_args["on"]:
            self.corc_thread.start()
            self.corc_thread.setPriority(QThread.Priority.TimeCriticalPriority)
        if self.xsens_args["on"]:
            self.xsens_thread.start()
            self.xsens_thread.setPriority(QThread.Priority.TimeCriticalPriority)
        if self.fp_args["on"]:
            self.fp_thread.start()
            self.fp_thread.setPriority(QThread.Priority.TimeCriticalPriority)
        if self.log_args["on"]:
            self.logger_thread.start()
            self.logger_thread.setPriority(QThread.Priority.TimeCriticalPriority)
        
    """
    Update Sensor Variable Callbacks
    """
    @Slot(dict)
    def update_corc(self,corc_response):
        self.corc_response = corc_response
    def update_xsens(self,xsens_response):
        self.xsens_response = xsens_response
    @Slot(dict)
    def update_forceplate(self, fp_response):
        self.fp_response = fp_response  
    @Slot(dict)
    def update_logger(self,logger_response):
        self.logger_response = logger_response

    """
    Update Main GUI Helper Functions and Callback
    """
    def update_gui(self):
        super().update_gui()

        # trial # tracks the logger's live take_num (increments on 'C') once logging is active;
        # falls back to session_data's initial value before that
        if hasattr(self, 'logger_response') and "take_num" in self.logger_response:
            trial_num = self.logger_response["take_num"]
        else:
            trial_num = self.session_data.get("take_num", 0)
        meta_txt = (f"Trial #: {trial_num}\n"
                    f"Chair Height: {self.session_data.get('chair_height', '-')}\n"
                    f"Foot Position: {self.session_data.get('foot_position', '-')}\n"
                    f"Momentum: {self.session_data.get('momentum_condition', '-')}\n")
        self.update_response_label(self.meta_label, meta_txt)

        if hasattr(self, 'corc_response'):
            # update rft info (FPS shown once, consolidated, in gui_label below)
            corc_data = self.corc_response["raw_data"]
            txt = ""
            self.update_response_label(self.corc_label,f"CORC Running Time:{corc_data[0]}s\n{txt}")

        if hasattr(self, 'xsens_response'):
            # update rft info (FPS shown once, consolidated, in gui_label below)
            timecode = self.xsens_response["timecode"]
            right_list = self.xsens_response["right"]["list"]
            left_list = self.xsens_response["left"]["list"]             
            
            # update the gui
            CoM = self.xsens_response["CoM"]
            if self.gui_args["on"] and self.gui_args["vel"]:
                try:
                    self.com_speed_bar.set_value(CoM[3])  # dx
                except:
                    print(CoM)
            
            # update COM point and frame
            if self.gui_args["on"] and self.gui_args["3d"]:
                try:
                    self.update_frame(self.com_frame,np.array(CoM[0:3]),SO3().R*0.1)
                except:
                    print(CoM)
                
            txt = f"XSENS Timecode:{timecode}\n"

            for i,key in  enumerate(['trunk_ie','trunk_aa','trunk_fe',
                        'hip_fe','hip_aa','hip_ie',
                        'knee_fe','knee_aa', 'knee_ie',
                        'ankle_fe','ankle_aa','ankle_ie']):
                txt += f"{key:15}: {left_list[i]:8.4f} {right_list[i]:8.4f}\n"
            txt += f"CoM Position\t: {CoM[0:3]}\n"
            txt += f"CoM Velocity\t: {CoM[3:6]}\n"
            self.update_response_label(self.xsens_label,f"{txt}")

            # large readout: knee & ankle flexion and 
            knee_right = self.xsens_response["right"]["dict"]["knee_fe"]
            knee_left = self.xsens_response["left"]["dict"]["knee_fe"]
            ankle_right = self.xsens_response["right"]["dict"]["ankle_fe"]
            ankle_left = self.xsens_response["left"]["dict"]["ankle_fe"]
            big_txt = f"Knee  L:{knee_left:6.1f} R:{knee_right:6.1f}\n"
            big_txt += f"Ankle L:{ankle_left:6.1f} R:{ankle_right:6.1f}\n"
            self.update_response_label(self.knee_com_label,big_txt)

        if hasattr(self, 'fp_response'):
            # update each plate's CoP marker + its own overlaid force reading
            # (no separate text label - see init_forceplate())
            summed_forces = self.fp_response["summed_forces"]
            cops = self.fp_response["cops"]
            for i, (f, cop) in enumerate(zip(summed_forces, cops)):
                if i == self.fp_force_only_idx:
                    self.fp_force_buf["t"].append(self.gui_elapsed_time)
                    self.fp_force_buf["f"].append(f)
                    self.fp_force_trace.setData(list(self.fp_force_buf["t"]), list(self.fp_force_buf["f"]))
                    self.fp_force_widget.setTitle(f"Plate {i} F={f:.1f} N")
                else:
                    self.fp_cop_plots[i].setData([cop[0]], [cop[1]])
                    self.fp_force_texts[i].setText(f"F={f:.1f} N\nCoP=({cop[0]:.3f}, {cop[1]:.3f})")
        if hasattr(self, 'logger_response'):
            print_text = self.logger_response["print_text"]
            self.update_response_label(self.logger_label,f"{print_text}")

        # replacing the base class's own single-source gui_label text written by super().update_gui() above
        fps_txt = f"GUI FPS: {self.gui_fps:.1f}   Elapsed: {self.gui_elapsed_time:.2f}s\n"
        if hasattr(self, 'corc_response'):
            fps_txt += f"CORC FPS: {self.corc_response['corc_fps']:.1f}\n"
        if hasattr(self, 'xsens_response'):
            fps_txt += f"Xsens FPS: {self.xsens_response['xsens_fps']:.1f}\n"
        if hasattr(self, 'fp_response'):
            fps_txt += f"ForcePlate FPS: {self.fp_response['fp_fps']:.1f}\n"
        if hasattr(self, 'logger_response'):
            fps_txt += f"Logger FPS: {self.logger_response['logger_fps']:.1f}\n"
        self.update_response_label(self.gui_label, fps_txt)
    
    """
    Cleanup
    """
    def close_worker_thread(self,worker):
        QMetaObject.invokeMethod(worker, "stop", Qt.ConnectionType.QueuedConnection)
    def close_workers(self):
        if hasattr(self, 'corc_response'):
            self.close_worker_thread(self.corc_worker)
        if hasattr(self, 'xsens_response'):
            self.close_worker_thread(self.xsens_worker)
        if hasattr(self, 'fp_response'):
            self.close_worker_thread(self.fp_worker)  # NEW: stop forceplate thread on quit
        if hasattr(self, 'logger_response'):
            self.close_worker_thread(self.logger_worker)
    @Slot()
    def close_app(self):
        self.num_closed_threads += 1
        if self.num_closed_threads == self.num_opened_threads:
            print("Shutting down app...")
            for plt in self.plt_items:
                plt.clear()
                plt.deleteLater()
            self.close()
            self.deleteLater()

if __name__ == "__main__":
    try:
        argv = sys.argv[1]
    except:
        argv ={
               "init_flags":{"corc":{"on":False,
                                     "ip":"127.0.0.1",
                                     "port":2048},
                            "xsens":{"on":True,
                                     "ip":"0.0.0.0",
                                     "port":9764},
                             "forceplate":{"on":True,
                                           "ip":"127.0.0.1",
                                           "port":2048,
                                           "num_plates":3},
                             "gui":{"on":True,
                                    "freq":30,
                                    "3d":False,
                                    "vel":True},
                             "log":{"on":True}
                             },
                "session_data":{
                    "exp_id":"exp2",
                    "subject_id":"test_j",
                    "take_num":0,
                    "var_num":1,
                    "chair_height":"test",
                    "foot_position":"test",
                    "momentum_condition":"test",
                }
               }
        
        argv = json.dumps(argv)

    init_args = json.loads(argv)
    app = QtWidgets.QApplication(sys.argv)
    w = sts_gui(init_args)
    
    title_str = f"sts-CORC-{init_args['session_data']['exp_id']}"+"-"
    title_str += init_args['session_data']['subject_id']+"-"
    w.setWindowTitle(title_str)
    w.show()

    app.exec()
    app.quit()
    import shiboken6
    shiboken6.delete(app)  # Immediate deletion
    sys.exit()


    # 169.254.45.31
