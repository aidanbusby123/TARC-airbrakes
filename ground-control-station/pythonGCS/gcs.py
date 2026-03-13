"""
Rocket Telemetry Visualizer — PyQt5 + OpenGL
─────────────────────────────────────────────
Install (Debian 11):
    sudo apt install python3-pyqt5 python3-pyqt5.qtopengl python3-opengl python3-serial
    # or via pip into conda:
    pip install PyQt5 PyOpenGL PyOpenGL_accelerate pyserial

Run:
    python3 rocket_telemetry.py
    python3 rocket_telemetry.py /dev/ttyUSB0
"""

import sys, os, struct, threading, time as time_module, csv
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QComboBox, QPushButton, QCheckBox, QGroupBox,
    QSizePolicy, QFrame, QStatusBar,
)
from PyQt5.QtCore    import Qt, QTimer
from PyQt5.QtGui     import QFont, QColor, QPainter, QPen
from PyQt5.QtOpenGL  import QGLWidget          # PyQt5 uses QGLWidget instead of QOpenGLWidget

from OpenGL.GL  import *
from OpenGL.GLU import *

import serial
import serial.tools.list_ports


# ══════════════════════════════════════════════════════════════════════════════
#  Telemetry state  (shared across threads)
# ══════════════════════════════════════════════════════════════════════════════
class TelemetryState:
    def __init__(self):
        self._lock = threading.Lock()
        self.time_val = self.alt = self.baro_alt = self.apogee = 0.0
        self.ax = self.ay = self.az = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.qw = 1.0
        self.qx = self.qy = self.qz = 0.0
        self.p_val = self.i_val = self.d_val = self.pid_val = 0.0
        self.brake_target = self.cd = 0.0

    def update_from_floats(self, f):
        def s(i): return f[i] if i < len(f) else 0.0
        with self._lock:
            self.time_val     = s(1)
            self.ax, self.ay, self.az   = s(2), s(3), s(4)
            self.vx, self.vy, self.vz   = s(11), s(12), s(13)
            self.qw, self.qx, self.qy, self.qz = s(20), s(21), s(22), s(23)
            self.apogee       = s(24)
            self.alt          = s(26)
            self.cd           = s(30)
            self.p_val        = s(31)
            self.i_val        = s(32)
            self.d_val        = s(33)
            self.pid_val      = s(34)
            self.brake_target = s(35)

    def snapshot(self):
        with self._lock:
            return {k: v for k, v in self.__dict__.items() if k != "_lock"}


STATE = TelemetryState()
HISTORY_WIDTH = 900

accel_x_hist = deque([0.0] * HISTORY_WIDTH, maxlen=HISTORY_WIDTH)
accel_y_hist = deque([0.0] * HISTORY_WIDTH, maxlen=HISTORY_WIDTH)
accel_z_hist = deque([0.0] * HISTORY_WIDTH, maxlen=HISTORY_WIDTH)
alt_hist     = deque([0.0] * HISTORY_WIDTH, maxlen=HISTORY_WIDTH)

log_rows = []
LOG_FIELDS = ["time", "alt", "apogee", "ax", "ay", "az", "vx", "vy", "vz"]


# ══════════════════════════════════════════════════════════════════════════════
#  Serial reader  (background thread)
# ══════════════════════════════════════════════════════════════════════════════
MSG_START = 0xFA

class SerialReader:
    def __init__(self):
        self.port         = None
        self._running     = False
        self._thread      = None
        self.print_serial = False
        # framing state
        self._reading    = False
        self._msg_type   = 0
        self._data_size  = 0
        self._bytes_read = 0
        self._buf        = bytearray()

    def open(self, port_name):
        self.close()
        try:
            self.port = serial.Serial(port_name, 115200, timeout=0.01)
            self._running = True
            self._thread  = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            print(f"Serial error: {e}")
            self.port = None
            return False

    def close(self):
        self._running = False
        if self.port and self.port.is_open:
            self.port.close()
        self.port = None

    def _run(self):
        while self._running:
            if not self.port or not self.port.is_open:
                time_module.sleep(0.05)
                continue
            try:
                while self.port.in_waiting > 0:
                    b = self.port.read(1)[0]
                    self._feed(b)
            except Exception as e:
                print(f"Serial read error: {e}")
                time_module.sleep(0.1)
            time_module.sleep(0.001)

    def _feed(self, b):
        if not self._reading:
            if b == MSG_START:
                self._reading    = True
                self._bytes_read = 0
                self._buf        = bytearray()
        elif self._bytes_read == 0:
            self._msg_type   = b
            self._bytes_read = 1
        elif self._bytes_read == 1:
            self._data_size  = b
            self._bytes_read = 2
        else:
            self._buf.append(b)
            self._bytes_read += 1
            if len(self._buf) == self._data_size:
                self._process(self._msg_type, bytes(self._buf))
                self._reading = False

    def _process(self, msg_type, data):
        if self.print_serial:
            print(f"[Serial] type={msg_type} size={len(data)}")
        if len(data) % 4 != 0:
            return
        n      = len(data) // 4
        floats = list(struct.unpack_from(f"<{n}f", data))
        STATE.update_from_floats(floats)
        snap = STATE.snapshot()
        accel_x_hist.append(snap["ax"])
        accel_y_hist.append(snap["ay"])
        accel_z_hist.append(snap["az"])
        alt_hist.append(snap["alt"])
        log_rows.append({
            "time":   snap["time_val"], "alt":    snap["alt"],
            "apogee": snap["apogee"],  "ax":     snap["ax"],
            "ay":     snap["ay"],      "az":     snap["az"],
            "vx":     snap["vx"],      "vy":     snap["vy"],
            "vz":     snap["vz"],
        })


READER = SerialReader()


# ══════════════════════════════════════════════════════════════════════════════
#  OBJ loader
# ══════════════════════════════════════════════════════════════════════════════
def load_obj(path):
    verts, faces = [], []
    if not os.path.exists(path):
        return verts, faces
    with open(path) as f:
        for line in f:
            t = line.strip()
            if t.startswith("v "):
                p = t.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif t.startswith("f "):
                p    = t.split()[1:]
                face = [int(x.split("/")[0]) - 1 for x in p]
                faces.append(face)
    return verts, faces


# ══════════════════════════════════════════════════════════════════════════════
#  OpenGL viewport  (QGLWidget — PyQt5 compatible)
# ══════════════════════════════════════════════════════════════════════════════
ROT_X90     = [1,0,0,0, 0,0,1,0, 0,-1,0,0, 0,0,0,1]
ACCEL_SCALE = 10

class RocketGLWidget(QGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.obj_verts, self.obj_faces = [], []
        self.setMinimumSize(640, 480)

    def initializeGL(self):
        glClearColor(0.04, 0.06, 0.10, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_COLOR_MATERIAL)
        glShadeModel(GL_SMOOTH)
        self.obj_verts, self.obj_faces = load_obj("rocket.obj")

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(60, w / max(h, 1), 0.1, 5000)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(0, 0, 600, 0, 0, 0, 0, 1, 0)

        snap = STATE.snapshot()

        # Lighting
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0); glEnable(GL_LIGHT1); glEnable(GL_LIGHT2)
        glLightfv(GL_LIGHT0, GL_POSITION, [ 400, 400,  500, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1.00, 0.78, 0.78, 1])
        glLightfv(GL_LIGHT1, GL_POSITION, [-400, 400,  500, 1])
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.78, 0.78, 1.00, 1])
        glLightfv(GL_LIGHT2, GL_POSITION, [0, 0, -500, 1])
        glLightfv(GL_LIGHT2, GL_DIFFUSE,  [1.00, 1.00, 1.00, 1])

        glTranslatef(0, 200, 0)

        # Acceleration vectors
        glDisable(GL_LIGHTING)
        glLineWidth(4)
        ax, ay, az = snap["ax"], snap["ay"], snap["az"]
        for end, rgb in [
            ((-ax * ACCEL_SCALE, 0, 0),  (1, 0, 0)),
            ((0, -az * ACCEL_SCALE, 0),  (0, 1, 0)),
            ((0, 0, -ay * ACCEL_SCALE),  (0, 0, 1)),
        ]:
            glBegin(GL_LINES)
            glColor3f(*rgb)
            glVertex3f(0, 0, 0)
            glVertex3f(*end)
            glEnd()
        glEnable(GL_LIGHTING)

        # Quaternion rotation (same Y↔Z swap as original Processing sketch)
        qw, qx, qy, qz = snap["qw"], snap["qx"], snap["qy"], snap["qz"]
        glMultMatrixf(ROT_X90)
        glMultMatrixf([
            1-2*(qy*qy+qz*qz), 2*(qx*qy+qw*qz),   2*(qx*qz-qw*qy),   0,
            2*(qx*qy-qw*qz),   1-2*(qx*qx+qz*qz), 2*(qy*qz+qw*qx),   0,
            2*(qx*qz+qw*qy),   2*(qy*qz-qw*qx),   1-2*(qx*qx+qy*qy), 0,
            0, 0, 0, 1,
        ])
        glMultMatrixf(ROT_X90)
        glTranslatef(0, -150, 0)

        # Draw model or fallback axes
        glColor3f(0.75, 0.78, 0.90)
        if self.obj_verts and self.obj_faces:
            glScalef(0.75, 0.75, 0.75)
            glBegin(GL_TRIANGLES)
            for face in self.obj_faces:
                for idx in face[:3]:
                    glVertex3f(*self.obj_verts[idx])
            glEnd()
        else:
            glDisable(GL_LIGHTING)
            glColor3f(1, 0.45, 0.1)
            glBegin(GL_LINES)
            for pt in [(80,0,0),(-80,0,0),(0,80,0),(0,-80,0),(0,0,160),(0,0,-160)]:
                glVertex3f(0, 0, 0)
                glVertex3f(*pt)
            glEnd()
            glEnable(GL_LIGHTING)


# ══════════════════════════════════════════════════════════════════════════════
#  Scrolling graph widget  (pure QPainter, no OpenGL)
# ══════════════════════════════════════════════════════════════════════════════
class GraphWidget(QWidget):
    TRACES = [
        ("AX",  QColor(255, 80,  80),  accel_x_hist,  5),
        ("AY",  QColor(80,  80,  255), accel_y_hist,  5),
        ("AZ",  QColor(80,  200, 80),  accel_z_hist,  5),
        ("ALT", QColor(220, 220, 220), alt_hist,      10),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet("background:#050810; border:1px solid #102030;")

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h   = self.width(), self.height()
        lane_h = h // len(self.TRACES)
        fn     = QFont("Courier New", 9)

        for idx, (name, color, hist, scale) in enumerate(self.TRACES):
            y_base = lane_h * idx + lane_h // 2
            data   = list(hist)
            n      = len(data)
            p.setPen(QPen(color, 1))
            if n >= 2:
                xs = w / n
                for i in range(1, n):
                    p.drawLine(
                        int((i-1)*xs), int(y_base - data[i-1]*scale),
                        int(i*xs),     int(y_base - data[i]*scale),
                    )
            p.setFont(fn)
            p.setPen(QPen(color))
            p.drawText(4, lane_h*idx + 13, name)
        p.end()


# ══════════════════════════════════════════════════════════════════════════════
#  Helpers: telemetry row + separator
# ══════════════════════════════════════════════════════════════════════════════
MONO = QFont("Courier New", 11)

def add_row(layout, label):
    row = QHBoxLayout()
    row.setSpacing(4)
    lbl = QLabel(label)
    lbl.setFont(MONO)
    lbl.setStyleSheet("color:#2e6e9e;")
    val = QLabel("—")
    val.setFont(MONO)
    val.setStyleSheet("color:#b8d8f8;")
    row.addWidget(lbl)
    row.addWidget(val)
    row.addStretch()
    layout.addLayout(row)
    return val

def hsep():
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setStyleSheet("color:#102030; margin:2px 0;")
    return f


# ══════════════════════════════════════════════════════════════════════════════
#  Dark stylesheet
# ══════════════════════════════════════════════════════════════════════════════
DARK = """
QMainWindow, QWidget { background:#080c14; color:#c0d8f0; }
QGroupBox {
    border:1px solid #102030; border-radius:4px; margin-top:10px;
    padding-top:8px; color:#2a5a80;
    font-family:'Courier New'; font-size:10px; letter-spacing:2px;
}
QGroupBox::title { subcontrol-origin:margin; left:8px; color:#1a4a70; }
QComboBox {
    background:#0c1520; border:1px solid #102030; border-radius:3px;
    color:#c0d8f0; padding:3px 8px;
    font-family:'Courier New'; font-size:10px;
}
QComboBox::drop-down { border:none; }
QPushButton {
    background:#0c1e30; border:1px solid #1a4a70; border-radius:3px;
    color:#3a90c8; font-family:'Courier New'; font-size:10px;
    padding:5px 12px; letter-spacing:1px;
}
QPushButton:hover   { background:#0e2840; border-color:#2a6a9a; color:#70b8e0; }
QPushButton:pressed { background:#080f1c; }
QCheckBox  { color:#2e6e9e; font-family:'Courier New'; font-size:10px; }
QStatusBar {
    background:#050810; color:#1a4060;
    font-family:'Courier New'; font-size:10px;
    border-top:1px solid #102030;
}
"""


# ══════════════════════════════════════════════════════════════════════════════
#  Main window
# ══════════════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROCKET TELEMETRY  //  GROUND STATION")
        self.setMinimumSize(1280, 800)
        self.setStyleSheet(DARK)
        self._connected = False

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ── Left sidebar ─────────────────────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(6)
        root.addLayout(left, 0)

        # Serial group
        sg = QGroupBox("SERIAL PORT")
        sl = QVBoxLayout(sg)
        self.port_combo  = QComboBox()
        self.refresh_btn = QPushButton("↺  REFRESH PORTS")
        self.connect_btn = QPushButton("▶  CONNECT")
        self.print_cb    = QCheckBox("PRINT RAW SERIAL")
        self._refresh_ports()
        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._toggle_connect)
        self.print_cb.toggled.connect(lambda v: setattr(READER, "print_serial", v))
        sl.addWidget(self.port_combo)
        sl.addWidget(self.refresh_btn)
        sl.addWidget(self.connect_btn)
        sl.addWidget(self.print_cb)
        left.addWidget(sg)

        # Telemetry group
        tg = QGroupBox("TELEMETRY")
        tl = QVBoxLayout(tg)
        tl.setSpacing(1)
        self.v_time   = add_row(tl, "TIME   ")
        self.v_alt    = add_row(tl, "ALT    ")
        self.v_baro   = add_row(tl, "BARO   ")
        self.v_apogee = add_row(tl, "APOGEE ")
        tl.addWidget(hsep())
        self.v_ax = add_row(tl, "AX     ")
        self.v_ay = add_row(tl, "AY     ")
        self.v_az = add_row(tl, "AZ     ")
        tl.addWidget(hsep())
        self.v_vx = add_row(tl, "VX     ")
        self.v_vy = add_row(tl, "VY     ")
        self.v_vz = add_row(tl, "VZ     ")
        tl.addWidget(hsep())
        self.v_p   = add_row(tl, "P      ")
        self.v_i   = add_row(tl, "I      ")
        self.v_d   = add_row(tl, "D      ")
        self.v_pid = add_row(tl, "PID    ")
        tl.addWidget(hsep())
        self.v_tdep = add_row(tl, "T DEP  ")
        self.v_cd   = add_row(tl, "CD     ")
        left.addWidget(tg)
        left.addStretch()

        # ── Right: 3-D viewport + graph ──────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(6)
        root.addLayout(right, 1)

        self.gl = RocketGLWidget()
        right.addWidget(self.gl, 1)

        gg = QGroupBox("TELEMETRY HISTORY")
        gl_lay = QVBoxLayout(gg)
        gl_lay.setContentsMargins(4, 4, 4, 4)
        self.graph = GraphWidget()
        gl_lay.addWidget(self.graph)
        right.addWidget(gg, 0)

        # Status bar
        self.sb = QStatusBar()
        self.setStatusBar(self.sb)
        self.sb.showMessage("DISCONNECTED  //  no port selected")

        # Refresh timer ~30 fps
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self._timer.start(33)

        # Pre-select port from command-line arg
        if len(sys.argv) > 1:
            idx = self.port_combo.findText(sys.argv[1])
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
            self._toggle_connect()

    # ── serial ───────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        current = self.port_combo.currentText()
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        for p in ports:
            self.port_combo.addItem(p)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def _toggle_connect(self):
        if self._connected:
            READER.close()
            self._connected = False
            self.connect_btn.setText("▶  CONNECT")
            self.sb.showMessage("DISCONNECTED")
        else:
            port = self.port_combo.currentText()
            if not port:
                self.sb.showMessage("ERROR: no port selected")
                return
            if READER.open(port):
                self._connected = True
                self.connect_btn.setText("■  DISCONNECT")
                self.sb.showMessage(f"CONNECTED  //  {port}  //  115200 baud")
            else:
                self.sb.showMessage(f"ERROR: could not open {port}")

    # ── update loop ──────────────────────────────────────────────────────────
    def _tick(self):
        snap = STATE.snapshot()
        def f(v): return f"{v:.3f}"
        self.v_time.setText(f(snap["time_val"]))
        self.v_alt.setText(f(snap["alt"]))
        self.v_baro.setText(f(snap["baro_alt"]))
        self.v_apogee.setText(f(snap["apogee"]))
        self.v_ax.setText(f(snap["ax"]))
        self.v_ay.setText(f(snap["ay"]))
        self.v_az.setText(f(snap["az"]))
        self.v_vx.setText(f(snap["vx"]))
        self.v_vy.setText(f(snap["vy"]))
        self.v_vz.setText(f(snap["vz"]))
        self.v_p.setText(f(snap["p_val"]))
        self.v_i.setText(f(snap["i_val"]))
        self.v_d.setText(f(snap["d_val"]))
        self.v_pid.setText(f(snap["pid_val"]))
        self.v_tdep.setText(f(snap["brake_target"]))
        self.v_cd.setText(f(snap["cd"]))
        self.gl.update()
        self.graph.update()

    def closeEvent(self, event):
        READER.close()
        if log_rows:
            with open("logfile.csv", "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=LOG_FIELDS)
                w.writeheader()
                w.writerows(log_rows)
            print("Log saved → logfile.csv")
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Rocket Telemetry")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())          # note: exec_() in PyQt5, exec() in PyQt6


if __name__ == "__main__":
    main()