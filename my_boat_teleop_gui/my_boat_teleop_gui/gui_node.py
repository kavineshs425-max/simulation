"""teleop_gui -- my_boat keyboard teleop in a window that does NOT need the focus.

    ros2 launch my_boat_teleop_gui teleop_gui.launch.py
    (or: ros2 run my_boat_teleop_gui teleop_gui)

* ON / OFF switch. While ON, w a s d / Shift / space are read from the
  keyboard WHATEVER window is focused (X11), so you can drive while looking at
  Gazebo or RViz. F9 turns it OFF from anywhere (turning ON needs a click, so
  a stray F9 can never start driving).
* Mode: Direction (w forward, s reverse, a/d turn with the bow thrusters
  assisting) or Actuator (one key per actuator, Shift = negative).
* Side panel: every parameter of my_boat_teleop, live. They are also ROS
  parameters of this node, so `ros2 param set /keyboard_teleop_gui ...` works.

Publishes the same topics as my_boat_teleop:
  /<ns>/thrusters/T1/thrust  T2/thrust  T3/thrust  (N)   T3/pos (rad)
"""

import math
import time

import rclpy
from python_qt_binding.QtCore import QRectF, Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QAbstractSpinBox, QApplication, QButtonGroup, QCheckBox, QDoubleSpinBox, QFormLayout, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton, QScrollArea,
    QSizePolicy, QVBoxLayout, QWidget)
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

from .core import DEFAULTS, MODES, TeleopCore, keymap
from .xkeys import GlobalKeys

STYLE = """
QWidget { font-family: "Inter", "Segoe UI", "Ubuntu", sans-serif; font-size: 13px; color: #15202b; }
QMainWindow, #root { background: #f3f5f8; }
#card { background: #ffffff; border: 1px solid #e1e6ec; border-radius: 10px; }
#title { font-size: 18px; font-weight: 800; letter-spacing: 1px; }
#sub { color: #6b7a8a; }
#section { color: #6b7a8a; font-size: 11px; font-weight: 700; letter-spacing: 1px; }
QPushButton { border: 1px solid #cfd7e0; border-radius: 8px; padding: 8px 14px; background: #ffffff; font-weight: 600; }
QPushButton:hover { border-color: #9fb0c2; }
QPushButton#power { font-size: 15px; font-weight: 800; padding: 12px 18px; min-width: 150px; }
QPushButton#power[on="true"] { background: #1e8e3e; border-color: #1e8e3e; color: white; }
QPushButton#power[on="false"] { background: #eef1f4; color: #3b4a5a; }
QPushButton#mode:checked { background: #1a66e0; border-color: #1a66e0; color: white; }
QPushButton#stop { background: #d93025; border-color: #d93025; color: white; font-size: 15px; font-weight: 800; padding: 12px; }
QPushButton#stop:hover { background: #b3261e; }
#warn { background: #fff4e5; color: #a15c00; border: 1px solid #ffd699; border-radius: 8px; padding: 7px 10px; }
#info { background: #e8f0fe; color: #1a4fa8; border: 1px solid #c6dafc; border-radius: 8px; padding: 7px 10px; }
#keycap { border: 1px solid #cfd7e0; border-bottom: 3px solid #b9c3cf; border-radius: 7px; background: #ffffff;
          font-weight: 800; font-size: 15px; min-width: 38px; min-height: 34px; qproperty-alignment: AlignCenter; }
#keycap[down="true"] { background: #f57c00; color: white; border-color: #d56a00; }
#keydesc { color: #3b4a5a; }
QGroupBox { background: #ffffff; border: 1px solid #e1e6ec; border-radius: 10px; margin-top: 14px; padding: 10px 8px 6px 8px; font-weight: 700; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #6b7a8a; }
QDoubleSpinBox, QLineEdit { border: 1px solid #cfd7e0; border-radius: 6px; padding: 4px 6px; background: white; min-width: 96px; }
QDoubleSpinBox:focus, QLineEdit:focus { border-color: #1a66e0; }
"""


class CenterBar(QWidget):
    """A bar that grows from zero, between asymmetric limits."""

    def __init__(self, name, unit, fmt='{:+.1f}'):
        super().__init__()
        self.name, self.unit, self.fmt = name, unit, fmt
        self.value, self.lo, self.hi = 0.0, -1.0, 1.0
        self.setMinimumHeight(46)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set(self, value, lo, hi):
        self.value, self.lo, self.hi = value, lo, hi
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.setFont(QFont(self.font().family(), 10, QFont.Bold))
        p.setPen(QColor('#3b4a5a'))
        p.drawText(0, 14, self.name)
        p.setPen(QColor('#15202b'))
        txt = self.fmt.format(self.value) + ' ' + self.unit
        p.drawText(QRectF(0, 0, w, 16), Qt.AlignRight | Qt.AlignVCenter, txt)
        track = QRectF(0, 22, w, 12)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor('#e9edf2'))
        p.drawRoundedRect(track, 6, 6)
        span = self.hi - self.lo
        if span <= 0:
            return
        zx = (0.0 - self.lo) / span * w
        vx = (min(max(self.value, self.lo), self.hi) - self.lo) / span * w
        p.setBrush(QColor('#1a66e0') if self.value >= 0 else QColor('#f57c00'))
        p.drawRoundedRect(QRectF(min(zx, vx), 22, abs(vx - zx), 12), 5, 5)
        p.setPen(QPen(QColor('#15202b'), 2))
        p.drawLine(int(zx), 19, int(zx), 37)
        p.setPen(QColor('#8a99a8'))
        p.setFont(QFont(self.font().family(), 8))
        p.drawText(QRectF(0, 34, w, 12), Qt.AlignLeft, self.fmt.format(self.lo))
        p.drawText(QRectF(0, 34, w, 12), Qt.AlignRight, self.fmt.format(self.hi))


class TeleopNode(Node):
    def __init__(self, core: TeleopCore, on_param):
        super().__init__('keyboard_teleop_gui')
        self.core = core
        self._on_param = on_param
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        for k in DEFAULTS:                       # launch/-p overrides -> core
            core.set_param(k, self.get_parameter(k).value)
        self.add_on_set_parameters_callback(self._params_cb)
        self.pubs = {}
        self._make_publishers()

    def _make_publishers(self):
        for p in self.pubs.values():
            self.destroy_publisher(p)
        base = '/' + self.core.params['boat_namespace'].strip('/') + '/thrusters/'
        self.base = base
        self.pubs = {k: self.create_publisher(Float64, base + t, 10) for k, t in
                     (('T1', 'T1/thrust'), ('T2', 'T2/thrust'), ('T3', 'T3/thrust'), ('steer', 'T3/pos'))}

    def _params_cb(self, params):
        for prm in params:
            if prm.name not in DEFAULTS:
                continue
            try:
                self.core.set_param(prm.name, prm.value)
            except (ValueError, TypeError) as e:
                return SetParametersResult(successful=False, reason=str(e))
            if prm.name == 'boat_namespace':
                self._make_publishers()
            self._on_param(prm.name)
        return SetParametersResult(successful=True)

    def publish(self):
        for k, v in self.core.commands().items():
            self.pubs[k].publish(Float64(data=float(v)))


class Window(QMainWindow):
    def __init__(self, node: TeleopNode, core: TeleopCore):
        super().__init__()
        self.node, self.core = node, core
        self.keys = GlobalKeys()
        self.local_down = set()                 # fallback: this window's own key events
        self.on = False
        self._f9 = False
        self._last_pub = 0.0
        self.setWindowTitle('my_boat teleop')
        self.setStyleSheet(STYLE)
        self._build()
        self.refresh_all()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.loop)
        self.timer.start(10)

    # ------------------------------------------------------------ layout
    def _card(self):
        f = QFrame(); f.setObjectName('card'); return f

    def _build(self):
        root = QWidget(); root.setObjectName('root')
        outer = QHBoxLayout(root); outer.setContentsMargins(14, 14, 14, 14); outer.setSpacing(14)

        # ---- main column
        main = QVBoxLayout(); main.setSpacing(12)
        head = QHBoxLayout()
        tv = QVBoxLayout()
        t = QLabel('my_boat teleop'); t.setObjectName('title'); tv.addWidget(t)
        self.subtitle = QLabel(''); self.subtitle.setObjectName('sub'); tv.addWidget(self.subtitle)
        head.addLayout(tv); head.addStretch(1)
        self.power = QPushButton('OFF'); self.power.setObjectName('power'); self.power.setFocusPolicy(Qt.NoFocus)
        self.power.clicked.connect(lambda: self.set_on(not self.on))
        head.addWidget(self.power)
        main.addLayout(head)

        self.banner = QLabel(''); self.banner.setWordWrap(True)
        main.addWidget(self.banner)

        # mode
        mc = self._card(); ml = QVBoxLayout(mc); ml.setContentsMargins(12, 10, 12, 12)
        s = QLabel('MODE'); s.setObjectName('section'); ml.addWidget(s)
        row = QHBoxLayout(); self.mode_group = QButtonGroup(self); self.mode_buttons = {}
        for m, label in (('direction', 'Direction   (w s = drive,  a d = turn)'), ('actuator', 'Actuator   (one key per thruster)')):
            b = QPushButton(label); b.setObjectName('mode'); b.setCheckable(True); b.setFocusPolicy(Qt.NoFocus)
            b.clicked.connect(lambda _c, mm=m: self.set_param('control_mode', mm))
            self.mode_group.addButton(b); self.mode_buttons[m] = b; row.addWidget(b)
        ml.addLayout(row)
        main.addWidget(mc)

        # keys
        kc = self._card(); self.kl = QGridLayout(kc); self.kl.setContentsMargins(12, 10, 12, 12)
        self.kl.setHorizontalSpacing(10); self.kl.setVerticalSpacing(6)
        main.addWidget(kc)
        self.keycaps = {}

        # commands
        cc = self._card(); cl = QVBoxLayout(cc); cl.setContentsMargins(12, 10, 12, 12)
        s = QLabel('COMMANDS'); s.setObjectName('section'); cl.addWidget(s)
        self.bars = {'T3': CenterBar('T3  stern drive', 'N'), 'T1': CenterBar('T1  port bow', 'N'),
                     'T2': CenterBar('T2  starboard bow', 'N'), 'steer': CenterBar('Steering', 'deg')}
        for b in self.bars.values():
            cl.addWidget(b)
        main.addWidget(cc)

        stop = QPushButton('ALL STOP   (space)'); stop.setObjectName('stop'); stop.setFocusPolicy(Qt.NoFocus)
        stop.clicked.connect(self.all_stop)
        main.addWidget(stop)
        self.status = QLabel(''); self.status.setObjectName('sub'); self.status.setWordWrap(True)
        main.addWidget(self.status)
        main.addStretch(1)
        mw = QWidget(); mw.setLayout(main); mw.setMinimumWidth(460)
        outer.addWidget(mw, 3)

        # ---- side panel: parameters
        side = QVBoxLayout(); side.setSpacing(4)
        s = QLabel('PARAMETERS'); s.setObjectName('section'); side.addWidget(s)
        self.fields = {}
        groups = [
            ('Step && feel', [('thrust_step', 'thrust step', 'N / tap', 0.1, 500, 1),
                             ('steer_step_deg', 'steering step', 'deg / tap', 0.1, 21, 1),
                             ('accel_gain', 'hold accel gain', 'x / repeat', 0, 10, 2),
                             ('accel_max', 'hold accel max', 'x', 1, 500, 1),
                             ('repeat_delay_ms', 'repeat delay', 'ms', 0, 2000, 0),
                             ('repeat_hz', 'repeat rate', 'Hz', 1, 100, 1)]),
            ('Thrust limits  (N)', [('t1_min_thrust', 'T1 min', 'N', -2000, 0, 3), ('t1_max_thrust', 'T1 max', 'N', 0, 2000, 3),
                                    ('t2_min_thrust', 'T2 min', 'N', -2000, 0, 3), ('t2_max_thrust', 'T2 max', 'N', 0, 2000, 3),
                                    ('t3_min_thrust', 'T3 min', 'N', -5000, 0, 1), ('t3_max_thrust', 'T3 max', 'N', 0, 5000, 1),
                                    ('steer_max', 'steering max', 'rad', 0.01, 1.57, 4)]),
            ('Output', [('publish_rate', 'publish rate', 'Hz', 1, 200, 1)]),
        ]
        for title, items in groups:
            g = QGroupBox(title); f = QFormLayout(g); f.setLabelAlignment(Qt.AlignLeft)
            if title.startswith('Step'):
                rz = QCheckBox('return to zero on release  (direction mode)')
                rz.setFocusPolicy(Qt.NoFocus)
                rz.setToolTip('Joystick: hold a key to ramp its axis up, release it and the axis goes back to 0.\n'
                              'Off: the value stays after release (like my_boat_teleop).')
                rz.toggled.connect(lambda on: self.set_param('return_to_zero', bool(on)))
                self.fields['return_to_zero'] = rz; f.addRow(rz)
            for key, label, unit, lo, hi, dec in items:
                sb = QDoubleSpinBox(); sb.setRange(lo, hi); sb.setDecimals(dec); sb.setSuffix('  ' + unit)
                sb.setButtonSymbols(QAbstractSpinBox.NoButtons); sb.setKeyboardTracking(False)
                sb.valueChanged.connect(lambda v, k=key: self.set_param(k, v))
                self.fields[key] = sb; f.addRow(label, sb)
            if title == 'Output':
                ns = QLineEdit(); ns.editingFinished.connect(lambda: self.set_param('boat_namespace', self.fields['boat_namespace'].text().strip() or 'my_boat'))
                self.fields['boat_namespace'] = ns; f.addRow('boat namespace', ns)
                self.zero_off = QCheckBox('send zeros when switched OFF'); self.zero_off.setChecked(True)
                self.zero_off.setFocusPolicy(Qt.NoFocus); f.addRow(self.zero_off)
            side.addWidget(g)
        reset = QPushButton('Reset to defaults'); reset.setFocusPolicy(Qt.NoFocus)
        reset.clicked.connect(self.reset_defaults); side.addWidget(reset)
        side.addStretch(1)
        sw = QWidget(); sw.setLayout(side)
        sc = QScrollArea(); sc.setWidget(sw); sc.setWidgetResizable(True); sc.setFrameShape(QFrame.NoFrame)
        sc.setMinimumWidth(310); sc.setStyleSheet('QScrollArea { background: transparent; }')
        outer.addWidget(sc, 2)
        self.setCentralWidget(root)
        self.resize(900, 700)

    def _build_keys(self):
        while self.kl.count():
            w = self.kl.takeAt(0).widget()
            if w:
                w.deleteLater()
        s = QLabel('KEYS'); s.setObjectName('section'); self.kl.addWidget(s, 0, 0, 1, 4)
        self.keycaps = {}
        km = keymap(self.core.mode)
        if self.core.mode == 'direction':
            rows = [('w', km['w']), ('s', km['s']), ('a', km['a']), ('d', km['d'])]
        else:
            rows = [('w', km['w']), ('W', km['W']), ('a', km['a']), ('A', km['A']),
                    ('d', km['d']), ('D', km['D']), ('s', km['s']), ('S', km['S'])]
        for i, (k, desc) in enumerate(rows):
            r, c = 1 + i // 2, (i % 2) * 2
            cap = QLabel(('Shift+' + k.lower()) if k.isupper() else k); cap.setObjectName('keycap')
            cap.setProperty('down', False)
            d = QLabel(desc); d.setObjectName('keydesc')
            self.kl.addWidget(cap, r, c); self.kl.addWidget(d, r, c + 1)
            self.keycaps[k] = cap
        if self.core.mode == 'direction':
            note = ('hold = more, up to the limit  ·  RELEASE = back to 0' if self.core.params['return_to_zero']
                    else 'hold = more  ·  the value STAYS after release (return to zero is off)')
            n = QLabel(note); n.setObjectName('keydesc'); self.kl.addWidget(n, 98, 0, 1, 4)
        cap = QLabel('space'); cap.setObjectName('keycap'); cap.setProperty('down', False)
        self.kl.addWidget(cap, 99, 0); self.kl.addWidget(QLabel('ALL-STOP     ·     F9 = switch OFF from any window'), 99, 1, 1, 3)
        self.keycaps['space'] = cap

    # ------------------------------------------------------------ state
    def set_param(self, name, value):
        if self.core.params.get(name) == value:
            return
        r = self.node.set_parameters([Parameter(name, value=value)])[0]
        if not r.successful:
            self.status.setText(f'refused: {r.reason}')
            self.refresh_all()

    def on_param(self, name):                 # from the ROS side (or the GUI)
        if name in ('control_mode', 'return_to_zero'):
            self._build_keys()
        self.refresh_all()

    def reset_defaults(self):
        self.node.set_parameters([Parameter(k, value=v) for k, v in DEFAULTS.items() if k != 'control_mode'])
        self.refresh_all()

    def set_on(self, on):
        self.on = on
        if not on and self.zero_off.isChecked():
            self.core.all_stop()
            self.node.publish()               # one zero set, then silence
        self.core.update(set(), time.monotonic())
        self.refresh_all()

    def all_stop(self):
        self.core.all_stop()
        if self.on:
            self.node.publish()
        self.refresh_bars()

    def refresh_all(self):
        p = self.core.params
        for k, w in self.fields.items():
            if w.hasFocus():
                continue
            w.blockSignals(True)
            if isinstance(w, QLineEdit):
                w.setText(str(p[k]))
            elif isinstance(w, QCheckBox):
                w.setChecked(bool(p[k]))
            else:
                w.setValue(float(p[k]))
            w.blockSignals(False)
        for m, b in self.mode_buttons.items():
            b.setChecked(p['control_mode'] == m)
        if not self.keycaps:
            self._build_keys()
        self.power.setText('ON' if self.on else 'OFF')
        self.power.setProperty('on', self.on)
        self.power.style().unpolish(self.power); self.power.style().polish(self.power)
        if self.on and self.keys.ok:
            self.banner.setObjectName('warn')
            self.banner.setText('Listening to the keyboard in EVERY window: typing w a s d anywhere drives the boat. '
                                'F9 (from any window) or the button switches it OFF.')
        elif self.on:
            self.banner.setObjectName('warn')
            self.banner.setText(f'Global keys unavailable ({self.keys.why}): keys work only while THIS window is focused.')
        else:
            self.banner.setObjectName('info')
            self.banner.setText('OFF: keys are ignored and nothing is published. Switch ON to drive.')
        self.banner.style().unpolish(self.banner); self.banner.style().polish(self.banner)
        self.subtitle.setText(f"{p['control_mode']} mode  ·  namespace /{p['boat_namespace'].strip('/')}")
        self.refresh_bars()

    def refresh_bars(self):
        p, c = self.core.params, self.core
        self.bars['T1'].set(c.t1, p['t1_min_thrust'], p['t1_max_thrust'])
        self.bars['T2'].set(c.t2, p['t2_min_thrust'], p['t2_max_thrust'])
        self.bars['T3'].set(c.t3, p['t3_min_thrust'], p['t3_max_thrust'])
        self.bars['steer'].set(math.degrees(c.steer), -math.degrees(p['steer_max']), math.degrees(p['steer_max']))

    # ------------------------------------------------------------ keys + loop
    def _typing_here(self):
        f = QApplication.focusWidget()
        return self.isActiveWindow() and isinstance(f, (QLineEdit, QDoubleSpinBox))

    def keyPressEvent(self, e):
        if not e.isAutoRepeat():
            self.local_down.add(self._name(e))

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat():
            self.local_down.discard(self._name(e))

    @staticmethod
    def _name(e):
        k = e.key()
        return {Qt.Key_W: 'w', Qt.Key_A: 'a', Qt.Key_S: 's', Qt.Key_D: 'd', Qt.Key_Space: 'space',
                Qt.Key_Shift: 'shift', Qt.Key_F9: 'f9'}.get(k, '')

    def loop(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        down = self.keys.down() if self.keys.ok else set(self.local_down)
        f9 = 'f9' in down
        if f9 and not self._f9 and self.on:
            self.set_on(False)
        self._f9 = f9
        if self._typing_here():
            down = set()
        now = time.monotonic()
        if self.on:
            if self.core.update(down - {'f9'}, now):
                self.refresh_bars()
            period = 1.0 / self.core.params['publish_rate']
            if now - self._last_pub >= period:
                # schedule from the LAST publish, not from now, so timer jitter
                # does not slow the rate; resync if we fell a whole period behind
                self._last_pub = self._last_pub + period if now - self._last_pub < 2 * period else now
                self.node.publish()
        shown = down if self.on else set()
        shift = 'shift' in shown and self.core.mode == 'actuator'
        for k, cap in self.keycaps.items():
            if k == 'space':
                d = 'space' in shown
            elif k.isupper():
                d = shift and k.lower() in shown
            else:
                d = k in shown and not (shift and self.core.mode == 'actuator')
            if cap.property('down') != d:
                cap.setProperty('down', d); cap.style().unpolish(cap); cap.style().polish(cap)
        if self.on:
            src = 'global X11 keyboard' if self.keys.ok else 'this window only'
            self.status.setText(f'ON  ·  keys from {src}  ·  {self.core.params["publish_rate"]:.0f} Hz to '
                                f'{self.node.base}T1/thrust, T2/thrust, T3/thrust, T3/pos')
        else:
            self.status.setText(f'OFF  ·  nothing published  ·  topics: {self.node.base}T1/thrust, T2/thrust, T3/thrust, T3/pos')

    def closeEvent(self, e):
        if self.on:
            self.core.all_stop()
            self.node.publish()
        super().closeEvent(e)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])
    core = TeleopCore()
    holder = {}
    node = TeleopNode(core, lambda name: holder['w'].on_param(name) if 'w' in holder else None)
    w = Window(node, core)
    holder['w'] = w
    w.show()
    try:
        app.exec_() if hasattr(app, 'exec_') else app.exec()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
