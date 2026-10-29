"""Which keys are held RIGHT NOW, whatever window has the focus (X11).

Uses libX11's XQueryKeymap through ctypes, so it needs no extra Python
package and no special permissions. It READS the key state; it does not grab
the keyboard -- the focused window still receives your typing as usual. (That
is also why, while the teleop is ON, typing w/a/s/d anywhere drives the boat.)

X11 only. Under Wayland XQueryKeymap sees just XWayland windows, so global
listening is not possible there; the GUI then falls back to its own window's
key events.
"""

import ctypes
import ctypes.util
import os

WATCH = {'w': 'w', 'a': 'a', 's': 's', 'd': 'd', 'space': 'space',
         'shift': ('Shift_L', 'Shift_R'), 'f9': 'F9'}


class GlobalKeys:
    def __init__(self):
        self.ok = False
        self.why = ''
        if not os.environ.get('DISPLAY'):
            self.why = 'no DISPLAY (not an X11 session)'
            return
        if os.environ.get('XDG_SESSION_TYPE') == 'wayland':
            self.why = 'Wayland session: global key state is not available'
            return
        name = ctypes.util.find_library('X11')
        if not name:
            self.why = 'libX11 not found'
            return
        x = ctypes.cdll.LoadLibrary(name)
        x.XOpenDisplay.restype = ctypes.c_void_p
        x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x.XStringToKeysym.restype = ctypes.c_ulong
        x.XStringToKeysym.argtypes = [ctypes.c_char_p]
        x.XKeysymToKeycode.restype = ctypes.c_ubyte
        x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        x.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char * 32]
        self._x = x
        self._d = x.XOpenDisplay(None)
        if not self._d:
            self.why = f"cannot open X display {os.environ.get('DISPLAY')}"
            return
        self._codes = {}
        for key, syms in WATCH.items():
            syms = (syms,) if isinstance(syms, str) else syms
            self._codes[key] = [x.XKeysymToKeycode(self._d, x.XStringToKeysym(s.encode())) for s in syms]
        self._buf = (ctypes.c_char * 32)()
        self.ok = True

    def down(self):
        """-> set of held keys among w a s d space shift f9."""
        if not self.ok:
            return set()
        self._x.XQueryKeymap(self._d, self._buf)
        bits = bytes(self._buf)
        return {k for k, codes in self._codes.items()
                if any(c and bits[c >> 3] & (1 << (c & 7)) for c in codes)}
