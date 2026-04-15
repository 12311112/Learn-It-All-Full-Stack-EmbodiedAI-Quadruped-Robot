import atexit
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass


@dataclass
class _Button:
    triggered: bool = False


class _Buttons:
    def __init__(self, a_triggered=False):
        self.A = _Button(False)
        self.A.triggered = a_triggered


class XBoxController:
    """
    Keyboard-backed controller compatible with the existing walk loop.

    Controls:
    - W/S: forward/backward
    - A/D: left/right strafe
    - Q/E: yaw left/right
    - Space: A button (toggle pause in walk loop)
    """

    def __init__(self, command_freq=20):
        self.command_freq = float(command_freq)
        self._key_hold_s = 0.18
        self._cmd_scale = 1.0

        self.buttons = _Buttons()
        self._lock = threading.Lock()
        self._running = True

        self._last_press = {
            "w": 0.0,
            "s": 0.0,
            "a": 0.0,
            "d": 0.0,
            "q": 0.0,
            "e": 0.0,
        }

        self._stdin_fd = None
        self._stdin_settings = None
        if sys.stdin.isatty():
            self._stdin_fd = sys.stdin.fileno()
            self._stdin_settings = termios.tcgetattr(self._stdin_fd)
            tty.setcbreak(self._stdin_fd)
            atexit.register(self.close)
            print(
                "[keyboard] enabled: W/S forward, A/D strafe, Q/E yaw, SPACE pause"
            )
        else:
            print("[keyboard] stdin is not a tty, keyboard control disabled")

        self._thread = threading.Thread(target=self._keyboard_worker, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._stdin_fd is not None and self._stdin_settings is not None:
            try:
                termios.tcsetattr(
                    self._stdin_fd, termios.TCSADRAIN, self._stdin_settings
                )
            except Exception:
                pass

    def _keyboard_worker(self):
        dt = 1.0 / max(self.command_freq, 1.0)
        while self._running:
            if self._stdin_fd is None:
                time.sleep(dt)
                continue

            readable, _, _ = select.select([sys.stdin], [], [], dt)
            if not readable:
                continue

            try:
                ch = sys.stdin.read(1)
            except Exception:
                continue

            now = time.time()
            ch = ch.lower()
            with self._lock:
                if ch in self._last_press:
                    self._last_press[ch] = now
                elif ch == " ":
                    self.buttons.A.triggered = True

    def _active(self, key, now):
        return (now - self._last_press[key]) <= self._key_hold_s

    def _compute_commands(self):
        now = time.time()
        forward = float(self._active("w", now)) - float(self._active("s", now))
        lateral = float(self._active("a", now)) - float(self._active("d", now))
        yaw = float(self._active("q", now)) - float(self._active("e", now))

        return [
            self._cmd_scale * forward,
            self._cmd_scale * lateral,
            self._cmd_scale * yaw,
        ]

    def get_last_command(self):
        with self._lock:
            cmds = self._compute_commands()
            a_triggered = self.buttons.A.triggered
            buttons = _Buttons(a_triggered=a_triggered)
            left_trigger = 0.0
            right_trigger = 0.0
            # Edge-trigger: expose once, then clear internal state.
            self.buttons.A.triggered = False

        return cmds, buttons, left_trigger, right_trigger
