# my_boat_teleop_gui

The my_boat keyboard teleop in a window, and **the window does not need the
focus**. While it's ON, `w a s d` / Shift / space are read from the keyboard
whatever window you're in (Gazebo, RViz, the SIMU page), so you can watch the
boat and drive it at the same time.

```bash
ros2 launch my_boat_gazebo my_boat_world.launch.py          # the v1 boat
ros2 launch my_boat_teleop_gui teleop_gui.launch.py         # this window (ros2 launch works here)
```

- **ON / OFF** (top right). OFF: keys are ignored and nothing is published.
  Switching OFF sends one set of zeros first (tick box in the panel).
  **F9 switches it OFF from any window**; switching ON takes a click, so a
  stray F9 never starts the boat.
- **Mode:**
  - **Direction:** `w` forward, `s` reverse, `a`/`d` turn, with the bow
    thrusters helping.
  - **Actuator:** `w a d s` = T3 / T1 / T2 / steering +, and Shift+key = −.
  - `space` = ALL-STOP in both modes.
- **Return to zero (direction mode, on by default):** it behaves like a
  joystick. Hold a key and its axis ramps up to the limit; release it and the
  axis goes back to 0. The axes are `w`/`s` for the stern thrust and `a`/`d`
  for steering, with the bow thrusters following. Holding both keys of an
  axis puts it at 0. Untick it (or `return_to_zero:=false`) for the old
  throttle behaviour, where the value stays after release. Actuator mode
  always keeps its values.
- **Keys light up** as they are pressed; **bars** show T1, T2, T3 and the steering
  against their (asymmetric) limits.
- **Side panel:** every parameter, applied live. They are also ROS parameters:
  `ros2 param set /keyboard_teleop_gui thrust_step 20.0`.

Same topics, keys, limits, zero-detent and direction-mode bow mapping as
`my_boat_teleop` (ported from `keyboard_teleop.cpp`). The differences:

| | terminal teleop | this window |
|---|---|---|
| keyboard | the terminal must be focused | any window (X11) |
| hold to accelerate | needs OS auto-repeat (often off over ssh/VNC) | its own repeat: `repeat_delay_ms` then `repeat_hz` |
| keys at once | one | several (e.g. forward + turn) |
| release in direction mode | value stays (throttle) | back to 0 (joystick), switchable |
| zero detent | stops at 0 for one repeat | parks at 0 until you release the key |
| `ros2 launch` | no keyboard (reads stdin) | works |
| `q` | quits | nothing (so typing q elsewhere is safe) |

**Safety:** while ON, typing `w a s d` in *any* window drives the boat. Switch
it OFF (or press F9) before typing elsewhere. While you type in this window's
own parameter fields, keys are not used for driving.

Global keys use X11 (`XQueryKeymap` through ctypes, no extra packages). They
only read the key state and never grab the keyboard. On Wayland it falls back
to keys while this window is focused.

Don't run it together with `my_boat_teleop`, `gazebo_thruster_bridge` or the
DP/allocator: they publish the same topics.
