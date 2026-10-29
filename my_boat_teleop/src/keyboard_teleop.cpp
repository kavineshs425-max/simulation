// =============================================================================
// keyboard_teleop.cpp
// =============================================================================
// Terminal keyboard teleop for the my_boat USV. Reads raw keystrokes and
// publishes std_msgs/Float64 commands to the thruster topics created by the
// Gazebo Thruster / JointPositionController plugins (bridged to ROS 2 by
// my_boat_gazebo/config/thrusters.yaml):
//
//   /<ns>/thrusters/T1/thrust   (N)    port  bow thruster
//   /<ns>/thrusters/T2/thrust   (N)    stbd  bow thruster
//   /<ns>/thrusters/T3/thrust   (N)    stern drive surge
//   /<ns>/thrusters/T3/pos      (rad)  stern steering angle
//
// Two control modes are selectable with the `control_mode` parameter:
//
// ACTUATOR mode (control_mode:=actuator, default) -- one key per actuator:
//   w / W : T3 stern thrust   +/-   (forward / reverse)
//   a / A : T1 port-bow thrust +/-
//   d / D : T2 stbd-bow thrust +/-
//   s / S : T3 steering angle  +/-  (1 deg per tap)
//   space : ALL-STOP (zero every command)
//   q     : quit (Ctrl-C also works)
//
// DIRECTION mode (control_mode:=direction) -- keys are driving intent:
//   w : forward   (propeller / stern thrust +)
//   s : reverse   (propeller / stern thrust -)
//   a : turn left  (steering rad -, T1 -, T2 -)
//   d : turn right (steering rad +, T1 +, T2 +)
//   space : ALL-STOP    q : quit (Ctrl-C also works)
//   The bow thrusters T1/T2 follow the steering angle, mapped from the steering
//   domain [-steer_max, +steer_max] onto the bow thrust range, so they assist
//   the turn proportionally to the commanded steering angle. They get the SAME
//   sign -- see sync_bow() for why that is not a typo.
//
// Feel:
//   * One tap = 1 unit (1 N of thrust, 1 deg of steering).
//   * PRESS-AND-HOLD accelerates: the longer a key auto-repeats, the bigger the
//     step (up to accel_max x), so you ramp to large values quickly but still
//     have 1-unit precision on single taps.
//   * ZERO DETENT: ramping the opposite way STOPS AT 0 (and resets the
//     acceleration) instead of shooting past, so you can always settle exactly
//     to zero. Press again from 0 to go negative. (space zeroes everything.)
//
// Values are clamped to the BOLLARD-MEASURED hardware limits, which are
// ASYMMETRIC (reverse ~= half of forward) and differ per unit because T2 is
// physically mirrored relative to T1:
//   T1  [-39.2266, +78.4532] N      T2  [-78.4532, +39.2266] N
//   T3 [-274.6,   +554.1  ] N       steering +/-0.3665 rad (+/-21 deg)
// These MUST match thruster_allocator/config/allocator_bollard.yaml and
// my_boat_gazebo/urdf/thrusters/my_boat_thruster_layout.xacro. Gazebo clamps to
// the xacro values, so a teleop limit that is too generous just means the
// status line lies about what the boat is actually being given.
// =============================================================================

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace
{
// Saved terminal settings, restored on exit (global so std::atexit can reach it).
struct termios g_old_termios;
bool g_termios_saved = false;

void restore_terminal()
{
  if (g_termios_saved)
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_termios);
    g_termios_saved = false;
  }
}
}  // namespace

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : rclcpp::Node("keyboard_teleop")
  {
    const std::string ns = this->declare_parameter<std::string>("boat_namespace", "my_boat");
    thrust_step_   = this->declare_parameter<double>("thrust_step", 1.0);          // N per tap
    steer_step_    = this->declare_parameter<double>("steer_step_deg", 1.0)
                     * M_PI / 180.0;                                               // rad per tap
    accel_gain_    = this->declare_parameter<double>("accel_gain", 0.6);           // growth / repeat
    accel_max_     = this->declare_parameter<double>("accel_max", 30.0);           // max x multiplier
    hold_reset_ms_ = this->declare_parameter<double>("hold_reset_ms", 150.0);      // gap that resets accel
    // Per-thruster, signed, asymmetric. Defaults are the bollard measurements.
    t1_min_ = this->declare_parameter<double>("t1_min_thrust", -39.2266);
    t1_max_ = this->declare_parameter<double>("t1_max_thrust", +78.4532);
    t2_min_ = this->declare_parameter<double>("t2_min_thrust", -78.4532);
    t2_max_ = this->declare_parameter<double>("t2_max_thrust", +39.2266);
    t3_min_ = this->declare_parameter<double>("t3_min_thrust", -274.6);
    t3_max_ = this->declare_parameter<double>("t3_max_thrust", +554.1);
    steer_max_     = this->declare_parameter<double>("steer_max", 0.3665);
    const double rate = this->declare_parameter<double>("publish_rate", 20.0);

    // Largest bow command that BOTH units can honour without one of them
    // clamping. T1 and T2 have mirrored limits, so their common range is the
    // smaller magnitude on each side (39.2266 N with the defaults). Direction
    // mode drives the pair together, so it uses this rather than either unit's
    // full range -- otherwise one saturates and the "balanced" pair yaws.
    bow_sync_max_ = std::min(std::min(-t1_min_, t1_max_),
                             std::min(-t2_min_, t2_max_));

    const std::string mode_str =
      this->declare_parameter<std::string>("control_mode", "actuator");
    mode_ = (mode_str == "direction") ? Mode::Direction : Mode::Actuator;

    const std::string base = "/" + ns + "/thrusters/";
    pub_t1_  = create_publisher<std_msgs::msg::Float64>(base + "T1/thrust", 10);
    pub_t2_  = create_publisher<std_msgs::msg::Float64>(base + "T2/thrust", 10);
    pub_t3_  = create_publisher<std_msgs::msg::Float64>(base + "T3/thrust", 10);
    pub_pos_ = create_publisher<std_msgs::msg::Float64>(base + "T3/pos", 10);

    setup_terminal();
    print_help();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KeyboardTeleop::tick, this));
  }

  ~KeyboardTeleop() override { restore_terminal(); }

private:
  // Non-canonical, no-echo, non-blocking stdin so single keys arrive instantly
  // without Enter. ISIG stays on so Ctrl-C still raises SIGINT.
  void setup_terminal()
  {
    tcgetattr(STDIN_FILENO, &g_old_termios);
    g_termios_saved = true;
    struct termios raw = g_old_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }

  void tick()
  {
    bool changed = false;
    char c;
    while (::read(STDIN_FILENO, &c, 1) == 1)
      changed = handle_key(c) || changed;

    publish();              // keep feeding the held command to the plugins
    if (changed) print_status();
  }

  // Returns the acceleration multiplier for this keystroke, growing while the
  // same key auto-repeats (held), resetting after a gap or on a key change.
  double accel_for(char c)
  {
    const auto now = std::chrono::steady_clock::now();
    const double gap_ms =
      std::chrono::duration<double, std::milli>(now - last_time_).count();
    if (c != last_key_ || gap_ms > hold_reset_ms_)
      repeat_ = 0;
    else
      ++repeat_;
    last_key_ = c;
    last_time_ = now;
    return std::min(accel_max_, 1.0 + repeat_ * accel_gain_);
  }

  bool handle_key(char c)
  {
    if (c == 'q') { restore_terminal(); rclcpp::shutdown(); return false; }
    if (c == ' ') { t1_ = t2_ = t3_ = steer_ = 0.0; repeat_ = 0; return true; }

    return (mode_ == Mode::Direction) ? handle_key_direction(c)
                                      : handle_key_actuator(c);
  }

  // ACTUATOR mode: one key drives one actuator directly.
  bool handle_key_actuator(char c)
  {
    const double a = accel_for(c);
    switch (c)
    {
      case 'w': step(t3_,    +thrust_step_ * a, t3_min_, t3_max_);       return true;
      case 'W': step(t3_,    -thrust_step_ * a, t3_min_, t3_max_);       return true;
      case 'a': step(t1_,    +thrust_step_ * a, t1_min_, t1_max_);       return true;
      case 'A': step(t1_,    -thrust_step_ * a, t1_min_, t1_max_);       return true;
      case 'd': step(t2_,    +thrust_step_ * a, t2_min_, t2_max_);       return true;
      case 'D': step(t2_,    -thrust_step_ * a, t2_min_, t2_max_);       return true;
      case 's': step(steer_, +steer_step_  * a, -steer_max_, steer_max_); return true;
      case 'S': step(steer_, -steer_step_  * a, -steer_max_, steer_max_); return true;
      default:  return false;
    }
  }

  // DIRECTION mode: keys are driving intent. Forward/reverse drive the stern
  // propeller; left/right drive the steering angle, and the bow thrusters are
  // derived from that angle by sync_bow().
  bool handle_key_direction(char c)
  {
    const double a = accel_for(c);
    switch (c)
    {
      case 'w': step(t3_,    +thrust_step_ * a, t3_min_, t3_max_);                    return true;
      case 's': step(t3_,    -thrust_step_ * a, t3_min_, t3_max_);                    return true;
      case 'a': step(steer_, -steer_step_  * a, -steer_max_, steer_max_); sync_bow(); return true;
      case 'd': step(steer_, +steer_step_  * a, -steer_max_, steer_max_); sync_bow(); return true;
      default:  return false;
    }
  }

  // Map the steering angle onto the bow thrusters so they assist the turn:
  // steering [-steer_max, +steer_max] -> bow thrust [-bow_sync_max, +bow_sync_max].
  //
  // T2 GETS THE SAME SIGN AS T1, NOT THE OPPOSITE. This used to be t2_ = -t1_,
  // which was correct only while the two bow units were mounted ANTI-PARALLEL
  // (T1 starboard-positive, T2 port-positive). They are now both
  // STARBOARD-POSITIVE (mount yaw -pi/2 on both, matching the allocator's
  // t1_dir = t2_dir = -1), and with the pair parallel and at the SAME
  // longitudinal station x = +1.03 m, opposite signs cancel EXACTLY:
  //     Fy = -T1 - T2 = -T1 + T1 = 0
  //     Mz = -l1*(T1 + T2)       = 0
  // i.e. the old code now contributes precisely nothing to the turn. Same sign
  // gives a bow side-force, which is what "assist the turn" means:
  //     right (steer > 0): T1 > 0, T2 > 0 -> bow pushed to starboard
  //     left  (steer < 0): T1 < 0, T2 < 0 -> bow pushed to port
  // If you ever flip t1_dir/t2_dir back to an opposed pair, restore t2_ = -t1_.
  void sync_bow()
  {
    t1_ = map(steer_, -steer_max_, steer_max_, -bow_sync_max_, bow_sync_max_);
    t2_ = t1_;
  }

  // Linear remap of x from [in_min, in_max] onto [out_min, out_max]
  // (Arduino-style map(), in floating point).
  static double map(double x, double in_min, double in_max,
                    double out_min, double out_max)
  {
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
  }

  // Apply delta with a ZERO DETENT: if the step would cross zero, stop at zero
  // and reset the hold acceleration, so the opposite key reliably brings the
  // value back to exactly 0 (and only goes negative on the next press).
  // lo/hi are the unit's own signed limits and need not be symmetric.
  void step(double & val, double delta, double lo, double hi)
  {
    double n = val + delta;
    if ((val > 0.0 && n < 0.0) || (val < 0.0 && n > 0.0))
    {
      n = 0.0;
      repeat_ = 0;
    }
    val = std::max(lo, std::min(hi, n));
  }

  void publish()
  {
    std_msgs::msg::Float64 m;
    m.data = t1_;    pub_t1_->publish(m);
    m.data = t2_;    pub_t2_->publish(m);
    m.data = t3_;    pub_t3_->publish(m);
    m.data = steer_; pub_pos_->publish(m);
  }

  void print_help() const
  {
    if (mode_ == Mode::Direction)
    {
      std::printf(
        "\n=== my_boat keyboard teleop [DIRECTION mode] ===\n"
        "  w : forward      (stern propeller thrust +)\n"
        "  s : reverse      (stern propeller thrust -)\n"
        "  a : turn left    (steer -, T1 -, T2 -)\n"
        "  d : turn right   (steer +, T1 +, T2 +)\n"
        "  space : ALL-STOP    q : quit (Ctrl-C also)\n"
        "  bow thrusters T1/T2 track the steering angle (mapped) to assist the turn.\n"
        "  tap = 1 unit; HOLD to accelerate; opposite key stops at 0.\n\n");
    }
    else
    {
      std::printf(
        "\n=== my_boat keyboard teleop [ACTUATOR mode] ===\n"
        "  w / W : T3 stern thrust   +/-  (forward / reverse)\n"
        "  a / A : T1 port-bow thrust +/-\n"
        "  d / D : T2 stbd-bow thrust +/-\n"
        "  s / S : T3 steering angle  +/-\n"
        "  space : ALL-STOP    q : quit (Ctrl-C also)\n"
        "  tap = 1 unit (1 N / 1 deg); HOLD to accelerate; opposite key stops at 0.\n\n");
    }
    std::fflush(stdout);
  }

  void print_status() const
  {
    std::printf("\rT1:%+7.1f N  T2:%+7.1f N  T3:%+7.1f N  steer:%+6.3f rad (%+5.1f deg)   ",
                t1_, t2_, t3_, steer_, steer_ * 180.0 / M_PI);
    std::fflush(stdout);
  }

  enum class Mode { Actuator, Direction };

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_t1_, pub_t2_, pub_t3_, pub_pos_;
  rclcpp::TimerBase::SharedPtr timer_;

  Mode mode_ = Mode::Actuator;
  // commanded state
  double t1_ = 0.0, t2_ = 0.0, t3_ = 0.0, steer_ = 0.0;
  // tuning
  double thrust_step_, steer_step_, accel_gain_, accel_max_, hold_reset_ms_;
  // Signed per-thruster limits (asymmetric; T2's are the mirror of T1's).
  double t1_min_, t1_max_, t2_min_, t2_max_, t3_min_, t3_max_;
  double steer_max_, bow_sync_max_;
  // hold-acceleration tracking
  char last_key_ = 0;
  long repeat_ = 0;
  std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::atexit(restore_terminal);          // restore TTY even on abnormal exit
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  restore_terminal();
  std::printf("\n");
  return 0;
}
