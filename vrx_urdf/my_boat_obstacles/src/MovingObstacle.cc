// MovingObstacle -- play back a pre-computed trajectory on a model, in SIM TIME.
//
// All scenario logic (boat-relative spawn, encounter geometry, waypoints,
// turns) lives in the Python spawner, which is unit-tested. This plugin is
// deliberately dumb: it receives timed samples and moves the model through
// them. That keeps the part that is hard to test (inside Gazebo) trivial.
//
//   <plugin filename="MovingObstacle" name="my_boat_obstacles::MovingObstacle">
//     <z>0.5</z>                                   <!-- model origin height -->
//     <run_topic>/my_boat_obstacles/run</run_topic>
//     <samples_topic>/my_boat_obstacles/samples</samples_topic>
//     <samples>tau x y yaw  tau x y yaw ...</samples> <!-- tau = s since RUN, world frame -->
//   </plugin>
//
// NOTHING MOVES UNTIL RUN. The spawner publishes gz.msgs.Double on run_topic:
//   data >= 0   the absolute SIM time at which tau = 0 (every obstacle listens
//               to the same topic, so they all start at the same sim instant,
//               and the spawner's ground truth uses the same number)
//   data <  0   not running: hold the first sample (this is what reset sends)
// Until the first message arrives the model holds its first sample. After the
// last sample it holds the last one. The spawner re-sends the current value
// every second, so a dropped message only delays it and an obstacle spawned
// later still picks up the value.
//
// NEW PATH AT RUN TIME. gz.msgs.StringMsg on samples_topic, text
//   "<model name> tau x y yaw tau x y yaw ..."
// replaces this model's samples (messages naming another model are ignored,
// so one topic serves every obstacle). This is how the spawner's set_start
// service moves an obstacle without respawning it.
//
// HOW IT MOVES: the model is non-static and has no gravity. Every PreUpdate it
// is TELEPORTED to the interpolated pose (WorldPoseCmd) AND given the matching
// body-frame velocity (Linear/AngularVelocityCmd). The teleport makes the
// path exact regardless of contacts, and the velocity keeps whatever
// physics/pose publishing happens between teleports consistent. A static model
// cannot be used: physics never moves a static body's collision, so the boat
// could sail through the obstacle while the lidar still saw it.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <sstream>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

namespace my_boat_obstacles
{
struct Sample
{
  double t, x, y, yaw;
};

class MovingObstacle : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "MovingObstacle must be attached to a model\n";
      return;
    }
    this->name = this->model.Name(_ecm);
    this->z = _sdf->Get<double>("z", 0.0).first;
    const auto topic = _sdf->Get<std::string>("run_topic", "/my_boat_obstacles/run").first;
    this->node.Subscribe(topic, &MovingObstacle::OnRun, this);

    std::vector<Sample> parsed;
    if (!Parse(_sdf->Get<std::string>("samples", "").first, parsed))
      return;
    this->samples = std::move(parsed);
    const auto samplesTopic =
        _sdf->Get<std::string>("samples_topic", "/my_boat_obstacles/samples").first;
    this->node.Subscribe(samplesTopic, &MovingObstacle::OnSamples, this);
    gzmsg << "MovingObstacle [" << this->name << "]: " << this->samples.size()
          << " samples, t " << this->samples.front().t << " .. "
          << this->samples.back().t << " s\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->samples.empty())
      return;
    const double t0 = this->runAt.load();
    // tau < samples.front().t (= 0) holds the first pose: not running yet
    const double t = t0 < 0.0
        ? this->samples.front().t - 1.0
        : std::chrono::duration<double>(_info.simTime).count() - t0;

    Sample p;
    double vx = 0.0, vy = 0.0, r = 0.0;
    const auto &S = this->samples;
    if (t <= S.front().t)
    {
      p = S.front();
    }
    else if (t >= S.back().t)
    {
      p = S.back();
    }
    else
    {
      // first sample strictly after t
      auto hi = std::upper_bound(S.begin(), S.end(), t,
          [](double v, const Sample &a) { return v < a.t; });
      const Sample &b = *hi;
      const Sample &a = *(hi - 1);
      const double dt = b.t - a.t;
      const double f = (t - a.t) / dt;
      const double dyaw = std::atan2(std::sin(b.yaw - a.yaw),
                                     std::cos(b.yaw - a.yaw));  // short way
      p = {t, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), a.yaw + f * dyaw};
      vx = (b.x - a.x) / dt;
      vy = (b.y - a.y) / dt;
      r = dyaw / dt;
    }

    // the model was just deleted (spawner remove/respawn) but this system has
    // not been unloaded yet: leave it alone, or gz logs "create a component
    // ... entity does not exist" every step
    if (!_ecm.HasEntity(this->model.Entity()))
      return;
    this->model.SetWorldPoseCmd(
        _ecm, gz::math::Pose3d(p.x, p.y, this->z, 0.0, 0.0, p.yaw));

    // velocity commands are BODY frame
    const double c = std::cos(p.yaw), s = std::sin(p.yaw);
    const gz::math::Vector3d vBody(c * vx + s * vy, -s * vx + c * vy, 0.0);
    const gz::math::Vector3d wBody(0.0, 0.0, r);
    const auto e = this->model.Entity();
    if (!_ecm.HasEntity(e))
      return;
    if (!_ecm.Component<gz::sim::components::LinearVelocityCmd>(e))
      _ecm.CreateComponent(e, gz::sim::components::LinearVelocityCmd(vBody));
    else
      _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(e, vBody);
    if (!_ecm.Component<gz::sim::components::AngularVelocityCmd>(e))
      _ecm.CreateComponent(e, gz::sim::components::AngularVelocityCmd(wBody));
    else
      _ecm.SetComponentData<gz::sim::components::AngularVelocityCmd>(e, wBody);
  }

private:
  bool Parse(const std::string &_text, std::vector<Sample> &_out) const
  {
    std::istringstream in(_text);
    Sample s;
    while (in >> s.t >> s.x >> s.y >> s.yaw)
      _out.push_back(s);
    if (_out.empty())
    {
      gzerr << "MovingObstacle [" << this->name << "]: no samples\n";
      return false;
    }
    for (size_t i = 1; i < _out.size(); ++i)
    {
      if (!(_out[i].t > _out[i - 1].t))
      {
        gzerr << "MovingObstacle [" << this->name
              << "]: sample times must be strictly increasing (index " << i
              << ")\n";
        return false;
      }
    }
    return true;
  }

  void OnSamples(const gz::msgs::StringMsg &_msg)
  {
    const std::string &text = _msg.data();
    const auto sp = text.find(' ');
    if (sp == std::string::npos || text.compare(0, sp, this->name) != 0)
      return;  // addressed to another obstacle
    std::vector<Sample> parsed;
    if (!Parse(text.substr(sp + 1), parsed))
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    this->samples = std::move(parsed);
    gzmsg << "MovingObstacle [" << this->name << "]: new path, "
          << this->samples.size() << " samples, starts at ("
          << this->samples.front().x << ", " << this->samples.front().y << ")\n";
  }

  void OnRun(const gz::msgs::Double &_msg)
  {
    this->runAt.store(_msg.data());
  }

  gz::transport::Node node;
  std::atomic<double> runAt{-1.0};
  std::mutex mutex;
  gz::sim::Model model{gz::sim::kNullEntity};
  std::string name;
  double z{0.0};
  std::vector<Sample> samples;
};
}  // namespace my_boat_obstacles

GZ_ADD_PLUGIN(my_boat_obstacles::MovingObstacle,
              gz::sim::System,
              my_boat_obstacles::MovingObstacle::ISystemConfigure,
              my_boat_obstacles::MovingObstacle::ISystemPreUpdate)
