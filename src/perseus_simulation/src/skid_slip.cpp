// Skid-steer slip for the Perseus simulation.
//
// On the real rover, loose sand scrubs the tyres sideways when it turns, so the body
// yaws far slower than the wheels' speeds say it should. In the simulation the wheels
// are velocity-driven and the ground is firm enough that the body follows them
// exactly, so nav2 never sees the failure it has to tune against.
//
// This world system reproduces it. Every physics step it
//
//   1. reads the wheels' joint speeds and works out the yaw rate the diff drive is
//      asking for:  w_wheels = r * (mean right wheel rad/s - mean left wheel rad/s) / W
//   2. reads the chassis' real yaw rate, and
//   3. applies a torque about the world z axis, k * (target - actual), where
//      target = yaw_efficiency * w_wheels.
//
// yaw_efficiency is the fraction of the commanded turn the rover actually achieves:
// 1.0 turns exactly as commanded (the plugin does nothing), 0.2 yaws at a fifth of
// it. Forward motion is not touched, so the wheels still drive normally and only the
// turning is dulled.
//
// The torque is a PI controller on the yaw RATE, not a model of the sand, so what it
// reproduces is "turns at a fraction of the commanded rate", which is what the
// controllers have to cope with. Its gain is sized from the rover's own yaw inertia,
// estimated from its links, so that an error closes in yaw_response_s seconds
// (torque = inertia / yaw_response_s * error). The wheels push the body through the
// ground with a torque of their own - about mu * weight * half the track - and a
// spring alone settles where the two balance, well short of the target. So the
// controller also integrates the error (time constant yaw_integral_s), raising the
// torque until the body is at the target; max_torque_nm must sit above what the
// wheels transmit. If the achieved ratio still comes out higher than yaw_efficiency,
// the wheels are out-pushing the cap, so raise it; if the yaw hunts, raise
// yaw_response_s and yaw_integral_s.
//
// The efficiency can be changed while the simulation runs, to find the value that
// matches the rover:
//   gz topic -t /skid_slip/yaw_efficiency -m gz.msgs.Double -p 'data: 0.2'
//
// SDF parameters (all optional; defaults match the Perseus model and its diff drive):
//   robot_model, base_link, left_wheels, right_wheels (space-separated joint names),
//   wheel_radius, wheel_separation, yaw_efficiency, yaw_response_s, yaw_integral_s,
//   yaw_stiffness (N m s/rad; 0 sizes it from the inertia), max_torque_nm,
//   active_above_rad_s, topic.

#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <sstream>
#include <string>
#include <vector>

namespace perseus_simulation
{
using gz::math::Vector3d;
using gz::sim::Entity;
using gz::sim::kNullEntity;

class SkidSlip : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
{
public:
    void Configure(const Entity& /*world*/, const std::shared_ptr<const sdf::Element>& sdf,
                   gz::sim::EntityComponentManager& /*ecm*/,
                   gz::sim::EventManager& /*eventMgr*/) override
    {
        auto get = [&](const char* name, auto def) {
            return sdf->Get<decltype(def)>(name, def).first;
        };
        _robotName = get("robot_model", std::string("perseus"));
        _baseLink = get("base_link", std::string("base_link"));
        _left = Split(get("left_wheels", std::string("front_left_wheel_joint rear_left_wheel_joint")));
        _right = Split(get("right_wheels", std::string("front_right_wheel_joint rear_right_wheel_joint")));
        _radius = get("wheel_radius", 0.147);
        _separation = get("wheel_separation", 0.714);
        _responseS = std::max(get("yaw_response_s", 0.02), 0.005);
        _integralS = std::max(get("yaw_integral_s", 0.3), 0.02);
        _stiffnessOverride = get("yaw_stiffness", 0.0);
        _maxTorque = get("max_torque_nm", 1000.0);
        _activeAbove = get("active_above_rad_s", 0.05);
        _efficiency = std::clamp(get("yaw_efficiency", 0.25), 0.0, 1.0);

        const std::string topic = get("topic", std::string("/skid_slip/yaw_efficiency"));
        _node.Subscribe(topic, &SkidSlip::OnEfficiency, this);
        gzmsg << "[SkidSlip] yaw efficiency " << _efficiency.load() << ", response " << _responseS
              << " s, cap " << _maxTorque << " N m; live on " << topic << "\n";
    }

    void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
    {
        if (info.paused)
            return;
        if (!FindEntities(ecm))
            return;

        const double s = _efficiency.load();
        if (s >= 1.0)
            return;  // nothing to add

        const auto wheels = WheelYawRate(ecm);
        if (!wheels)
            return;
        const double commanded = *wheels;

        gz::sim::Link base(_base);
        const auto w = base.WorldAngularVelocity(ecm);
        if (!w)
            return;

        // Fade in with the commanded turn, so a rover driving straight or standing
        // still is left to the ordinary physics rather than held to a zero yaw rate.
        const double fade = std::min(1.0, std::abs(commanded) / std::max(_activeAbove, 1e-6));
        const double target = s * commanded;
        const double dt = std::chrono::duration<double>(info.dt).count();
        const double error = target - w->Z();
        // The integral is itself a torque, and is dropped whenever the turn stops, so
        // nothing carries over from one turn to the next.
        if (std::abs(commanded) < 0.5 * _activeAbove)
            _integral = 0.0;
        else
            _integral = std::clamp(_integral + _stiffness / _integralS * error * dt, -_maxTorque,
                                   _maxTorque);
        const double torque =
            std::clamp((_stiffness * error + _integral) * fade, -_maxTorque, _maxTorque);
        if (info.iterations % 2000 == 1 && fade > 0.5)
            gzdbg << "[SkidSlip] wheels " << commanded << " rad/s, body " << w->Z() << ", target "
                  << target << ", torque " << torque << " N m (integral " << _integral << ")\n";
        base.AddWorldWrench(ecm, Vector3d::Zero, Vector3d(0, 0, torque));
    }

private:
    static std::vector<std::string> Split(const std::string& text)
    {
        std::vector<std::string> out;
        std::istringstream in(text);
        for (std::string word; in >> word;)
            out.push_back(word);
        return out;
    }

    void OnEfficiency(const gz::msgs::Double& msg)
    {
        const double value = std::clamp(msg.data(), 0.0, 1.0);
        _efficiency = value;
        gzmsg << "[SkidSlip] yaw efficiency set to " << value << "\n";
    }

    /// The yaw rate the wheels are commanding, from their joint speeds, or nullopt
    /// until every wheel's speed is available.
    std::optional<double> WheelYawRate(const gz::sim::EntityComponentManager& ecm) const
    {
        auto meanSpeed = [&](const std::vector<Entity>& joints) -> std::optional<double> {
            double sum = 0.0;
            for (const Entity e : joints) {
                const auto v = gz::sim::Joint(e).Velocity(ecm);
                if (!v || v->empty())
                    return std::nullopt;
                sum += v->front();
            }
            return joints.empty() ? std::nullopt : std::optional<double>(sum / joints.size());
        };
        const auto l = meanSpeed(_leftJoints);
        const auto r = meanSpeed(_rightJoints);
        if (!l || !r)
            return std::nullopt;
        // Both sides turn +y: a positive speed is forward on each.
        return _radius * (*r - *l) / _separation;
    }

    bool FindEntities(gz::sim::EntityComponentManager& ecm)
    {
        if (_found)
            return true;
        const Entity robot = ecm.EntityByComponents(gz::sim::components::Model(),
                                                    gz::sim::components::Name(_robotName));
        if (robot == kNullEntity)
            return false;  // not spawned yet
        gz::sim::Model model(robot);

        _base = model.LinkByName(ecm, _baseLink);
        if (_base == kNullEntity) {
            if (!_warned)
                gzwarn << "[SkidSlip] " << _robotName << " has no link " << _baseLink << "\n";
            _warned = true;
            return false;
        }
        for (const auto& [names, out] : {std::pair{&_left, &_leftJoints}, std::pair{&_right, &_rightJoints}}) {
            out->clear();
            for (const auto& name : *names) {
                const Entity joint = model.JointByName(ecm, name);
                if (joint == kNullEntity) {
                    if (!_warned)
                        gzwarn << "[SkidSlip] " << _robotName << " has no joint " << name << "\n";
                    _warned = true;
                    return false;
                }
                gz::sim::Joint(joint).EnableVelocityCheck(ecm, true);
                out->push_back(joint);
            }
        }
        gz::sim::Link(_base).EnableVelocityChecks(ecm, true);

        // Yaw inertia about the centre of mass, from the links' masses and positions
        // (their own inertia about z is small beside the lever arms, and the wheels
        // spin about y, so only the base's is added).
        double mass = 0.0;
        Vector3d weighted = Vector3d::Zero;
        std::vector<std::pair<double, Vector3d>> points;
        for (const Entity l : model.Links(ecm)) {
            const auto in = ecm.Component<gz::sim::components::Inertial>(l);
            if (!in)
                continue;
            const double m = in->Data().MassMatrix().Mass();
            const Vector3d p = gz::sim::worldPose(l, ecm).Pos() +
                               gz::sim::worldPose(l, ecm).Rot().RotateVector(in->Data().Pose().Pos());
            points.emplace_back(m, p);
            mass += m;
            weighted += m * p;
        }
        double izz = 0.0;
        if (mass > 0.0) {
            const Vector3d centre = weighted / mass;
            for (const auto& [m, p] : points)
                izz += m * ((p.X() - centre.X()) * (p.X() - centre.X()) +
                            (p.Y() - centre.Y()) * (p.Y() - centre.Y()));
        }
        if (const auto in = ecm.Component<gz::sim::components::Inertial>(_base))
            izz += in->Data().MassMatrix().Izz();
        _inertia = std::max(izz, 0.5);
        _stiffness = _stiffnessOverride > 0.0 ? _stiffnessOverride : _inertia / _responseS;
        _found = true;
        gzmsg << "[SkidSlip] tracking " << _robotName << " (" << mass << " kg): " << _leftJoints.size()
              << " left and " << _rightJoints.size() << " right wheels, yaw inertia " << _inertia
              << " kg m^2, gain " << _stiffness << " N m s/rad\n";
        return true;
    }

    std::string _robotName, _baseLink;
    std::vector<std::string> _left, _right;
    std::vector<Entity> _leftJoints, _rightJoints;
    Entity _base{kNullEntity};
    double _radius{0.147}, _separation{0.714}, _stiffness{150.0}, _maxTorque{1000.0};
    double _responseS{0.02}, _integralS{0.3}, _integral{0.0}, _stiffnessOverride{0.0}, _inertia{1.0};
    double _activeAbove{0.05};
    std::atomic<double> _efficiency{0.25};
    bool _found{false}, _warned{false};
    gz::transport::Node _node;
};
}  // namespace perseus_simulation

GZ_ADD_PLUGIN(perseus_simulation::SkidSlip, gz::sim::System,
              perseus_simulation::SkidSlip::ISystemConfigure,
              perseus_simulation::SkidSlip::ISystemPreUpdate)
