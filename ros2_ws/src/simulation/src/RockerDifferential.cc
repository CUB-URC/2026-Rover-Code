// rocker differential constraint: q_right + ratio * q_left = 0
#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>

namespace rover
{
class RockerDifferential : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    auto get = [&](const std::string &k, double d) {
      return _sdf->HasElement(k) ? _sdf->Get<double>(k) : d; };
    auto getS = [&](const std::string &k, const std::string &d) {
      return _sdf->HasElement(k) ? _sdf->Get<std::string>(k) : d; };

    leftName_ = getS("left_joint", "rocker_left_joint");
    rightName_ = getS("right_joint", "rocker_right_joint");
    ratio_ = get("ratio", 1.0);
    k_ = get("stiffness", 3000.0);
    d_ = get("damping", 60.0);
    maxT_ = get("max_torque", 600.0);

    left_ = gz::sim::Joint(model.JointByName(_ecm, leftName_));
    right_ = gz::sim::Joint(model.JointByName(_ecm, rightName_));
    if (!left_.Valid(_ecm) || !right_.Valid(_ecm))
    {
      gzerr << "[RockerDifferential] joints '" << leftName_ << "' / '" << rightName_
            << "' not found in model - differential disabled\n";
      return;
    }
    left_.EnablePositionCheck(_ecm, true);
    left_.EnableVelocityCheck(_ecm, true);
    right_.EnablePositionCheck(_ecm, true);
    right_.EnableVelocityCheck(_ecm, true);
    ok_ = true;
    gzmsg << "[RockerDifferential] " << rightName_ << " = -" << ratio_ << " x " << leftName_
          << "  (K=" << k_ << ", D=" << d_ << ")\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!ok_ || _info.paused)
      return;
    auto pl = left_.Position(_ecm), pr = right_.Position(_ecm);
    auto vl = left_.Velocity(_ecm), vr = right_.Velocity(_ecm);
    if (!pl || !pr || pl->empty() || pr->empty())
      return;  // components appear after the first step
    const double c = (*pr)[0] + ratio_ * (*pl)[0];
    double cd = 0.0;
    if (vl && vr && !vl->empty() && !vr->empty())
      cd = (*vr)[0] + ratio_ * (*vl)[0];
    double lam = -(k_ * c + d_ * cd);
    lam = std::clamp(lam, -maxT_, maxT_);
    left_.SetForce(_ecm, {ratio_ * lam});
    right_.SetForce(_ecm, {lam});

    worst_ = std::max(worst_, std::abs(c));
    if (_info.simTime - lastLog_ > std::chrono::seconds(10))
    {
      lastLog_ = _info.simTime;
      const double deg = worst_ * 57.2958;
      if (deg > 3.0)
        gzwarn << "[RockerDifferential] linkage error up to " << deg
               << " deg - raise linkage_stiffness_nm_per_rad\n";
      worst_ = 0.0;
    }
  }

private:
  gz::sim::Joint left_, right_;
  std::string leftName_, rightName_;
  double ratio_{1.0}, k_{3000.0}, d_{60.0}, maxT_{600.0}, worst_{0.0};
  std::chrono::steady_clock::duration lastLog_{0};
  bool ok_{false};
};
}  // namespace rover

GZ_ADD_PLUGIN(rover::RockerDifferential, gz::sim::System,
              rover::RockerDifferential::ISystemConfigure,
              rover::RockerDifferential::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(rover::RockerDifferential, "rover::RockerDifferential")
