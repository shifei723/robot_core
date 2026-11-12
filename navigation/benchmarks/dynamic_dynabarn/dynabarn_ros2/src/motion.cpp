#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <gz/common/Animation.hh>
#include <gz/common/KeyFrame.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

namespace dynabarn {
// Prescribed absolute-world motion, matching Classic Entity::UpdateAnimation.
// Gazebo common PoseAnimation retains spline interpolation and loop semantics.
class Motion: public gz::sim::System, public gz::sim::ISystemConfigure,
              public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
 gz::sim::Model model{gz::sim::kNullEntity};
 std::unique_ptr<gz::common::PoseAnimation> animation;
 gz::transport::Node node;
 gz::transport::Node::Publisher publisher;
 double lastPublish=-1.,phase=0.;
 std::string name;
 public:
 void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
                gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
  model=gz::sim::Model(entity);name=model.Name(ecm);
  if(const char *value=std::getenv("DYNABARN_PHASE"))phase=std::stod(value);
  if(!std::isfinite(phase)||phase<0)throw std::runtime_error("Invalid motion phase");
  auto config=sdf->Clone();const double period=config->Get<double>("duration");
  if(!std::isfinite(period)||period<=0)throw std::runtime_error("Invalid DynaBARN period");
  animation=std::make_unique<gz::common::PoseAnimation>(name,period,true);
  double previous=-1.;
  for(auto key=config->GetElement("keyframe");key;key=key->GetNextElement("keyframe")) {
   const double time=key->Get<double>("time");const auto p=key->Get<gz::math::Vector3d>("position");
   if(!std::isfinite(time)||time<=previous||time>period||!p.IsFinite())throw std::runtime_error("Invalid DynaBARN keyframe");
   auto frame=animation->CreateKeyFrame(time);frame->Translation(p);frame->Rotation(gz::math::Quaterniond::Identity);previous=time;
  }
  if(animation->KeyFrameCount()<2||std::abs(previous-period)>1e-6)throw std::runtime_error("Incomplete DynaBARN motion");
  publisher=node.Advertise<gz::msgs::Pose_V>("/dynabarn/ground_truth");
 }
 void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(info.paused||!animation)return;
  const double t=std::chrono::duration<double>(info.simTime).count();
  animation->Time(std::fmod(std::max(0.,t+phase),animation->Length()));
  gz::common::PoseKeyFrame frame(0);animation->InterpolatedKeyFrame(frame);
  model.SetWorldPoseCmd(ecm,gz::math::Pose3d(frame.Translation(),frame.Rotation()));
 }
 void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
  const double t=std::chrono::duration<double>(info.simTime).count();
  if(info.paused || (t>=lastPublish && t-lastPublish<.049))return;
  lastPublish=t;gz::msgs::Pose_V msg;
  const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
  msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
  auto pose=msg.add_pose();pose->set_name(name);pose->set_id(model.Entity());
  pose->mutable_header()->CopyFrom(msg.header());
  auto parent=pose->mutable_header()->add_data();parent->set_key("frame_id");parent->add_value("world");
  auto child=pose->mutable_header()->add_data();child->set_key("child_frame_id");child->add_value(name);
  const auto actual=gz::sim::worldPose(model.Entity(),ecm);
  pose->mutable_position()->set_x(actual.Pos().X());pose->mutable_position()->set_y(actual.Pos().Y());pose->mutable_position()->set_z(actual.Pos().Z());
  pose->mutable_orientation()->set_w(actual.Rot().W());pose->mutable_orientation()->set_x(actual.Rot().X());pose->mutable_orientation()->set_y(actual.Rot().Y());pose->mutable_orientation()->set_z(actual.Rot().Z());
  publisher.Publish(msg);
 }
};
}
GZ_ADD_PLUGIN(dynabarn::Motion,gz::sim::System,dynabarn::Motion::ISystemConfigure,dynabarn::Motion::ISystemPreUpdate,dynabarn::Motion::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(dynabarn::Motion,"dynabarn::Motion")
