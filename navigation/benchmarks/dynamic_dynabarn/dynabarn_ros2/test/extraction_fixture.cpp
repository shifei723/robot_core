#include <string>
namespace ignition { namespace math { inline namespace v4 {
template<class T> class Vector3 {public:Vector3(const T&,const T&,const T&);};
template<class T> class Quaternion {public:Quaternion(const T&,const T&,const T&);};
}}}
namespace gazebo { namespace common {
class PoseKeyFrame {public:void Translation(const ignition::math::Vector3<double>&);void Rotation(const ignition::math::Quaternion<double>&);};
class PoseAnimation {public:PoseAnimation(const std::string&,double,bool);PoseKeyFrame *CreateKeyFrame(double);};
}
class obs_fixture {public:void Load(int);};
void obs_fixture::Load(int) {
 auto anim=new common::PoseAnimation("fixture",4.5,true);
 auto k=anim->CreateKeyFrame(0.);k->Translation(ignition::math::Vector3<double>(-2.25,3.,0.));k->Rotation(ignition::math::Quaternion<double>(0.,0.,0.));
 k=anim->CreateKeyFrame(1.25);k->Translation(ignition::math::Vector3<double>(0.,1.,0.));k->Rotation(ignition::math::Quaternion<double>(0.,0.,0.));
 k=anim->CreateKeyFrame(4.5);k->Translation(ignition::math::Vector3<double>(6.,-2.5,0.));k->Rotation(ignition::math::Quaternion<double>(0.,0.,0.));
}}
