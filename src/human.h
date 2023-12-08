#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rbdyn_urdf/urdf.h>
#include <mc_rtc/logging.h>
#include <mc_robots/api.h>

namespace mc_robots
{

  struct MC_ROBOTS_DLLAPI HumanRobotModule : public mc_rbdyn::RobotModule
  {
  public:
    HumanRobotModule(bool fixed, bool canonical);

  protected:
    std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;
    std::vector< std::map<std::string, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;
    std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;
    std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  public:
    std::vector<std::string> virtualLinks;
    std::map< std::string, std::vector<double> > halfSitting;
    mc_rbdyn_urdf::Limits limits;
    std::vector<std::string> excludedLinks;
    std::vector<std::string> gripperJoints;
  };

} // namespace mc_robots


extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"human", "humanFixed", "humanCanonical", "humanFixedCanonical"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    ROBOT_MODULE_CHECK_VERSION("human")
    if(name == "human")
    {
      return new mc_robots::HumanRobotModule(false, false);
    }
    else if(name == "humanFixed")
    {
      return new mc_robots::HumanRobotModule(true, false);
    }
    else if(name == "humanCanonical")
    {
      return new mc_robots::HumanRobotModule(false, true);
    }
    else if(name == "humanFixedCanonical")
    {
      return new mc_robots::HumanRobotModule(true, true);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Human module cannot create an object of type {}", name);
      return nullptr;
    }
  }
}
