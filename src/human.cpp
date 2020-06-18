#include "human.h"
#include "config.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <mc_rtc/logging.h>
#include <fstream>

namespace bfs = boost::filesystem;

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_robots
{
HumanRobotModule::HumanRobotModule(bool fixed, bool hands)
 : RobotModule(HUMAN_DESCRIPTION_PATH, "human")
 {
  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links */
 	virtualLinks.push_back("base_link");
  virtualLinks.push_back("hip2torso_1");
  virtualLinks.push_back("hip2torso_2");
  virtualLinks.push_back("torso2head_1");
  virtualLinks.push_back("torso2head_2");
  virtualLinks.push_back("torso2lshoulder_1");
  virtualLinks.push_back("torso2lshoulder_2");
  virtualLinks.push_back("forearm2lwrist_1");
  virtualLinks.push_back("torso2rshoulder_1");
  virtualLinks.push_back("torso2rshoulder_2");
  virtualLinks.push_back("forearm2rwrist_1");
  virtualLinks.push_back("torso2lleg_1");
  virtualLinks.push_back("torso2lleg_2");
  virtualLinks.push_back("shin2lankle_1");
  virtualLinks.push_back("torso2rleg_1");
  virtualLinks.push_back("torso2rleg_2");
  virtualLinks.push_back("shin2rankle_1");

  /* Gripper links not included in NoHands model */
  if(!hands){
    excludedLinks.push_back("LHandThumb0Link");
    excludedLinks.push_back("LHandThumb1Link");
    excludedLinks.push_back("LHandThumb2Link");
    excludedLinks.push_back("LHandIndex1Link");
    excludedLinks.push_back("LHandIndex2Link");
    excludedLinks.push_back("LHandIndex3Link");
    excludedLinks.push_back("LHandMiddle1Link");
    excludedLinks.push_back("LHandMiddle2Link");
    excludedLinks.push_back("LHandMiddle3Link");
    excludedLinks.push_back("LHandRing1Link");
    excludedLinks.push_back("LHandRing2Link");
    excludedLinks.push_back("LHandRing3Link");
    excludedLinks.push_back("LHandBaby1Link");
    excludedLinks.push_back("LHandBaby2Link");
    excludedLinks.push_back("LHandBaby3Link");
    excludedLinks.push_back("RHandThumb0Link");
    excludedLinks.push_back("RHandThumb1Link");
    excludedLinks.push_back("RHandThumb2Link");
    excludedLinks.push_back("RHandIndex1Link");
    excludedLinks.push_back("RHandIndex2Link");
    excludedLinks.push_back("RHandIndex3Link");
    excludedLinks.push_back("RHandMiddle1Link");
    excludedLinks.push_back("RHandMiddle2Link");
    excludedLinks.push_back("RHandMiddle3Link");
    excludedLinks.push_back("RHandRing1Link");
    excludedLinks.push_back("RHandRing2Link");
    excludedLinks.push_back("RHandRing3Link");
    excludedLinks.push_back("RHandBaby1Link");
    excludedLinks.push_back("RHandBaby2Link");
    excludedLinks.push_back("RHandBaby3Link");
  }

  /* Gripper joints to include in half posture if hands */
  if(hands){
    gripperJoints.push_back("RHand");
    gripperJoints.push_back("RHandThumbLink1");
    gripperJoints.push_back("RHandThumbLink2");
    gripperJoints.push_back("RHandThumbLink3");
    gripperJoints.push_back("RHandIndexLink1");
    gripperJoints.push_back("RHandIndexLink2");
    gripperJoints.push_back("RHandIndexLink3");
    gripperJoints.push_back("RHandMiddleLink1");
    gripperJoints.push_back("RHandMiddleLink2");
    gripperJoints.push_back("RHandMiddleLink3");
    gripperJoints.push_back("RHandRingLink1");
    gripperJoints.push_back("RHandRingLink2");
    gripperJoints.push_back("RHandRingLink3");
    gripperJoints.push_back("RHandBabyLink1");
    gripperJoints.push_back("RHandBabyLink2");
    gripperJoints.push_back("RHandBabyLink3");
    gripperJoints.push_back("LHand");
    gripperJoints.push_back("LHandThumbLink1");
    gripperJoints.push_back("LHandThumbLink2");
    gripperJoints.push_back("LHandThumbLink3");
    gripperJoints.push_back("LHandIndexLink1");
    gripperJoints.push_back("LHandIndexLink2");
    gripperJoints.push_back("LHandIndexLink3");
    gripperJoints.push_back("LHandMiddleLink1");
    gripperJoints.push_back("LHandMiddleLink2");
    gripperJoints.push_back("LHandMiddleLink3");
    gripperJoints.push_back("LHandRingLink1");
    gripperJoints.push_back("LHandRingLink2");
    gripperJoints.push_back("LHandRingLink3");
    gripperJoints.push_back("LHandBabyLink1");
    gripperJoints.push_back("LHandBabyLink2");
    gripperJoints.push_back("LHandBabyLink3");
  }

  /* Default posture joint values in degrees */
 	halfSitting["Torso_0"] = { 0.0 };
  halfSitting["Torso_1"] = { 0.0 };
  halfSitting["Torso_2"] = { 0.0 };
 	halfSitting["Head_0"] = { 0.0 };
  halfSitting["Head_1"] = { 0.0 };
  halfSitting["Head_2"] = { 0.0 };
 	halfSitting["LArm_0"] = { 0.0 };
  halfSitting["LArm_1"] = { 0.0 };
  halfSitting["LArm_2"] = { 0.0 };
 	halfSitting["LElbow"] = { 0.0 };
 	halfSitting["LForearm"] = { 0.0 };
 	halfSitting["LWrist_0"] = { 0 };
  halfSitting["LWrist_1"] = { 0 };
 	halfSitting["RArm_0"] = { 0.0 };
  halfSitting["RArm_1"] = { 0.0 };
  halfSitting["RArm_2"] = { 0.0 };
 	halfSitting["RElbow"] = { 0.0 };
  halfSitting["RForearm"] = { 0.0 };
 	halfSitting["RWrist_0"] = { 0.0 };
  halfSitting["RWrist_1"] = { 0.0 };
 	halfSitting["LLeg_0"] = { 0.0 };
  halfSitting["LLeg_1"] = { 0.0 };
  halfSitting["LLeg_2"] = { 0.0 };
 	halfSitting["LShin_0"] = { 0.0 };
 	halfSitting["LAnkle_0"] = { 0.0 };
  halfSitting["LAnkle_1"] = { 0.0 };
 	halfSitting["RLeg_0"] = { 0.0 };
  halfSitting["RLeg_1"] = { 0.0 };
  halfSitting["RLeg_2"] = { 0.0 };
 	halfSitting["RShin_0"] = { 0.0 };
 	halfSitting["RAnkle_0"] = { 0.0 };
  halfSitting["RAnkle_1"] = { 0.0 };
  if(hands){
    for(const auto& gripJ : gripperJoints){
      halfSitting[gripJ] = { 0 };
    }
  }

  /* Grippers */
  if(hands){
    _grippers = {{"l_gripper", {"LHand"}, true}, {"r_gripper", {"RHand"}, true}};
  }

  /* Reference joint order */
  _ref_joint_order = {
 	"Torso_0", // 0
  "Torso_1", // 1
  "Torso_2", // 2
 	"Head_0", // 3
  "Head_1", // 4
  "Head_2", // 5
 	"LArm_0", // 6
  "LArm_1", // 7
  "LArm_2", // 8
 	"LElbow", // 9
 	"LForearm", // 10
 	"LWrist_0", // 11
  "LWrist_1", // 12
 	"RArm_0", // 13
  "RArm_1", // 14
  "RArm_2", // 15
 	"RElbow", // 16
 	"RForearm", // 17
 	"RWrist_0", // 18
  "RWrist_1", // 19
  "LLeg_0", // 20
  "LLeg_1", // 21
  "LLeg_2", // 22
 	"LShin_0", // 23
 	"LAnkle_0", // 24
  "LAnkle_1", // 25
 	"RLeg_0", // 26
  "RLeg_1", // 27
  "RLeg_2", // 28
 	"RShin_0", // 29
 	"RAnkle_0", // 30
  "RAnkle_1" // 31
  };
  if(hands){
    _ref_joint_order.push_back("LHand"); // 32
    _ref_joint_order.push_back("RHand"); // 33
  }

  /* Read URDF file */
  readUrdf("human", fixed, excludedLinks);

  /* Collision hulls */
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  /* Joint limits */
  _bounds = nominalBounds(limits);

  /* Halfsit posture */
  _stance = halfSittingPose(mb);

  /* Critical self collisions */
  _minimalSelfCollisions = {
    mc_rbdyn::Collision("HeadLink", "LArmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HeadLink", "LForearmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HeadLink", "LHandLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HeadLink", "RArmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HeadLink", "RForearmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HeadLink", "RHandLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("TorsoLink", "LForearmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("TorsoLink", "LHandLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("TorsoLink", "RForearmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("TorsoLink", "RHandLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HipsLink", "LForearmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HipsLink", "LHandLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HipsLink", "RForearmLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("HipsLink", "RHandLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("LLegLink", "RLegLink", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("LShinLink", "RShinLink", 0.03, 0.01, 0.)
  };
  _commonSelfCollisions = _minimalSelfCollisions;

  /* Default kinematic tree root pose */
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.987}};

 }


 void HumanRobotModule::readUrdf(const std::string & robotName,
                                     bool fixed,
                                     const std::vector<std::string> & filteredLinks)
  {
    std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
    std::ifstream ifs(urdfPath);
    if(ifs.is_open())
    {
      std::stringstream urdf;
      urdf << ifs.rdbuf();
      /* Consider robot as fixed base for now */
      mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), fixed, filteredLinks);
      mb = res.mb;
      mbc = res.mbc;
      mbg = res.mbg;
      limits = res.limits;

      _visual = res.visual;
      _collisionTransforms = res.collision_tf;
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Could not open Human model at {}", urdfPath);
      throw("Failed to open Human model");
    }
  }


  std::map<std::string, std::vector<double>> HumanRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::vector<double>> res;
    for (const auto & j : mb.joints())
    {
      if(halfSitting.count(j.name()))
      {
        res[j.name()] = halfSitting.at(j.name());
        for (auto & ji : res[j.name()])
        {
          ji = M_PI*ji / 180;
        }
      }
      else if(j.name() != "Root" && j.dof() > 0)
      {
        mc_rtc::log::warning("Joint {} has {} dof, but is not part of half sitting posture.", j.name(), j.dof());
      }
    }
    return res;
  }


  std::map<std::string, std::pair<std::string, std::string> > HumanRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
  {
    std::string convexPath = path + "/convex/";

    std::map<std::string, std::pair<std::string, std::string> > res;
    for(const auto & f : files)
    {
      bfs::path fpath = bfs::path(convexPath)/(f.second.second+"-ch.txt");
      if (bfs::exists(fpath))
      {
       res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
      }
    }
    return res;
  }


  std::vector< std::map<std::string, std::vector<double> > > HumanRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
  {
    std::vector< std::map<std::string, std::vector<double> > > res(0);
    res.push_back(limits.lower);
    res.push_back(limits.upper);
    {
      auto mvelocity = limits.velocity;
      for (auto & mv : mvelocity)
      {
        for (auto & mvi : mv.second)
        {
          mvi = -mvi;
        }
      }
      res.push_back(mvelocity);
    }
    res.push_back(limits.velocity);
    {
      auto mtorque = limits.torque;
      for (auto & mt : mtorque)
      {
        for (auto & mti : mt.second)
        {
          mti = -mti;
        }
      }
      res.push_back(mtorque);
    }
    res.push_back(limits.torque);
    return res;
  }

  std::map<std::string, std::pair<std::string, std::string>> HumanRobotModule::stdCollisionsFiles(const rbd::MultiBody &/*mb*/) const
  {
    std::map<std::string, std::pair<std::string, std::string>> res;
    for(const auto & b : mb.bodies())
    {
      // Filter out virtual links without convex files
      if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
      {
        res[b.name()] = {b.name(), b.name()};
      }
    }
    return res;
  }

}
