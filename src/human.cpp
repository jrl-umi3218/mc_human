#include "human.h"
#include "config.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <mc_rtc/logging.h>
#include <fstream>

#include <RBDyn/parsers/urdf.h>

namespace bfs = boost::filesystem;

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_robots
{
HumanRobotModule::HumanRobotModule(bool fixed, bool canonical)
 : RobotModule(HUMAN_DESCRIPTION_PATH, "human")
 {
   std::string canonicalName = "human";
  if(fixed) canonicalName += "Fixed";
  if(canonical) canonicalName += "Canonical";
  _canonicalParameters = {canonicalName};

  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links */
  auto gripperLinks = std::vector<std::string>{};
  if(!canonical)
  {
    for(const auto & link : { "HandThumb", "HandIndex", "HandMiddle", "HandRing", "HandBaby" })
    for(const auto & side : {"L", "R"})
    {
      for(size_t i = 0; i <= 2; ++i)
      {
        if(link == std::string{"HandThumb"})
        {
          virtualLinks.push_back(fmt::format("{}{}{}Link", side, link, i));
        }
        else
        {
          virtualLinks.push_back(fmt::format("{}{}{}Link", side, link, i+1));
        }
        gripperLinks.push_back(virtualLinks.back());
        mc_rtc::log::info("Adding virtual {}", virtualLinks.back());
      }
    }
    virtualLinks.push_back("LHand");
    virtualLinks.push_back("RHand");
  }

  /* Default posture joint values in degrees */
 	halfSitting["Torso_0"] = { 0.0 };
  halfSitting["Torso_1"] = { 0.0 };
  halfSitting["Torso_2"] = { 0.0 };
 	halfSitting["Head_0"] = { 0.0 };
  halfSitting["Head_1"] = { 0.0 };
  halfSitting["Head_2"] = { 0.0 };
 	halfSitting["LArm_0"] = { -70.0 };
  halfSitting["LArm_1"] = { 0.0 };
  halfSitting["LArm_2"] = { 0.0 };
 	// halfSitting["LElbow"] = { 0.0 };
 	halfSitting["LForearm_0"] = { -45.0 };
  halfSitting["LForearm_1"] = { 0.0 };
 	halfSitting["LWrist_0"] = { 0 };
  halfSitting["LWrist_1"] = { 0 };
 	halfSitting["RArm_0"] = { 70.0 };
  halfSitting["RArm_1"] = { 0.0 };
  halfSitting["RArm_2"] = { 0.0 };
 	// halfSitting["RElbow"] = { 0.0 };
  halfSitting["RForearm_0"] = { 45.0 };
  halfSitting["RForearm_1"] = { 0.0 };
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
  halfSitting["RHand"] = { 0.0 };
  halfSitting["LHand"] = { 0.0 };
  if(canonical)
  {
    for(const auto & gLink : gripperLinks)
    {
      halfSitting[gLink] = { 0.0 };
    }
  }

  /* Grippers */
  _grippers = {{"l_gripper", {"LHand"}, true}, {"r_gripper", {"RHand"}, true}};

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
 	// "LElbow", // 9
 	"LForearm_0", // 9
  "LForearm_1", // 10
 	"LWrist_0", // 11
  "LWrist_1", // 12
 	"RArm_0", // 13
  "RArm_1", // 14
  "RArm_2", // 15
 	// "RElbow", // 16
 	"RForearm_0", // 16
  "RForearm_1", // 17
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
  "RAnkle_1", // 31
  "LHand", // 32
  "RHand" // 33
  };

  /* Read URDF file */
  init(rbd::parsers::from_urdf_file(urdf_path,
        rbd::parsers::ParserParameters{}
          .fixed(fixed)
          .filtered_links(virtualLinks)
          .remove_filtered_links(false)));

  
  for(auto & visual : _visual)
  {
    for(size_t i = 0; i < visual.second.size(); i++)
    {
      auto & v = visual.second[i];
      v.material.type = rbd::parsers::Material::Type::COLOR;
      auto & color = boost::get<rbd::parsers::Material::Color>(v.material.data);
      color.r = 1.;
      color.g = 1.;
      color.b = 1.;
      color.a = 0.2;
      if (visual.first == "HipsLink")
      {
        color.a = 0.2;
      }
      if (visual.first == "TorsoLink")
      {
        color.a = 0.2;
      }
      if (visual.first == "HeadLink")
      {
        color.a = 1;
      }
      
      
    }
  }

  /* Collision hulls */
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  // Accelerometer 
  _bodySensors.emplace_back("Accelerometer", "HipsLink", sva::PTransformd(Eigen::Vector3d(0, 0, 0.1095)));
  // // Floating base
  _bodySensors.emplace_back("FloatingBase", "HipsLink", sva::PTransformd::Identity());

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

  std::map<std::string, std::vector<double>> HumanRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::vector<double>> res;
    for (const auto & j : mb.joints())
    {
      if(halfSitting.count(j.name()))
      { res[j.name()] = halfSitting.at(j.name());
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

    // // Lambda function to associate bodies and the corresponding files
    // auto addBody = [&res](const std::string & body, const std::string & file) { res[body] = {body, file}; };

    // addBody("HipsLinkFull", "HipsLinkFull");
    // addBody("TorsoLinkFull", "TorsoLinkFull");
    // addBody("HeadLinkFull", "HeadLinkFull");
    // addBody("LArmLinkFull", "LArmLinkFull");
    // addBody("LElbowLink", "LElbowLink");
    // addBody("LForearmLink", "LForearmLink");
    // addBody("LWrist", "LWrist");
    // addBody("RArmLinkFull", "RArmLinkFull");
    // addBody("RElbowLink", "RElbowLink");
    // addBody("RForearmLink", "RForearmLink");
    // addBody("RWrist", "RWrist");
    // addBody("LLegLink", "LLegLink");
    // addBody("LShinLink", "LShinLink");
    // addBody("LAnkleLinkFull", "LAnkleLinkFull");
    // addBody("RLegLink", "RLegLink");
    // addBody("RShinLink", "RShinLink");
    // addBody("RAnkleLinkFull", "RAnkleLinkFull");
    // addBody("LThumb0thPhalange", "LThumb0thPhalange");
    // addBody("LThumb1stPhalange", "LThumb1stPhalange");
    // addBody("LThumb2ndPhalange", "LThumb2ndPhalange");
    // addBody("LIndex1stPhalange", "LIndex1stPhalange");
    // addBody("LIndex2ndPhalange", "LIndex2ndPhalange");
    // addBody("LIndex3rdPhalange", "LIndex3rdPhalange");
    // addBody("LMiddle1stPhalange", "LMiddle1stPhalange");
    // addBody("LMiddle2ndPhalange", "LMiddle2ndPhalange");
    // addBody("LMiddle3rdPhalange", "LMiddle3rdPhalange");
    // addBody("LRing1stPhalange", "LRing1stPhalange");
    // addBody("LRing2ndPhalange", "LRing2ndPhalange");
    // addBody("LRing3rdPhalange", "LRing3rdPhalange");
    // addBody("LBaby1stPhalange", "LBaby1stPhalange");
    // addBody("LBaby2ndPhalange", "LBaby2ndPhalange");
    // addBody("LBaby3rdPhalange", "LBaby3rdPhalange");
    // addBody("RThumb0thPhalange", "RThumb0thPhalange");
    // addBody("RThumb1stPhalange", "RThumb1stPhalange");
    // addBody("RThumb2ndPhalange", "RThumb2ndPhalange");
    // addBody("RIndex1stPhalange", "RIndex1stPhalange");
    // addBody("RIndex2ndPhalange", "RIndex2ndPhalange");
    // addBody("RIndex3rdPhalange", "RIndex3rdPhalange");
    // addBody("RMiddle1stPhalange", "RMiddle1stPhalange");
    // addBody("RMiddle2ndPhalange", "RMiddle2ndPhalange");
    // addBody("RMiddle3rdPhalange", "RMiddle3rdPhalange");
    // addBody("RRing1stPhalange", "RRing1stPhalange");
    // addBody("RRing2ndPhalange", "RRing2ndPhalange");
    // addBody("RRing3rdPhalange", "RRing3rdPhalange");
    // addBody("RBaby1stPhalange", "RBaby1stPhalange");
    // addBody("RBaby2ndPhalange", "RBaby2ndPhalange");
    // addBody("RBaby3rdPhalange", "RBaby3rdPhalange");

    return res;
  }

}
