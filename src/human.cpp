#include "human.h"

#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>

#include <fstream>

#include <config.h>


#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;


namespace mc_robots
{
HumanRobotModule::HumanRobotModule()
 : RobotModule(HUMAN_DESCRIPTION_PATH, "human")
 {

  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links */
 	virtualLinks.push_back("base_link");

  /* Init joint values in degrees */
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


  _ref_joint_order = {
 	"Torso_0", // 2
  "Torso_1",
  "Torso_2", // 4
 	"Head_0",
  "Head_1", // 6
  "Head_2",
 	"LArm_0", // 8
  "LArm_1",
  "LArm_2", // 10
 	"LElbow",
 	"LForearm", // 12
 	"LWrist_0",
  "LWrist_1", // 14
 	"RArm_0",
  "RArm_1", // 16
  "RArm_2",
 	"RElbow", // 18
 	"RForearm",
 	"RWrist_0", // 20
  "RWrist_1",
  "LLeg_0", // 22
  "LLeg_1",
  "LLeg_2", // 24
 	"LShin_0",
 	"LAnkle_0", // 26
  "LAnkle_1",
 	"RLeg_0", // 28
  "RLeg_1",
  "RLeg_2", // 20
 	"RShin_0",
 	"RAnkle_0", // 32
  "RAnkle_1" 
  };

  
  /* Read URDF file */
  readUrdf("human", filteredLinks);

  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);

  
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

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.987}};

 }


 void HumanRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
  {
    std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
    std::ifstream ifs(urdfPath);
    if(ifs.is_open())
    {
      std::stringstream urdf;
      urdf << ifs.rdbuf();
      /* Consider robot as fixed base for now with root at base_footprint */
      mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks);
      mb = res.mb;
      mbc = res.mbc;
      mbg = res.mbg;
      limits = res.limits;

      _visual = res.visual;
      _collisionTransforms = res.collision_tf;
    }
    else
    {
      LOG_ERROR("Could not open Human model at " << urdfPath)
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
        LOG_WARNING("Joint " << j.name() << " has " << j.dof() << " dof, but is not part of half sitting posture.");
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

  const std::map<std::string, std::pair<std::string, std::string> > & HumanRobotModule::convexHull() const
  {
    return _convexHull;
  }


  const std::vector< std::map<std::string, std::vector<double> > > & HumanRobotModule::bounds() const
  {
    return _bounds;
  }

  const std::map<std::string, std::vector<double> > & HumanRobotModule::stance() const
  {
    return _stance;
  }

}
