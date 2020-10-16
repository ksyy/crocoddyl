///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "cost.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/costs/com-position.hpp"
// #include "crocoddyl/multibody/costs/centroidal-momentum.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/frame-rotation.hpp"
#include "crocoddyl/multibody/costs/frame-translation.hpp"
#include "crocoddyl/multibody/costs/frame-velocity.hpp"
#include "crocoddyl/multibody/costs/contact-friction-cone.hpp"
#include "crocoddyl/multibody/costs/contact-wrench-cone.hpp"
#include "crocoddyl/multibody/costs/pair-collisions.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include <crocoddyl/core/fwd.hpp>

namespace crocoddyl {
namespace unittest {

const std::vector<CostModelTypes::Type> CostModelTypes::all(CostModelTypes::init_all());

std::ostream& operator<<(std::ostream& os, CostModelTypes::Type type) {
  switch (type) {
    case CostModelTypes::CostModelState:
      os << "CostModelState";
      break;
    case CostModelTypes::CostModelControl:
      os << "CostModelControl";
      break;
    case CostModelTypes::CostModelCoMPosition:
      os << "CostModelCoMPosition";
      break;
    // case CostModelTypes::CostModelCentroidalMomentum:
    //   os << "CostModelCentroidalMomentum";
    //   break;
    case CostModelTypes::CostModelFramePlacement:
      os << "CostModelFramePlacement";
      break;
    case CostModelTypes::CostModelFrameRotation:
      os << "CostModelFrameRotation";
      break;
    case CostModelTypes::CostModelFrameTranslation:
      os << "CostModelFrameTranslation";
      break;
    case CostModelTypes::CostModelFrameVelocity:
      os << "CostModelFrameVelocity";
      break;
    case CostModelTypes::CostModelPairCollisions:
      os << "CostModelPairCollisions";
      break;
    case CostModelTypes::NbCostModelTypes:
      os << "NbCostModelTypes";
      break;
    default:
      break;
  }
  return os;
}

CostModelFactory::CostModelFactory() {}
CostModelFactory::~CostModelFactory() {}

boost::shared_ptr<crocoddyl::CostModelPairCollisions> CostModelFactory::create_CostModelPairCollisions(
      boost::shared_ptr<crocoddyl::StateMultibody> state, 
      boost::shared_ptr<crocoddyl::ActivationModelAbstract> activation,
      std::size_t nu) const
{
  const pinocchio::Model& pin_model = *(state->get_pinocchio().get()); 
  boost::shared_ptr<pinocchio::GeometryModel> geom_model = boost::make_shared<pinocchio::GeometryModel>();

  std::string link_name = pin_model.names[2]; // Depends on the model used, so why not 2?
  pinocchio::FrameIndex pin_link_id = pin_model.getFrameId(link_name);
  pinocchio::JointIndex pin_joint_id = pin_model.getJointId(link_name);
  Eigen::Vector3d pos_body(-0.025,0,-.225);

  pinocchio::GeomIndex ig_robot = geom_model->addGeometryObject(pinocchio::GeometryObject("simple_robot_limb", pin_link_id, pin_model.frames[pin_link_id].parent, boost::shared_ptr<hpp::fcl::Capsule>(new hpp::fcl::Capsule(0, 0.45)), pinocchio::SE3(Eigen::Matrix3d::Identity(),pos_body)),pin_model);


  Eigen::Vector3d capsule_pose(-0.3,-0.1,0.75);
  Eigen::Vector2d capsule_size(.3, .02);
  
  pinocchio::GeomIndex ig_obs = geom_model->addGeometryObject(
    pinocchio::GeometryObject("simple_obs",
                              pin_model.getFrameId("universe"),
                              pin_model.frames[pin_model.getFrameId("universe")].parent,
                              boost::shared_ptr<hpp::fcl::Capsule>(new hpp::fcl::Capsule(capsule_size(0), capsule_size(1))),
                              pinocchio::SE3(Eigen::Matrix3d::Identity(), capsule_pose)),
                              pin_model);
  geom_model->addCollisionPair(pinocchio::CollisionPair(ig_robot, ig_obs));


  return boost::make_shared<crocoddyl::CostModelPairCollisions>(
           state, activation, nu, 
           geom_model, 
           0, pin_joint_id);
}

boost::shared_ptr<crocoddyl::CostModelAbstract> CostModelFactory::create(CostModelTypes::Type cost_type,
                                                                         StateModelTypes::Type state_type,
                                                                         ActivationModelTypes::Type activation_type,
                                                                         std::size_t nu) const {
  StateModelFactory state_factory;
  ActivationModelFactory activation_factory;
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::StateMultibody> state =
      boost::static_pointer_cast<crocoddyl::StateMultibody>(state_factory.create(state_type));
  crocoddyl::FrameIndex frame_index = state->get_pinocchio()->frames.size() - 1;
  pinocchio::SE3 frame_SE3 = pinocchio::SE3::Random();
  if (nu == std::numeric_limits<std::size_t>::max()) {
    nu = state->get_nv();
  }
  switch (cost_type) {
    case CostModelTypes::CostModelState:
      cost = boost::make_shared<crocoddyl::CostModelState>(
          state, activation_factory.create(activation_type, state->get_ndx()), state->rand(), nu);
      break;
    case CostModelTypes::CostModelControl:
      cost = boost::make_shared<crocoddyl::CostModelControl>(state, activation_factory.create(activation_type, nu),
                                                             Eigen::VectorXd::Random(nu));
      break;
    case CostModelTypes::CostModelCoMPosition:
      cost = boost::make_shared<crocoddyl::CostModelCoMPosition>(state, activation_factory.create(activation_type, 3),
                                                                 Eigen::Vector3d::Random(), nu);
      break;
    // case CostModelTypes::CostModelCentroidalMomentum:
    //   cost = boost::make_shared<crocoddyl::CostModelCentroidalMomentum>(state_,
    //                                                                      activation_factory.create(activation_type,
    //                                                                      6), Vector6d::Random(), nu);
    //   break;
    case CostModelTypes::CostModelFramePlacement:
      cost = boost::make_shared<crocoddyl::CostModelFramePlacement>(
          state, activation_factory.create(activation_type, 6), crocoddyl::FramePlacement(frame_index, frame_SE3), nu);
      break;
    case CostModelTypes::CostModelFrameRotation:
      cost = boost::make_shared<crocoddyl::CostModelFrameRotation>(
          state, activation_factory.create(activation_type, 3),
          crocoddyl::FrameRotation(frame_index, frame_SE3.rotation()), nu);
      break;
    case CostModelTypes::CostModelFrameTranslation:
      cost = boost::make_shared<crocoddyl::CostModelFrameTranslation>(
          state, activation_factory.create(activation_type, 3),
          crocoddyl::FrameTranslation(frame_index, frame_SE3.translation()), nu);
      break;
    case CostModelTypes::CostModelFrameVelocity:
      cost = boost::make_shared<crocoddyl::CostModelFrameVelocity>(
          state, activation_factory.create(activation_type, 6),
          crocoddyl::FrameMotion(frame_index, pinocchio::Motion::Random()), nu);
      break;
    case CostModelTypes::CostModelPairCollisions:
      cost = create_CostModelPairCollisions(state, activation_factory.create(activation_type, 3), nu);
      break;
    default:
      throw_pretty(__FILE__ ": Wrong CostModelTypes::Type given");
      break;
  }
  return cost;
}

boost::shared_ptr<crocoddyl::CostModelAbstract> create_random_cost(StateModelTypes::Type state_type) {
  static bool once = true;
  if (once) {
    srand((unsigned)time(NULL));
    once = false;
  }

  CostModelFactory factory;
  CostModelTypes::Type rand_type = static_cast<CostModelTypes::Type>(rand() % CostModelTypes::NbCostModelTypes);
  return factory.create(rand_type, state_type, ActivationModelTypes::ActivationModelQuad);
}

}  // namespace unittest
}  // namespace crocoddyl
