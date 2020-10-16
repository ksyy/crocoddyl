///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, LAAS-CNRS, Airbus
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_COSTS_PAIR_COLLISIONS_HPP_
#define CROCODDYL_MULTIBODY_COSTS_PAIR_COLLISIONS_HPP_

#include "crocoddyl/core/activations/norm2-barrier.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/cost-base.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/fcl.hpp>

#include <string>

namespace crocoddyl {

/**
 * Uses *HPP-FCL*, via *Pinocchio*, to compute the distance between a robot link and an
 * environment obstacle.
 *
 * If the distance is above a threshold (see ActivationModelNorm2BarrierTpl), the cost and its derivatives are 0.
 * Else, values for the cost and its derivatives are computed as follows:
 *
 * With \f$p\f$ the point on the robot's link nearest to the obstacle, and \f$p_{ref}\f$
 * its counterpart on the obstacle.
 *
 * \f$cost = a \left( p - p_{ref} \right) = 0.5\left (\left \| r \right \|_2 - threshold  \right )^2\f$
 *
 * Noting \f$J_p\f$ the joint jacobian computed at \f$p\f$, and \f$J = J_p\left[ 1:3 ; ... \right]\f$
 * the submatrix comprised of the 3 top rows.
 *
 * \f$R_x = \left(\begin{array}{@{}c|c@{}} J & 0 \end{array}\right)\f$
 *
 * \f$L_x = \left(\begin{array}{@{}c|c@{}} J^T A_r & 0 \end{array}\right)\f$
 *
 * \f$L_{xx} = \left(\begin{array}{@{}c|c@{}} 
 * J A_{rr} J^T & 0 \\ 
 * \hline 
 * 0 & 0 \end{array}\right)\f$
 *
 * For more information about those calculations, see this <a href="cost-model-for-obstacle-avoidance.pdf">this report</a>.
 */
template <typename _Scalar>
class CostModelPairCollisionsTpl : public CostModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelAbstractTpl<Scalar> Base;
  typedef CostDataPairCollisionsTpl<Scalar> Data;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef CostDataAbstractTpl<Scalar> CostDataAbstract;
  typedef ActivationModelAbstractTpl<Scalar> ActivationModelAbstract;
  typedef ActivationModelNorm2BarrierTpl<Scalar> Activation;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef pinocchio::GeometryModel GeometryModel;
  
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /** Initialise the collision cost model
   *
   * Note that it only works for *one* collision pair at the moment.
   *
   * \param state Multibody state
   * \param activation Activation model (if Norm2Barrier is used, **nr** should be 3)
   * \param nu Dimension of control
   * \param geom_model Pinocchio geometric model
   * \param pair_id Collision pair id in the geometric model
   * \param joint_id Joint id in the Pinocchio model (used for Jacobian computation)
   */
  CostModelPairCollisionsTpl(boost::shared_ptr<StateMultibody> state,
                             boost::shared_ptr<ActivationModelAbstract> activation,
                             const std::size_t& nu,
                             boost::shared_ptr<GeometryModel> geom_model,
                             const pinocchio::PairIndex& pair_id,
                             const pinocchio::JointIndex& joint_id);

  /** Initialise the collision cost model
   *
   * Note that it only works for *one* collision pair at the moment.
   *
   * \param state Multibody state
   * \param threshold Distance threshold for this collision 
   * \param nu Dimension of control
   * \param geom_model Pinocchio geometric model
   * \param pair_id Collision pair id in the geometric model
   * \param joint_id Joint id in the Pinocchio model (used for Jacobian computation)
   */
  CostModelPairCollisionsTpl(boost::shared_ptr<StateMultibody> state,
                             const Scalar& threshold,
                             const std::size_t& nu,
                             boost::shared_ptr<GeometryModel> geom_model,
                             const pinocchio::PairIndex& pair_id,
                             const pinocchio::JointIndex& joint_id);

  virtual ~CostModelPairCollisionsTpl();

  virtual void calc(const boost::shared_ptr<CostDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);
  virtual void calcDiff(const boost::shared_ptr<CostDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<CostDataAbstract> createData(DataCollectorAbstract* const data);

  const pinocchio::GeometryModel& get_geomModel() const;  

 protected:
  
  using Base::activation_;
  using Base::state_;

 private:
  boost::shared_ptr<pinocchio::GeometryModel > geom_model_;
  pinocchio::PairIndex pair_id_;
  pinocchio::JointIndex joint_id_;
};

template <typename _Scalar>
struct CostDataPairCollisionsTpl : public CostDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;
  typedef typename MathBase::Matrix3xs Matrix3xs;
  typedef typename MathBase::Matrix6xs Matrix6xs;
  typedef typename MathBase::Matrix6s Matrix6s;
  typedef typename MathBase::Vector6s Vector6s;

  template <template <typename Scalar> class Model>
  CostDataPairCollisionsTpl(Model<Scalar>* const model, DataCollectorAbstract* const data)
      : Base(model, data),
        geom_data(pinocchio::GeometryData(model->get_geomModel())),
        J(Matrix6xs::Zero(6, model->get_state()->get_nv())),
        Arr_J(MatrixXs::Zero(model->get_activation()->get_nr(), model->get_state()->get_nv())) {
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty("Invalid argument: the shared data should be derived from DataCollectorMultibody");
    }
    // Avoids data casting at runtime
    pinocchio = d->pinocchio;    
  }

  pinocchio::GeometryData geom_data;
  pinocchio::DataTpl<Scalar>* pinocchio;

  Matrix6xs J;
  MatrixXs Arr_J;
  
  using Base::shared;
  using Base::activation;
  using Base::cost;
  using Base::Lx;
  using Base::Lxx;
  using Base::r;
  using Base::Rx;
};

}  // namespace crocoddyl

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "crocoddyl/multibody/costs/pair-collisions.hxx"

#endif  // CROCODDYL_MULTIBODY_COSTS_PAIR_COLLISIONS_HPP_
