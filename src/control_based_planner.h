#pragma once
#include <pinocchio/fwd.hpp>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/config.h>

#include "ompl_planner.h"
#include "planning_world.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


template<typename DATATYPE>
using ValidityCheckerTpl_ptr = std::shared_ptr<ValidityCheckerTpl<DATATYPE>>;


using ValidityCheckerd_ptr = ValidityCheckerTpl_ptr<double>;
using ValidityCheckerf_ptr = ValidityCheckerTpl_ptr<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerf = ValidityCheckerTpl<float>;


template<typename DATATYPE>
class MobileRobotModel
{
  
//  typedef std::shared_ptr <ob::CompoundStateSpace> CompoundStateSpace_ptr;
 private:
    ob::CompoundStateSpace* space;
    const std::shared_ptr<oc::SpaceInformation> &si;
    const double carLength;
    int n;

 public:
  
     MobileRobotModel(const  std::shared_ptr<oc::SpaceInformation>& si_): si(si_), carLength(0.2){
        // std::cout << "MobileRobotModel" << std::endl;

        space = si->getStateSpace()->as<ob::CompoundStateSpace>();
        // std::cout << "get space.." << std::endl;
        n = space->getDimension();
        // std::cout << "space dimension (in mobile robot): " << n << std::endl;
    }
     void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const
     {
        auto state_vec = state2vector<DATATYPE>(state, si.get());
        auto theta = state_vec[2];
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        dstate.resize(n);
        // for(int i = 0; i < dstate.size() - 1; i++) {
        //     std:: cout<< "u[" << i << "]: " << u[i] << std::endl;
        // }

        dstate[0] = u[0] * cos(theta);
        dstate[1] = u[0] * sin(theta);
        //dstate[2] = u[0] * tan(u[1]) / carLength;
        dstate[2] = u[1];
        for(int i = 3; i < dstate.size(); i++) 
            dstate[i] = u[i-1];

        // std:: cout<< "dstate size: " << dstate.size() << std::endl;
        // for(int i = 0; i < dstate.size(); i++) {
        //     std:: cout<< "dstate[" << i << "]: " << dstate[i] << " ";
        // }
        // std:: cout<< std::endl;
     }
  
     void update(ob::State *state, const std::valarray<double> &dstate) const
     {
        // std:: cout << "start update" << " " << dstate.size() << std::endl;
        // std:: cout << "before update" <<" " << state2eigen<DATATYPE>(state, si.get()) << std::endl;
        auto s = state->as<ob::CompoundState>();
        for (size_t i = 0; i < dstate.size(); ++i) {
            // std:: cout << "update: " << i << std::endl;
            if(i != 2){
                auto v = space->as<ob::RealVectorStateSpace>(i);
                auto sub = s->as<ob::RealVectorStateSpace::StateType>(i);
                sub->as<ob::RealVectorStateSpace::StateType>()->values[0] += dstate[i];
                v->enforceBounds(sub);
            }
            else{
                auto v = space->as<ob::SO2StateSpace>(i);
                auto sub = s->as<ob::SO2StateSpace::StateType>(i);
                sub->as<ob::SO2StateSpace::StateType>()->value += dstate[i];
                v->enforceBounds(sub);
            }
        }
        // std:: cout << "finish update" <<" " << state2eigen<DATATYPE>(state, si.get()) << std::endl;
        // std:: cout << "finish update" << std::endl;
     }
  
 };
  

 template<typename F>
 class EulerIntegrator
 {

 public:
  
     EulerIntegrator(const std::shared_ptr<oc::SpaceInformation>& si, double timeStep): ode_(si)//: space_(
        //si->getStateSpace()->as<ob::CompoundStateSpace>()
     //), timeStep_(timeStep), ode_(si)
     {
        std:: cout << "EulerIntegrator" << std::endl;
        space_ = si->getStateSpace()->as<ob::CompoundStateSpace>();
        std:: cout << "EulerIntegrator: finish space" << std::endl;
        timeStep_ = timeStep;
        std:: cout << "EulerIntegrator: finish timeSteps" << std::endl;
     }
  
     void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const
     {
        // std::cout << "propagate" << std::endl;
        double t = 0;
        std::valarray<double> dstate;
        space_->copyState(result, start);
        while (t+timeStep_ < duration + std::numeric_limits<double>::epsilon())
        {
            // std::cout << "propagate: while" << " " << timeStep_ << " " << duration << std::endl;
            ode_(result, control, dstate);
            ode_.update(result, timeStep_ * dstate);
            t += timeStep_;
        }
        if (t + std::numeric_limits<double>::epsilon() < duration)
        {
            ode_(result, control, dstate);
            ode_.update(result, (duration-t) * dstate);
        }
     }
  
     double getTimeStep() const
     {
         return timeStep_;
     }
  
     void setTimeStep(double timeStep)
     {
         timeStep_ = timeStep;
     }
  
 private:
  
     const ob::CompoundStateSpace* space_;
     double                   timeStep_;
     F                        ode_;
 };


template<typename DATATYPE>
class MobileStatePropagator : public oc::StatePropagator
{
 public:
     MobileStatePropagator(const std::shared_ptr<oc::SpaceInformation>& si) : oc::StatePropagator(si),
                                                     integrator_(si, 0.0)
     {
     }
  
     void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override
     {
         integrator_.propagate(state, control, duration, result);
     }
  
     void setIntegrationTimeStep(double timeStep)
     {
         integrator_.setTimeStep(timeStep);
     }
  
     double getIntegrationTimeStep() const
     {
         return integrator_.getTimeStep();
     }
  
     EulerIntegrator<MobileRobotModel<DATATYPE>> integrator_;
 };
  


template<typename DATATYPE>
class ControlBasedPlannerTpl {
    typedef std::shared_ptr <ob::CompoundStateSpace> CompoundStateSpace_ptr;
    typedef std::shared_ptr <oc::SpaceInformation> SpaceInformation_ptr;
    typedef std::shared_ptr <ob::ProblemDefinition> ProblemDefinition_ptr;

    typedef std::shared_ptr <oc::RealVectorControlSpace> ControlSpace_ptr;

    typedef ob::CompoundStateSpace CompoundStateSpace;
    typedef oc::SpaceInformation SpaceInformation;
    typedef ob::ProblemDefinition ProblemDefinition;
    typedef oc::RealVectorControlSpace ControlSpace;

    using ValidityChecker = ValidityCheckerTpl<DATATYPE>;
    using ValidityChecker_ptr = ValidityCheckerTpl_ptr<DATATYPE>;


    DEFINE_TEMPLATE_EIGEN(DATATYPE)

    CompoundStateSpace_ptr cs;
    SpaceInformation_ptr si;
    ControlSpace_ptr cspace;
    std::shared_ptr <MobileStatePropagator<DATATYPE>> propagator;

    std::shared_ptr <oc::SimpleSetup> ss;

    PlanningWorldTpl_ptr<DATATYPE> world;
    ValidityCheckerTpl_ptr<DATATYPE> valid_checker;
    size_t dim;
    std::vector<DATATYPE> lower_joint_limits, upper_joint_limits;
    std::vector<bool> is_revolute;

public:
    VectorX random_sample_nearby(VectorX const &start_state);

    ControlBasedPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world);

    void build_state_space();

    PlanningWorldTpl_ptr<DATATYPE> get_world() { return world; }

    size_t get_dim() { return dim; }

    std::pair <std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
    plan(VectorX const &start_state, std::vector<VectorX> const &goal_states, const std::string &planner_name = "RRTConnect",
        const double &time = 1.0, const double& range = 0.0, const bool &verbose = false, const double& integration_step=0.01);
};


template<typename DATATYPE>
using ControlBasedPlannerTpl_ptr = std::shared_ptr<ValidityCheckerTpl<DATATYPE>>;


using ControlBasedPlannerTpld_ptr = ControlBasedPlannerTpl_ptr<double>;
using ControlBasedPlannerTplf_ptr = ControlBasedPlannerTpl_ptr<float>;
using ControlBasedPlannerTpld = ControlBasedPlannerTpl<double>;
using ControlBasedPlannerTplf = ControlBasedPlannerTpl<float>;

