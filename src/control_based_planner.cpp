#include "control_based_planner.h"
#include "pinocchio_model.h"


#define DEFINE_TEMPLATE_ControlBased(DATATYPE) template class ValidityCheckerTpl<DATATYPE>; template class ControlBasedPlannerTpl<DATATYPE>;

DEFINE_TEMPLATE_ControlBased(double)

DEFINE_TEMPLATE_ControlBased(float)

#define PI 3.14159265359

template<typename DATATYPE>
void ControlBasedPlannerTpl<DATATYPE>::build_state_space(void) {
    cs = std::make_shared<CompoundStateSpace>();
    dim = 0;
    std::string const joint_prefix = "JointModel";
    int robot_id = 0;
    for (auto robot: world->getArticulations()) {
        auto dim_i = 0;
        robot_id += 1;
        auto model = robot->getPinocchioModel();
        auto joint_types = model.getJointTypes();
        auto d = robot->getQposDim(); // TODO!!! only construct for move group joints
        auto indices = robot->getMoveGroupJointIndices();
        ASSERT(d == indices.size(), "QposDim != size of the movegroup joints");
        for (size_t i = 0; i < d; i++) {
            auto id = indices[i];
            auto joint_type = joint_types[id];
            std:: cout << "Joint type " << joint_type << " " << joint_prefix << std::endl;
            std:: string space_type = "fixed";
            if (joint_type[joint_prefix.size()] == 'P' || (joint_type[joint_prefix.size()] == 'R' &&
                                                        joint_type[joint_prefix.size() + 1] != 'U'))// PRISMATIC and REVOLUTE
            {
                auto bound = model.getJointLimit(id);
                auto subspcae = std::make_shared<ob::RealVectorStateSpace>(bound.rows());
                auto ob_bounds = ob::RealVectorBounds(bound.rows());
                dim_i += bound.rows();
                for (size_t j = 0; j < bound.rows(); j++) {
                    std:: cout << "Add subspace " << bound(j, 0) << " " << bound(j, 1) << std::endl;
                    lower_joint_limits.push_back(bound(j, 0));
                    upper_joint_limits.push_back(bound(j, 1));
                    ob_bounds.setLow(j, bound(j, 0)), ob_bounds.setHigh(j, bound(j, 1));
                }
                subspcae->setBounds(ob_bounds);
                cs->addSubspace(subspcae, 1.0);
                space_type = "bounded";
            } else if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] == 'U') {
                cs->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
                lower_joint_limits.push_back(-PI);
                upper_joint_limits.push_back(PI);
                dim_i += 1;
                space_type = "unbounded";
            }
            if(robot_id == 1)
            {
                if(dim_i == 1 || dim_i == 2){
                    ASSERT(space_type == "bounded", "The first/second joint should not be bounded");
                }
                if(dim_i == 3){
                    ASSERT(space_type == "unbounded", "The third joint must be a revolute joint unbounded");
                }
            }
            if (joint_type[joint_prefix.size()] == 'R' || joint_type[joint_prefix.size()] == 'P') {
                if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] != 'U')
                    is_revolute.push_back(true);
                else
                    is_revolute.push_back(false);
            }
        }
        ASSERT(dim_i == robot->getQposDim(), "Dim of bound is different from dim of qpos " +  std::to_string(dim_i) + " " + std::to_string(robot->getQposDim()));
        dim += dim_i;
    }
}


namespace oc = ompl::control;
namespace ob = ompl::base;
namespace og = ompl::geometric;

  


template<typename DATATYPE>
ControlBasedPlannerTpl<DATATYPE>::ControlBasedPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world):world(world) {
    build_state_space();
    //std::cout << "State space dimension: " << dim << std::endl;
    cspace = std::make_shared<oc::RealVectorControlSpace>(cs, dim-1);

    //std::cout << "Control space dimension: " << cspace->getDimension() << std::endl;
    ss = std::make_shared<oc::SimpleSetup>(cspace);
    //std:: cout << "Simple setup" << std::endl;
    si = ss->getSpaceInformation();
    //std:: cout << "Space information" << std::endl;

    valid_checker = std::make_shared<ValidityChecker>(world, si);
    ss->setStateValidityChecker(valid_checker);
    //std:: cout << "State validity checker" << std::endl;

    propagator = std::make_shared<MobileStatePropagator<DATATYPE>>(si);
    ss->setStatePropagator(propagator);
    //std:: cout << "State propagator" << std::endl;
}

template<typename DATATYPE>
std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
ControlBasedPlannerTpl<DATATYPE>::plan(
    VectorX const &start_state, 
    std::vector<VectorX> const &goal_states, 
    const std::string &planner_name,
    const double &time,
    const double& range, 
    const bool& verbose,
    const double& integration_step
) {
    ASSERT(start_state.rows() == goal_states[0].rows(), "Length of start state and goal state should be equal");
    ASSERT(start_state.rows() == dim, "Length of start state and problem dimension should be equal");
    if (verbose == false)
        ompl::msg::noOutputHandler();


    // set the bounds for the control space
    auto cbounds = ob::RealVectorBounds(dim-1);
    cbounds.setLow(-1.0);
    cbounds.setHigh(1.0);
    cspace->setBounds(cbounds);



    ob::ScopedState<> start(cs);
    start = eigen2vector<DATATYPE, double>(start_state);

    bool invalid_start = !valid_checker->_isValid(start_state);
    if (invalid_start) {
        ASSERT(false, "invalid start state!! (collision)");
    }

    auto goals = std::make_shared<ob::GoalStates>(si);

    int tot_enum_states = 1, tot_goal_state = 0;
    for (int i = 0; i < dim; i++) 
        tot_enum_states *= 3;

    for (int ii = 0; ii < goal_states.size(); ii++)
        for (int i = 0; i < tot_enum_states; i++) {
            std::vector<double> tmp_state;
            int tmp = i;
            bool flag = true;
            for (int j = 0; j < dim; j++) {
                tmp_state.push_back(goal_states[ii](j));
                int dir = tmp % 3;
                tmp /= 3;
                if (dir != 0 && is_revolute[j] == false) {
                    flag = false;
                    break;
                }
                if (dir == 1) {
                    if (tmp_state[j] - 2 * PI > lower_joint_limits[j]) 
                        tmp_state[j] -= 2 * PI;
                    else {
                        flag = false;
                        break;
                    }
                }
                else if (dir == 2) {
                    if (tmp_state[j] + 2 * PI < upper_joint_limits[j]) 
                        tmp_state[j] += 2 * PI;
                    else {
                        flag = false;
                        break;
                    }                
                }
            }
            if (flag) {
                ob::ScopedState<> goal(cs);
                goal = tmp_state; 
                goals->addState(goal);
                tot_goal_state += 1;
            }
        }
    if (verbose)
        std::cout << "number of goal state: " << tot_goal_state << std::endl;

    ss->clear();
    ss->setStartState(start);
    ss->setGoal(goals);
    ss->setup();
    si->setPropagationStepSize(range);
    propagator->setIntegrationTimeStep(integration_step);

    if (verbose)
        std::cout << "OMPL setup" << std::endl;
    ob::PlannerStatus solved = ss->solve(time);
    
    if (solved) {
        if (verbose)
            std::cout << "Solved!" << std::endl;
        auto control_path = ss->getSolutionPath(); // #.asGeometric();

        //auto geo_path = std::dynamic_pointer_cast<og::PathGeometric>(path);
        size_t len = control_path.getStateCount();
        Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(len + invalid_start, dim);
        if (verbose)
            std::cout << "Result size " << len << " " << dim << std::endl;
        if (invalid_start) {
            for (int j = 0; j < dim; j++)
                ret(0, j) = start_state(j);
        }
        for (size_t i = 0; i < len; i++) {
            auto res_i = state2eigen<DATATYPE>(control_path.getState(i), si.get());
            //std::cout << "Size_i " << res_i.rows() << std::endl;
            ASSERT(res_i.rows() == dim, "Result dimension is not correct!");
            for (size_t j = 0; j < dim; j++)
                ret(invalid_start + i, j) = res_i[j];
        }
        return std::make_pair(solved.asString(), ret);
    }
    else {
        Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(0, dim);
        return std::make_pair(solved.asString(), ret);
    }
}
