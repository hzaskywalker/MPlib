#include "dubins_planner.h"
#include "pinocchio_model.h"

using namespace dubins;


#define DEFINE_TEMPLATE_OMPL(DATATYPE) template class ValidityCheckerTpl<DATATYPE>; template class OMPLPlannerTpl<DATATYPE>;

DEFINE_TEMPLATE_OMPL(double)

DEFINE_TEMPLATE_OMPL(float)

#define PI 3.14159265359

template<typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_state_space(void) {
    cs = std::make_shared<CompoundStateSpace>();
    dim = 0;
    std::string const joint_prefix = "JointModel";
    for (auto robot: world->getArticulations()) {
        auto dim_i = 0;
        auto model = robot->getPinocchioModel();
        auto joint_types = model.getJointTypes();
        auto d = robot->getQposDim(); // TODO!!! only construct for move group joints
        auto indices = robot->getMoveGroupJointIndices();
        ASSERT(d == indices.size(), "QposDim != size of the movegroup joints");

        // 0, 1, 2 will be dubins
        // skjfaslkdfjlasjfsl
        auto dubins_space = std::make_shared<ob::DubinsStateSpace>(1.0);
        auto bound0 = model.getJointLimit(indices[0]);
        auto bound1 = model.getJointLimit(indices[1]);
        lower_joint_limits.push_back(bound0(0, 0));
        lower_joint_limits.push_back(bound1(0, 0));
        upper_joint_limits.push_back(bound0(0, 1));
        upper_joint_limits.push_back(bound1(0, 1));



        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, bound0(0, 0));
        bounds.setHigh(0, bound0(0, 1));
        bounds.setLow(1, bound1(0, 0));
        bounds.setHigh(1, bound1(0, 1));
        dubins_space->setBounds(bounds);
        cs->addSubspace(dubins_space, 1.0);
        dim_i = 3.;
        is_revolute.push_back(false);
        is_revolute.push_back(false);
        is_revolute.push_back(false);

        for (size_t i = 3; i < d; i++) {
            auto id = indices[i];
            auto joint_type = joint_types[id];
            if (joint_type[joint_prefix.size()] == 'P' || (joint_type[joint_prefix.size()] == 'R' &&
                                                        joint_type[joint_prefix.size() + 1] != 'U'))// PRISMATIC and REVOLUTE
            {
                auto bound = model.getJointLimit(id);
                auto subspcae = std::make_shared<ob::RealVectorStateSpace>(bound.rows());
                auto ob_bounds = ob::RealVectorBounds(bound.rows());
                dim_i += bound.rows();
                for (size_t j = 0; j < bound.rows(); j++) {
                    lower_joint_limits.push_back(bound(j, 0));
                    upper_joint_limits.push_back(bound(j, 1));
                    ob_bounds.setLow(j, bound(j, 0)), ob_bounds.setHigh(j, bound(j, 1));
                }
                subspcae->setBounds(ob_bounds);
                cs->addSubspace(subspcae, 1.0);
            } else if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] == 'U') {
                cs->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
                lower_joint_limits.push_back(-PI);
                upper_joint_limits.push_back(PI);
                dim_i += 1;
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



template<typename DATATYPE>
OMPLPlannerTpl<DATATYPE>::OMPLPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world):world(world) {
    build_state_space();
    si = std::make_shared<SpaceInformation>(cs);
    valid_checker = std::make_shared<ValidityChecker>(world, si);
    si->setStateValidityChecker(valid_checker);

    pdef = std::make_shared<ob::ProblemDefinition>(si);
}

template<typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> OMPLPlannerTpl<DATATYPE>::random_sample_nearby(VectorX const &start_state) {
    int cnt = 0;
    while (true) {
        DATATYPE ratio = (DATATYPE) (cnt + 1) / 1000;
        VectorX new_state = start_state;
        for (int i = 0; i < dim; i++) {
            DATATYPE r = (DATATYPE) rand() / RAND_MAX * 2 - 1;
            new_state[i] += (upper_joint_limits[i] - lower_joint_limits[i]) * ratio * r;
            if (new_state[i] < lower_joint_limits[i])
                new_state[i] = lower_joint_limits[i];
            else if (new_state[i] > upper_joint_limits[i])
                new_state[i] = upper_joint_limits[i];
        }
        if (valid_checker->_isValid(new_state)) {
            std::cout << "successfully sampled a new state with a perturbation of " << ratio * 100 << "% joint limits." << std::endl;
            return new_state;
        }
        cnt += 1;
        if (cnt > 1000)
            return start_state;
    }
}

template<typename DATATYPE>
std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
OMPLPlannerTpl<DATATYPE>::plan(VectorX const &start_state, std::vector<VectorX> const &goal_states, const std::string &planner_name,
                                const double &time, const double& range, const bool& verbose) {
    ASSERT(start_state.rows() == goal_states[0].rows(), "Length of start state and goal state should be equal");
    ASSERT(start_state.rows() == dim, "Length of start state and problem dimension should be equal");
    if (verbose == false)
        ompl::msg::noOutputHandler();

    ob::ScopedState<> start(cs);
    start = eigen2vector<DATATYPE, double>(start_state);

    bool invalid_start = !valid_checker->_isValid(start_state);
    if (invalid_start) {
        //std::cout << "invalid start state!! (collision)" << std::endl;
        VectorX new_start_state = random_sample_nearby(start_state);
        start = eigen2vector<DATATYPE, double>(new_start_state);
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

    pdef->clearStartStates();
    pdef->clearGoal();
    pdef->clearSolutionPaths();
    pdef->clearSolutionNonExistenceProof();
    //pdef->setStartAndGoalStates(start, goal);
    pdef->setGoal(goals);
    pdef->addStartState(start);
    ob::PlannerPtr planner;
    if (planner_name == "RRTConnect")
    {
        auto rrt_connect = std::make_shared<og::RRTConnect>(si);
        if (range > 1E-6)
            rrt_connect->setRange(range);
        planner = rrt_connect;
    }
    else
        throw std::runtime_error("Planner Not implemented");

    planner->setProblemDefinition(pdef);
    planner->setup();
    if (verbose)
        std::cout << "OMPL setup" << std::endl;
    ob::PlannerStatus solved = planner->ob::Planner::solve(time);
    if (solved) {
        if (verbose)
            std::cout << "Solved!" << std::endl;
        ob::PathPtr path = pdef->getSolutionPath();
        auto geo_path = std::dynamic_pointer_cast<og::PathGeometric>(path);
        size_t len = geo_path->getStateCount();
        Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(len + invalid_start, dim);
        if (verbose)
            std::cout << "Result size " << len << " " << dim << std::endl;
        if (invalid_start) {
            for (int j = 0; j < dim; j++)
                ret(0, j) = start_state(j);
        }
        for (size_t i = 0; i < len; i++) {
            auto res_i = state2eigen<DATATYPE>(geo_path->getState(i), si.get());
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
