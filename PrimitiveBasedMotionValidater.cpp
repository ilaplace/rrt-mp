#include <ompl/base/MotionValidator.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include "PrimitiveCollusionPoints.cpp"
#include "StateDis.cpp"
#include "PrimitiveAdapter.cpp"

#pragma once
class PrimitiveBasedMotionValidater : public ompl::base::MotionValidator
{
private:
    PrimitiveAdapter m_adapter;
public:
    PrimitiveBasedMotionValidater(ompl::base::SpaceInformationPtr si, const PrimitiveAdapter& adapter);
    ~PrimitiveBasedMotionValidater();
    bool checkMotion(const ompl::base::State *state1, const ompl::base::State *state2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;
};

PrimitiveBasedMotionValidater::PrimitiveBasedMotionValidater(ompl::base::SpaceInformationPtr si, const PrimitiveAdapter& adapter) : ompl::base::MotionValidator(si), m_adapter(adapter){}

PrimitiveBasedMotionValidater::~PrimitiveBasedMotionValidater(){}

bool PrimitiveBasedMotionValidater::checkMotion(const ompl::base::State *state1, const ompl::base::State *state2) const {
    auto s1 = state1->as<ompl::base::StateDis::StateType>();
    auto s2 = state2->as<ompl::base::StateDis::StateType>();
    // if(m_adapter.hasCollusionPoints()){
    //     std::cout << "Has Collusion points" << std::endl;
    // }

    // Create a primitive object from the given states to search
    ::StateSpace init(s1->getX(), s1->getY(),s1->getYaw(), s1->getSpeed());
    ::StateSpace fin(s2->getX(), s2->getY(),s2->getYaw(), s2->getSpeed());
    Primitive primCandidate(init, fin);

    auto prim = m_adapter.findPrimitive(primCandidate);


    // If there is such valid primitive
    if(prim.m_flag == 1){
        auto shiftedprim = m_adapter.shiftPrimitive(prim, init);
        
        auto validity = si_->getStateValidityChecker();

          
        // Create a state to check the validity of the point
        auto statePtr = si_->allocState();

        // Cast the state
        auto state =  statePtr->as<ompl::base::StateDis::StateType>();


       for(auto stated : shiftedprim.getStateTrajectory().getData()){
            state->setXY(stated.getX(),stated.getY());
            if(!validity->isValid(state)){
                return false; 
                }
       }        
        
        // load it's collusion points shifted
        auto points = m_adapter.getCollusionPoints(prim.getId(), s1->getX(), s1->getY());
        
      

        // Check if the motion is collusion free
        for(auto point : points.get()){
            
            state->setXY(point.first, point.second);
            
            // If any point is in collusion
            if(!validity->isValid(state)){
                return false; 
                }
        }return true;

    }else
        return false;
}

bool PrimitiveBasedMotionValidater::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const{
     std::cout << "This should not appear" << std::endl;
     return true;
 }



class MyGoal : public ompl::base::GoalState{
    public:
    MyGoal(const ompl::base::SpaceInformationPtr &si) : ompl::base::GoalState(si){}

    bool isSatisfied(const ompl::base::State *st) const override{

        //TODO: Check if bounds matter here
        ompl::base::StateDis statespace(50);
        return statespace.equalStates(state_,st);   
    }
    bool isSatisfied(const ompl::base::State *st, double *distance) const override{
        bool result = isSatisfied(st);
        if (distance != NULL)
        {
            if (true)
                    {
                         *distance = std::numeric_limits<double>::infinity();
                //*distance = std::numeric_limits<double>::max();
            }
            // else
            // {
            //     if (true)
            //         *distance = 1;
            //     else
            //         *distance = 100;
            // }
        }
        return result;
    }
};

namespace ompl{
    namespace base{
        class PrimitiveCostObjective : public OptimizationObjective{
            private:
                PrimitiveAdapter m_adapter;
            public:
                PrimitiveCostObjective(const SpaceInformationPtr &si, const PrimitiveAdapter& adapter);
                Cost stateCost (const State *s) const override;
                Cost motionCost (const State *s1, const State *s2) const override;

        };

        PrimitiveCostObjective::PrimitiveCostObjective(const SpaceInformationPtr &si, const PrimitiveAdapter& adapter) :OptimizationObjective(si), m_adapter(adapter) {}

        Cost PrimitiveCostObjective::motionCost(const State *state1, const State *state2) const{
            auto s1 = state1->as<StateDis::StateType>();
            auto s2 = state2->as<StateDis::StateType>();

            ::StateSpace init(s1->getX(), s1->getY(),s1->getYaw(), s1->getSpeed());
            ::StateSpace fin(s2->getX(), s2->getY(),s2->getYaw(), s2->getSpeed());
            Primitive primCandidate(init, fin);

            auto prim = m_adapter.findPrimitive(primCandidate);

            if(prim.m_flag == 1){
        //std::cout <<"Found primitive's id "<< prim.m_id << std::endl;
             return Cost(prim.getCost());
            }else
            return Cost(3);
        }

        Cost PrimitiveCostObjective::stateCost (const State *s) const{
            return Cost(1.0);
        }
    }
}
