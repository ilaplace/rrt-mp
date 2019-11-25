#include <unistd.h>
#include "State.cpp"
#include <iostream>
#include <fstream>
#include "SimpleTrajectory.h"
#include "PrimitivesCollection.cpp"
#include "State.cpp"
#include "Primitive.cpp"
#include "PrimitiveAdapter.cpp"
#include "doctest.h"
#include <boost/math/constants/constants.hpp>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "StateDis.cpp"
#include "PrimitiveBasedMotionValidater.cpp"
#include "ompl/geometric/PathGeometric.h"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

TEST_CASE("Sampling test"){
    ob::StateSpacePtr space(new ob::StateDis(50));
    auto statePtr = space->allocState();
    auto sampler = space->allocStateSampler();
    sampler->sampleUniform(statePtr);
    auto state =  statePtr->as<ob::StateDis::StateType>();
    int x = state->getX();
    int y = state->getY();
    double w = state->getYaw();
    int v = state->getSpeed();
    std::cout << "(" << x << "," << y << "," << w<< ","<< v<< ")" << std::endl;
    ompl::base::ScopedState<> s(space);
}

bool isStateValid(const ob::State *state){
    return true;
}
TEST_CASE("planning"){

    
    PrimitivesCollection collect;
    collect.loadDatabase();
    
    PrimitiveAdapter adapter(collect);

    auto space = std::make_shared<ob::StateDis>(50);

    //space->setLongestValidSegmentFraction(0.99);
    space->setAdapter(adapter);
    
    auto si(std::make_shared<ob::SpaceInformation>(space));
    //ob::SpaceInformationPtr si(space.get());

    si->setStateValidityChecker(isStateValid);
    si->setMotionValidator(std::make_shared<PrimitiveBasedMotionValidater>(si, adapter));

    auto startPtr = si->allocState();
    auto start =  startPtr->as<ob::StateDis::StateType>();
    start->setXY(0,0);
    start->setYaw(0);
    start->setSpeed(0);

    auto goalPtr = si->allocState();
    auto goal = goalPtr->as<ob::StateDis::StateType>();
    goal->setXY(15,15);
    goal->setYaw(0);
    goal->setSpeed(0);

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    auto goalChecker(std::make_shared<MyGoal>(si));
    goalChecker->setState(goal);
    pdef->setGoal(goalChecker);

    //pdef->setStartAndGoalStates(start,goal);
    pdef->addStartState(start);
    
    auto rrt(std::make_shared<og::RRTstar>(si));
    //rrt->setGoalBias(0.5);
    auto planner(rrt);

    planner->setProblemDefinition(pdef);

    planner->setup();

    
    //std::cout << "Timeout 30 seconds" << std::endl;
    ob::PlannerStatus solved;// = planner->ob::Planner::solve(30.0);
    
    //WARN(bool(solved));

    SUBCASE("solve for 20"){
        si->printSettings(std::cout);
        space->setBounds(35);
        std::cout << "Timeout 20 seconds" << std::endl;
        solved = planner->ob::Planner::solve(20.0);
        //  auto y = bool(solved);
        //  std::cout << y;
        WARN(bool(solved));
    }
    
    SUBCASE("solve in a space 200x200"){
        planner->clear();
        space->setBounds(100);
        std::cout << "Timeout 40 seconds" << std::endl;
        solved = planner->ob::Planner::solve(40.0);
        WARN(bool(solved));
    
    if(!bool(solved)){
        std::cout << "Failed to solve will try for 40 seconds more" << std::endl;
        planner->clear();
        solved = planner->ob::Planner::solve(80.0);
        WARN(bool(solved));
    }
    if(!bool(solved)){
        planner->clear();
        solved = planner->ob::Planner::solve(120.0);
        CHECK(bool(solved));
    }
    if(!bool(solved)){
        //planner->clear();
        solved = planner->ob::Planner::solve(120.0);
        CHECK(bool(solved));
    }
        
     }
    // SUBCASE("solve for 120"){
    //     std::cout << "Timeout 60 seconds" << std::endl;
    //     solved = planner->ob::Planner::solve(60.0);
    // }
  

     if (solved){
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Found solution:" << std::endl;

         auto pathGeo = path->as<og::PathGeometric>();
         auto result = pathGeo->getStates(); 
         std::ofstream myfile;
         myfile.open("deneme.txt");
        if (myfile.is_open()){
             for(auto state : result)
                myfile << state;  
            myfile.close();
        }else{
            std::cout << "Unable to open file"<< std::endl;
        }
             
         //path->print(std::cout);

     }
     else
         std::cout << "No solution found" << std::endl;
}