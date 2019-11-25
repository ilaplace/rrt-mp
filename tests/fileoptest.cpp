#include <unistd.h>
#include "State.cpp"
#include <iostream>
#include <fstream>
#include "SimpleTrajectory.h"
#include <vector>
#include "PrimitivesCollection.cpp"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "State.cpp"
#include "Primitive.cpp"
#include "doctest.h"


using namespace std;

StateSpace save(){

    StateSpace initalState(1,2,3,4);
    std::ofstream ofs("mk");
    boost::archive::text_oarchive oa(ofs);
    oa << initalState;
    return initalState;

}
StateSpace loadState(){
    StateSpace init;
    ifstream ifs("mk");
    boost::archive::text_iarchive ia(ifs);
    ia >> init;
    return init;
}

SimpleTrajectory<StateSpace> saveTraj(){
    StateSpace initalState(1,2,3,4);
    StateSpace finalState(5,6,7,8);
    SimpleTrajectory<StateSpace> traj(initalState);
    traj.add(finalState);

    std::ofstream ofs("mk");
    boost::archive::text_oarchive oa(ofs);
    oa << traj;
    std::cout << "Saved trajectory:" << traj << std::endl;
    return traj;
}
SimpleTrajectory<StateSpace> loadTraj(){
    SimpleTrajectory<StateSpace> traj;
    std::ifstream ifs("mk");
    boost::archive::text_iarchive ia(ifs);
    ia >> traj;
    std::cout << "Loadad trajectory:" << traj << std::endl;
    return traj;
}

Primitive savePrim(){
    
    StateSpace initalState(1,2,3,4);
    StateSpace finalState(5,6,7,8);
    Primitive prim(1,initalState, finalState,12, 1, 1);
    SimpleTrajectory<StateSpace> traj(initalState);
    traj.add(finalState);
    prim.addIntermediaryStateTrajectory(traj);
    std::ofstream ofs("mk");
    boost::archive::text_oarchive oa(ofs);
    oa << prim;
    std::cout << "Saved primitive:" << prim << std::endl;

    return prim;

}
Primitive loadPrim(){
    Primitive prim;
    std::ifstream ifs("mk");
    boost::archive::text_iarchive ia(ifs);
    ia >> prim;
    auto traj =  prim.m_stateSpaceTrajectory;
    std::cout << "Loadad primitive:" << prim << std::endl;
    return prim; 
    
}

PrimitivesCollection saveKoleksyon(){
    auto prim1 = loadPrim();
    auto prim2 = loadPrim();

    std::ofstream ofs("mk");
    boost::archive::text_oarchive oa(ofs);

    PrimitivesCollection kolye;
    kolye.addPrimitive(prim1);
    kolye.addPrimitive(prim2);

    oa << kolye;
    std::cout << "Saved kolye:" << kolye << std::endl;
    return kolye;   
}
PrimitivesCollection loadKolye(){
    PrimitivesCollection kolye;
    std::ifstream ifs("mk");
    boost::archive::text_iarchive ia(ifs);
    ia >> kolye;
    std::cout << "Loaded kolye:" << kolye << std::endl;
    return kolye;
}

TEST_CASE("testing the serializition and deserialization") {
    CHECK(save() == loadState());
    CHECK(saveTraj() == loadTraj());
    
}
TEST_CASE("Save collections"){
    CHECK(savePrim() == loadPrim());
    CHECK(saveKoleksyon() == loadKolye());
}

