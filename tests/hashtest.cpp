#include "doctest.h"
#include <boost/functional/hash.hpp>
#include "State.cpp"
#include "Primitive.cpp"
#include <boost/unordered_set.hpp>

TEST_CASE("Hash the state"){
    boost::hash<StateSpace> state_hasher;
    StateSpace state1(1,2,3,4);
    StateSpace state2(4,5,6,7);
    StateSpace state3(10,11,12,13);
    StateSpace state4 = state1;

    CHECK(state_hasher(state1) == state_hasher(state4));

    CHECK(state_hasher(state1) != state_hasher(state2));
    CHECK(state_hasher(state1) != state_hasher(state3));

}
TEST_CASE("Hash the primitive"){
    boost::hash<Primitive> primitive_hasher;

    //Prim 1
    StateSpace initalState(1,2,3,4);
    StateSpace finalState(5,6,7,8);
    Primitive prim(1,initalState, finalState,12, 1, 1);
    SimpleTrajectory<StateSpace> traj(initalState);
    traj.add(finalState);
    prim.addIntermediaryStateTrajectory(traj);

    //Prim 2
    StateSpace initalState2(10, 12,13,14);
    StateSpace finalState2(14,15,16,17);
    Primitive prim2(1,initalState2, finalState2,12, 1, 1);

    Primitive prim3 = prim2;

    CHECK(primitive_hasher(prim) != primitive_hasher(prim2));
    CHECK(primitive_hasher(prim3) == primitive_hasher(prim2));
    
    boost::unordered_set<Primitive> prims;
    prims.emplace(prim);
    prims.emplace(prim2);

    auto it = prims.find(prim2);
    CHECK(it->m_initialState == initalState2);

    // Prim to found
    // A primitive with the same initial and final states but different ids etc.
    Primitive prim_dif(100,initalState2, finalState2,12231, 1, 1);
    auto ite = prims.find(prim_dif);
    CHECK(ite->m_initialState == initalState2);

}
