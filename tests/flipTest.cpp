#include <unistd.h>
#include "State.cpp"
#include <iostream>
#include <fstream>
#include "SimpleTrajectory.h"
#include "PrimitivesCollection.cpp"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "State.cpp"
#include "Primitive.cpp"
#include "PrimitiveAdapter.cpp"
#include "doctest.h"
#include <boost/math/constants/constants.hpp>

using namespace std;

Primitive shifter(Primitive candidatePrimitive){
    StateSpace init = candidatePrimitive[0];
    StateSpace fin = candidatePrimitive[1];

    // Move the initial state to origin
    double x0_n = 0;
    double y0_n = 0;

    // Move the final state with it
    double xf_n = fin[0] - init[0];
    double yf_n = fin[1] - init[0];
    // ShiftVector shift(candidatePrimitive);
    // auto [xf_n, yf_n] = shift.get();

    double pi = boost::math::constants::pi<double>();
    
    // TODO: theta0 deişince final state de değişio
    //If in first quadrant
    if(init[2]>=0 & init[2]<= pi/2 ){
        StateSpace initNormalized(x0_n, y0_n, init[2], init[3]);
        StateSpace finNormalized(xf_n, yf_n, fin[2], fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return primm;
    }
    else if(init[2]>=pi/2 & init[2]<= pi ){
        StateSpace initNormalized(x0_n, y0_n, pi-init[2], init[3]);
        StateSpace finNormalized(-xf_n, yf_n, pi-fin[2], fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return primm;
    }
    else if(init[2]>=pi & init[2]<= 3*pi/2 ){
        StateSpace initNormalized(x0_n, y0_n, init[2]-pi, init[3]);
        StateSpace finNormalized(-xf_n, -yf_n, fin[2]-pi, fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return primm;
    }
    else{
        StateSpace initNormalized(x0_n, y0_n, 2*pi-init[2], init[3]);
        StateSpace finNormalized(xf_n, -yf_n, 2*pi-init[2], init[3]);
        Primitive primm(initNormalized,finNormalized);
        return primm;
    }

    
}
// Shift the primitive where it originally was using initialState
Primitive shiftBack(Primitive primitive, StateSpace to){
    //shift vector is a vector with the same length of the trajectory 

    //move only the x and y components
    StateSpace too(to[0], to[1],0,0);
    
    SimpleTrajectory<StateSpace> shiftTrajectory;
    for(int i =0; i < primitive.m_numberIntermStateSpaces+1 ; i ++)
        shiftTrajectory.add(too);

    // Move the whole trajectory 
    primitive.m_stateSpaceTrajectory = primitive.m_stateSpaceTrajectory + shiftTrajectory;

    double pi = boost::math::constants::pi<double>();

    
    if(to[2]>=pi/2 & to[2]<= pi ){
        auto traj = primitive.m_stateSpaceTrajectory.getData();

        // If don't iterate over reference states you'd create copies of it hence not changing the trajectory
        for (auto &state : traj){
            state.setTheta(pi - state[2]);
            state.setX(-state[1]);
            //cout << state << endl;
        }
        primitive.m_stateSpaceTrajectory.setData(traj);
        //cout << primitive << endl;
        return primitive;
        
    }
}

Primitive generatePrim(){
    double pi = boost::math::constants::pi<double>();
    StateSpace init(1,1,pi-0.785434214142,0);
    StateSpace fin(0,0,pi-0.7854,4);
    Primitive primm(init,fin);

    return primm;
}

SimpleTrajectory<StateSpace> generatePath(){
    double pi = boost::math::constants::pi<double>();

    SimpleTrajectory<StateSpace> traj;
    for (size_t i = 0; i < 10; i++){
        StateSpace init(i,i,pi-0.785434214142,0);
        traj.add(init);
    }
    return traj;
}

SimpleTrajectory<Primitive> decryptTrajectory(SimpleTrajectory<StateSpace> trajectory){
    std::vector<StateSpace> traj = trajectory.getData();
    SimpleTrajectory<Primitive> primitiveTrajectory;
    StateSpace init, fin;
    PrimitivesCollection collect;
    collect.loadDatabase();
    PrimitiveAdapter adapter(collect);
    //std::vector<StateSpace>::iterator it;
    for(auto it = traj.begin(); it!= traj.end()-1; it++){
        init = *it;
        fin = *std::next(it);
        //fin = *(it++);
        Primitive prim(init,fin);
        Primitive primim = adapter.findPrimitive(prim);
        auto newprim = adapter.shiftPrimitive(primim, prim[0]);
        primitiveTrajectory.add(newprim);
    }
    return primitiveTrajectory;


}
// TEST_CASE("flip"){

//     // The points to be connected
//     auto prim = generatePrim();

//     //The corresponding primitive from the database
//     auto correspons = shifter(prim);

//     cout << "Shifted prims candidate: " << correspons << endl;
//     PrimitivesCollection collect;
//     std::ifstream ifs("file");
//     boost::archive::text_iarchive ia(ifs);
//     ia >> collect;
    
//     auto primitives = collect.getPrimitives();
//     auto primitive = primitives.find(correspons);
//     Primitive primim = *primitive;
//     auto newprim = shiftBack(primim, prim[0]);

    
//     cout << "Original Primitive Id: " << primitive->m_id << endl;
//     cout << "InitialState: " << primitive->m_initialState << endl;
//     cout << "FinalState: " << primitive->m_finalState << endl;
//     //cout << primitive->m_stateSpaceTrajectory << endl;
//     //cout << primim << endl;

// }

// TEST_CASE("flipPro"){

//     // The points to be connected
//     auto prim = generatePrim();

//     //The corresponding primitive from the database
//     auto correspons = shifter(prim);

//     cout << "Shifted prims candidate: " << correspons << endl;
//     PrimitivesCollection collect;
//     collect.loadDatabase();
    
//     Primitive primim = collect.findPrimitive(correspons);
//     auto newprim = shiftBack(primim, prim[0]);

    
//     cout << "Original Primitive Id: " << primim.m_id << endl;
//     cout << "InitialState: " << primim.m_initialState << endl;
//     cout << "FinalState: " << primim.m_finalState << endl;
//     //cout << primitive->m_stateSpaceTrajectory << endl;
//     //cout << primim << endl;

// }
TEST_CASE("muchmorepro"){

    // The points to be connected
    auto prim = generatePrim();

    PrimitivesCollection collect;
    collect.loadDatabase();
    
    PrimitiveAdapter adapter(collect);
    Primitive primim = adapter.findPrimitive(prim);
    auto newprim = adapter.shiftPrimitive(primim, prim[0]);
    
    
    cout << "Original Primitive Id: " << newprim.m_id << endl;
    cout << "InitialState: " << newprim.m_initialState << endl;
    cout << "FinalState: " << newprim.m_finalState << endl;
    //cout << newprim << endl;

}

TEST_CASE("Primitive trajectory"){
    auto primitivesPath = decryptTrajectory(generatePath());
    for(auto primitive : primitivesPath.getData())
        cout << primitive.m_initialState;
}