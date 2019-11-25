#include "PrimitivesCollection.cpp"
#include <boost/math/constants/constants.hpp>
#include "SimpleTrajectory.h"
#include "PrimitiveCollusionPoints.cpp"
#include <math.h>
#pragma once 

class PrimitiveAdapter
{
private:
    PrimitivesCollection m_collection;
    PrimitiveCollusionPointsCollection m_collusionPoints;
public:
    PrimitiveAdapter(/* args */) = default;
    PrimitiveAdapter(const PrimitivesCollection&);
    
    /* Find the primitive from the database */
    Primitive findPrimitive(Primitive) const;

    /*Shift the primitives from the databse to real coordinates */
    Primitive shiftPrimitive(Primitive, StateSpace to) const;
    
    ~PrimitiveAdapter() = default;

    // From a stataspace trajectory find the corresponding primitive trajectory in the database
    SimpleTrajectory<Primitive> decryptTrajectory(SimpleTrajectory<StateSpace>);

    PrimitiveAdapter& operator=(const PrimitiveAdapter&);

    // Load the all collusion points to memory
    void loadCollusionPoints();

    // Load the collusion point of the primitive with given id
    CollusionPoints getCollusionPoints(unsigned int, int, int) const;

    bool hasCollusionPoints() const;
};

PrimitiveAdapter::PrimitiveAdapter(const PrimitivesCollection &collection){
    m_collection = collection;

}
Primitive PrimitiveAdapter::findPrimitive(Primitive candidatePrimitive)const {
    StateSpace init = candidatePrimitive[0];
    StateSpace fin = candidatePrimitive[1];

    // Move the initial state to origin
    double x0_n = 0;
    double y0_n = 0;

    // Move the final state with it
    double xf_n = fin[0] - init[0];
    double yf_n = fin[1] - init[1];
    // ShiftVector shift(candidatePrimitive);
    // auto [xf_n, yf_n] = shift.get();

    double pi = boost::math::constants::pi<double>();
    
    // orientation 
    auto theta_pos =  init[2];
   // if(theta_pos < 0)
     //   theta_pos = theta_pos + 2*pi;
    theta_pos = fmod(theta_pos, 2*pi);

    //If in first quadrant 
    if((theta_pos>=0) & (theta_pos<= pi/2) ){
        StateSpace initNormalized(x0_n, y0_n, init[2], init[3]);
        StateSpace finNormalized(xf_n, yf_n, fin[2], fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return m_collection.findPrimitive(primm);
    }
    else if((theta_pos>=pi/2) & (theta_pos<= pi )){
        StateSpace initNormalized(x0_n, y0_n, pi-init[2], init[3]);
        StateSpace finNormalized(-xf_n, yf_n, pi-fin[2], fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return m_collection.findPrimitive(primm);
    }
    else if((theta_pos>=pi) & (theta_pos<= 3*pi/2) ){
        StateSpace initNormalized(x0_n, y0_n, init[2]-pi, init[3]);
        StateSpace finNormalized(-xf_n, -yf_n, fin[2]-pi, fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return m_collection.findPrimitive(primm);
    }
    else{
        StateSpace initNormalized(x0_n, y0_n, 2*pi-init[2], init[3]);
        StateSpace finNormalized(xf_n, -yf_n, 2*pi-fin[2], fin[3]);
        Primitive primm(initNormalized,finNormalized);
        return m_collection.findPrimitive(primm);
    }
}

Primitive PrimitiveAdapter::shiftPrimitive(Primitive primitive, StateSpace to)const{
    
    //move only the x and y components
    StateSpace too(to[0], to[1],0,0);
    
    SimpleTrajectory<StateSpace> shiftTrajectory;
    for(int i =0; i < primitive.m_numberIntermStateSpaces+1 ; i ++)
        shiftTrajectory.add(too);

    // Move the whole trajectory 
    //primitive.m_stateSpaceTrajectory = primitive.m_stateSpaceTrajectory + shiftTrajectory;

    double pi = boost::math::constants::pi<double>();

    // auto normalized_inital_state = primitive[0];
    // auto theta_pos = normalized_inital_state[2];
    auto theta_pos =  to[2];
    
    //if(theta_pos < 0)
        //theta_pos = theta_pos + 2*pi;

    theta_pos = fmod(theta_pos, 2*pi);
    
    // first quadrant
    if((theta_pos>=0) & (theta_pos<= pi/2)) {
        primitive.m_stateSpaceTrajectory = primitive.m_stateSpaceTrajectory + shiftTrajectory;
        return primitive;
    }

    // If in the second quadrant
    else if((theta_pos>=pi/2) & (theta_pos<= pi )){
        auto traj = primitive.m_stateSpaceTrajectory.getData();
         SimpleTrajectory<StateSpace> newTraj;
        // If don't iterate over reference states you'd create copies of it hence not changing the trajectory
        for (auto &state : traj){
            //state.setTheta(fmod(pi - state[2],2*pi));
          //  state.setX(-state[0]);
             newTraj.add(StateSpace(-state[0], state[1], fmod(pi-state[2],2*pi), state[3]));
        }
        primitive.m_stateSpaceTrajectory.setData(newTraj.getData());

        // not sure about this though
        primitive.m_stateSpaceTrajectory = primitive.m_stateSpaceTrajectory + shiftTrajectory;
        return primitive;
        
    }
    // third quadrant
    else if((theta_pos>=pi) & (theta_pos<= 3*pi/2) ){
        auto traj = primitive.m_stateSpaceTrajectory.getData();
        SimpleTrajectory<StateSpace> newTraj;
        for (auto &state : traj){
            //state.setTheta(fmod( state[2]-pi,2*pi));
           // state.setX(-state[0]);
            //state.setY(-state[1]);
            newTraj.add(StateSpace(-state[0], -state[1], fmod(state[2]-pi,2*pi), state[3]));
        }
        primitive.m_stateSpaceTrajectory.setData(newTraj.getData());
        primitive.m_stateSpaceTrajectory = primitive.m_stateSpaceTrajectory + shiftTrajectory;
        
        return primitive;
    }

    // forth quadrant
    else{
        auto traj = primitive.m_stateSpaceTrajectory.getData();
        SimpleTrajectory<StateSpace> newTraj;
        for (auto &state : traj){
            //state.setTheta(fmod(2*pi - state[2],2*pi));
            //state.setY(-state[1]);
            newTraj.add(StateSpace(state[0],-state[1],fmod(2*pi - state[2],2*pi), state[3]));
        }
        primitive.m_stateSpaceTrajectory.setData(newTraj.getData());
        primitive.m_stateSpaceTrajectory = primitive.m_stateSpaceTrajectory + shiftTrajectory;
    
        return primitive;
    }
}

SimpleTrajectory<Primitive> PrimitiveAdapter::decryptTrajectory(SimpleTrajectory<StateSpace> trajectory){
      std::vector<StateSpace> traj = trajectory.getData();
    SimpleTrajectory<Primitive> primitiveTrajectory;
    StateSpace init, fin;
    for(auto it = traj.begin(); it!= traj.end()-1; it++){
        init = *it;
        fin = *std::next(it);
        Primitive prim(init,fin);
        Primitive primim = findPrimitive(prim);

        // shift if a primitive found
        if(primim.m_flag != 0){
            
            auto newprim = shiftPrimitive(primim, prim[0]);
            primitiveTrajectory.add(newprim);

        }
        // if not found return the candidate
        else{
            primitiveTrajectory.add(prim);
        }
        
    }
    return primitiveTrajectory;


}

PrimitiveAdapter& PrimitiveAdapter::operator=(const PrimitiveAdapter & adapter){
    m_collection = adapter.m_collection;
    return *this;
}

void PrimitiveAdapter::loadCollusionPoints(){
    m_collusionPoints.loadCollusionPoints();

}
CollusionPoints PrimitiveAdapter::getCollusionPoints(unsigned int id, int x, int y) const{
    auto mk = m_collusionPoints.getCollusionPoints(id,x,y);
    return mk;
}
// bool PrimitiveAdapter::hasCollusionPoints() const{
//     if(m_collusionPoints != nullptr)
//         return true;
//     else 
//         return false;
// }