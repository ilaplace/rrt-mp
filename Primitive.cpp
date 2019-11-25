#include <iostream>
#include "State.cpp"
#include "SimpleTrajectory.h"
//#include "PrimitiveCollusionPoints.cpp"
#include <boost/serialization/vector.hpp>
#include <boost/functional/hash.hpp>


#pragma once
    // Always keep the trajector in memory
    // or load when necesey like in the end?
    // should I care about this in here 
    // maybe later I can create a lighter derived class

class Primitive
{
private:
    
    //SimpleTrajectory<ControlSpace> m_controlSpaceTrajectory;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version){
        ar &m_id;
        ar &m_numberIntermStateSpaces;
        ar &m_stateSpaceTrajectory;
        ar &m_initialState;
        ar &m_finalState;
        ar &m_cost;
        ar &m_flag;
    
    }

public:
    
    unsigned int m_id;
    StateSpace m_initialState;
    StateSpace m_finalState;
    int m_numberIntermStateSpaces;
    double m_cost;
    bool m_flag;
   // PrimitiveCollusionPoints m_points;
    SimpleTrajectory<StateSpace> m_stateSpaceTrajectory;
    Primitive();
    Primitive(StateSpace, StateSpace);
    Primitive(int, StateSpace, StateSpace, int, double, bool);
    ~Primitive();
    void addIntermediaryStateTrajectory(SimpleTrajectory<StateSpace> traj);
    SimpleTrajectory<StateSpace> getStateTrajectory() const;
    friend std::ostream& operator<<(std::ostream &out, const Primitive &prim); 
    friend bool operator== (const Primitive &lhs, const Primitive &rhs);
    friend bool operator!= (const Primitive &lhs, const Primitive &rhs);
    friend std::size_t hash_value(Primitive const& p);
    StateSpace operator[](unsigned int i);
    StateSpace getInitial();
    StateSpace getFinal();
    unsigned int getId() const;
    double getCost() const;
    void rotate(double radiens);
};

Primitive::Primitive(){}
Primitive::Primitive(StateSpace initState, StateSpace finalState) : m_initialState(initState), m_finalState(finalState){}
Primitive::Primitive(int id, StateSpace initState, StateSpace finalState, int numberOfStates, double cost, bool flag) : m_id(id) ,m_initialState(initState), m_finalState(finalState), m_numberIntermStateSpaces(numberOfStates), m_cost(cost), m_flag(flag){}
Primitive::~Primitive(){}

void Primitive::addIntermediaryStateTrajectory(SimpleTrajectory<StateSpace> trajectory){
    m_stateSpaceTrajectory = trajectory;
}

SimpleTrajectory<StateSpace> Primitive::getStateTrajectory() const {
    return m_stateSpaceTrajectory;
}

std::ostream& operator << (std::ostream &out, const Primitive &primitive){
    auto traj = primitive.getStateTrajectory();
    auto array = traj.getData();
    //out << primitive.m_initialState;
    for (auto state : array)
        out << state << std::endl;
   // out << primitive.m_finalState;
    return out;

}

bool operator == (const Primitive &lhs, const Primitive &rhs){
    return lhs.m_initialState == rhs.m_initialState &&
        lhs.m_finalState == rhs.m_finalState;

}
bool operator != (const Primitive &lhs, const Primitive &rhs){
    return lhs.m_initialState != rhs.m_initialState &&
        lhs.m_finalState != rhs.m_finalState;

}
StateSpace Primitive::operator[](unsigned int i){
    StateSpace states [2] = {m_initialState, m_finalState};
    return states[i];
}

StateSpace Primitive::getInitial(){
    return m_initialState;
}
StateSpace Primitive::getFinal(){
    return m_finalState;
}

std::size_t hash_value(Primitive const& prim){
    boost::hash<StateSpace> state_hasher;
    std::size_t seed = 0;
    boost::hash_combine(seed, state_hasher(prim.m_initialState));
    boost::hash_combine(seed, state_hasher(prim.m_finalState));
    return seed;
}

double Primitive::getCost() const {
    return m_cost;
}
unsigned int Primitive::getId() const {
    return m_id;
}
// class Hash{
//     public:
//     size_t operator()(const Primitive & prim) const {
//     std::size_t h1 = std::hash<int>()(prim.m_initialState.m_x);
//     std::size_t h1 = std::hash<int>()(prim.m_initialState.m_x);


//     return h1;
//     };
// };
