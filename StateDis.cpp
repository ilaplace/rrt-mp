#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include <boost/math/constants/constants.hpp>
#include "PrimitiveAdapter.cpp"
#include "Primitive.cpp"
#include "State.cpp"
#pragma once 
namespace ompl
{
    namespace base
    {

    
class StateDis : public CompoundStateSpace
{
private:
    PrimitiveAdapter m_adapter;
    bool m_metric = true;
public:
    class StateType : public CompoundStateSpace::StateType
    {
        public:
        StateType() = default;

        int getX() const {
            return as<DiscreteStateSpace::StateType>(0)->value;
        }
        int getY() const {
            return as<DiscreteStateSpace::StateType>(1)->value;
        }
        double getYaw() const {
            double pi = boost::math::constants::pi<double>();
            double val = as<DiscreteStateSpace::StateType>(2)->value;
            return val*15*pi/180;
        }
        int getSpeed() const {
            return as<DiscreteStateSpace::StateType>(3)->value*2;
        }

        void setX(int x){
            as<DiscreteStateSpace::StateType>(0)->value = x;
        }
        void setY(int y){
            as<DiscreteStateSpace::StateType>(1)->value = y;
        }
         void setXY(int x, int y){
            setX(x);
            setY(y);
        }
        void setYaw(int yaw){
             as<DiscreteStateSpace::StateType>(2)->value = yaw;
        }
        void setYaw(double yaw){
            double pi = boost::math::constants::pi<double>();
            as<DiscreteStateSpace::StateType>(2)->value = (int)yaw*180/15/pi;
        }
        void setSpeed(int speed){
            as<DiscreteStateSpace::StateType>(3)->value = speed/2;
        }
    };
    StateDis(int);
    StateDis(int x, int y);
    StateDis();
    ~StateDis () override = default;
    /*bool isMetricSpace () const override;
    unsigned int getDimension() const override;
    double distance();*/
    bool isCompound () const override;
    State *allocState() const override;
    void freeState(State* state) const override;
    bool equalStates(const State* state1, const State* state2)const override;
    void setAdapter(const PrimitiveAdapter&);
    const PrimitiveAdapter& getAdapter();
    double distance (const State *state1, const State *state2) const override;
    void setBounds(int);
    void setNonMetricDistance();

    friend std::ostream& operator<<(std::ostream &out, const State *s);
    /* Transfor the discrete states to small trajectory primitives */
      
};
StateDis::StateDis(){}
StateDis::StateDis(int bound) 
{
        setName("SE2_V_Discrete");
        // x
        addSubspace(std::make_shared<DiscreteStateSpace>(-bound,bound),1.0);
        // y
        addSubspace(std::make_shared<DiscreteStateSpace>(-bound,bound),1.0);
        // yaw
        addSubspace(std::make_shared<DiscreteStateSpace>(0,24),1.0);
        // speed
        addSubspace(std::make_shared<DiscreteStateSpace>(0,2),1.0); 
        lock();   
}
StateDis::StateDis(int x, int y) 
{
        setName("SE2_V_Discrete");
        // x
        addSubspace(std::make_shared<DiscreteStateSpace>(0,x),1.0);
        // y
        addSubspace(std::make_shared<DiscreteStateSpace>(0,y),1.0);
        // yaw
        addSubspace(std::make_shared<DiscreteStateSpace>(0,24),1.0);
        // speed
        addSubspace(std::make_shared<DiscreteStateSpace>(0,2),1.0); 
        lock();   
}
void StateDis::setNonMetricDistance(){
    m_metric = false;
}
State* StateDis::allocState() const {
    auto *state = new StateType();
    allocStateComponents(state);
    return state;

}

void StateDis::freeState(State *state) const{
    auto *cstate = static_cast<CompoundState *>(state);
     for (unsigned int i = 0; i < componentCount_; ++i)
        {auto c =  components_[i];
         components_[i]->freeState(cstate->components[i]);}
     delete[] cstate->components;
     delete cstate;
}

void StateDis::setAdapter(const PrimitiveAdapter& adapter){
    m_adapter = adapter;
} 
    
double StateDis::distance (const State *state1, const State *state2) const{
    
  //  if(false){
    if(m_metric){   
       return CompoundStateSpace::distance(state1,state2);
    }else{
            
        auto s1 = state1->as<StateDis::StateType>();
        auto s2 = state2->as<StateDis::StateType>();

        ::StateSpace init(s1->getX(), s1->getY(),s1->getYaw(), s1->getSpeed());
        ::StateSpace fin(s2->getX(), s2->getY(),s2->getYaw(), s2->getSpeed());
        Primitive primCandidate(init, fin);

        auto prim = m_adapter.findPrimitive(primCandidate);

        if(equalStates(state1,state2))
            return 0;
        else
            return prim.getCost();
    }

    


    
}
 bool StateDis::isCompound () const {
     return true;
 }

bool StateDis::equalStates(const State* state1, const State* state2)const {
    auto s1 = state1->as<StateDis::StateType>();
    auto s2 = state2->as<StateDis::StateType>();

    // For debug purposeses
    ::StateSpace init(s1->getX(), s1->getY(),s1->getYaw(), s1->getSpeed());
    ::StateSpace fin(s2->getX(), s2->getY(),s2->getYaw(), s2->getSpeed());
    
    //TODO: Check
    return s1->getX() == s2->getX() &&
            s1->getY() == s2->getY() &&
            s1->getYaw() == s2->getYaw() &&
            s1->getSpeed() == s2->getSpeed();
    
}

std::ostream& operator<<(std::ostream &out, const State *s){
     auto state = s->as<StateDis::StateType>();
    int x = state->getX();
    int y = state->getY();
    double w = state->getYaw();
    int v = state->getSpeed();
    out << x << " " << y << " " << w<< " "<< v << std::endl;
    return out;
 }

 void StateDis::setBounds(int bound){
    auto vec = getSubspaces();
     
    //set the given boundries for the x and y space
    vec[0]->as<DiscreteStateSpace>()->setBounds(-bound, bound);
    vec[1]->as<DiscreteStateSpace>()->setBounds(-bound, bound);
 }

 const PrimitiveAdapter& StateDis::getAdapter(){
    return m_adapter;
}
    }
    }

