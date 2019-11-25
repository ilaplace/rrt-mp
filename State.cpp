#include <vector>
#include <iostream>
#include <boost/archive/text_oarchive.hpp>      
#include <boost/archive/text_iarchive.hpp>
#include <boost/functional/hash_fwd.hpp>
# include <math.h>
#pragma once

class StateSpace{

     //friend class Hash;
     double m_x;
     double m_y;
     double m_theta;
     double m_v; 

     friend class boost::serialization::access;
     template<class Archive>
     void serialize(Archive &ar, const unsigned int version){
             ar &m_x;
             ar &m_y;
             ar &m_theta;
             ar &m_v;
     }

     public:
        StateSpace(){};
        StateSpace(double x, double y, double theta, double v) : m_x(x),m_y(y), m_theta(theta), m_v(v){};

        bool operator == (const StateSpace& other) const {
            return other.m_x == m_x &&
                    other.m_y == m_y &&
                   fabs(other.m_theta - m_theta) < 0.01 &&
                    other.m_v == m_v;   
        }
        bool operator != (const StateSpace& other) const {
            return other.m_x != m_x &&
                    other.m_y != m_y &&
                    fabs(other.m_theta - m_theta) >= 0.01 &&
                    other.m_v != m_v; 
        }
        
        friend std::size_t hash_value(StateSpace const& state){
                std::size_t seed = 0;
                int theta = int(state.m_theta*100);
                boost::hash_combine(seed, state.m_x);
                boost::hash_combine(seed, state.m_y);
                boost::hash_combine(seed, theta);
                boost::hash_combine(seed, state.m_v);
                return seed;
        }

        friend std::ostream& operator<<(std::ostream& out, const StateSpace& StateSpace){
            out << StateSpace.m_x << "," 
                    << StateSpace.m_y << ","
                    << StateSpace.m_theta << ","
                    << StateSpace.m_v;

            return out;
        };

        void setTheta(double theta){
                m_theta = theta;
        }

        void setX(double x){
                m_x =x;
        }
        double getX(){
                return m_x;
        }
        double getY(){
                return m_y;
        }
        void setY(double y){
                m_x = y;
        }
        void setV(double v){
                m_v = v;
        }
        /* Sum the x and y only
         */
        friend StateSpace operator+(StateSpace lhs, const StateSpace &rhs){
                lhs.m_x += rhs.m_x;
                lhs.m_y += rhs.m_y;
                return lhs;
        }

        friend std::vector<StateSpace> operator+(std::vector<StateSpace> lhs, const std::vector<StateSpace> &rhs){
                for (std::size_t i=0; i < lhs.size(); i++){
                        lhs[i] = lhs[i] + rhs[i];
                }
                return lhs;
        }


        ~StateSpace(){};

        /*  Returns the state's m_x, m_y, m_theta, m_v respectively
         */
        double operator[](unsigned int i){
                double state [4] = {m_x, m_y,m_theta, m_v};
                return state[i];
        };

};

class ControlSpace{
        double m_a;
        double m_w;
        ControlSpace();
        
        public:
                ControlSpace(double a, double w ) : m_a(a), m_w(w){};
                        bool operator == (const ControlSpace& other) const {
                        return other.m_a == m_a &&
                                other.m_w == m_w;
                 
        }
                friend std::ostream& operator<<(std::ostream& out, const ControlSpace& controlSpace){
            out << "(" << controlSpace.m_a << "," 
                    << controlSpace.m_w << ")";

            return out;
        };


};