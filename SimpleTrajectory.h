#include "State.cpp"
#include <vector>
#include <boost/serialization/vector.hpp>

#pragma once 
template<typename _T>
class SimpleTrajectory
{
private:
    std::vector<_T> m_data;
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version){
        ar & m_data;
    }
    
public:
    SimpleTrajectory();
    SimpleTrajectory(std::vector<_T>);
    SimpleTrajectory(_T state);
    ~SimpleTrajectory();
    void add(_T state);
        void render();
    std::vector<_T> getData();
    template <typename U>
    friend bool operator == (const SimpleTrajectory<U>& rhs, const SimpleTrajectory<U>& lhs);
    template <typename U>
    friend bool operator != (const SimpleTrajectory<U>& rhs, const SimpleTrajectory<U>& lhs);
    template <typename U> // all instantiations of this template are my friends
    friend std::ostream& operator<< (std::ostream& out, const SimpleTrajectory<U>& trajectory);
    template <typename U>
    friend SimpleTrajectory<U> operator+(SimpleTrajectory<U> lhs, const SimpleTrajectory<U>& rhs);
    void setData(std::vector<_T> data);
};



template <typename _T>
std::ostream& operator<<(std::ostream& out, const SimpleTrajectory<_T>& trajectory){
   // auto m_data = trajectory.getData();
    for(auto data : trajectory.m_data)
        out << data << std::endl;
    return out;
}
template <typename _T>
SimpleTrajectory<_T>::SimpleTrajectory(){}

template <typename _T> 
SimpleTrajectory<_T>::SimpleTrajectory(std::vector<_T> vec) : m_data(vec)
{
}

// remove this because results in duplicated initialstates in trajectory
template <typename _T>
SimpleTrajectory<_T>::SimpleTrajectory(_T state){
    m_data.push_back(state);
}

template <typename _T>
SimpleTrajectory<_T>::~SimpleTrajectory()
{
}
template <typename _T>
void SimpleTrajectory<_T>::render(){

    for(const auto &val : m_data){
        std::cout << val << std::endl;
    }
}

template <typename _T>
void SimpleTrajectory<_T>::add(_T state){
    m_data.push_back(state);
}

template <typename _T>
std::vector<_T> SimpleTrajectory<_T>::getData(){
    return m_data;
}
template <typename _T>
void SimpleTrajectory<_T>::setData(std::vector<_T> data){
    m_data = data;
    }

template <typename _T>
bool operator == (const SimpleTrajectory<_T>& right,const SimpleTrajectory<_T>& left ){
    return left.m_data == right.m_data;
}

template <typename _T>
bool operator != (const SimpleTrajectory<_T>& right,const SimpleTrajectory<_T>& left ){
    return left.m_data != right.m_data;
}

template <typename _T>
SimpleTrajectory<_T> operator+(SimpleTrajectory<_T> lhs, const SimpleTrajectory<_T>& rhs){
    lhs.m_data = lhs.m_data + rhs.m_data;
    return lhs;
}
