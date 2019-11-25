#include "Primitive.cpp"
#include <boost/serialization/vector.hpp>
#include <boost/unordered_set.hpp>
#include <boost/serialization/boost_unordered_set.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <fstream>

#pragma once 

class PrimitivesCollection
{
private:
    friend class boost::serialization::access;
    //std::vector<Primitive> m_data;
    boost::unordered_set<Primitive> m_primitives;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version){
            //ar & m_data;
            ar & m_primitives;
    }
  
public:
    PrimitivesCollection(/* args */);
    PrimitivesCollection(PrimitivesCollection&& other);
    PrimitivesCollection(const PrimitivesCollection& other);
    ~PrimitivesCollection();
    void addPrimitive(Primitive);
    void loadDatabase();
    Primitive findPrimitive(Primitive) const;
    boost::unordered_set<Primitive> getPrimitives();
    friend std::ostream& operator<<(std::ostream &out, const PrimitivesCollection &kolye);
    friend bool operator==(const PrimitivesCollection& lhs, const PrimitivesCollection &rhs);
    PrimitivesCollection& operator=(const PrimitivesCollection&);
};

PrimitivesCollection::PrimitivesCollection(/* args */) {}

PrimitivesCollection::~PrimitivesCollection() {}

PrimitivesCollection::PrimitivesCollection(const PrimitivesCollection& other){
    m_primitives = other.m_primitives;
}

PrimitivesCollection::PrimitivesCollection(PrimitivesCollection&& other){
    m_primitives = other.m_primitives;
}

void PrimitivesCollection::addPrimitive(Primitive primitive){
    //m_data.push_back(primitive);
    m_primitives.emplace(primitive);
}

void PrimitivesCollection::loadDatabase(){
    std::cout << "Database Loaded" << std::endl;
    std::ifstream ifs("resources/file");
    boost::archive::text_iarchive ia(ifs);
    ia >> *this;
}

// TODO: Return something more genereic in case you cannat find the primitive
Primitive PrimitivesCollection::findPrimitive(Primitive candidatePrimitive) const {
    auto foundPrimitivePointer = m_primitives.find(candidatePrimitive);
    if(foundPrimitivePointer != m_primitives.end()){
        return *foundPrimitivePointer;
    }else{
        candidatePrimitive.m_flag = 0;
        candidatePrimitive.m_cost = 100;
        return candidatePrimitive;
    }

}

boost::unordered_set <Primitive> PrimitivesCollection::getPrimitives(){
    return m_primitives;
}


bool operator==(const PrimitivesCollection& lhs, const PrimitivesCollection &rhs){
    //return lhs.m_data == rhs.m_data;
    return  lhs.m_primitives == rhs.m_primitives;

}
std::ostream& operator<<(std::ostream &out, const PrimitivesCollection &kolye){
    for (auto data : kolye.m_primitives)
        out << data;
    return out;
 }
 
PrimitivesCollection& PrimitivesCollection::operator=(const PrimitivesCollection& other){
    //std::swap(m_primitives, other.m_primitives);
    m_primitives = other.m_primitives;
    return *this;
}