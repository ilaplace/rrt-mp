#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>  
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/utility.hpp>
#include <utility>
#include <vector>
#include <map>
#include <fstream>
#pragma once

class CollusionPoints
{
private:
    friend class boost::serialization::access;  
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version){
            ar &m_points;
    }
    std::vector<std::pair<double,double>> m_points;
  //  unsigned int m_id;
    unsigned int m_number;
public:
    CollusionPoints() = default;
    void addPoint(double, double);
    ~CollusionPoints() = default;
    std::vector<std::pair<double,double>> get();
};


void CollusionPoints::addPoint(double x, double y){
    m_points.push_back(std::make_pair(x,y));
}

std::vector<std::pair<double,double>> CollusionPoints::get(){
    return m_points;
}

class PrimitiveCollusionPointsCollection{
    private:
        friend class boost::serialization::access;  
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version){
            ar & m_collusionCollection;     
        }
        std::map<unsigned int, CollusionPoints> m_collusionCollection;
    public:
        PrimitiveCollusionPointsCollection() = default;
        ~PrimitiveCollusionPointsCollection() = default;
        void add(unsigned int id, CollusionPoints point);
        void loadCollusionPoints();

        // Load the collusion points shifted
        CollusionPoints getCollusionPoints(unsigned int, int, int) const;

};

void PrimitiveCollusionPointsCollection::add(unsigned int id, CollusionPoints point){
    m_collusionCollection.insert(std::make_pair(id, point));
}

void PrimitiveCollusionPointsCollection::loadCollusionPoints(){
    
    std::ifstream ifs("resources/collusionPointsdb");
    boost::archive::text_iarchive ia(ifs);
    ia >> *this;

    std::cout << "Collusion Points are Loaded" << std::endl;
}

CollusionPoints PrimitiveCollusionPointsCollection::getCollusionPoints(unsigned int id, int x, int y) const{
    
    auto points = m_collusionCollection.find(id);
    // If found a primitive with the given id
    CollusionPoints cp;
    if(points != m_collusionCollection.end()){
        auto originalPoints = points->second;
        
        // Shift the points to the given location
        for(auto state : originalPoints.get()){
            cp.addPoint(state.first + x, state.second + y); 
        }
        // Not really the originals though!
        return cp;

        
    }else
    {
        CollusionPoints cp;
        return cp;
    }
    
}

