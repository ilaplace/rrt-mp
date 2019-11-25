#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include "State.cpp"
#include "SimpleTrajectory.h"
#include "Primitive.cpp"
#include "PrimitivesCollection.cpp"
#include "PrimitiveCollusionPoints.cpp"

using namespace std;
/*
1. loop as the number of motion primitives
1. read the number of states of the motion primitive
3. loop with the number of states primitives */
void loadPrimitives(){
    fstream fin;
     
     // TODO: Store it on the heap
     PrimitivesCollection collect;
     //PrimitivesCollection *collect = new PrimitivesCollection;
    
    try{fin.open("./resources/trajNew.csv", ios::in);
    }
    catch(const std::exception& e){
        std::cerr << e.what() << '\n';
    }
    
    string x;
    
    //first line useless
    getline(fin, x);

    unsigned int numberOfMotionPrims = 5208;

    for (size_t i = 0; i < numberOfMotionPrims; i++){   
        string line, word;
        getline(fin, line);
        vector<string> row;
        stringstream s(line);

        //fragment the row into words
        while (getline(s,word,',')){
            row.push_back(word);
        }
        unsigned int numberOfStates = stoi(row[1]);
        //number of states of the motion primitive
        //cout << row[1] << endl; 
        cout << "For int:" << i << " ---"<< line << endl;

        //the inital state of the motion primitive (in the first header line)
        StateSpace initalState(stod(row[3]),stod(row[4]),stod(row[5]),stod(row[6]));
       // cout << "Initial state" << initalState << endl;
        
        //Get the final state from the second header line
        SimpleTrajectory<StateSpace> trajectory;

        getline(fin, line);
        stringstream n(line);
        //cout<< line <<endl;
        vector<string> secondrow;
        while (getline(n,word,',')){
            secondrow.push_back(word);
        }
        
        StateSpace finalstate(stod(secondrow[3]),stod(secondrow[4]),stod(secondrow[5]),stod(secondrow[6]));
        //cout << "final state" << finalstate << endl;
        
        Primitive primitive(stoi(row[0]),initalState, finalstate, stoi(row[1]), stod(secondrow[1]), stod(secondrow[0]));

        for (size_t j = 0; j < numberOfStates; j++)
        {
            getline(fin, line);
            //motion primitive intermediary states
            stringstream x(line);
            //cout<< line <<endl;
            vector<string> interrow;
            while (getline(x,word,',')){
                interrow.push_back(word);
            }
        
            StateSpace interState(stod(interrow[1]),stod(interrow[2]),stod(interrow[3]),stod(interrow[4]));
            trajectory.add(interState);
            
        }
    //cout << "trajectory:" <<endl;
    //trajectory.render();
    primitive.addIntermediaryStateTrajectory(trajectory);    
    collect.addPrimitive(primitive);

    }
    // scope to make sure destructor of the output text archive is called
    {
        ofstream ofs("resources/file");
        boost::archive::text_oarchive archive(ofs);
        archive << collect;
    }
    {
        PrimitivesCollection kolye;
        std::ifstream ifs("resources/file");
        boost::archive::text_iarchive ia(ifs);
        ia >> kolye;
    }
}
//load collusion points for each primitive
void loadCollusionPoints(){
    
    fstream fin;
    PrimitiveCollusionPointsCollection kolye;
     
    try{fin.open("./resources/collusionPoints.csv", ios::in);
    }
    catch(const std::exception& e){
        std::cerr << e.what() << '\n';
    }
    
    string x;
    //first line useless
    getline(fin, x);

    unsigned int numberOfMotionPrims = 5208;

    for (size_t i = 0; i < numberOfMotionPrims; i++){   
        string line, word;
        getline(fin, line);
        vector<string> row;
        stringstream s(line);

        //fragment the row into words
        while (getline(s,word,',')){
            row.push_back(word);
        }

        //the inital state of the motion primitive (in the first header line)
        CollusionPoints collusionPoints;
     
        for (size_t j = 0; j < stoul(row[1]); j++)
        {
            getline(fin, line);
            //motion primitive intermediary states
            stringstream x(line);
            //cout<< line <<endl;
            vector<string> interrow;
            while (getline(x,word,',')){
                interrow.push_back(word);
            }
        
            collusionPoints.addPoint(stod(interrow[0]), stod(interrow[1]));            
        }
        kolye.add(stoul(row[0]), collusionPoints);
    }
    
    {
        ofstream ofs("resources/collusionPointsdb");
        boost::archive::text_oarchive archive(ofs);
        archive << kolye;
    }

    {
        PrimitiveCollusionPointsCollection kolye;
        std::ifstream ifs("resources/collusionPointsdb");
        boost::archive::text_iarchive ia(ifs);
        ia >> kolye;
    }
}

int main(int argc, char const *argv[])
{
    loadPrimitives();
    //loadCollusionPoints();
    return 0;
}
