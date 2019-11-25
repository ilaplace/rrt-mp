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
#include "doctest.h"
#include <chrono>
#include <algorithm>

using namespace std;

TEST_CASE("Load the database"){
    auto start = chrono::high_resolution_clock().now();
    PrimitivesCollection collect;
    std::ifstream ifs("resources/file");
    boost::archive::text_iarchive ia(ifs);
    ia >> collect;

    auto stop = chrono::high_resolution_clock().now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start); 

    cout << "Time it took to load was " << duration.count() << " milisecond" << endl;
    
    StateSpace initalState(0,0,0,0);
    StateSpace finalState(-2,-2,0,4);
    Primitive prim(3,initalState, finalState,12, 1, 1);

    auto primitives = collect.getPrimitives();
    auto primitive = primitives.find(prim);

    // for( primitive = primitives.begin(); primitive != primitives.end(); primitive++)
    // {
    //     cout << primitive->m_id << " -- " << primitive->m_initialState << " ---- "  << primitive->m_finalState<< "\n" ;
    // }
    if(primitive != primitives.end()){
        std::cout << "Loaded primitive ID: " << primitive->m_id << std::endl;
       // cout << primitive->m_stateSpaceTrajectory << endl;
    }
    else
        cout << "Not found the primitive" << endl;
    //CHECK(primitive.m_finalState == destination);
    CHECK(primitive->m_id == 3);



   // SUBCASE("Load the primitive with ID 537")
    {

        StateSpace init(0,0,0,0);
        StateSpace fin(2,0,1.5708,4);
        Primitive primm(123,init,fin,123,1,1);

        auto primitive2 = primitives.find(primm);
        if(primitive2 != primitives.end()){
            std::cout << "Loaded primitive ID: " << primitive2->m_id << std::endl;
            //cout << primitive2->m_stateSpaceTrajectory << endl;
            
        }
        else
            cout << "Not found the primitive" << endl;
    }

    //"Load the primitive with ID 1839"
    {
        StateSpace init(0,0,0.7854,0);
        StateSpace fin(-2,-1,3.1416,4);
        Primitive primm(123,init,fin,123,1,1);

        auto primitive2 = primitives.find(primm);
        if(primitive2 != primitives.end()){
            std::cout << "Loaded primitive ID: " << primitive2->m_id << std::endl;
           // cout << primitive2->m_stateSpaceTrajectory << endl;
            
        }
        else
            cout << "Not found the primitive" << endl;
    }

    StateSpace init(0,0,1.5708,2);
    StateSpace fin(-2,0,0.7854,0);
    Primitive primm(123,init,fin,123,1,1);

    auto primitive2 = primitives.find(primm);
    if(primitive2 != primitives.end()){
        std::cout << "Loaded primitive ID: " << primitive2->m_id << std::endl;
        //cout << primitive2->m_stateSpaceTrajectory << endl;
        
    }
    else
        cout << "Not found the primitive" << endl;
    //CHECK(primitive.m_finalState == destination);
    CHECK(primitive2->m_id == 4252);


    // 5206
    StateSpace init5207(0,0,1.5708,4);
    StateSpace fin5207(1,-1,-0.7854,0);
    Primitive prim5207(5207,init5207,fin5207,123,1,1);
    auto primitive5206 = primitives.find(prim5207);

    if(primitive5206 != primitives.end()){
        std::cout << "Loaded primitive Initial State: " << primitive5206->m_initialState << std::endl;
        std::cout << "Loaded primitive Fİnal State: " << primitive5206->m_finalState << std::endl;       
    }
    else
        cout << "Not found the primitive with id 5206" << endl;
    CHECK(primitive5206->m_id == 5206);



//"Load the primitive with ID 5169"
    {
        StateSpace init(0,0,1.5708,4);
        StateSpace fin(1,-2,1.5708,4);
        Primitive primm(123,init,fin,123,1,1);

        auto primitive2 = primitives.find(primm);
        if(primitive2 != primitives.end()){
            std::cout << "Loaded primitive ID: " << primitive2->m_id << std::endl;
           // cout << primitive2->m_stateSpaceTrajectory << endl;
            
        }
        else
            cout << "Not found the primitive" << endl;
        CHECK(primitive2->m_id == 5169);
    }


    //"Load the primitive with ID 5205"
    {
        StateSpace init(0,0,1.5708,4);
        StateSpace fin(1,-1,-1.5708,4);
        Primitive primm(123,init,fin,123,1,1);

        auto primitive2 = primitives.find(primm);
        if(primitive2 != primitives.end()){
            std::cout << "Loaded primitive ID: " << primitive2->m_id << std::endl;
           // cout << primitive2->m_stateSpaceTrajectory << endl;
        }
        else
            cout << "Not found the primitive with ID 5205" << endl;

        CHECK(primitive2->m_id == 5205);
    }

     // Prim 5400
     
     {
        StateSpace init(0,0,1.5708,4);
        StateSpace fin(2,2,-0.7854,4);
        Primitive primm(123,init,fin,123,1,1);

        auto primitive2 = primitives.find(primm);
        if(primitive2 != primitives.end()){
            std::cout << "Loaded primitive ID: " << primitive2->m_id << std::endl;
           // cout << primitive2->m_stateSpaceTrajectory << endl;
            
        }
        else
            cout << "Not found the primitive with ID 5205" << endl;
        
        CHECK(primitive2->m_id == 5400);
    }

}

