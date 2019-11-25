/* This test is to see how cost changes with respect to number of iterations.
No collusion checking is set, and a map of 10x10 is used. 
*/

#include "doctest.h"
#include "PrimitiveEnvironment.cpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <chrono>

std::pair<double, std::chrono::duration<double>> runEnvironment(int maxIterations){

    // create a map from map 100x100
    boost::filesystem::path path("./resources/map_cropped-10.ppm");
    PrimitiveEnvironment env(path.string().c_str());

    ompl::base::IterationTerminationCondition cnd(maxIterations);
    auto condition = ompl::base::PlannerTerminationCondition(cnd);
    env.setPlannerTerminationCondition(&condition);
    
    // State Validity checker is NOT set, so assuming every state is valid
   bool solved = false;
   auto start = std::chrono::high_resolution_clock::now();
   env.plannerSetup(6,6,0,0, 0,0,0,0);
   env.setNonDistantMetric();
   solved = env.plan();
   auto finish = std::chrono::high_resolution_clock::now();

   std::chrono::duration<double> elapsed = finish - start;
   // force finding a solution
    if(!solved){

   //    // forget the prior solution
       env.clearSolution();
       env.plannerSetup(8,8,0,0, 0,0,0,0);
       solved = env.plan();
    }

   return std::make_pair(env.getCost(),elapsed);
} 

TEST_CASE("Cost vs Iteration"){

     std::vector<int> data {5,10, 15, 30, 50, 70, 100, 1000};
     std::vector<int> costs;
     std::vector<double> timings;
     std::ofstream file;
    
    for(auto& i : data) {
        auto maxIteration = i*10; 

        std::cout << "Max number of iterations "<< maxIteration << std::endl; // log the current input data

        std::pair<double, std::chrono::duration<double>> res = runEnvironment(maxIteration);
       
        costs.push_back(res.first);

        timings.push_back(res.second.count());
    }
    file.open("resources/costs.txt");
   for (size_t i = 0; i < data.size(); i++){
      file << data[i] << " " << costs[i] << " " <<timings[i] << std::endl; 
   }
   file.close();
}
   