/* This test is to see how cost changes with respect to number of iterations.
No collusion checking is set, and a map of 10x10 is used. 
*/

#include "doctest.h"
#include "PrimitiveEnvironment.cpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <chrono>


TEST_CASE("Cost vs Iteration"){

    std::vector<int> data {5,10, 15, 30, 50, 70, 100, 1000, 2000};
    std::vector<double> costs;
    std::vector<double> timings;
    std::ofstream file;

    boost::filesystem::path path("./resources/map_cropped-10.ppm");
    PrimitiveEnvironment env(path.string().c_str());
    env.setNonDistantMetric();
   
    env.plannerSetup(8,8,0,0, 0,0,0,0);
    for(auto& i : data) {
        auto maxIteration = i*10; 
        
        ompl::base::IterationTerminationCondition cnd(maxIteration);
        auto condition = ompl::base::PlannerTerminationCondition(cnd);
        env.setPlannerTerminationCondition(&condition);

        // State Validity checker is NOT set, so assuming every state is valid
       
        auto start = std::chrono::high_resolution_clock::now();
        
        env.plan();
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        
        costs.push_back(env.getCost());
        timings.push_back(elapsed.count());
    }
    file.open("resources/costs.txt");
   for (size_t i = 0; i < data.size(); i++){
      file << data[i] << " " << costs[i] << " " <<timings[i] << std::endl; 
   }
   file.close();
}
   