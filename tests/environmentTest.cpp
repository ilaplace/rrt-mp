#include "doctest.h"
#include "PrimitiveEnvironment.cpp"
#include <boost/filesystem.hpp>

TEST_CASE("Is the environment working"){
    // create a map from map 100x100
    boost::filesystem::path path("./resources/map_cropped-100.ppm");
    PrimitiveEnvironment env(path.string().c_str());

    ompl::base::IterationTerminationCondition cnd(1000000);
    auto condition = ompl::base::PlannerTerminationCondition(cnd);
   // env.setNonDistantMetric();
    env.setImageBasedStateValidityChecker();
    env.setPlannerTerminationCondition(&condition);
    env.plannerSetup(60,60,0,0);
    env.plan();
    env.recordSolution();
    //saves result as 1000x1000, can't modify the image used here now
    env.save("resources/result.ppm");
    env.saveAsText("resources/result.csv");
}