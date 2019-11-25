
#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include "StateDis.cpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include "PrimitiveBasedMotionValidater.cpp"
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "SimpleTrajectory.h"
#include <boost/filesystem.hpp>
#include <ompl/base/StateStorage.h>
namespace ob = ompl::base;

 
class PrimitiveEnvironment{
    private:
        int m_scalingCoefficient = 10;
        int m_maxWidth;
        int m_maxHeight;
        ompl::PPM m_ppm;
        ompl::PPM m_ppmToPlot;
        ompl::PPM m_treeOnMap;
        ompl::base::PlannerPtr planner;
        ompl::base::ProblemDefinitionPtr m_pdef;
        ompl::base::SpaceInformationPtr m_si;
        SimpleTrajectory<Primitive> m_result;
        ompl::base::OptimizationObjectivePtr m_objective;
        ompl::base::PlannerTerminationCondition *m_condition;
         

        
    public:
    double getCost();    
    void setScalingCoefficient(int);
    void saveTheTreeOnMap(const ompl::base::PlannerData&);
    PrimitiveEnvironment(const char *ppm_file);
    ~PrimitiveEnvironment();
    void setImageBasedStateValidityChecker();
    void clearSolution();
    void setPlannerTerminationCondition(ompl::base::PlannerTerminationCondition *ptc);
    void setNonDistantMetric();
    bool isStateValid(const ob::State *state) const;
    void plannerSetup(unsigned int x_f,
                unsigned int y_f,
                double w_f,
                unsigned int v_f,
                unsigned int x_0 = 10, 
                unsigned int y_0 = 10,
                double w_0 = 0,
                unsigned int v_0 = 0
                );
    bool plan();

    void recordSolution();
    void save(const char *filename);
    void saveAsText(const char *filename);
};


bool PrimitiveEnvironment::plan(){

    auto solved = planner->solve(*m_condition);

    ompl::base::PlannerData plannerData(m_si);
    planner->getPlannerData(plannerData);
    if(bool(solved)){
        saveTheTreeOnMap(plannerData);
    }
    return bool(solved);
}


void PrimitiveEnvironment::setNonDistantMetric(){
    auto space = m_si->getStateSpace();
    space->as<ompl::base::StateDis>()->setNonMetricDistance();
    m_si = std::make_shared<ob::SpaceInformation>(space);
    
}
 void PrimitiveEnvironment::clearSolution(){

      if (!m_pdef){
            OMPL_WARN("Missing problem definition");
            return;
            }
     
     m_pdef->clearSolutionPaths();
 }
void PrimitiveEnvironment::setImageBasedStateValidityChecker(){
     m_si->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
}
void PrimitiveEnvironment::setScalingCoefficient(int coefficient){
    m_scalingCoefficient = coefficient;
}
 void PrimitiveEnvironment::setPlannerTerminationCondition(ompl::base::PlannerTerminationCondition *ptc){
     m_condition = ptc;
 }
 void PrimitiveEnvironment::saveTheTreeOnMap(const ompl::base::PlannerData &plannerData){

    // auto stateStorage = plannerSetpnerData.extractStateStorage();
    // auto states = stateStorage->getStates();
    auto space = m_si->getStateSpace();
    auto sp = space->as<ompl::base::StateDis>();
    auto adapter = sp->getAdapter();
   // auto interpolPath = adapter.decryptTrajectory(traj);

    // First add the graph to a primtive trajectory
    SimpleTrajectory<Primitive> primitiveTrajectory;

    for (size_t i = 1; i < plannerData.numVertices(); i++){
        std::vector< unsigned int > connectedEdges;
        plannerData.getEdges(i, connectedEdges);

        for(auto j : connectedEdges){

            auto init_ostate = plannerData.getVertex(i).getState()->as<ompl::base::StateDis::StateType>();
            auto fin_ostate = plannerData.getVertex(j).getState()->as<ompl::base::StateDis::StateType>();

            Primitive prim(
                StateSpace(init_ostate->getX(),init_ostate->getY(),init_ostate->getYaw(),init_ostate->getSpeed()),
                StateSpace(fin_ostate->getX(),fin_ostate->getY(),fin_ostate->getYaw(),fin_ostate->getSpeed())
                );
            
            Primitive primim = adapter.findPrimitive(prim);

            // shift if a primitive found
            if(primim.m_flag != 0){
                
                auto newprim = adapter.shiftPrimitive(primim, prim[0]);
                primitiveTrajectory.add(newprim);

            }
            // if not found return the candidate
            else{
                primitiveTrajectory.add(prim);
            }
        }   
    }
    
    // log the tree  
    for(auto primitive : primitiveTrajectory.getData()){
        for(auto state : primitive.getStateTrajectory().getData()){
    
    auto x_onMap =  std::min( (unsigned int)(state.getX()*m_scalingCoefficient) ,m_treeOnMap.getWidth()-1);
    auto y_onMap = std::min( (unsigned int)(state.getY()*m_scalingCoefficient),  m_treeOnMap.getHeight()-1);
        ompl::PPM::Color &c = m_treeOnMap.getPixel(x_onMap, y_onMap);
        c.red = 0;
        c.green = 0;
        c.blue = 255;
    }
    }  
    m_treeOnMap.saveFile("tree.ppm");
 }

PrimitiveEnvironment::PrimitiveEnvironment(const char *ppm_file){
    bool ok = false;
    try{
        m_ppm.loadFile(ppm_file);
        
        // Map to plot
        boost::filesystem::path path("./resources/map_cropped-1000.ppm");
        m_ppmToPlot.loadFile(path.string().c_str());

        // Map to print  the tree
        m_treeOnMap.loadFile(path.string().c_str());
        ok = true;
    }
    catch(ompl::Exception &ex){
        OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
    }
    if (ok){
        PrimitivesCollection collect;
        collect.loadDatabase();
        
        PrimitiveAdapter adapter(collect);
        //PrimitiveAdapter* adapter = new PrimitiveAdapter(collect);
        adapter.loadCollusionPoints();

        auto space = std::make_shared<ob::StateDis>(m_ppm.getWidth(),m_ppm.getHeight());

        space->setAdapter(adapter);
        
        m_si = std::make_shared<ob::SpaceInformation>(space);
        m_objective = std::make_shared<ob::PrimitiveCostObjective>(m_si, adapter);
    
       // m_si->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
        m_si->setMotionValidator(std::make_shared<PrimitiveBasedMotionValidater>(m_si, adapter));
    }
}

PrimitiveEnvironment::~PrimitiveEnvironment(){}

bool PrimitiveEnvironment::isStateValid(const ob::State *state) const{

        auto st = state->as<ob::StateDis::StateType>();
        
        // Make sure the point is inside the map
        auto x_onMap =  std::min((unsigned int)st->getX() ,m_ppm.getWidth()-1);
        auto y_onMap = std::min((unsigned int)st->getY(), m_ppm.getHeight()-1);


        const ompl::PPM::Color &c = m_ppm.getPixel(x_onMap, y_onMap);

        return c.red > 127 && c.green > 127 && c.blue > 127;
     }
    
void PrimitiveEnvironment::plannerSetup(unsigned int x_f,
                                unsigned int y_f,
                                double w_f,
                                unsigned int v_f,
                                unsigned int x_0, 
                                unsigned int y_0,
                                double w_0,
                                unsigned int v_0
                                ){
    auto startPtr = m_si->allocState();
    auto start =  startPtr->as<ob::StateDis::StateType>();
    //start->setXY(10,10);
    start->setXY(x_0, y_0);
    start->setYaw(w_0);
    start->setSpeed(v_0);

    auto goalPtr = m_si->allocState();
    auto goal = goalPtr->as<ob::StateDis::StateType>();
    // goal->setXY(20,20);
    goal->setXY(x_f,y_f);
    goal->setYaw(w_f);
    goal->setSpeed(v_f);

    m_pdef = std::make_shared<ob::ProblemDefinition>(m_si);

    //m_pdef->setOptimizationObjective(m_objective);
    
    auto goalChecker(std::make_shared<MyGoal>(m_si));
    goalChecker->setState(goal);
    m_pdef->setGoal(goalChecker);
    m_pdef->addStartState(start);
    
    auto rrt(std::make_shared<ompl::geometric::RRTstar>(m_si));
   // auto plannerSetpner(rrt);
    planner = rrt;

    planner->setProblemDefinition(m_pdef);
    planner->setup();
    m_si->printSettings(std::cout);
    
//     auto solved = plannerSetpner->solve(*m_condition);
    
//     ompl::base::PlannerSetpnerData plannerSetpnerData(m_si);
//     plannerSetpner->getPlannerSetpnerData(plannerSetpnerData);
//     if(bool(solved)){
//         saveTheTreeOnMap(plannerSetpnerData);
//     }
//     //save the whole tree
//    // out.open("resources/tree.txt");
//     //std::cout << "Number of iterations made " << plannerSetpner->numIterations() <<std::endl;
    
//     return bool(solved);
}
void PrimitiveEnvironment::recordSolution()
     {
        if (!m_pdef || !m_pdef->hasSolution()){
            OMPL_WARN("Missing problem definition or no solution found");
            return;
            }
        auto path = m_pdef->getSolutionPath();
        
        auto p = path->as<ompl::geometric::PathGeometric>();
        auto space = m_si->getStateSpace();
        auto sp = space->as<ompl::base::StateDis>();
        auto adapter = sp->getAdapter();

        SimpleTrajectory<StateSpace> traj;
        auto result = p->getStates();
        for(auto state : result){
            auto std = state->as<ompl::base::StateDis::StateType>();
            StateSpace st(std->getX(),std->getY(),std->getYaw(),std->getSpeed());
            traj.add(st);

        auto x_onMap =  std::min(std->getX()*m_scalingCoefficient ,(int)m_ppmToPlot.getWidth()-1);
        auto y_onMap = std::min(std->getY()*m_scalingCoefficient, (int)m_ppmToPlot.getHeight()-1);
    
        
        ompl::PPM::Color &c = m_ppmToPlot.getPixel(x_onMap, y_onMap);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
        std::cout << traj;
        auto interpolPath = adapter.decryptTrajectory(traj);
        m_result = interpolPath;
        
         for(auto primitive : m_result.getData()){
            for(auto state : primitive.getStateTrajectory().getData()){
                
                auto x_onMap =  std::min(int(state.getX()*m_scalingCoefficient) ,(int)m_ppmToPlot.getWidth()-1);
                auto y_onMap = std::min(int(state.getY()*m_scalingCoefficient), (int)m_ppmToPlot.getHeight()-1);
                // The result 
                ompl::PPM::Color &c = m_ppmToPlot.getPixel(x_onMap, y_onMap);
                c.red = 255;
                c.green = 0;
                c.blue = 0;
                // The result on the whole tree
                ompl::PPM::Color &k = m_treeOnMap.getPixel(x_onMap, y_onMap);
                k.red = 255;
                k.green = 0;
                k.blue = 0;
             }
         }
     }
void PrimitiveEnvironment::save(const char *filename)
     {
         if (!m_si)
             return;
         m_ppmToPlot.saveFile(filename);
        m_treeOnMap.saveFile("tree.ppm");
     }

void PrimitiveEnvironment::saveAsText(const char *filename){
    
        ob::PathPtr path = m_pdef->getSolutionPath();
        
        if(path != nullptr){
            auto pathGeo = path->as<ompl::geometric::PathGeometric>();
            auto result = pathGeo->getStates(); 
            std::ofstream myfile;

            //char directory[] = "resources/";
            //strcat(directory,filename);
            myfile.open(filename);

            if (myfile.is_open()){
                for(auto state : m_result.getData())
                    myfile << state;  
                myfile.close();
            }else{
                std::cout << "Unable to open file"<< std::endl;
            }
        }else
            OMPL_WARN("Can't save path as no solution found");

}

double PrimitiveEnvironment::getCost(){
  if (!m_pdef || !m_pdef->hasSolution()){
            OMPL_WARN("Missing problem definition or no solution found");
            return 1;
            }

    auto solution_vector = m_pdef->getSolutions();
    std::vector <double> costs;
    for(auto solution : solution_vector)
        costs.push_back(solution.cost_.value());

    return costs.front();
}