#ifndef RAISIMSIMULATOR_H
#define RAISIMSIMULATOR_H

#include <eigen3/Eigen/Core>
#include <raisim/World.hpp>
#include "raisim/RaisimServer.hpp"
#include <memory> 

using namespace raisim;
using namespace std;

class RaisimSimulator
{
public:
    RaisimSimulator(const float rsStep, Path binaryPath);
    ~RaisimSimulator();
    void updateSimulation();
private:
    void initialize(const float rsStep);
    void addModel();

    const float rsStep;
    int numDOF;
    Path binaryPath;
    World world;
    unique_ptr<RaisimServer> server;
    shared_ptr<ArticulatedSystem> hexapodLegModel;

};

#endif
