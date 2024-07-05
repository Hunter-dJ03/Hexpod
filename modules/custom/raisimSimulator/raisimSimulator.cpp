#include "raisimSimulator.h"
#include "raisim/World.hpp"
#include "matplotlibcpp.h"
#include "raisim/RaisimServer.hpp"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <chrono>
#include <memory> 

using namespace std;
using namespace raisim;

// Constructor - Sets key variables and starts initilisation
RaisimSimulator::RaisimSimulator(const float rsStep, Path binaryPath) : rsStep(rsStep), binaryPath(binaryPath)
{
    initialize(rsStep);
}

// Destructor - Kill the Raisim Simulation
RaisimSimulator::~RaisimSimulator() {
    server->killServer();
}

void RaisimSimulator::updateSimulation()
{
    /*
    NEEDED VARIABLES 

    from existing math
        current angles
        next angles
        current angular velocity
        desired angular velocity
    
    preset
        ki
        kp
    
    

    velocityError = desiredAngularVelocities - currentAngularVelocities

    if (i==0) {
        integralError = zeroVelocity -= currentAngles;
    } else {
        integralError = desiredAngles -= currentAngles;
    }
    controlledAngularVelocities = currentAngularVelocities += vecDynElementMultiply(Kp,velocityError) += vecDynElementMultiply(Ki,integralError);

    hexapod->setGeneralizedVelocity(controlledAngularVelocities);

    */
}

// Initilise the Raisim Simulation
void RaisimSimulator::initialize(const float rsStep) {
    // Make World and Size
    world.setTimeStep(rsStep/1000);
    auto ground = world.addGround(-2);

    // Add Hexapod Leg Model to world
    addModel();

    // Build and launch Server
    server = make_unique<RaisimServer>(&world);
    server->launchServer(8080);

    // Wait for server connection
    cout<<"Awaiting Connection to raisim server"<<endl;
    while (!server->isConnected());
    cout<<"Server Connected"<<endl;
}

// Add Hexapod Leg Model to the Simulation
void RaisimSimulator::addModel()
{
    // Add the model to the world
    hexapodLegModel = shared_ptr<ArticulatedSystem>(world.addArticulatedSystem(binaryPath.getDirectory() + "/models/hexapod/urdf/hexapodLeg.urdf"));
    hexapodLegModel->setName("HexapodLegModel");

    // Remove Collision Meshes
    for (int i = 0; i <= 3; ++i) {
        for (int j = i + 1; j <= 3; ++j) {
            hexapodLegModel->ignoreCollisionBetween(i, j);
    }


}

}
