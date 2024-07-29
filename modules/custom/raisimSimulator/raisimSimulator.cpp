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
RaisimSimulator::RaisimSimulator(const float rsStep, Path binaryPath, const string URDFName) : rsStep(rsStep), binaryPath(binaryPath), URDFName(URDFName)
{
    initialize(rsStep);
}

// Destructor - Kill the Raisim Simulation
RaisimSimulator::~RaisimSimulator()
{
    server->killServer();
}

// Send joint angles to simulation model with Vector3d input
void RaisimSimulator::setSimAngle(Eigen::VectorXd angs)
{
    // Send joint angles to simulation model
    hexapodLegModel->setGeneralizedCoordinate(angs);
}

// Send joint angles to simulation model with joint variable inputs
void RaisimSimulator::setSimAngle(float th1, float th2, float th3)
{
    // Convert input variables in combined Vector3d type
    Eigen::Vector3d angs = {th1, th2, th3};

    // Send joint angles to simulation model
    hexapodLegModel->setGeneralizedCoordinate(angs);
}

void RaisimSimulator::setSimVelocity(Eigen::Vector3d nextAngles, Eigen::Vector3d desiredAngularVelocities)
{
    // Get Current Angles
    raisim::VecDyn currentAnglesVecDyn = hexapodLegModel->getGeneralizedCoordinate();
    raisim::VecDyn currentAngularVelocitiesVecDyn = hexapodLegModel->getGeneralizedVelocity();
    
    // Convert raisim::VecDyn type to Eigen::Vector3d
    Eigen::Vector3d currentAngles = {currentAnglesVecDyn[0], currentAnglesVecDyn[1], currentAnglesVecDyn[2]};
    Eigen::Vector3d currentAngularVelocities = {currentAngularVelocitiesVecDyn[0], currentAngularVelocitiesVecDyn[1], currentAngularVelocitiesVecDyn[2]};

    // Find controlled angular velocities through PI controller
    Eigen::Vector3d controlledAngularVelocities = currentAngularVelocities + Kp.cwiseProduct(desiredAngularVelocities - currentAngularVelocities) + Ki.cwiseProduct(nextAngles - currentAngles);

    // Set the simulation model joint velocities
    hexapodLegModel->setGeneralizedVelocity(controlledAngularVelocities);
}

// Initilise the Raisim Simulation
void RaisimSimulator::initialize(const float rsStep)
{
    // Make World and Size
    world.setTimeStep(rsStep / 1000);
    auto ground = world.addGround(-2);

    // Add Hexapod Leg Model to world
    addModel();

    // Build and launch Server
    server = make_unique<RaisimServer>(&world);
    server->launchServer(8080);

    // Wait for server connection
    cout << "Awaiting Connection to raisim server" << endl;
    while (!server->isConnected())
        ;
    cout << "Server Connected" << endl;
}

// Add Hexapod Leg Model to the Simulation
void RaisimSimulator::addModel()
{
    // Add the model to the world
    hexapodLegModel = shared_ptr<ArticulatedSystem>(world.addArticulatedSystem(binaryPath.getDirectory() + "/models/hexapod/urdf/" + URDFName));
    hexapodLegModel->setName("HexapodLegModel");

    // Remove Collision Meshes
    for (int i = 0; i <= 3; ++i)
    {
        for (int j = i + 1; j <= 3; ++j)
        {
            hexapodLegModel->ignoreCollisionBetween(i, j);
        }
    }
}
