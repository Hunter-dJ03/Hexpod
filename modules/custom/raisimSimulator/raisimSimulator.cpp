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

RaisimSimulator::RaisimSimulator(const float rsStep) : rsStep(rsStep) 
{
    initialize(rsStep);
}

RaisimSimulator::~RaisimSimulator() {
    server->killServer();
}

void RaisimSimulator::initialize(const float rsStep) {
    world.setTimeStep(rsStep/1000);
    auto ground = world.addGround(-2);

    // Build Server
    // server.world_
    server = make_unique<raisim::RaisimServer>(&world);
    server->launchServer(8080);

    cout<<"Awaiting Connection to raisim server"<<endl;
    while (!server->isConnected());
    cout<<"Server Connected"<<endl;
}

