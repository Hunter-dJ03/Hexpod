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

using namespace std;
using namespace raisim;

RaisimSimulator::RaisimSimulator(const float rsStep) {
    initialize(rsStep);
}

RaisimSimulator::~RaisimSimulator() {
    
}

void RaisimSimulator::initialize(const float rsStep) {
    world.setTimeStep(rsStep/1000);
    auto ground = world.addGround(-2);

    // Build Server
    RaisimServer server(&world);
    server.launchServer(8080);

    cout<<"Awaiting Connection to raisim server"<<endl;
    while (!server.isConnected());
    cout<<"Server Connected"<<endl;
}

