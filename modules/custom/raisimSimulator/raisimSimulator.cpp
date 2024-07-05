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

RaisimSimulator::RaisimSimulator(const float rsStep) {
    initialize(rsStep);
}

RaisimSimulator::~RaisimSimulator() {
    // Cleanup Raisim resources
}

void RaisimSimulator::initialize(const float rsStep) {
    world.setTimeStep(rsStep/1000);
    auto ground = world.addGround(-2);
}


// Other methods...

