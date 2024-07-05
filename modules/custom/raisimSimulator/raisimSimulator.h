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
    RaisimSimulator(const float rsStep);
    ~RaisimSimulator();

    void initialize(const float rsStep);
    // Other simulation-related methods...

private:
    const float rsStep;
    World world;
    unique_ptr<raisim::RaisimServer> server;
};

#endif
