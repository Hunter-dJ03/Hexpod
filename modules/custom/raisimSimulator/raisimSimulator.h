#ifndef RAISIMSIMULATOR_H
#define RAISIMSIMULATOR_H

#include <eigen3/Eigen/Core>
#include <raisim/World.hpp>

using namespace raisim;

class RaisimSimulator
{
public:
    RaisimSimulator(const float rsStep);
    ~RaisimSimulator();

    void initialize(const float rsStep);
    // Other simulation-related methods...

private:
    World world;
    // Raisim specific members and methods...
};

#endif
