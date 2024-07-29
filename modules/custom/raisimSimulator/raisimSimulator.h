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
    RaisimSimulator(const float rsStep, Path binaryPath, const string URDFName);
    ~RaisimSimulator();
    void setSimAngle(Eigen::VectorXd angs);
    void setSimAngle(float th1, float th2, float th3);
    void setSimVelocity(Eigen::Vector3d nextAngles, Eigen::Vector3d desiredAngularVelocities);

    unique_ptr<RaisimServer> server;
private:
    void initialize(const float rsStep);
    void addModel();

    const float rsStep;
    Eigen::Vector3d Kp = {1.02, 1.08, 1.03};
    Eigen::Vector3d Ki = {3.5, 6.12, 9.69};


    string URDFName;
    Path binaryPath;
    World world;
    shared_ptr<ArticulatedSystem> hexapodLegModel;

};

#endif
