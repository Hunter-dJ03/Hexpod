#ifndef RAISIMSIMULATOR_H
#define RAISIMSIMULATOR_H

#include <eigen3/Eigen/Core>
#include <raisim/World.hpp>
#include <Eigen/Dense>
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
    void setSimVelocity(Eigen::VectorXd nextAngles, Eigen::VectorXd desiredAngularVelocities);

    Eigen::VectorXd convertVecDynToEigen(const raisim::VecDyn& vecDyn);

    unique_ptr<RaisimServer> server;
    shared_ptr<ArticulatedSystem> hexapodLegModel;
private:
    void initialize(const float rsStep);
    void addModel();

    const float rsStep;
    
    Eigen::VectorXd Kp = (Eigen::VectorXd(18) << 1.02, 1.08, 1.03, 1.02, 1.08, 1.03, 1.02, 1.08, 1.03, 1.02, 1.08, 1.03, 1.02, 1.08, 1.03, 1.02, 1.08, 1.03).finished();
    Eigen::VectorXd Ki = (Eigen::VectorXd(18) << 3.5, 6.12, 9.69, 3.5, 6.12, 9.69, 3.5, 6.12, 9.69, 3.5, 6.12, 9.69, 3.5, 6.12, 9.69, 3.5, 6.12, 9.69).finished();


    string URDFName;
    Path binaryPath;
    World world;

};

#endif
