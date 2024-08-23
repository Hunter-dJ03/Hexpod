#include "hexapod.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "../modules/custom/utilities/utils.h"
#include "../modules/custom/raisimSimulator/raisimSimulator.h"

#include "matplotlibcpp.h"

#include <iostream>
#include <fmt/core.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <bitset>

using namespace std;
namespace plt = matplotlibcpp;
using namespace this_thread; // sleep_for, sleep_until
using chrono::system_clock;

HexapodControl::HexapodControl(unsigned int id, std::unique_ptr<ArduinoController> arduino, RSTimedLoop &rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath)
    : id(id), arduino(move(arduino)), rsLoop(rsLoop), arduinoConnected(arduinoConnected), rsStep(rsStep)
{
    if (raisimSimulator)
    {
        simulator = make_unique<RaisimSimulator>(rsStep, binaryPath, "HexapodControlV2b.urdf");
    }

    currentAngles = Eigen::VectorXd(18);
    pos = Eigen::VectorXd(18);

    // moveToCurled();
}

HexapodControl::~HexapodControl()
{
    if (arduino) {
        arduino.reset();
    }
    if (simulator) {
        simulator.reset(); 
    }
}

Eigen::MatrixXd HexapodControl::getJacobian(int legNum) const
{
    Eigen::MatrixXd Jac(3, 3);

    Jac(0, 0) = -sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*(coxaX + tibiaX*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*cos(currentAngles[legNum*3+1]));
    Jac(0, 1) = -cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*(tibiaX*sin(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*sin(currentAngles[legNum*3+1]));
    Jac(0, 2) = -tibiaX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*sin(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]);
    Jac(1, 0) = cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*(coxaX + tibiaX*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*cos(currentAngles[legNum*3+1]));
    Jac(1, 1) = -sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*(tibiaX*sin(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*sin(currentAngles[legNum*3+1]));
    Jac(1, 2) = -tibiaX*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*sin(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]);
    Jac(2, 0) = 0;
    Jac(2, 1) = tibiaX*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*cos(currentAngles[legNum*3+1]);
    Jac(2, 2) = tibiaX*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]);

    // cout << "Jac" << endl << Jac << endl;

    return Jac;
}

// Perform forward kinematics of the HexapodControl leg to get end affector positions
void HexapodControl::updatePos()
{
    for (int legNum = 0; legNum < 6; legNum++) {
    
        pos(legNum*3) = coxaX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0]) + bodyLegOffsets[legNum]*cos(bodyLegAngles[legNum]) + tibiaX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*cos(currentAngles[legNum*3+1]);
        pos(legNum*3+1) = coxaX*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0]) + bodyLegOffsets[legNum]*sin(bodyLegAngles[legNum]) + tibiaX*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2])*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0]) + femurX*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*cos(currentAngles[legNum*3+1]);
        pos(legNum*3+2) = coxaZ + tibiaX*sin(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*sin(currentAngles[legNum*3+1]);
    }
}
// Print End Affector Positions
void HexapodControl::printPos() const
{
    for (int legNum = 0; legNum < 6; legNum++) {
        cout << "Leg " << legNum+1 << " Position:  (" << pos(legNum*3) << ", " << pos(legNum*3+1) << ", " << pos(legNum*3+2) << ")" << endl;
    }
}

// Move to straight leg position
void HexapodControl::moveToZero()
{
    Eigen::VectorXd zeroAnglesVector(18);       
    float zeroAngles[3] = {0, 0 * M_PI / 180, 360 * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        zeroAnglesVector[i] = zeroAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(zeroAnglesVector);
}
// Move to basic standing position
void HexapodControl::moveToBasic()
{
    Eigen::VectorXd basicAnglesVector(18);
    float basicAngles[3] = {0 * M_PI / 180, 40 * M_PI / 180, (360 - 102) * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        basicAnglesVector[i] = basicAngles[i % 3]; // Repeat the set of 3 angles
    }

    setAngs(basicAnglesVector);
}
// Move to position that should fold back past limit when power disabled
void HexapodControl::moveToOff()
{
    Eigen::VectorXd offAnglesVector(18);
    float offAngles[3] = {0 * M_PI / 180, 90 * M_PI / 180, (360 - 163) * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(offAnglesVector);
}
// Move to curled position
void HexapodControl::moveToCurled()
{
    Eigen::VectorXd offAnglesVector(18);
    float offAngles[3] = {0 * M_PI / 180, 135 * M_PI / 180, (360 - 158) * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(offAnglesVector);
}

// Move to initial position for walk cycle
void HexapodControl::stand()
{
    Eigen::VectorXd nextAngles(18);

    Eigen::VectorXd legstandPos(18);

    double tempPos[18] = {
        0.235586, 0.362771, 0,
        0.382555, 0, 0,
        0.235586, -0.362771, 0,
        -0.235586, -0.362771, 0,
        -0.382555, 0, 0,
        -0.235586, 0.362771, 0
    };

    legstandPos = Eigen::Map<Eigen::VectorXd>(tempPos, 18);

    moveLegsToPos(legstandPos);

    // Update tempPos with new values
    double tempPos2[18] = {
        0.235586, 0.362771, -0.113248,
        0.382555, 0, -0.113248,
        0.235586, -0.362771, -0.113248,
        -0.235586, -0.362771, -0.113248,
        -0.382555, 0, -0.113248,
        -0.235586, 0.362771, -0.113248
    };

    legstandPos = Eigen::Map<Eigen::VectorXd>(tempPos2, 18);

    moveLegsToPos(legstandPos);
}

void HexapodControl::moveLegsToPos(const Eigen::VectorXd& desiredPos) {

    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;


    Eigen::VectorXd posOffset(18);
    posOffset = desiredPos - pos;

    int dur = 1000;

    // cout<<endl<< "Leg " << legNum <<endl;
    // cout<< "currentPos: " << currentPos.transpose() <<endl;
    // cout<< "desiredPos: " << desiredPos.transpose() <<endl;
    // cout<< "posOffset: " << posOffset.transpose() <<endl;
    // cout<< "distance: " << dis <<endl;
    // cout<< "desiredSpatialVelocity: " << desiredSpatialVelocity.transpose() <<endl;
    // cout<< "legJointVelocity: " << legJointVelocity.transpose() <<endl;


    // Update time for real time loop
    rsLoop.updateTimeDelay();

    for (int i = 0; i <= dur / rsStep; i++) {

        desiredAngularVelocities*=0;

        for (int legNum = 0; legNum < 6; legNum++) {

            desiredSpatialVelocity << posOffset(legNum*3), posOffset(legNum*3+1), posOffset(legNum*3+2);

            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * (desiredSpatialVelocity / (dur/1000));

            for (int joint = 0; joint < 3; joint++) {
                desiredAngularVelocities[legNum*3+joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        setAngs(nextAngles);
        rsLoop.realTimeDelay();
    }
    
}


void HexapodControl::moveLegToPos(const Eigen::Vector3d& desiredPos, const int legNum) {

    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    // Eigen::Vector3d nextPos;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;


    Eigen::Vector3d currentPos = {pos((legNum-1)*3),pos((legNum-1)*3+1),pos((legNum-1)*3+2)};
    Eigen::Vector3d posOffset = desiredPos - currentPos;

    int dis = posOffset.norm()*2000;

    int dur = 1000;

    // desiredSpatialVelocity = posOffset/(dur);
    desiredSpatialVelocity = posOffset/(dur/1000);
    
    // Test Control Variables
    // float radius = 0.07; // meters
    // double period = 2;   // HZ
    // double cycles = 1;

    // Find Duration
    // int dur = period * cycles * 1000; // ms

    // cout<<endl<< "Leg " << legNum <<endl;
    // cout<< "currentPos: " << currentPos.transpose() <<endl;
    // cout<< "desiredPos: " << desiredPos.transpose() <<endl;
    // cout<< "posOffset: " << posOffset.transpose() <<endl;
    // cout<< "distance: " << dis <<endl;
    // cout<< "desiredSpatialVelocity: " << desiredSpatialVelocity.transpose() <<endl;
    // cout<< "legJointVelocity: " << legJointVelocity.transpose() <<endl;


    // Update time for real time loop
    rsLoop.updateTimeDelay();

    for (int i = 0; i <= dur / rsStep; i++) {

        desiredAngularVelocities*=0;

        jacobian = getJacobian(legNum-1);
        jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

        for (int joint = 0; joint < 3; joint++) {
            desiredAngularVelocities[(legNum-1)*3+joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
        }

        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        setAngs(nextAngles);
        rsLoop.realTimeDelay();
    }
    
}

// Set Angles Overload for 1x3 eigen vector
void HexapodControl::setAngs(const Eigen::VectorXd &angs)
{
    currentAngles = angs;
    sendAngs();

    updatePos();
}
// Send the angles of the servo motors to the arduino
void HexapodControl::sendAngs()
{

    Eigen::VectorXd modifiedAngs = currentAngles;

    if (arduinoConnected)
    {
        std::vector<std::bitset<11>> angleBinaryRepresentation(modifiedAngs.size());
        for (int i =0; i < modifiedAngs.size(); i++) {
            double baseValue = modifiedAngs[i]* 180 / M_PI;
            if ((i+1)%3==0) {
                baseValue = -baseValue+360;
            }

            baseValue+=angleInits[i%3];

            if (i == 10 || i == 11 || i == 13 || i == 14 || i == 16 || i == 17) {
                baseValue = 180 - baseValue;
            };

            if (!i%3) {
                if (baseValue > angleInits[i%3] + 60) {
                    cout << "Coxa out of range, "<< baseValue << ", capping arduino at " << angleInits[i%3] + 60 << endl;
                    baseValue = angleInits[i%3] + 60;
                } else if (baseValue < angleInits[i%3] - 60) {
                    cout << "Coxa out of range, "<< baseValue << ", capping arduino at " << angleInits[i%3] - 60 << endl;
                    baseValue = angleInits[i%3] - 60;
                }
            } else if (i%3) {
                if (baseValue > 180) {
                    cout << "Femur out of range, "<< baseValue << ", capping arduino at " << 180 << endl;
                    baseValue = 180;
                } else if (baseValue < 0) {
                    cout << "Femur out of range, "<< baseValue << ", capping arduino at " << 0 << endl;
                    baseValue = 0;
                }
            } else {
                if (baseValue > 180) {
                    cout << "Tibia out of range, "<< baseValue << ", capping arduino at " << 180 << endl;
                    baseValue = 180;
                } else if (baseValue < 0) {
                    cout << "Tibia out of range, "<< baseValue << ", capping arduino at " << 0 << endl;
                    baseValue = 0;
                }
            }

            cout << baseValue << endl;

            bitset<11> bit(static_cast<uint16_t>(Utils::toFixedPoint(baseValue, 1)));
            angleBinaryRepresentation[i] = bit;
        }

        // for (const auto& byte : angleBinaryRepresentation) {
        //     std::cout << std::bitset<11>(byte) << " ";
        // }
        // cout<<endl;

        size_t totalBits = angleBinaryRepresentation.size() * 11;
        size_t totalBytes = (totalBits + 7) / 8; // Round up to the nearest byte

        // Create a vector to hold the resulting bytes
        std::vector<uint8_t> result(totalBytes, 0);

        // Iterate through the bitsets and pack them into the result vector
        size_t bitIndex = 0;
        for (const auto& bitset : angleBinaryRepresentation) {
            for (size_t i = 0; i < 11; ++i, ++bitIndex) {
                if (bitset[i]) {
                    result[bitIndex / 8] |= (1 << (bitIndex % 8));
                }
            }
        }

        // for (const auto& byte : result) {
        //     std::cout << std::bitset<8>(byte) << " ";
        // }
        // cout<<endl;

        arduino->sendBitSetCommand(result);
    }

    if (simulator) {

        // Subtract the values at the specified indices from 180
        modifiedAngs(10) = - modifiedAngs(10);
        modifiedAngs(11) = - modifiedAngs(11);
        modifiedAngs(13) = - modifiedAngs(13);
        modifiedAngs(14) = - modifiedAngs(14);
        modifiedAngs(16) = - modifiedAngs(16);
        modifiedAngs(17) = - modifiedAngs(17);

        simulator->setSimAngle(modifiedAngs);
    }
}
