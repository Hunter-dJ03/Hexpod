#include "hexapod.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "../modules/custom/utilities/utils.h"
#include "../modules/custom/raisimSimulatorFull/raisimSimulator.h"

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
        simulator = make_unique<RaisimSimulator>(rsStep, binaryPath, "hexapodCustom.urdf");
    }

    currentAngles = Eigen::VectorXd(18);
    pos = Eigen::VectorXd(18);
    desiredAngles = Eigen::VectorXd(18);
    currentAngularVelocities = Eigen::VectorXd(18);

    active = false;

    moveToCurled();
}

HexapodControl::~HexapodControl()
{
    if (arduino)
    {
        arduino.reset();
    }
    if (simulator)
    {
        simulator.reset();
    }
}

Eigen::MatrixXd HexapodControl::getJacobian(int legNum) const
{
    Eigen::MatrixXd Jac(3, 3);

    Jac(0, 0) = -sin(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (coxaX + tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * cos(currentAngles[legNum * 3 + 1]));
    Jac(0, 1) = -cos(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (tibiaX * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * sin(currentAngles[legNum * 3 + 1]));
    Jac(0, 2) = -tibiaX * cos(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]);
    Jac(1, 0) = cos(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (coxaX + tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * cos(currentAngles[legNum * 3 + 1]));
    Jac(1, 1) = -sin(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * (tibiaX * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * sin(currentAngles[legNum * 3 + 1]));
    Jac(1, 2) = -tibiaX * sin(bodyLegAngles[legNum] + currentAngles[legNum * 3 + 0]) * sin(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]);
    Jac(2, 0) = 0;
    Jac(2, 1) = tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]) + femurX * cos(currentAngles[legNum * 3 + 1]);
    Jac(2, 2) = tibiaX * cos(currentAngles[legNum * 3 + 1] + currentAngles[legNum * 3 + 2]);

    return Jac;
}

// Perform forward kinematics of the HexapodControl leg to get end affector positions
void HexapodControl::updatePos()
{
    // for (int legNum = 0; legNum < 6; legNum++) {

    //     pos(legNum*3) = coxaX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0]) + bodyLegOffsets[legNum]*cos(bodyLegAngles[legNum]) + tibiaX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*cos(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*cos(currentAngles[legNum*3+1]);
    //     pos(legNum*3+1) = coxaX*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0]) + bodyLegOffsets[legNum]*sin(bodyLegAngles[legNum]) + tibiaX*cos(currentAngles[legNum*3+1] + currentAngles[legNum*3+2])*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0]) + femurX*sin(bodyLegAngles[legNum] + currentAngles[legNum*3+0])*cos(currentAngles[legNum*3+1]);
    //     pos(legNum*3+2) = coxaZ + tibiaX*sin(currentAngles[legNum*3+1] + currentAngles[legNum*3+2]) + femurX*sin(currentAngles[legNum*3+1]);
    // }

    pos = doFK(currentAngles);
}

Eigen::VectorXd HexapodControl::doFK(Eigen::VectorXd angs)
{
    Eigen::VectorXd position(18);

    for (int legNum = 0; legNum < 6; legNum++)
    {

        position(legNum * 3) = coxaX * cos(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) + bodyLegOffsets[legNum] * cos(bodyLegAngles[legNum]) + tibiaX * cos(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) * cos(angs[legNum * 3 + 1] + angs[legNum * 3 + 2]) + femurX * cos(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) * cos(angs[legNum * 3 + 1]);
        position(legNum * 3 + 1) = coxaX * sin(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) + bodyLegOffsets[legNum] * sin(bodyLegAngles[legNum]) + tibiaX * cos(angs[legNum * 3 + 1] + angs[legNum * 3 + 2]) * sin(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) + femurX * sin(bodyLegAngles[legNum] + angs[legNum * 3 + 0]) * cos(angs[legNum * 3 + 1]);
        position(legNum * 3 + 2) = coxaZ + tibiaX * sin(angs[legNum * 3 + 1] + angs[legNum * 3 + 2]) + femurX * sin(angs[legNum * 3 + 1]);
    }

    return position;
}

// Print End Affector Positions
void HexapodControl::printPos() const
{
    for (int legNum = 0; legNum < 6; legNum++)
    {
        cout << "Leg " << legNum + 1 << " Position:  (" << pos(legNum * 3) << ", " << pos(legNum * 3 + 1) << ", " << pos(legNum * 3 + 2) << ")" << endl;
    }
}

// Move to straight leg position
void HexapodControl::moveToZero()
{
    Eigen::VectorXd zeroAnglesVector(18);
    float zeroAngles[3] = {0, 0 * M_PI / 180, 360 * M_PI / 180};
    for (int i = 0; i < 18; ++i)
    {
        zeroAnglesVector[i] = zeroAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(zeroAnglesVector);
}
// Move to basic standing position
void HexapodControl::moveToBasic()
{
    Eigen::VectorXd basicAnglesVector(18);
    float basicAngles[3] = {0 * M_PI / 180, 40 * M_PI / 180, (360 - 102) * M_PI / 180};
    for (int i = 0; i < 18; ++i)
    {
        basicAnglesVector[i] = basicAngles[i % 3]; // Repeat the set of 3 angles
    }

    setAngs(basicAnglesVector);
}
// Move to position that should fold back past limit when power disabled
void HexapodControl::moveToOff()
{
    Eigen::VectorXd offAnglesVector(18);
    float offAngles[3] = {0 * M_PI / 180, 90 * M_PI / 180, (360 - 163) * M_PI / 180};
    for (int i = 0; i < 18; ++i)
    {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }

    setAngs(offAnglesVector);
    // moveLegsToPos(offAnglesVector);
}

// Move to curled position
void HexapodControl::moveToCurled()
{
    Eigen::VectorXd offAnglesVector(18);
    float offAngles[3] = {0 * M_PI / 180, 135 * M_PI / 180, (360 - 158) * M_PI / 180};
    for (int i = 0; i < 18; ++i)
    {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }

    setAngs(offAnglesVector);
}

// Move to initial position for walk cycle
void HexapodControl::off()
{

    Eigen::VectorXd legOffPos(18);

    double tempPosA[18] = {
        0.235586, 0.362771, 0.0,
        0.382555, 0.0, 0.0,
        0.235586, -0.362771, 0.0,
        -0.235586, -0.362771, 0.0,
        -0.382555, 0.0, 0.0,
        -0.235586, 0.362771, 0.0};

    legOffPos = Eigen::Map<Eigen::VectorXd>(tempPosA, 18);
    moveLegsToPos(legOffPos);

    double tempPosB[18] = {
        0.19, 0.29, 0.0,
        0.29, 0, 0.0,
        0.19, -0.29, 0.0,
        -0.19, -0.29, 0.0,
        -0.29, 0, 0.0,
        -0.19, 0.29, 0.0};

    legOffPos = Eigen::Map<Eigen::VectorXd>(tempPosB, 18);
    moveLegsToPos(legOffPos);
}

// Move to initial position for walk cycle
void HexapodControl::stand()
{

    Eigen::VectorXd legstandPos(18);

    double tempPosA[18] = {
        0.235586, 0.362771, 0,
        0.382555, 0, 0,
        0.235586, -0.362771, 0,
        -0.235586, -0.362771, 0,
        -0.382555, 0, 0,
        -0.235586, 0.362771, 0};

    legstandPos = Eigen::Map<Eigen::VectorXd>(tempPosA, 18);
    moveLegsToPos(legstandPos);

    legstandPos = Eigen::Map<Eigen::VectorXd>(standPos, 18);
    moveLegsToPos(legstandPos);
}

void HexapodControl::moveLegsToPos(const Eigen::VectorXd &desiredPos)
{

    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    Eigen::VectorXd posOffset(18);
    posOffset = desiredPos - pos;
    int dur = 1000;

    // cout<< "currentPos: " << pos.transpose() <<endl;
    // cout<< "desiredPos: " << desiredPos.transpose() <<endl;
    // cout<< "posOffset: " << posOffset.transpose() <<endl;
    // cout<< "desiredSpatialVelocity: " << desiredSpatialVelocity.transpose() <<endl;
    // cout<< "legJointVelocity: " << legJointVelocity.transpose() <<endl;

    // Update time for real time loop
    // rsLoop.updateTimeDelay();
    for (int i = 0; i < dur / rsStep; i++)
    {
        desiredAngularVelocities *= 0;

        for (int legNum = 0; legNum < 6; legNum++)
        {
            desiredSpatialVelocity << posOffset(legNum * 3), posOffset(legNum * 3 + 1), posOffset(legNum * 3 + 2);
            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * (desiredSpatialVelocity / (dur / 1000));
            for (int joint = 0; joint < 3; joint++)
            {
                desiredAngularVelocities[legNum * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);
        simulator->setSimVelocity(nextAngles, desiredAngularVelocities);

        currentAngles = nextAngles;
        updatePos();

        simulator->server->integrateWorldThreadSafe();
        rsLoop.realTimeDelay();
    }

    desiredAngles = currentAngles;
}

void HexapodControl::moveLegToPos(const Eigen::Vector3d &desiredPos, const int legNum)
{

    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    // Eigen::Vector3d nextPos;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    Eigen::Vector3d currentPos = {pos((legNum - 1) * 3), pos((legNum - 1) * 3 + 1), pos((legNum - 1) * 3 + 2)};
    Eigen::Vector3d posOffset = desiredPos - currentPos;

    int dis = posOffset.norm() * 2000;

    int dur = 1000;

    // desiredSpatialVelocity = posOffset/(dur);
    desiredSpatialVelocity = posOffset / (dur / 1000);

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

    for (int i = 0; i <= dur / rsStep; i++)
    {

        desiredAngularVelocities *= 0;

        jacobian = getJacobian(legNum - 1);
        jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
        legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

        for (int joint = 0; joint < 3; joint++)
        {
            desiredAngularVelocities[(legNum - 1) * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
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
        for (int i = 0; i < modifiedAngs.size(); i++)
        {
            double baseValue = modifiedAngs[i] * 180 / M_PI;
            if ((i + 1) % 3 == 0)
            {
                baseValue = -baseValue + 360;
            }

            baseValue += angleInits[i % 3];

            if (i == 10 || i == 11 || i == 13 || i == 14 || i == 16 || i == 17)
            {
                baseValue = 180 - baseValue;
            };

            if (!i % 3)
            {
                if (baseValue > angleInits[i % 3] + 60)
                {
                    cout << "Coxa out of range, " << baseValue << ", capping arduino at " << angleInits[i % 3] + 60 << endl;
                    baseValue = angleInits[i % 3] + 60;
                }
                else if (baseValue < angleInits[i % 3] - 60)
                {
                    cout << "Coxa out of range, " << baseValue << ", capping arduino at " << angleInits[i % 3] - 60 << endl;
                    baseValue = angleInits[i % 3] - 60;
                }
            }
            else if (i % 3)
            {
                if (baseValue > 180)
                {
                    cout << "Femur out of range, " << baseValue << ", capping arduino at " << 180 << endl;
                    baseValue = 180;
                }
                else if (baseValue < 0)
                {
                    cout << "Femur out of range, " << baseValue << ", capping arduino at " << 0 << endl;
                    baseValue = 0;
                }
            }
            else
            {
                if (baseValue > 180)
                {
                    cout << "Tibia out of range, " << baseValue << ", capping arduino at " << 180 << endl;
                    baseValue = 180;
                }
                else if (baseValue < 0)
                {
                    cout << "Tibia out of range, " << baseValue << ", capping arduino at " << 0 << endl;
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
        for (const auto &bitset : angleBinaryRepresentation)
        {
            for (size_t i = 0; i < 11; ++i, ++bitIndex)
            {
                if (bitset[i])
                {
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

    if (simulator)
    {

        // Subtract the values at the specified indices from 180
        // modifiedAngs(10) = - modifiedAngs(10);
        // modifiedAngs(11) = - modifiedAngs(11);
        // modifiedAngs(13) = - modifiedAngs(13);
        // modifiedAngs(14) = - modifiedAngs(14);
        // modifiedAngs(16) = - modifiedAngs(16);
        // modifiedAngs(17) = - modifiedAngs(17);

        simulator->setSimAngle(modifiedAngs);

        desiredAngles = modifiedAngs;
    }
}

void HexapodControl::jacobianTest(const int &style)
{
    // Test Control Variables
    float radius = 0.06; // meters
    double period = 2;   // secs
    double cycles = 5;

    // Find Duration
    int dur = period * cycles * 1000; // ms

    // Create vector variables for control calculations
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    // Eigen::Vector3d nextPos;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    // // Set Start Position
    // float offAngles[3] = {0, 40 * M_PI / 180, 258 * M_PI / 180};
    // for (int i = 0; i < 18; ++i) {
    //     nextAngles[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    // }
    // setAngs(nextAngles);

    operationDuration = dur / rsStep + 2000 / rsStep;

    // Simulation Cycle
    for (int i = 0; i <= dur / rsStep + 2000 / rsStep; i++)
    {
        cout << operationDuration << endl;
        if (operationDuration <= 0)
        {
            // cout << "exit" << endl;
            break;
        }

        if (i < 2000 / rsStep)
        {
            simulator->setSimVelocity(desiredAngles, Eigen::VectorXd::Zero(18));
            rsLoop.realTimeDelay();
            // Integrate the Simulator server
            simulator->server->integrateWorldThreadSafe();
            operationDuration--;
            continue;
        }

        // desiredSpatialVelocity << 0,0,0;
        // Desired spatial velocity of XYZ

        for (int legNum = 0; legNum < 6; legNum++)
        {

            if (style == 0)
            {
                // cout<<endl<<"Jacobian test in X-Y plane" <<endl;
                desiredSpatialVelocity << 0,
                    radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                    -radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000));
            }
            else if (style == 1)
            {
                // cout<<endl<<"Jacobian test in X-Z plane" <<endl;
                desiredSpatialVelocity << radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                    0,
                    -radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000));
            }
            else if (style == 2)
            {
                // cout<<endl<<"Jacobian test in Y-Z plane" <<endl;
                desiredSpatialVelocity << radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)),
                    radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000)),
                    0;
            }

            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

            for (int joint = 0; joint < 3; joint++)
            {
                desiredAngularVelocities[legNum * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        // Send the angles to the arduino and simulation as desired
        // setAngs(nextAngles);

        // Send Velcoties to the simulator
        simulator->setSimVelocity(nextAngles, desiredAngularVelocities);

        currentAngles = nextAngles;
        updatePos();

        // Implement Real time delay
        rsLoop.realTimeDelay();
        // Integrate the Simulator server
        simulator->server->integrateWorldThreadSafe();

        operationDuration--;
    }

    desiredAngles = currentAngles;
}

void HexapodControl::walk(double vel, double ang)
{
    // Normalize the angles to the range -pi to pi
    auto normalizeAngle = [](double angle) -> double {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    };

    // Normalize both angles to the -pi to pi range
    ang = normalizeAngle(ang);
    lastAngle = normalizeAngle(lastAngle);

    // Calculate the difference and normalize to [-pi, pi] range
    double angleDifference = normalizeAngle(ang - lastAngle);

    // Check if the difference exceeds pi/2
    if (abs(angleDifference) > M_PI / 2) {
        for (size_t i = 0; i < standing.size(); ++i) {
            standing[i] = !standing[i];
        }
    }

    // Update lastAngle to the current angle for future comparisons
    lastAngle = ang;

    // Full step duration (support and sweep)
    double T = 2.0; // S

    double D;
    double S;

    if (!temp) {
        D = 0.050;
        temp = 1;
    } else {
        D = 0.10;
    }

    D = 0.08;

    double desiredPos[18];
    Eigen::VectorXd posOffset(18);
    double horizontalDistance[6];  // Array to store horizontal distances
    double angle[6];               // Array to store angles

    for (size_t j = 0; j<6; j++) {
        desiredPos[j*3] = standPos[j*3] - D * cos(ang) * (standing[j] * 2 - 1);
        desiredPos[j*3+1] = standPos[j*3+1] - D * sin(ang) * (standing[j] * 2 - 1);
        // desiredPos[j*3+2] = standPos[j*3+2];
    

        double x = desiredPos[j*3] - pos[j*3];
        double y = desiredPos[j*3+1] - pos[j*3+1];

        // Calculate horizontal distance (sqrt(x^2 + y^2))
        horizontalDistance[j] = std::sqrt(x * x + y * y);

        // Calculate angle (atan2(y, x))
        angle[j] = std::atan2(y, x);

        // std::cout << "Set " << j << ": " << horizontalDistance[j] << "\n";
        // std::cout << "Set " << j << ": " << angle[j] << "\n";
    }

    S = D;

    double w = 0.060;
    double t = 0.0;

    vector<double> a = {-1.0 / 2.0, -2.0 / T, 0, 160.0 / pow(T, 3), -480.0 / pow(T, 4), 384.0 / pow(T, 5), 0};
    vector<double> b = {0, 0, 0, 512.0 / pow(T, 3), -3072.0 / pow(T, 4), 6144.0 / pow(T, 5), -4096.0 / pow(T, 6)};

    // Create vector variables for control calculations
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    // Eigen::Vector3d nextPos;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    operationDuration = T/2 * 1000 / rsStep;

    // Simulation Cycle
    for (int i = 0; i <= T/2 * 1000 / rsStep; i++)
    {
        t+= 0.001 * rsStep;

        // cout << t << ", " << i << endl;
        if (operationDuration < 0)
        {
            // cout << "exit" << endl;
            break;
        }

        // desiredSpatialVelocity << 0,0,0;
        // Desired spatial velocity of XYZ

        // cout << t <<endl;

        for (int legNum = 0; legNum < 6; legNum++)
        {

            double horizontalMovement;
            double verticalMovement;

            if (standing[legNum])
            {
                // Standing Calculations
                horizontalMovement = (2.0 * horizontalDistance[legNum]) / (T);
                verticalMovement = 0;

            } else {
                // Swing calculations
                horizontalMovement = horizontalDistance[legNum] * (6 * a[6] * pow(t, 5) + 5 * a[5] * pow(t, 4) + 4 * a[4] * pow(t, 3) + 3 * a[3] * pow(t, 2) + 2 * a[2] * t + a[1]);
                verticalMovement =   w * (6 * b[6] * pow(t, 5) + 5 * b[5] * pow(t, 4) + 4 * b[4] * pow(t, 3) + 3 * b[3] * pow(t, 2) + 2 * b[2] * t + b[1]);
                // horizontalMovement = (2.0 * S) / (T);
                // verticalMovement = 0;
            }

            desiredSpatialVelocity << 
                    horizontalMovement * cos(angle[legNum]),
                    horizontalMovement * sin(angle[legNum]),
                    verticalMovement;

            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

            for (int joint = 0; joint < 3; joint++)
            {
                desiredAngularVelocities[legNum * 3 + joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);

        // Send the angles to the arduino and simulation as desired
        // setAngs(nextAngles);

        // Send Velcoties to the simulator
        simulator->setSimVelocity(nextAngles, desiredAngularVelocities);

        currentAngles = nextAngles;
        updatePos();

        // Implement Real time delay
        rsLoop.realTimeDelay();
        // Integrate the Simulator server
        simulator->server->integrateWorldThreadSafe();

        operationDuration--;
    }

    desiredAngles = currentAngles;

    for (size_t i = 0; i < standing.size(); ++i) {
        standing[i] = !standing[i];
    }
}