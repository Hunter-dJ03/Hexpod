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

Hexapod::Hexapod(unsigned int id, std::unique_ptr<ArduinoController> arduino, RSTimedLoop &rsLoop, bool arduinoConnected, bool raisimSimulator, float rsStep, Path binaryPath)
    : id(id), arduino(move(arduino)), rsLoop(rsLoop), arduinoConnected(arduinoConnected), rsStep(rsStep)
{
    if (raisimSimulator)
    {
        simulator = make_unique<RaisimSimulator>(rsStep, binaryPath, "hexapodV2b.urdf");
    }

    currentAngles = Eigen::VectorXd(18);
}

Hexapod::~Hexapod()
{
    if (arduino) {
        arduino.reset();
    }
    if (simulator) {
        simulator.reset(); 
    }
}

Eigen::MatrixXd Hexapod::getJacobian(int legNum) const
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

    // Jac(0, 0) = -sin(currentAngles[0]) * (0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]) + 0.044925);
    // Jac(0, 1) = -cos(currentAngles[0]) * (0.221426 * sin(currentAngles[1] + currentAngles[2]) + 0.1183145 * sin(currentAngles[1]));
    // Jac(0, 2) = -0.221426 * sin(currentAngles[1] + currentAngles[2]) * cos(currentAngles[0]);
    // Jac(1, 0) = cos(currentAngles[0]) * (0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]) + 0.044925);
    // Jac(1, 1) = -sin(currentAngles[0]) * (0.221426 * sin(currentAngles[1] + currentAngles[2]) + 0.1183145 * sin(currentAngles[1]));
    // Jac(1, 2) = -0.221426 * sin(currentAngles[1] + currentAngles[2]) * sin(currentAngles[0]);
    // Jac(2, 0) = 0;
    // Jac(2, 1) = 0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]);
    // Jac(2, 2) = 0.221426 * cos(currentAngles[1] + currentAngles[2]);

    // cout << "Jac" << endl << Jac << endl;

    return Jac;
}

// Perform inverse kinematics to get desired control engles
Eigen::Vector3d Hexapod::doLegIK(float x, float y, float z) const
{

    // cout<<endl<<x<<","<<y<<","<<z<<","<<endl;

    float dx = x/1000;
    float dy = y/1000;
    float dz = z/1000 - coxaZ;

    // cout<<endl<<dx<<","<<dy<<","<<dz<<endl;

    float H = sqrt(pow(dx, 2) + pow(dy, 2)) - coxaX;
    float L = sqrt(pow(H, 2) + pow(dz, 2));

    // cout<<endl<<H<<","<<L<<","<<endl;

    float omega = atan2(dz, H);
    float beta = acos(Utils::constrain((pow(femurX, 2) + pow(L, 2) - pow(tibiaX, 2)) / (2 * femurX * L), -1, 1));
    float phi = acos(Utils::constrain((pow(femurX, 2) + pow(tibiaX, 2) - pow(L, 2)) / (2 * femurX * tibiaX), -1, 1));

    // cout<<endl<<omega<<","<<beta<<","<<phi<<","<<endl;

    float th1 = atan2(dy, dx);
    float th2 = beta + omega;
    float th3 = M_PI + phi;

    Eigen::Vector3d angs = {th1, th2, th3};

    return angs;
}

// Perform inverse kinematics to get desired control engles
Eigen::Vector3d Hexapod::doBodyIK(float x, float y, float z) const
{

    // cout<<endl<<x<<","<<y<<","<<z<<","<<endl;

    float dx = x/1000 * cos(-bodyLegAngles[1]) - y/1000 * sin(-bodyLegAngles[1]) - bodyLegOffsets[1];
    float dy = x/1000 * sin(-bodyLegAngles[1]) + y/1000 * cos(-bodyLegAngles[1]);

    // float dx = x/1000;
    // float dy = y/1000;
    float dz = z/1000 - coxaZ;

    // cout<<endl<<dx<<","<<dy<<","<<dz<<endl;

    float H = sqrt(pow(dx, 2) + pow(dy, 2)) - coxaX;
    float L = sqrt(pow(H, 2) + pow(dz, 2));

    // cout<<endl<<H<<","<<L<<","<<endl;

    float omega = atan2(dz, H);
    float beta = acos(Utils::constrain((pow(femurX, 2) + pow(L, 2) - pow(tibiaX, 2)) / (2 * femurX * L), -1, 1));
    float phi = acos(Utils::constrain((pow(femurX, 2) + pow(tibiaX, 2) - pow(L, 2)) / (2 * femurX * tibiaX), -1, 1));

    // cout<<endl<<omega<<","<<beta<<","<<phi<<","<<endl;

    float th1 = atan2(dy, dx);
    float th2 = beta + omega;
    float th3 = M_PI + phi;

    Eigen::Vector3d angs = {th1, th2, th3};

    return angs;
}


// Perform forward kinematics of the hexapod leg to get end affector position
Eigen::Vector3d Hexapod::doFK() const
{
    // Eigen::Vector3d pos(
    //     cos(currentAngles[0]) * (0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]) + 0.044925),
    //     sin(currentAngles[0]) * (0.221426 * cos(currentAngles[1] + currentAngles[2]) + 0.1183145 * cos(currentAngles[1]) + 0.044925),
    //     0.221426 * sin(currentAngles[1] + currentAngles[2]) + 0.1183145 * sin(currentAngles[1]) + 0.01065);

    Eigen::Vector3d pos(

        coxaX*cos(bodyLegAngles[1] + currentAngles[0]) + bodyLegAngles[1]*cos(bodyLegAngles[1]) + tibiaX*cos(bodyLegAngles[1] + currentAngles[0])*cos(currentAngles[1] + currentAngles[2]) + femurX*cos(bodyLegAngles[1] + currentAngles[0])*cos(currentAngles[1]),
        coxaX*sin(bodyLegAngles[1] + currentAngles[0]) + bodyLegAngles[1]*sin(bodyLegAngles[1]) + tibiaX*cos(currentAngles[1] + currentAngles[2])*sin(bodyLegAngles[1] + currentAngles[0]) + femurX*sin(bodyLegAngles[1] + currentAngles[0])*cos(currentAngles[1]),
        coxaZ + tibiaX*sin(currentAngles[1] + currentAngles[2]) + femurX*sin(currentAngles[1]));

    return pos;
}

// Move to straight leg position
void Hexapod::moveToZero()
{
    Eigen::VectorXd zeroAnglesVector(18);       
    float zeroAngles[3] = {0, 0, 360 * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        zeroAnglesVector[i] = zeroAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(zeroAnglesVector);
}
// Move to basic standing position
void Hexapod::moveToBasic()
{
    Eigen::VectorXd basicAnglesVector(18);
    float basicAngles[3] = {0 * M_PI / 180, 40 * M_PI / 180, (360 - 102) * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        basicAnglesVector[i] = basicAngles[i % 3]; // Repeat the set of 3 angles
    }

    setAngs(basicAnglesVector);
}
// Move to position that should fold back past limit when power disabled
void Hexapod::moveToOff()
{
    Eigen::VectorXd offAnglesVector(18);
    float offAngles[3] = {0 * M_PI / 180, 90 * M_PI / 180, (360 - 163) * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        offAnglesVector[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(offAnglesVector);
}

// Move to initial position for walk cycle
void Hexapod::walkInit()
{
    Eigen::VectorXd nextAngles(18);

    // Set Start Position
    float offAngles[3] = {0, 50 * M_PI / 180, 270 * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        nextAngles[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(nextAngles);

    
}

void Hexapod::doJacobianTest(const int &style)
{

    // Test Control Variables
    float radius = 0.07; // meters
    double period = 2;   // HZ
    double cycles = 8;

    // Find Duration
    int dur = period * cycles * 1000; // ms

    // Create vector variables for plotting
    // vector<double> t(dur / rsStep);
    // vector<vector<double>> jointVelocity(3, vector<double>(dur / rsStep));
    // vector<vector<double>> jointPosition(3, vector<double>(dur / rsStep));
    // vector<vector<double>> spatialVelocity(3, vector<double>(dur / rsStep));
    // vector<vector<double>> spatialPosition(3, vector<double>(dur / rsStep));

    // Create vector variables for control calculations
    Eigen::Vector3d desiredSpatialVelocity;
    Eigen::Vector3d legJointVelocity;
    Eigen::VectorXd desiredAngularVelocities(18);
    Eigen::VectorXd nextAngles(18);
    // Eigen::Vector3d nextPos;
    Eigen::MatrixXd jacobian;
    Eigen::MatrixXd jacobianPseudoInverse;

    // Set Start Position
    float offAngles[3] = {0, 50 * M_PI / 180, 270 * M_PI / 180};
    for (int i = 0; i < 18; ++i) {
        nextAngles[i] = offAngles[i % 3]; // Repeat the set of 3 angles
    }
    setAngs(nextAngles);

    // Slight delay for servo motors to get to starting angle
    if (arduinoConnected)
    {
        sleep_for(chrono::milliseconds(2000));
    }

    // Update time for real time loop
    rsLoop.updateTimeDelay();

    // Simulation Cycle
    for (int i = 0; i <= dur / rsStep; i++)
    {
        // desiredSpatialVelocity << 0,0,0;
        // Desired spatial velocity of XYZ

        for (int legNum = 0; legNum < 6; legNum++) {

            desiredSpatialVelocity << 0,
                radius * 2 / period * M_PI * cos(2 / period * M_PI * (i) * (rsStep / 1000)) * (2*legStatus[legNum]-1),
                -radius * 2 / period * M_PI * sin(2 / period * M_PI * (i) * (rsStep / 1000));

            jacobian = getJacobian(legNum);
            jacobianPseudoInverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
            legJointVelocity = jacobianPseudoInverse * desiredSpatialVelocity;

            for (int joint = 0; joint < 3; joint++) {
                desiredAngularVelocities[legNum*3+joint] = legJointVelocity[joint]; // Repeat the set of 3 angles
            }
        }
        

        // Convert Spatial velocities to joint Angular velocities
        

        // Find next aqngles using discrete integration
        nextAngles = currentAngles + desiredAngularVelocities * (rsStep / 1000);
        // nextPos = doFK();

        // Set cycle variables for plotting
        // t[i] = i * rsStep;
        // for (int j = 0; j <= 2; j++)
        // {
        //     // pos
        //     jointVelocity[j][i] = desiredAngularVelocities[j];
        //     jointPosition[j][i] = nextAngles[j];
        //     spatialVelocity[j][i] = desiredSpatialVelocity[j];
        //     spatialPosition[j][i] = nextPos[j];
        // }

        // Output desired information
        if (true)
        {
            cout << endl
                 << i * rsStep << endl;
            // cout << endl << i << endl;
            // cout << "Current Angles" << endl<< currentAngles <<endl;
            // cout << "Desired Spatial Velocity" << endl<<desiredSpatialVelocity<<endl;
            // cout << "Jacobian" << endl<<jacobian<<endl;
            // cout << "Inverse Jacobian" << endl<<jacobianPseudoInverse<<endl;
            cout << "Desired Angular Velocity" << endl << desiredAngularVelocities <<endl;
            // cout << "Next Angles" << endl << nextAngles <<endl;
            // cout << "Next Pos" << endl << nextPos <<endl;
        }

        // Send the angles to the arduino and simulation as desired        
        setAngs(nextAngles);

        // Send Velcoties to the simulator
        // simulator->setSimVelocity(nextAngles, desiredAngularVelocities);

        // Implement Real time delay
        rsLoop.realTimeDelay();
        
        // Integrate the Simulator server 
        // simulator->server->integrateWorldThreadSafe();
    }

    // Plot graphs
    /*
    if (false)
    {

        /****** Spacial Position
        plt::figure_size(1200, 380 * 1.5);

        plt::subplot(3, 1, 1);
        plt::plot(t, spatialPosition[0]);
        plt::title("Spatial Position - Position over time");
        plt::ylabel("X (m)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 2);
        plt::plot(t, spatialPosition[1]);
        plt::ylabel("Y (m)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 3);
        plt::plot(t, spatialPosition[2]);
        plt::ylabel("Z (m)");
        plt::xlabel("Time (ms)");

        /****** Spacial Velocity
        plt::figure_size(1200, 380 * 1.5);

        plt::subplot(3, 1, 1);
        plt::plot(t, spatialVelocity[0]);
        plt::title("Spacial Velocity - Velocity over time");
        plt::ylabel("X (m/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 2);
        plt::plot(t, spatialVelocity[1]);
        plt::ylabel("Y (m/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 3);
        plt::plot(t, spatialVelocity[2]);
        plt::ylabel("Z (m/s)");
        plt::xlabel("Time (ms)");

        /****** Joint Velocity
        plt::figure_size(1200, 380 * 1.5);

        plt::subplot(3, 1, 1);
        plt::plot(t, jointVelocity[0]);
        plt::title("Joint Velocity - Velocity over time");
        plt::ylabel("Coxa (rad/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 2);
        plt::plot(t, jointVelocity[1]);
        plt::ylabel("Femur (rad/s)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 3);
        plt::plot(t, jointVelocity[2]);
        plt::ylabel("Tibia (rad/s)");
        plt::xlabel("Time (ms)");

        /****** Joint Position
        plt::figure_size(1200, 380 * 1.5);

        plt::subplot(3, 1, 1);
        plt::plot(t, jointPosition[0]);
        plt::title("Joint Position - Position over time");
        plt::ylabel("Coxa (rads)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 2);
        plt::plot(t, jointPosition[1]);
        plt::ylabel("Femur (rads)");
        // plt::ylabel("Time (ms)");

        plt::subplot(3, 1, 3);
        plt::plot(t, jointPosition[2]);
        plt::ylabel("Tibia (rads)");
        plt::xlabel("Time (ms)");

        plt::show();
    }
    */
    return;
}

// Test the IK position control in X Y Z axis individually
void Hexapod::doLegIKTest()
{
    // Set Start Position
    Eigen::Vector3d posik = {220, 0, -170};
    moveToPos(posik);

    // Set interpolation scale
    int scale = 160;

    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test X movement
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[0] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test Y movement
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 4; i++)
    {
        posik[1] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test Z Movement
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[2] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();
}

// Test the IK position control in X Y Z axis individually
void Hexapod::doBodyIKTest()
{
    // Set Start Position
    Eigen::Vector3d posik = {220, 0, -170};
    moveToPos(posik);

    // Set interpolation scale
    int scale = 160;

    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test X movement
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[0] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[0] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test Y movement
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 4; i++)
    {
        posik[1] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[1] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();

    // Test Z Movement
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale * 2; i++)
    {
        posik[2] -= 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    for (int i = 1; i <= scale; i++)
    {
        posik[2] += 0.5;
        moveToPos(posik);
        rsLoop.realTimeDelay();
        ;
    }
    sleep_for(chrono::milliseconds(2000));
    rsLoop.updateTimeDelay();
}

// Set Angles Overload for 3 individual angle input values
void Hexapod::setAngs(float coxa, float femur, float tibia)
{
    currentAngles(0) = coxa;
    currentAngles(1) = femur;
    currentAngles(2) = tibia;
    sendAngs();
}
// Set Angles Overload for 1x3 eigen vector
void Hexapod::setAngs(const Eigen::VectorXd &angs)
{
    currentAngles = angs;
    sendAngs();
}
// Send the angles of the servo motors to the arduino
void Hexapod::sendAngs()
{

    Eigen::VectorXd modifiedAngs = currentAngles;

    // Subtract the values at the specified indices from 180
    modifiedAngs(10) = - modifiedAngs(10);
    modifiedAngs(11) = - modifiedAngs(11);
    modifiedAngs(13) = - modifiedAngs(13);
    modifiedAngs(14) = - modifiedAngs(14);
    modifiedAngs(16) = - modifiedAngs(16);
    modifiedAngs(17) = - modifiedAngs(17);

    if (arduinoConnected)
    {
        std::vector<std::bitset<11>> angleBinaryRepresentation(modifiedAngs.size());
        for (int i =0; i < modifiedAngs.size(); i++) {
            double baseValue = modifiedAngs[i]* 180 / M_PI;
            if ((i+1)%3==0) {
                baseValue = -baseValue+360;
            }
            bitset<11> bit(static_cast<uint16_t>(Utils::toFixedPoint(baseValue+angleInits[i%3], 1)));
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
        simulator->setSimAngle(modifiedAngs);
    }
}

// Move to Position Overload for 3 individual position input values
void Hexapod::moveToPos(float x, float y, float z)
{
    sendPos(x, y, z);
};
// Move to Position Overload for 1x3 eigen vector
void Hexapod::moveToPos(const Eigen::Vector3d &pos)
{
    // cout <<endl<< pos <<endl;
    sendPos(pos[0], pos[1], pos[2]);
}
// Move to a desired 3D coordinate
void Hexapod::sendPos(float x, float y, float z)
{
    // cout<<endl<<x<<","<<y<<","<<z<<","<<endl;

    // Get Angles for set position
    Eigen::Vector3d angs = doBodyIK(x, y, z);

    cout << endl
         << angs << endl;

    // Move to angles
    setAngs(angs);
}
