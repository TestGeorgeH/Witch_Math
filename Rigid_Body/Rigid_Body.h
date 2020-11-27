#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <functional>

using namespace Eigen;
using Eigen::Matrix3d;
using Eigen::Vector3d;

struct ExternalInfluences
{
    Vector3d totalForce;
    Vector3d totalTorque; 
};

struct Derivatives
{
    Vector3d linearMomentum;
    Vector3d angularMomentum;
};

struct BodyPosition
{
    Vector3d positionVector; // Do get-ers?
    Matrix3d rotationMatrix;
};

class RigidBody
{
private:
    double mass;
    Matrix3d inertiaTensorBody;

    Derivatives derivatives;
    ExternalInfluences externalInfluences;

    void rotationMatrixToNormal();

public:
    BodyPosition bodyPosition;

    RigidBody(double massInp, Matrix3d inertiaTensorBodyInp, BodyPosition bodyPositionInp, Derivatives derivativesInp);
    void makeStepByRungeKutt(double h);
    void view();
};

RigidBody* CylinderRigidBody(double r,double h);
