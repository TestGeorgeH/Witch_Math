#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include <functional>

#define Cos(th) cos(M_PI/180*(th))
#define Sin(th) sin(M_PI/180*(th))

using namespace Eigen;
using Eigen::Matrix3d;
using Eigen::Vector3d;

#define FLOORLEVEL -3

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
    Vector3d getCollisionPlacementCylinder(double r,double h, double e);
public:
    BodyPosition bodyPosition;

    RigidBody(double massInp, Matrix3d inertiaTensorBodyInp, BodyPosition bodyPositionInp, Derivatives derivativesInp);
    void makeStepByRungeKutt(double h);
    void view();

    bool isCollidedCylinder(double r, double h);
    void floorCollisionHanlerCylinder(Vector3d collisionPlacement);
    void solve(double step, double r, double h, double e);

};

RigidBody* CylinderRigidBody(double r,double h);
Vector3d* CylinderHitbox(double r,double h);
