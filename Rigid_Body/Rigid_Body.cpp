#include "Rigid_Body.h"
#include <iostream>

double euler(double h, double f_x0, double dfdx_x0)
{
    double result = f_x0 + h * dfdx_x0;
    return result;
}

Vector3d euler(double h, Vector3d f_x0, Vector3d dfdx_x0)
{
    Vector3d result;
    for (int i=0; i<3; i++)
    {
        result(i) = euler(h, f_x0(i), dfdx_x0(i));
    }
    return result;
}

Matrix3d euler(double h, Matrix3d f_x0, Matrix3d dfdx_x0)
{
    Matrix3d result;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            result(i,j) = euler(h, f_x0(i,j), dfdx_x0(i,j));
        }
    }
    return result;
}

Matrix3d starFunction(Vector3d x)
{
    Matrix3d result;
    result << 0, -x(2), x(1),
              x(2), 0, -x(0),
              -x(1), x(0), 0;
    return result;
}

RigidBody::RigidBody(double massInp, Matrix3d inertiaTensorBodyInp, Vector3d positionVectorInp, Matrix3d rotationMatrixInp, Vector3d linearMomentumInp, Vector3d angularMomentumInp)
{
    mass = massInp;
    inertiaTensorBody = inertiaTensorBodyInp;
    positionVector = positionVectorInp;
    rotationMatrix = rotationMatrixInp;
    linearMomentum = linearMomentumInp;
    angularMomentum = angularMomentumInp;

    inertiaTensorInv = inertiaTensorBody.inverse();
    speedVector << 0, 0, 0;
    rotationVector << 0, 0, 0;
    totalForce << 0, 0, 0;
    totalTorque << 0, 0, 0;
}

void RigidBody::computeStepByEuler(double h)
{
    linearMomentum = euler(h, linearMomentum, totalForce);
    angularMomentum = euler(h, angularMomentum, totalTorque);
    inertiaTensorInv = (rotationMatrix)*(inertiaTensorBody.inverse())*(rotationMatrix.transpose());
    speedVector = linearMomentum/mass;
    rotationVector = inertiaTensorInv*angularMomentum;
    
    positionVector = euler(h, positionVector, speedVector);

    rotationMatrix = euler(h, rotationMatrix, starFunction(rotationVector)*rotationMatrix);
}

void RigidBody::viewPositionVector(){ std::cout << positionVector << '\n'; }

void RigidBody::viewRotationMatrix(){ std::cout << rotationMatrix << '\n'; }

int main()
{
    double mass = 1;
    Matrix3d cubeInertiaTensor, rotationMatrix;
    rotationMatrix << 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1;
    cubeInertiaTensor << 1, 0, 0,
                         0, 1, 0,
                         0, 0, 1;
    Vector3d positionVector, linearMomentum, angularMomentum;
    positionVector << 0, 0, 0;
    linearMomentum << 0.1, 0, 0;
    angularMomentum << 0.5, 1, 0;

    RigidBody body(mass, cubeInertiaTensor, positionVector, rotationMatrix, linearMomentum, angularMomentum);
    
    for (int i=0; i<30; i++)
    {
        body.viewPositionVector();
        body.viewRotationMatrix();
        std::cout << '\n';
        body.computeStepByEuler(0.01);
    }
}