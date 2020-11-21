#include "Rigid_Body.h"
#include <iostream>

double euler(double h, double f_x0, double dfdt_x0)
{
    double result = f_x0 + h * dfdt_x0;
    return result;
}

Vector3d euler(double h, Vector3d f_x0, Vector3d dfdt_x0)
{
    Vector3d result;
    for (int i=0; i<3; i++)
    {
        result(i) = euler(h, f_x0(i), dfdt_x0(i));
    }
    return result;
}

Matrix3d euler(double h, Matrix3d f_x0, Matrix3d dfdt_x0)
{
    Matrix3d result;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            result(i,j) = euler(h, f_x0(i,j), dfdt_x0(i,j));
        }
    }
    return result;
}

double euler(double h, double f_x0, RigidBody body, double f(double h, RigidBody body))
{
    double result = f_x0 + h * (f(h, body));
    return result;
}

Vector3d euler(double h, Vector3d f_x0, RigidBody body, Vector3d f(double h, RigidBody body))
{
    Vector3d result;
    for (int i=0; i<3; i++)
    {
        result(i) = f_x0(i) + h * f(h, body)(i);
    }
    return result;
}

Matrix3d euler(double h, Matrix3d f_x0, RigidBody body, Matrix3d f(double h, RigidBody body))
{
    Matrix3d result;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            result(i,j) = f_x0(i,j) + h * f(h, body)(i,j);
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
    totalForce << 0, 0, 0;
    totalTorque << 0, 0, 0;
}


Vector3d RigidBody::linearMomentumMetod(double h)
{
    return euler(h, linearMomentum, totalForce);
}

Vector3d RigidBody::angularMomentumMetod(double h)
{
    return euler(h, angularMomentum, totalTorque);
}

Vector3d RigidBody::speedVectorMetod(double h)
{
    return (linearMomentumMetod(h)/mass);
}

Vector3d speedVectorFunction(double h, RigidBody body)
{
    return (body.linearMomentumMetod(h)/(body.mass));
}

Vector3d RigidBody::rotationVectorMetod(double h)
{
    return (inertiaTensorInv*angularMomentumMetod(h));
}

Matrix3d RigidBody::dRdtMetod(double h)
{
    return (starFunction(rotationVectorMetod(h))*rotationMatrix);
}

Matrix3d dRdtFunction(double h, RigidBody body)
{
    return (starFunction(body.rotationVectorMetod(h))*(body.rotationMatrix));
}

void RigidBody::computeStepByEuler(double h)
{
    inertiaTensorInv = (rotationMatrix)*(inertiaTensorBody.inverse())*(rotationMatrix.transpose());
    positionVector = euler(h, positionVector, *this, speedVectorFunction);
    rotationMatrix = euler(h, rotationMatrix, *this, dRdtFunction);
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
    
    for (int i=0; i<3000; i++)
    {
        body.viewPositionVector();
        body.viewRotationMatrix();
        std::cout << '\n';
        body.computeStepByEuler(0.01);
    }
}