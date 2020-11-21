#include "Rigid_Body.h"

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
    quaternion << 0, 0, 0, 0;
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

void RigidBody::debugComputations()
{
    R_t = rotationMatrix.transpose();
    R_min1 = rotationMatrix.inverse();
    detR = rotationMatrix.determinant();
}

void RigidBody::computeQuaternion()
{
    quaternion(0) = sqrt(1 + rotationMatrix(0,0) + rotationMatrix(1,1) + rotationMatrix(2,2))/2;
    quaternion(1) = (rotationMatrix(2,1) - rotationMatrix(1,2))/(4*quaternion(0));
    quaternion(2) = (rotationMatrix(0,2) - rotationMatrix(2,0))/(4*quaternion(0));
    quaternion(3) = (rotationMatrix(1,0) - rotationMatrix(0,1))/(4*quaternion(0));
}

void RigidBody::computeStepByEuler(double h)
{
    inertiaTensorInv = (rotationMatrix)*(inertiaTensorBody.inverse())*(rotationMatrix.transpose());
    positionVector = euler(h, positionVector, *this, speedVectorFunction);
    rotationMatrix = euler(h, rotationMatrix, *this, dRdtFunction);
    computeQuaternion();
    debugComputations();
}

void RigidBody::viewPositionVector(){ std::cout << positionVector << '\n'; }

void RigidBody::viewRotationMatrix(){ std::cout << rotationMatrix << '\n'; }

RigidBody* CylinderRigidBody(double r,double h)
{
    double mass = 1;
    Matrix3d cylinderInertiaTensor, rotationMatrix;
    rotationMatrix << 1, 0, 0,
                      0, 1, 0,
                      0, 0, 1;
    cylinderInertiaTensor << (mass/12) * (3*r*r + h*h), 0, 0,
                         0, (mass/12) * (3*r*r + h*h), 0,
                         0, 0, (mass * r*r / 2.0);
    Vector3d positionVector, linearMomentum, angularMomentum;
    positionVector << 0, 0, 0;
    linearMomentum << 0, 0, 0;
    angularMomentum << 20, 40, 8;
    auto result = new RigidBody(mass, cylinderInertiaTensor, positionVector, rotationMatrix, linearMomentum, angularMomentum);
    return result;
}
