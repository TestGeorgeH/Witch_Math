#include "Rigid_Body.h"

using namespace std;

Matrix3d starFunction(Vector3d x)
{
    Matrix3d result;
    result << 0, -x(2), x(1),
              x(2), 0, -x(0),
              -x(1), x(0), 0;
    return result;
}

void RigidBody::rotationMatrixToNormal()
{
    Matrix3d A = bodyPosition.rotationMatrix;
    Matrix3d R;
    R << 0, 0, 0,
         0, 0, 0,
         0, 0, 0;
    double t_1_2, t_1_3, t_2_3;
    R.col(0) = A.col(0);
    t_1_2 = A.col(1).dot(R.col(0))/(R.col(0).dot(R.col(0)));
    R.col(1) = A.col(1) - t_1_2*R.col(0);
    t_1_3 = A.col(2).dot(R.col(0))/(R.col(0).dot(R.col(0)));
    t_2_3 = A.col(2).dot(R.col(1))/(R.col(1).dot(R.col(1)));
    R.col(2) = A.col(2) - t_1_3*R.col(0)  - t_2_3*R.col(1);
    bodyPosition.rotationMatrix = R;
}

RigidBody::RigidBody(double massInp, Matrix3d inertiaTensorBodyInp, BodyPosition bodyPositionInp, Derivatives derivativesInp)
{
    mass = massInp;
    inertiaTensorBody = inertiaTensorBodyInp;
    bodyPosition.positionVector = bodyPositionInp.positionVector;
    bodyPosition.rotationMatrix = bodyPositionInp.rotationMatrix;
    derivatives.linearMomentum = derivativesInp.linearMomentum;
    derivatives.angularMomentum = derivativesInp.angularMomentum;

    externalInfluences.totalForce << 0, 0, 0;
    externalInfluences.totalTorque << 0, 0, 0;
}


template <typename T>
T baseStepByRungeKutt(double h, T currentState, const function<T (double h, T state)>& f)
{
    T k1, k2, k3, k4;
    k1 = f(h, currentState);
    k2 = f(h, currentState + (h/2)*k1);
    k3 = f(h, currentState + (h/2)*k2);
    k4 = f(h, currentState + h*k3);
    return currentState + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
}

void RigidBody::makeStepByRungeKutt(double step)
{
    ExternalInfluences theExternalInfluences = externalInfluences;
    auto totalForceFunction = [theExternalInfluences](double h, Vector3d currentState){ return theExternalInfluences.totalForce; };
    auto totalTorqueFunction = [theExternalInfluences](double h, Vector3d currentState){ return theExternalInfluences.totalTorque; };

    auto linearMomentumFunction = [totalForceFunction](double h, Vector3d currentState)
    {
        Vector3d result = baseStepByRungeKutt <Vector3d> (h, currentState, totalForceFunction);
        return result;
    };
    
    auto angularMomentumFunction = [totalTorqueFunction](double h, Vector3d currentState)
    {
        Vector3d result = baseStepByRungeKutt <Vector3d> (h, currentState, totalTorqueFunction);
        return result;
    };

    derivatives.linearMomentum = linearMomentumFunction(step, derivatives.linearMomentum);
    derivatives.angularMomentum = angularMomentumFunction(step, derivatives.angularMomentum);

    auto theMass = mass;
    Vector3d theLinearMomentum = derivatives.linearMomentum;
    auto linearVelocityFunction = [theMass, theLinearMomentum, linearMomentumFunction](double h, Vector3d currentState)
    {
        Vector3d result = linearMomentumFunction(h, theLinearMomentum)/theMass;
        return result;
    };
    bodyPosition.positionVector = baseStepByRungeKutt <Vector3d> (step, bodyPosition.positionVector, linearVelocityFunction);
    Matrix3d theRotationMatrix = bodyPosition.rotationMatrix;
    Matrix3d theInertiaTensorBody = inertiaTensorBody;
    Vector3d theAngularMomentum = derivatives.angularMomentum; 
    auto rotationMatrixDerivative = [theInertiaTensorBody, theRotationMatrix, angularMomentumFunction, theAngularMomentum](double h, Matrix3d currentState)
    {
        Vector3d angularMomentum = angularMomentumFunction(h, theAngularMomentum);
        Matrix3d inertiaTensorInv = (theRotationMatrix)*(theInertiaTensorBody.inverse())*(theRotationMatrix.transpose());
        Vector3d angularVelocity = inertiaTensorInv * angularMomentum;
        Matrix3d starMatrix = starFunction(angularVelocity);
        Matrix3d result = starMatrix * currentState;
        return result;
    };

    bodyPosition.rotationMatrix = baseStepByRungeKutt <Matrix3d> (step, bodyPosition.rotationMatrix, rotationMatrixDerivative);
    rotationMatrixToNormal();
}

void RigidBody::view()
{
    cout << "angularMomentum\n" << derivatives.angularMomentum << "\n\n";
    cout << "linearMomentum\n" << derivatives.linearMomentum << "\n\n";
    cout << "totalForce\n" << externalInfluences.totalForce << "\n\n";
    cout << "totalTorque\n" << externalInfluences.totalTorque << "\n\n";
    cout << "positionVector\n" << bodyPosition.positionVector << "\n\n";
    cout << "rotationMatrix\n" << bodyPosition.rotationMatrix << "\n\n\n";
}

RigidBody* CylinderRigidBody(double r,double h)
{
    double mass = 1;
    Matrix3d cylinderInertiaTensor;
    Derivatives derivatives;
    BodyPosition bodyPosition;

    bodyPosition.rotationMatrix << 1, 0, 0,
                                   0, 1, 0,
                                   0, 0, 1;
    cylinderInertiaTensor << (mass/12) * (3*r*r + h*h), 0, 0,
                              0, (mass * r*r / 2.0), 0,
                              0, 0, (mass/12) * (3*r*r + h*h);
    bodyPosition.positionVector << 0, 0, 0;
    
    derivatives.linearMomentum << 0, 0, 0;
    derivatives.angularMomentum << 0, 50, 5;

    auto result = new RigidBody(mass, cylinderInertiaTensor, bodyPosition, derivatives);
    return result;
}