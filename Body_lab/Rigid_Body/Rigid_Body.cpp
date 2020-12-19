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

    externalInfluences.totalForce << 0, -10, 0;
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
    // Linear movement
    ExternalInfluences theExternalInfluences = externalInfluences;
    auto totalForceFunction = [theExternalInfluences](double h, Vector3d currentState){ return theExternalInfluences.totalForce; };
    auto linearMomentumFunction = [totalForceFunction](double h, Vector3d currentState)
    {
        Vector3d result = baseStepByRungeKutt <Vector3d> (h, currentState, totalForceFunction);
        return result;
    };
    derivatives.linearMomentum = linearMomentumFunction(step, derivatives.linearMomentum);
    auto theMass = mass;
    Vector3d theLinearMomentum = derivatives.linearMomentum;
    auto linearVelocityFunction = [theMass, theLinearMomentum, linearMomentumFunction](double h, Vector3d currentState)
    {
        Vector3d result = linearMomentumFunction(h, theLinearMomentum)/theMass;
        return result;
    };
    
    bodyPosition.positionVector = baseStepByRungeKutt <Vector3d> (step, bodyPosition.positionVector, linearVelocityFunction);

    // Angular movement
    auto totalTorqueFunction = [theExternalInfluences](double h, Vector3d currentState){ return theExternalInfluences.totalTorque; };
    auto angularMomentumFunction = [totalTorqueFunction](double h, Vector3d currentState)
    {
        Vector3d result = baseStepByRungeKutt <Vector3d> (h, currentState, totalTorqueFunction);
        return result;
    };
    derivatives.angularMomentum = angularMomentumFunction(step, derivatives.angularMomentum);

    Matrix3d theRotationMatrix = bodyPosition.rotationMatrix;
    Matrix3d theInertiaTensorBody = inertiaTensorBody;
    Vector3d theAngularMomentum = derivatives.angularMomentum; 
    auto rotationMatrixDerivative = [theInertiaTensorBody, angularMomentumFunction, theAngularMomentum](double h, Matrix3d currentState)
    {
        Vector3d angularMomentum = angularMomentumFunction(h, theAngularMomentum);
        Matrix3d inertiaTensorInv = (currentState)*(theInertiaTensorBody.inverse())*(currentState.transpose());
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
    cout << '\n';
    cout << "positionVector\n" << bodyPosition.positionVector << "\n\n";
    cout << "rotationMatrix\n" << bodyPosition.rotationMatrix << "\n\n";
    cout << "linearMomentum\n" << derivatives.linearMomentum << "\n\n";
    cout << "angularMomentum\n" << derivatives.angularMomentum << "\n\n";
    cout << "totalForce\n" << externalInfluences.totalForce << "\n\n";
    cout << "totalTorque\n" << externalInfluences.totalTorque << "\n\n";
    cout << '\n';
}

Vector3d* CylinderHitbox(double r,double h) // Whole idea with hitbox does not work
{
    int size = 720;
    Vector3d* result = new Vector3d[size];
    int resI = 0;
    Vector3d temp;

    for(int i=0; i<360; i++)
    {
        temp << r*Cos(i), h/2, r*Sin(i);
        result[resI++] = temp;
        temp << r*Cos(i), -h/2, r*Sin(i);
        result[resI++] = temp;
    }

    return result;
}

bool RigidBody::isCollidedCylinder(double r, double h)
{
    double collideTolerance = COLLIDETOLERANCE;
    double floorLevel = FLOORLEVEL;
    Vector3d* hitBox = CylinderHitbox(r, h);
    Vector3d n;
    n << 0, 1, 0;
    for (int i = 0; i<720; i++)
    {
        Vector3d collisionPlacement = bodyPosition.rotationMatrix * hitBox[i];
        Matrix3d inertiaTensorInv = bodyPosition.rotationMatrix * inertiaTensorBody.inverse() * bodyPosition.rotationMatrix.transpose();
        Vector3d p_dot = derivatives.linearMomentum/mass + ((inertiaTensorInv * derivatives.angularMomentum).cross(collisionPlacement));
        bool isBelowFloor = (collisionPlacement + bodyPosition.positionVector).dot(n) < floorLevel;
        bool isGoesToFloor = p_dot.dot(n) < -collideTolerance;
        
        if (isBelowFloor && isGoesToFloor)
        {   
            cout << "Goes to floor with speed " << p_dot.dot(n) << '\n';
            return true;
        }
    }
    return false;
}

Vector3d RigidBody::getCollisionPlacementCylinder(double r, double h, double e)
{
    double floorLevel = FLOORLEVEL;
    Vector3d* hitBox = CylinderHitbox(r, h);
    for (int i = 0; i<720; i++)
    {
        double distance = (bodyPosition.rotationMatrix * hitBox[i] + bodyPosition.positionVector)[1] - floorLevel;
        if ( 0 < distance && distance < e)
        {
            cout << "dist " << distance << '\n';
            return hitBox[i];
        }
    }
    Vector3d zero;
    zero << 0, 0, 0;
    return zero;
}

void RigidBody::floorCollisionHanlerCylinder(Vector3d collisionPlacement)
{
    Vector3d colPlWithRotation = bodyPosition.rotationMatrix * collisionPlacement;
    Matrix3d inertiaTensorInv = bodyPosition.rotationMatrix * inertiaTensorBody.inverse() * bodyPosition.rotationMatrix.transpose();
    Vector3d p_dot = derivatives.linearMomentum/mass + ((inertiaTensorInv * derivatives.angularMomentum).cross(colPlWithRotation));
    Vector3d n;
    n << 0, 1, 0;
    double numerator = (-1 * n.dot(p_dot));
    double term1 = 1/mass;
    double term2 = n.dot((inertiaTensorInv * (colPlWithRotation.cross(n))).cross(colPlWithRotation));
    Vector3d force = 2 * (numerator/(term1 + term2)) * n;

    cout << "\nForce\n" << force << '\n';
    cout << "colPlWithRotation\n" << colPlWithRotation << '\n';
    cout << "colPlWithRotation x Force\n" << colPlWithRotation.cross(force) << '\n';

    derivatives.linearMomentum += force;
    derivatives.angularMomentum += colPlWithRotation.cross(force);
}

void RigidBody::solve(double step, double r, double h, double e)
{
    double currentStep = step;
    RigidBody mainCopy = *this;
    RigidBody tempCopy = *this;
    Vector3d collisionPlacement;
    collisionPlacement << 0, 0, 0;
    Vector3d zero;
    zero << 0, 0, 0;

    tempCopy.makeStepByRungeKutt(step);

    if (tempCopy.isCollidedCylinder(r, h))
    {
        while (true)
        {
            cout << "Current step " << currentStep << '\n';
            tempCopy = mainCopy;
            tempCopy.makeStepByRungeKutt(currentStep);

            collisionPlacement = tempCopy.getCollisionPlacementCylinder(r, h, e);
            if (collisionPlacement != zero)
            {
                cout << "FOUND COLLISION\n";
                break;
            }
            if (!tempCopy.isCollidedCylinder(r, h))
            {
                mainCopy.makeStepByRungeKutt(currentStep);
                cout << "DO THE STEP\n";
            }
            else
                currentStep = currentStep/2;;
        }
        cout << "\nBody before\n\n";
        view();
        floorCollisionHanlerCylinder(collisionPlacement);
        cout << "\nBody after\n\n";
        view();
    }

    makeStepByRungeKutt(currentStep); 
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
    derivatives.angularMomentum << 5, 3, 1;

    auto result = new RigidBody(mass, cylinderInertiaTensor, bodyPosition, derivatives);
    return result;
}