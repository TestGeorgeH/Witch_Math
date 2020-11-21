#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace Eigen;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class RigidBody
{
public:
    // Constant things
    double mass;
    Matrix3d inertiaTensorBody;
    // Matrix3d inertiaTensorBodyInv;

    // Variables of system's state
    Vector3d positionVector;
    Matrix3d rotationMatrix; // With this ones you can draw the body
    Vector3d linearMomentum;
    Vector3d angularMomentum;

    // Variables for computing system's state
    Matrix3d inertiaTensorInv;

    // Pre-computed quantities
    // (This is all(*) information you need for computing other variables)
    Vector3d totalForce;
    Vector3d totalTorque; // It's like angular force, but not

    Vector4d quaternion;

    RigidBody(double massInp, Matrix3d inertiaTensorBodyInp, Vector3d positionVectorInp, Matrix3d rotationMatrixInp, Vector3d linearMomentumInp, Vector3d angularMomentumInp);
    void updateTotalForce(Vector3d totalForceInp);
    void updateTotalTorque(Vector3d totalTorqueInp);
    void computeStepByEuler(double h);

    void computeQuaternion();

    void viewPositionVector();
    void viewRotationMatrix();

    Vector3d linearMomentumMetod(double h);
    Vector3d angularMomentumMetod(double h);
    Vector3d speedVectorMetod(double h);
    Vector3d rotationVectorMetod(double h);
    Matrix3d dRdtMetod(double h);

    // DEBUG CHECKERS
    Matrix3d R_t;
    Matrix3d R_min1;
    double detR;
    void debugComputations();
    

};

RigidBody* CubeRigidBody();

double euler(double h, double f_x0, double dfdt_x0);
Vector3d euler(double h, Vector3d f_x0, Vector3d dfdt_x0);
Matrix3d euler(double h, Matrix3d f_x0, Matrix3d dfdt_x0);
double euler(double h, double f_x0, RigidBody body, double f(double h, RigidBody body));
Vector3d euler(double h, Vector3d f_x0, RigidBody body, Vector3d f(double h, RigidBody body));
Matrix3d euler(double h, Matrix3d f_x0, RigidBody body, Matrix3d f(double h, RigidBody body));

Matrix3d starFunction(Vector3d x);
