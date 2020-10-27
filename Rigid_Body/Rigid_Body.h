#include <eigen3/Eigen/Dense>

using namespace Eigen;
using Eigen::Matrix3d;
using Eigen::Vector3d;

class RigidBody
{
private:
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
    Vector3d speedVector;
    Vector3d rotationVector;

    // Pre-computed quantities
    // (This is all(*) information you need for computing other variables)
    Vector3d totalForce;
    Vector3d totalTorque; // It's like angular force, but not

public:
    RigidBody(double massInp, Matrix3d inertiaTensorBodyInp, Vector3d positionVectorInp, Matrix3d rotationMatrixInp, Vector3d linearMomentumInp, Vector3d angularMomentumInp);
    void updateTotalForce(Vector3d totalForceInp);
    void updateTotalTorque(Vector3d totalTorqueInp);
    void computeStepByEuler(double h);

    void viewPositionVector();
    void viewRotationMatrix();
};

double euler(double h, double f_x0, double dfdx_x0);
Vector3d euler(double h, Vector3d f_x0, Vector3d dfdx_x0);
Matrix3d euler(double h, Matrix3d f_x0, Matrix3d dfdx_x0);