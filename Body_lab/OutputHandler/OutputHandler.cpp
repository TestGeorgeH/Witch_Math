#include <GL/glut.h>
#include <GL/glu.h>
#include "../Rigid_Body/Rigid_Body.h"

#define DEF_D 1
#define Cos(th) cos(M_PI/180*(th))
#define Sin(th) sin(M_PI/180*(th))

#define RADIUS 1
#define HEIGHT 0.1
#define STEP 0.001

RigidBody rigidBody = *(CylinderRigidBody(RADIUS, HEIGHT));


void Idle() {
    glutPostRedisplay();
}

void drawFloor(double d)
{
    glBegin(GL_QUADS);
    glColor3d(0.5,0.5,0.5);
    glVertex3f(-100, -100, 2*d);
    glVertex3f(100, -100, 2*d);
    glVertex3f(100, 100, 2*d);
    glVertex3f(-100, 100, 2*d);

    glEnd();
}

void drawCylinder(double r, double h) {
    int j, i, k;
    glBegin(GL_QUAD_STRIP);
    for (j=0;j<=360;j+=DEF_D) {
        glColor3f(Cos(j),Sin(j),0.5);
        glVertex3f(r*Cos(j),h/2,r*Sin(j));
        glColor3f(0.2,Cos(j),Sin(j));
        glVertex3f(r*Cos(j),-h/2,r*Sin(j));
    }
    glEnd();

    for (i=1;i>=-1;i-=2) {
        glBegin(GL_TRIANGLE_FAN);
        glColor3f(0.0,0.0,1.0);
        glVertex3f(0,i*h/2,0);
        for (k=0;k<=360;k+=DEF_D) {
            glColor3f(1,0.0,0.0);
            glVertex3f(i*Cos(k)*r,i*h/2,Sin(k)*r);
        }
        glEnd();
    }
}

void Display()
{
    glViewport(0, 0, 1000, 1000);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 1, 1, 500);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    rigidBody.makeStepByRungeKutt(STEP);
    // rigidBody.view();
    if (rigidBody.isCollidedCylinder(RADIUS, HEIGHT, 0.05))
        rigidBody.floorCollisionHanlerCylinder(RADIUS, HEIGHT, 0.05);
    glPushMatrix();
    
    drawFloor(FLOORLEVEL);

    BodyPosition rigidBodyPosition = rigidBody.bodyPosition;

    glTranslated(rigidBodyPosition.positionVector(0), rigidBodyPosition.positionVector(1), rigidBodyPosition.positionVector(2) - 5);
    GLdouble rotationMatrixForOpenGL[16] = {rigidBodyPosition.rotationMatrix(0,0), rigidBodyPosition.rotationMatrix(1,0), rigidBodyPosition.rotationMatrix(2,0), 0,
                                            rigidBodyPosition.rotationMatrix(0,1), rigidBodyPosition.rotationMatrix(1,1), rigidBodyPosition.rotationMatrix(2,1), 0,
                                            rigidBodyPosition.rotationMatrix(0,2), rigidBodyPosition.rotationMatrix(1,2), rigidBodyPosition.rotationMatrix(2,2), 0,
                                            0, 0, 0, 1}; // The matrix is column major
    glMultMatrixd(rotationMatrixForOpenGL);
    drawCylinder(RADIUS, HEIGHT);
    glPopMatrix();
    glFlush();
    glutSwapBuffers();
}

int main(int argc, char * argv [])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 900);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("WitchMath");
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
    glEnable(GL_DEPTH_TEST);
    glutMainLoop();

    return 0;
}

// g++ OutputHandler/OutputHandler.cpp Rigid_Body/Rigid_Body.cpp -o main -lGL -lGLU -lglut