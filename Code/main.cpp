/*
	Name: Adeshvir Dhillon
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <gl/glut.h>
#include <utility>
#include <vector>

using namespace std;

const int vWidth = 650;    // Viewport width in pixels
const int vHeight = 500;    // Viewport height in pixels

// THE BODY IS THE BASE PART

// Robot Measurements
	// Body Measurements
float robotBodyWidth = 8.0;
float robotBodyLength = 5.0;
float robotBodyDepth = 3.0;
	float robotBodyChestWidth = 7.75;
	float robotBodyChestLength = 3.0;
	float robotBodyChestDepth = 2.0;
		float robotBodyChestCubeWidth = 1.5;
		float robotBodyChestCubeLength = 1.25;
		float robotBodyChestCubeDepth = 0.5;
	float robotBodyMiddleWidth = 2.0;
	float robotBodyMiddleLength = 4.0;
	float robotBodyMiddleDepth = 1.5;
	float robotBodyWaistWidth = 7.25;
	float robotBodyWaistLength = 2.0;
	float robotBodyWaistDepth = 2.25;
	float robotBodyBackpackWidth = 5.0;
	float robotBodyBackpackLength = 5.0;
	float robotBodyBackpackDepth = 1.5;
		float robotBodyCylinderRadiusTop = 0.4;
		float robotBodyCylinderRadiusBottom = 0.4;
		float robotBodyCylinderDepth = 5.0;

	// Head Measurements
float robotHeadWidth = 3.2;
float robotHeadLength = 3.2;
float robotHeadDepth = 3.2;
	float robotHeadBandWidth = 0.5;
	float robotHeadBandLength = 0.6;
	float robotHeadBandDepth = robotHeadLength + 0.5;
	float robotHeadBandBackWidth = 0.5;
	float robotHeadBandBackLength = 0.5;
	float robotHeadBandBackDepth = 0.5;
	float robotHeadRedTopWidth = 0.35;
	float robotHeadRedTopLength = 0.35;
	float robotHeadRedTopDepth = 0.4;
	float robotHeadAntennaLength = 2.5;
	float robotHeadAntennaRadius = 0.15;
	float robotHeadBrowLength = 1.25;
	float robotHeadBrowRadius = 0.1;
	float robotHeadEyeWidth = 0.15;
	float robotHeadEyeLength = 0.10;
	float robotHeadEyeDepth = 0.1;
	float robotHeadRedBottomWidth = 0.5;
	float robotHeadRedBottomLength = 0.5;
	float robotHeadRedBottomDepth = 0.5;

	// Arm Measurements
float robotArmWidth = 2.0;
float robotArmLength = 10.0;
float robotArmDepth = 2.0;
	float robotShoulderWidth = 2.75;
	float robotShoulderLength = 2.5;
	float robotShoulderDepth = 2.5;
	float robotHandWidth = 2.05;
	float robotHandLength = 1.5;
	float robotHandDepth = 2.05;
	float cannonCylinderRadiusTop = 1.0;
	float cannonCylinderRadiusBottom = cannonCylinderRadiusTop;
	float cannonCylinderDepth = 0.35 * robotArmLength;
		float cannonCylinderMiniRadiusTop = 0.25;
		float cannonCylinderMiniRadiusBottom = cannonCylinderMiniRadiusTop;
		float cannonCylinderMiniDepth = 0.75 * robotArmLength;
		float cannonTorusRadiusInner = 0.125 * cannonCylinderRadiusTop;
		float cannonTorusRadiusOuter = cannonCylinderRadiusTop + 0.1;
		float cannonArmWidth = 0.5 * robotArmWidth;
		float cannonArmLength = 2.0;
		float cannonArmDepth = 1.0;

	// Hip Measurements
float robotHipWidth = 7.75;
float robotHipLength = 3.0;
float robotHipDepth = 3.0;
	float robotHipMiddleWidth = 2.0;
	float robotHipMiddleLength = 2.5;
	float robotHipMiddleDepth = 2.0;
		float robotHipCubeWidth = 1.75;
		float robotHipCubeLength = 1.75;
		float robotHipCubeDepth = 1.5;

	// Leg Measurements
float robotLegWidth = 3.0;
float robotLegLength = 10.0;
float robotLegDepth = 2.5;
	float robotKneeWidth = 3.5;
	float robotKneeLength = 2.75;
	float robotKneeDepth = 2.5;
	float robotFeetWidth = 3.5;
	float robotFeetLength = 1.0;
	float robotFeetDepth = 4.25;

	// Joint Angles
float robotAngle = 0.0;
float upperAngle = 0.0;
float headAngle = 0.0;
float leftArmAngle = 0.0;
float rightArmAngle = 0.0;
float cannonAngle = 90;
float cannonSpinAngle = 0.0;
float hipAngle = 0.0;
float leftLegAngle = 0.0;
float rightLegAngle = 0.0;

enum Joint {ROBOT, UPPER_BODY, HEAD, LEFT_ARM, RIGHT_ARM, HIP, LEFT_LEG, RIGHT_LEG, NONE}; // Possible joint values (for adjusting joint angles)
Joint activeJoint = NONE; // Holds the current join, when NONE -> no joint is selected

// Flags
bool cannonStop = true;
bool walkStop = true;
bool walkFlag = true; // true = right leg step | false = left leg step

GLUquadric* quadric; // For quadric surfaces

// Material Colours
	// Robot Blue
GLfloat blue_ambient[] = { 0.101f,0.146f,0.229f,1.0f };
GLfloat blue_specular[] = { 0.337f,0.486f,0.765f,1.0f };
GLfloat blue_diffuse[] = { 0.101f,0.146f,0.229f,1.0f };
GLfloat blue_shininess[] = { 32.0F };

	// Robot Grey
GLfloat grey_ambient[] = { 0.133f,0.16f,0.169f,1.0f };
GLfloat grey_specular[] = { 0.443f,0.533f,0.565f,1.0f };
GLfloat grey_diffuse[] = { 0.222f,0.267f,0.282f,1.0f };
GLfloat grey_shininess[] = { 32.0F };

	// Robot Red
GLfloat red_ambient[] = { 0.247f,0.066f,0.038f,1.0f };
GLfloat red_specular[] = { 0.824f,0.220f,0.125f,1.0f };
GLfloat red_diffuse[] = { 0.412f,0.110f,0.063f,1.0f };
GLfloat red_shininess[] = { 32.0F };

	// Robot White
GLfloat white_ambient[] = { 0.251f,0.268f,0.262f,1.0f };
GLfloat white_specular[] = { 0.835f,0.894f,0.875f,1.0f };
GLfloat white_diffuse[] = { 0.418f,0.447f,0.438f,1.0f };
GLfloat white_shininess[] = { 32.0F };

	// Robot Yellow
GLfloat yellow_ambient[] = { 0.274f,0.194f,0.045f,1.0f };
GLfloat yellow_diffuse[] = { 0.913f,0.647f,0.149f,1.0f };
GLfloat yellow_specular[] = { 0.457f,0.324f,0.075f,1.0f };
GLfloat yellow_shininess[] = { 32.0F };

GLfloat light_position0[] = { 4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_position1[] = { 4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);
void functionKeys(int key, int x, int y);
void animationHandler(int param);
void drawRobot();
void drawBody();
void drawHead();
void drawLeftArm();
void drawRightArm();
void drawHip();
void drawLeftLeg();
void drawRightLeg();

void cannonSpin(int param);
void robotWalk(int param);
void resetRobotWalk(int param);
void resetRobot(int param);

int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(vWidth, vHeight);
	glutInitWindowPosition(200, 30);
	glutCreateWindow("Mobile Suit RX-78-2 Gundam");

	initOpenGL(vWidth, vHeight);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(functionKeys);

	// Prints help output to Terminal/Debugger
	cout << "\n[Help]\nPress a joint key, then use the UP and DOWN arrow keys to adjust the joint's angle.\n\tR - Entire Robot\n\tT - Upper Body\n\tF - Head\n\tG - Hip\n\tQ - Right Arm\n\tE - Left Arm\n\tA - Right Leg\n\tD - Left Leg\n\n\tX - Deselect Joint\n\tZ - Reset Robot\n\nAnimations (case sensitive):\n\tw - Start WALKING ANIMATION\n\tW - Stop WALKING ANIMATION\n\ts - Start CANNON SPINNING ANIMATION\n\tS - Stop CANNON SPINNING ANIMATION\n\n";

	glutMainLoop();

	return 0;
}

void initOpenGL(int w, int h) {
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);   // This second light is currently off

	glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
	glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see 
	glClearColor(0.4F, 0.4F, 0.4F, 0.0F);  // Color and depth for glClear
	glClearDepth(1.0f);
	glEnable(GL_NORMALIZE);    // Renormalize normal vectors 
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	gluLookAt(0.0, 6.0, 30.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	drawRobot();

	glutSwapBuffers();
}

void drawRobot() {
	glPushMatrix();
		glRotatef(robotAngle, 0.0, 1.0, 0.0); // Ensures that the entire robot moves with the robot angle
		glPushMatrix();
			glRotatef(upperAngle, 0.0, 1.0, 0.0); // Ensures that the entire upper body moves with the upper angle
			drawBody();
			drawHead();
			drawLeftArm();
			drawRightArm();
		glPopMatrix();
		drawHip();
		drawLeftLeg();
		drawRightLeg();
	glPopMatrix();
}

// BASE PART
void drawBody() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, blue_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, blue_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, blue_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, blue_shininess);

	glPushMatrix();
		glTranslatef(0.0, 0.95, 0.0);
		glScalef(robotBodyWidth, robotBodyLength, robotBodyDepth);
		glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
		glTranslatef(0.0, 1.7, 1.0);
		glPushMatrix();
			glRotatef(45, 1.0, 0.0, 0.0);
			glScalef(robotBodyChestWidth, robotBodyChestLength, robotBodyChestDepth);
			glutSolidCube(1.0);
		glPopMatrix();
	glPopMatrix();

	glPushMatrix();
		glTranslatef(0.0, 0.325, 1.5);
		glPushMatrix();
			glRotatef(15, 1.0, 0.0, 0.0);
			glScalef(robotBodyMiddleWidth, robotBodyMiddleLength, robotBodyMiddleDepth);
			glutSolidCube(1.0);
		glPopMatrix();
	glPopMatrix();

	glMaterialfv(GL_FRONT, GL_AMBIENT, yellow_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, yellow_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, yellow_shininess);

	glPushMatrix();
		glTranslatef(2.5, 1.5, 2.0);
		glPushMatrix();
			glRotatef(45, 1.0, 0.0, 0.0);
			glScalef(robotBodyChestCubeWidth, robotBodyChestCubeLength, robotBodyChestCubeDepth);
			glutSolidCube(1.0);
		glPopMatrix();
	glPopMatrix();

	glPushMatrix();
		glTranslatef(-2.5, 1.5, 2.0);
		glPushMatrix();
			glRotatef(45, 1.0, 0.0, 0.0);
			glScalef(robotBodyChestCubeWidth, robotBodyChestCubeLength, robotBodyChestCubeDepth);
			glutSolidCube(1.0);
		glPopMatrix();
	glPopMatrix();

	glMaterialfv(GL_FRONT, GL_AMBIENT, red_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, red_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, red_shininess);

	glPushMatrix();
		glTranslatef(0.0, -2.45, 0.0);
		glPushMatrix();
		glScalef(robotBodyWaistWidth, robotBodyWaistLength, robotBodyWaistDepth);
		glutSolidCube(1.0);
		glPopMatrix();
	glPopMatrix();

	glMaterialfv(GL_FRONT, GL_AMBIENT, grey_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, grey_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, grey_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, grey_shininess);

	glPushMatrix();
		glTranslatef(0.0, 0.6, -2.0);
		glScalef(robotBodyBackpackWidth, robotBodyBackpackLength, robotBodyBackpackDepth);
		glutSolidCube(1.0);
	glPopMatrix();

	glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

	glPushMatrix();
		glTranslatef(-3.65, 6.0, -2.25);
		glPushMatrix();
			glRotatef(90, 1.0, 0.0, 0.0);
			glRotatef(30, 0.0, 1.0, 0.0);
			quadric = gluNewQuadric();
			gluCylinder(quadric, robotBodyCylinderRadiusTop, robotBodyCylinderRadiusBottom, robotBodyCylinderDepth, 20, 20);
			gluDisk(quadric, 0.0, robotBodyCylinderRadiusTop, 20, 1);
			gluDeleteQuadric(quadric);
		glPopMatrix();
	glPopMatrix();

	glPushMatrix();
		glTranslatef(3.65, 6.0, -2.25);
		glPushMatrix();
			glRotatef(90, 1.0, 0.0, 0.0);
			glRotatef(-30, 0.0, 1.0, 0.0);
			quadric = gluNewQuadric();
			gluCylinder(quadric, robotBodyCylinderRadiusTop, robotBodyCylinderRadiusBottom, robotBodyCylinderDepth, 20, 20);
			gluDisk(quadric, 0.0, robotBodyCylinderRadiusTop, 20, 1);
			gluDeleteQuadric(quadric);
		glPopMatrix();
	glPopMatrix();
}

void drawHead() {
	glPushMatrix();
		glRotatef(headAngle, 0.0, 1.0, 0.0);

		glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
		glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
		glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

		glPushMatrix();
			glTranslatef(0.0, (0.5 * 7.0 + 0.5 * robotHeadLength), 0.0);
			glPushMatrix();
				glScalef(robotHeadWidth, robotHeadLength, robotHeadDepth);
				glutSolidCube(1.0);
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0.0, (0.5 * 7.0 + robotHeadLength + 0.1), 0.05);
			glPushMatrix();
				glScalef(robotHeadBandWidth, robotHeadBandLength, robotHeadBandDepth);
				glutSolidCube(1.0);
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0.0, (0.5 * 7.0 + robotHeadLength - 0.1), -1.55);
			glPushMatrix();
				glScalef(robotHeadBandBackWidth, robotHeadBandBackLength, robotHeadBandBackDepth);
				glutSolidCube(1.0);
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0.1, (0.5 * 7.0 + robotHeadLength - 0.4), 1.75);
			glPushMatrix();
				glRotatef(-90, 1.0, 0.0, 0.0);
				glRotatef(50, 0.0, 1.0, 0.0);
				glutSolidCone(robotHeadAntennaRadius, robotHeadAntennaLength, 20, 20);
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(-0.1, (0.5 * 7.0 + robotHeadLength - 0.4), 1.75);
			glPushMatrix();
				glRotatef(-90, 1.0, 0.0, 0.0);
				glRotatef(-50, 0.0, 1.0, 0.0);
				glutSolidCone(robotHeadAntennaRadius, robotHeadAntennaLength, 20, 20);
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0.1, (0.5 * 7.0 + robotHeadLength - 0.45), 1.75);
			glPushMatrix();
				glRotatef(90, 1.0, 0.0, 0.0);
				glRotatef(65, 0.0, 1.0, 0.0);
				glutSolidCone(robotHeadBrowRadius, robotHeadBrowLength, 20, 20);
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(-0.1, (0.5 * 7.0 + robotHeadLength - 0.45), 1.75);
			glPushMatrix();
				glRotatef(90, 1.0, 0.0, 0.0);
				glRotatef(-65, 0.0, 1.0, 0.0);
				glutSolidCone(robotHeadBrowRadius, robotHeadBrowLength, 20, 20);
			glPopMatrix();
		glPopMatrix();

		glMaterialfv(GL_FRONT, GL_AMBIENT, red_ambient);
		glMaterialfv(GL_FRONT, GL_SPECULAR, red_specular);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diffuse);
		glMaterialfv(GL_FRONT, GL_SHININESS, red_shininess);

		glPushMatrix();
			glTranslatef(0.0, (0.5 * 7.0 + robotHeadLength - 0.4), 1.8);
			glPushMatrix();
				glScalef(robotHeadRedTopWidth, robotHeadRedTopLength, robotHeadRedTopDepth);
				glutSolidOctahedron();
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(0.0, (0.5 * 7.0 + robotHeadLength - 2.9), 1.75);
			glPushMatrix();
				glScalef(robotHeadRedBottomWidth, robotHeadRedBottomLength, robotHeadRedBottomDepth);
				glutSolidCube(1.0);
			glPopMatrix();
		glPopMatrix();

		glMaterialfv(GL_FRONT, GL_AMBIENT, yellow_ambient);
		glMaterialfv(GL_FRONT, GL_SPECULAR, yellow_specular);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow_diffuse);
		glMaterialfv(GL_FRONT, GL_SHININESS, yellow_shininess);

		glPushMatrix();
			glTranslatef(0.45, (0.5 * 7.0 + robotHeadLength - 0.9), 1.75);
			glPushMatrix();
				glRotatef(-10, 0.0, 0.0, 1.0);
				glScalef(robotHeadEyeWidth, robotHeadEyeLength, robotHeadEyeDepth);
				glutSolidDodecahedron();
			glPopMatrix();
		glPopMatrix();

		glPushMatrix();
			glTranslatef(-0.45, (0.5 * 7.0 + robotHeadLength - 0.9), 1.75);
			glPushMatrix();
				glRotatef(10, 0.0, 0.0, 1.0);
				glScalef(robotHeadEyeWidth, robotHeadEyeLength, robotHeadEyeDepth);
				glutSolidDodecahedron();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void drawLeftArm() {
	glPushMatrix();
		glTranslatef(-(0.5 * robotBodyWidth + 0.5 * robotArmWidth), (0.35 * robotArmLength), 0.0);
		glRotatef(leftArmAngle, 1.0, 0.0, 0.0);
		glTranslatef((0.5 * robotBodyWidth + 0.5 * robotArmWidth), -(0.35 * robotArmLength), 0.0);

		glTranslatef((0.5 * robotBodyWidth + 0.5 * robotArmWidth + 0.4), 0.0, 0.0);
		glPushMatrix();
			glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

			glPushMatrix();
				glTranslatef(0.0, -1.25, 0.0);
				glPushMatrix();
					glScalef(robotArmWidth, robotArmLength, robotArmDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0, 2.6, 0.0);
				glPushMatrix();
					glScalef(robotShoulderWidth, robotShoulderLength, robotShoulderDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glMaterialfv(GL_FRONT, GL_AMBIENT, grey_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, grey_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, grey_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, grey_shininess);

			glPushMatrix();
				glTranslatef(0.0, -6.0, 0.0);
				glPushMatrix();
					glScalef(robotHandWidth, robotHandLength, robotHandDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void drawRightArm() {
	glPushMatrix();
		glTranslatef(-(0.5 * robotBodyWidth + 0.5 * robotArmWidth), (0.35 * robotArmLength), 0.0);
		glRotatef(rightArmAngle, 1.0, 0.0, 0.0);
		glTranslatef((0.5 * robotBodyWidth + 0.5 * robotArmWidth), -(0.35 * robotArmLength), 0.0);

		glTranslatef(-(0.5 * robotBodyWidth + 0.5 * robotArmWidth + 0.4), 0.0, 0.0);
		glPushMatrix();
			glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

			glPushMatrix();
				glTranslatef(0.0, -1.25, 0.0);
				glPushMatrix();
					glScalef(robotArmWidth, robotArmLength, robotArmDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0, 2.6, 0.0);
				glPushMatrix();
					glScalef(robotShoulderWidth, robotShoulderLength, robotShoulderDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glMaterialfv(GL_FRONT, GL_AMBIENT, grey_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, grey_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, grey_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, grey_shininess);

			glPushMatrix();
				glTranslatef(0.0, -6.0, 0.0);
				glPushMatrix();
					glScalef(robotHandWidth, robotHandLength, robotHandDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0, -(0.25 * robotArmLength - 1.0), (0.7 * robotArmWidth + 1.0));
				glRotatef(cannonAngle, 1.0, 0.0, 0.0);

				glPushMatrix();
					quadric = gluNewQuadric();
					gluCylinder(quadric, cannonCylinderRadiusTop, cannonCylinderRadiusBottom, cannonCylinderDepth, 20, 20);
					gluDisk(quadric, 0.0, cannonCylinderRadiusTop, 20, 1);

					glMaterialfv(GL_FRONT, GL_AMBIENT, red_ambient);
					glMaterialfv(GL_FRONT, GL_SPECULAR, red_specular);
					glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diffuse);
					glMaterialfv(GL_FRONT, GL_SHININESS, red_shininess);

					glPushMatrix();
						glTranslatef(0.0, 0.0, 1.0);
						gluDisk(quadric, 0.0, cannonCylinderRadiusTop, 20, 1);
					glPopMatrix();
					gluDeleteQuadric(quadric);
				glPopMatrix();

				glMaterialfv(GL_FRONT, GL_AMBIENT, grey_ambient);
				glMaterialfv(GL_FRONT, GL_SPECULAR, grey_specular);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, grey_diffuse);
				glMaterialfv(GL_FRONT, GL_SHININESS, grey_shininess);

				glPushMatrix();
					glTranslatef(0.0, 0.0, 7.25);
					glutSolidTorus(cannonTorusRadiusInner, cannonTorusRadiusOuter, 20, 20);
				glPopMatrix();

				glPushMatrix();
					glTranslatef(0.0, -0.4, 0.1);
					glPushMatrix();
						glScalef(cannonArmWidth, cannonArmLength, cannonArmDepth);
						glutSolidCube(1.0);
					glPopMatrix();
				glPopMatrix();

				glPushMatrix();
					glMaterialfv(GL_FRONT, GL_AMBIENT, red_ambient);
					glMaterialfv(GL_FRONT, GL_SPECULAR, red_specular);
					glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diffuse);
					glMaterialfv(GL_FRONT, GL_SHININESS, red_shininess);

					glRotatef(cannonSpinAngle, 0.0, 0.0, 1.0);
					glTranslatef(0.0, 0.0, 0.1);
					glPushMatrix();
						glTranslatef(0.0, 0.0, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(0.7, 0.0, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(-0.7, 0.0, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(0.0, 0.7, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(0.0, -0.7, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(0.5, 0.5, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(-0.5, 0.5, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(0.5, -0.5, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();

					glPushMatrix();
						glTranslatef(-0.5, -0.5, 0.0);
						glPushMatrix();
							quadric = gluNewQuadric();
							gluCylinder(quadric, cannonCylinderMiniRadiusTop, cannonCylinderMiniRadiusBottom, cannonCylinderMiniDepth, 20, 20);
							gluDisk(quadric, 0.0, cannonCylinderMiniRadiusTop, 20, 1);
							gluDeleteQuadric(quadric);
						glPopMatrix();
					glPopMatrix();
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void drawHip() {
	glPushMatrix();
		glRotatef(hipAngle, 0.0, 1.0, 0.0);
		glPushMatrix();
			glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

			glPushMatrix();
				glTranslatef(0.0, -(1.655 * robotHipLength), 0.0);
				glPushMatrix();
					glScalef(robotHipWidth, robotHipLength, robotHipDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0, -(1.655 * robotHipLength), 1.0);
				glPushMatrix();
					glScalef(robotHipMiddleWidth, robotHipMiddleLength, robotHipMiddleDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(0.0, -(1.655 * robotHipLength), -1.0);
				glPushMatrix();
					glScalef(robotHipMiddleWidth, robotHipMiddleLength, robotHipMiddleDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glMaterialfv(GL_FRONT, GL_AMBIENT, yellow_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, yellow_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, yellow_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, yellow_shininess);

			glPushMatrix();
				glTranslatef(2.25, -(1.655 * robotHipLength), 1.5);
				glPushMatrix();
					glScalef(robotHipCubeWidth, robotHipCubeLength, robotHipCubeDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(-2.25, -(1.655 * robotHipLength), 1.5);
				glPushMatrix();
					glScalef(robotHipCubeWidth, robotHipCubeLength, robotHipCubeDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(2.25, -(1.655 * robotHipLength), -1.5);
				glPushMatrix();
					glScalef(robotHipCubeWidth, robotHipCubeLength, robotHipCubeDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(-2.25, -(1.655 * robotHipLength), -1.5);
				glPushMatrix();
					glScalef(robotHipCubeWidth, robotHipCubeLength, robotHipCubeDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void drawLeftLeg() {
	glPushMatrix();
		glTranslatef((0.325 * robotHipWidth), (-0.725 * robotLegLength), 0.0);
		glRotatef(leftLegAngle, 1.0, 0.0, 0.0);
		glTranslatef(-(0.325 * robotHipWidth), -(-0.725 * robotLegLength), 0.0);
		glPushMatrix();
			glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

			glPushMatrix();
				glTranslatef((0.325 * robotHipWidth), -(1.655 * robotHipLength + 6.5), 0.0);
				glPushMatrix();
					glScalef(robotLegWidth, robotLegLength, robotLegDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef((0.325 * robotHipWidth), -(1.655 * robotHipLength + 6.5 + 0.5), 0.5);
				glPushMatrix();
					glScalef(robotKneeWidth, robotKneeLength, robotKneeDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glMaterialfv(GL_FRONT, GL_AMBIENT, red_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, red_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, red_shininess);

			glPushMatrix();
				glTranslatef((0.325 * robotHipWidth), -(1.655 * robotHipLength + 6.5 + 4.5), 0.75);
				glPushMatrix();
					glScalef(robotFeetWidth, robotFeetLength, robotFeetDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void drawRightLeg() {
	glPushMatrix();
		glTranslatef((0.325 * robotHipWidth), (-0.725 * robotLegLength), 0.0);
		glRotatef(rightLegAngle, 1.0, 0.0, 0.0);
		glTranslatef(-(0.325 * robotHipWidth), -(-0.725 * robotLegLength), 0.0);
		glPushMatrix();
			glMaterialfv(GL_FRONT, GL_AMBIENT, white_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, white_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, white_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, white_shininess);

			glPushMatrix();
				glTranslatef(-(0.325 * robotHipWidth), -(1.655 * robotHipLength + 6.5), 0.0);
				glPushMatrix();
					glScalef(robotLegWidth, robotLegLength, robotLegDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glPushMatrix();
				glTranslatef(-(0.325 * robotHipWidth), -(1.655 * robotHipLength + 6.5 + 0.5), 0.5);
				glPushMatrix();
					glScalef(robotKneeWidth, robotKneeLength, robotKneeDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();

			glMaterialfv(GL_FRONT, GL_AMBIENT, red_ambient);
			glMaterialfv(GL_FRONT, GL_SPECULAR, red_specular);
			glMaterialfv(GL_FRONT, GL_DIFFUSE, red_diffuse);
			glMaterialfv(GL_FRONT, GL_SHININESS, red_shininess);

			glPushMatrix();
				glTranslatef(-(0.325 * robotHipWidth), -(1.655 * robotHipLength + 6.5 + 4.5), 0.75);
				glPushMatrix();
					glScalef(robotFeetWidth, robotFeetLength, robotFeetDepth);
					glutSolidCube(1.0);
				glPopMatrix();
			glPopMatrix();
		glPopMatrix();
	glPopMatrix();
}

void reshape(int w, int h) {
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)w / h, 0.2, 40.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0.0, 6.0, 22.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}

void cannonSpin(int param) {
	if (!cannonStop) {
		cannonSpinAngle += 2.0;
		glutPostRedisplay();
		glutTimerFunc(10, cannonSpin, 0);
	}
}

void robotWalk(int param) {
	if (!walkStop) {
		if (walkFlag) {
			if (rightLegAngle > -46) {
				rightLegAngle -= 2.0;
			}
			if (hipAngle < 15) {
				hipAngle += 1.0;
			}
			if (leftLegAngle < 46) {
				leftLegAngle += 2.0;
			}

			if ((rightLegAngle == -46) && (hipAngle == 15) && (leftLegAngle == 46)) {
				walkFlag = false;
			}
		}
		else {
			if (rightLegAngle < 46) {
				rightLegAngle += 2.0;
			}
			if (hipAngle > -15) {
				hipAngle -= 1.0;
			}
			if (leftLegAngle > -46) {
				leftLegAngle -= 2.0;
			}

			if ((rightLegAngle == 46) && (hipAngle == -15) && (leftLegAngle == -46)) {
				walkFlag = true;
			}
		}
		glutPostRedisplay();
		glutTimerFunc(10, robotWalk, 0);
	}
}

void resetRobotWalk(int param) {
	if ((rightLegAngle != 0) || (hipAngle != 0) || (leftLegAngle != 0)) {
		if (rightLegAngle > 0) {
			rightLegAngle -= 2.0;
		}
		else if (rightLegAngle < 0) {
			rightLegAngle += 2.0;
		}

		if (hipAngle > 0) {
			hipAngle -= 1.0;
		}
		else if (hipAngle < 0) {
			hipAngle += 1.0;
		}

		if (leftLegAngle > 0) {
			leftLegAngle -= 2.0;
		}
		else if (leftLegAngle < 0) {
			leftLegAngle += 2.0;
		}
		glutPostRedisplay();
		glutTimerFunc(10, resetRobotWalk, 0);
	}
}

void resetRobot(int param) {
	if ((robotAngle != 0) || (upperAngle != 0) || (headAngle != 0) || (leftArmAngle != 0) || (rightArmAngle != 0) || (hipAngle != 0) || (rightLegAngle != 0) || (leftLegAngle != 0)) {
		if (robotAngle > 0) {
			robotAngle -= 2.0;
		}
		else if (robotAngle < 0) {
			robotAngle += 2.0;
		}

		if (upperAngle > 0) {
			upperAngle -= 2.0;
		}
		else if (upperAngle < 0) {
			upperAngle += 2.0;
		}

		if (headAngle > 0) {
			headAngle -= 2.0;
		}
		else if (headAngle < 0) {
			headAngle += 2.0;
		}

		if (leftArmAngle > 0) {
			leftArmAngle -= 2.0;
		}
		else if (leftArmAngle < 0) {
			leftArmAngle += 2.0;
		}

		if (rightArmAngle > 0) {
			rightArmAngle -= 2.0;
		}
		else if (rightArmAngle < 0) {
			rightArmAngle += 2.0;
		}

		if (hipAngle > 0) {
			hipAngle -= 1.0;
		}
		else if (hipAngle < 0) {
			hipAngle += 1.0;
		}

		if (rightLegAngle > 0) {
			rightLegAngle -= 2.0;
		}
		else if (rightLegAngle < 0) {
			rightLegAngle += 2.0;
		}

		if (leftLegAngle > 0) {
			leftLegAngle -= 2.0;
		}
		else if (leftLegAngle < 0) {
			leftLegAngle += 2.0;
		}
		glutPostRedisplay();
		glutTimerFunc(10, resetRobot, 0);
	}
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'r':
	case 'R':
		activeJoint = ROBOT;
		break;
	case 't':
	case 'T':
		activeJoint = UPPER_BODY;
		break;
	case 'f':
	case 'F':
		activeJoint = HEAD;
		break;
	case 'e':
	case 'E':
		activeJoint = LEFT_ARM;
		break;
	case 'q':
	case 'Q':
		activeJoint = RIGHT_ARM;
		break;
	case 'g':
	case 'G':
		activeJoint = HIP;
		break;
	case 'd':
	case 'D':
		activeJoint = LEFT_LEG;
		break;
	case 'a':
	case 'A':
		activeJoint = RIGHT_LEG;
		break;
	case 'x':
	case 'X':
		activeJoint = NONE;
		break;
	case 'w':
		walkStop = false;
		glutTimerFunc(10, robotWalk, 0);
		break;
	case 'W':
		walkStop = true;
		glutTimerFunc(10, resetRobotWalk, 0);
		break;
	case 's':
		cannonStop = false;
		glutTimerFunc(10, cannonSpin, 0);
		break;
	case 'S':
		cannonStop = true;
		break;
	case 'z':
	case 'Z':
		cannonStop = true;
		walkStop = true;
		glutTimerFunc(10, resetRobot, 0);
		break;
	}
	glutPostRedisplay();   // Trigger a window redisplay
}

void functionKeys(int key, int x, int y) {
	switch (key) {
		case GLUT_KEY_F1: // Help Key, prints to Terminal/Debugger
			cout << "\n[Help]\nPress a joint key, then use the UP and DOWN arrow keys to adjust the joint's angle.\n\tR - Entire Robot\n\tT - Upper Body\n\tF - Head\n\tG - Hip\n\tQ - Right Arm\n\tE - Left Arm\n\tA - Right Leg\n\tD - Left Leg\n\n\tX - Deselect Joint\n\tZ - Reset Robot\n\nAnimations (case sensitive):\n\tw - Start WALKING ANIMATION\n\tW - Stop WALKING ANIMATION\n\ts - Start CANNON SPINNING ANIMATION\n\tS - Stop CANNON SPINNING ANIMATION\n\n";
			break;

		case GLUT_KEY_UP:
			if (activeJoint == ROBOT) {
				if (robotAngle == 360) {
					robotAngle = 2;
				}
				else {
					robotAngle += 2.0;
				}
			}
			else if (activeJoint == UPPER_BODY && upperAngle > -46) {
				upperAngle -= 2.0;
			}
			else if (activeJoint == HEAD && headAngle > -46) {
				headAngle -= 2.0;
			}
			else if (activeJoint == LEFT_ARM && leftArmAngle > -210) {
				leftArmAngle -= 2.0;
			}
			else if (activeJoint == RIGHT_ARM && rightArmAngle > -210) {
				rightArmAngle -= 2.0;
			}
			else if (activeJoint == HIP && hipAngle > -15) {
				hipAngle -= 1.0;
			}
			else if (activeJoint == LEFT_LEG && leftLegAngle > -46) {
				leftLegAngle -= 2.0;
			}
			else if (activeJoint == RIGHT_LEG && rightLegAngle > -46) {
				rightLegAngle -= 2.0;
			}
			break;
		
		case GLUT_KEY_DOWN:
			if (activeJoint == ROBOT) {
				if (robotAngle == -360) {
					robotAngle = -2;
				}
				else {
					robotAngle -= 2.0;
				}
			}
			else if (activeJoint == UPPER_BODY && upperAngle < 46) {
				upperAngle += 2.0;
			}
			else if (activeJoint == HEAD && headAngle < 46) {
				headAngle += 2.0;
			}
			else if (activeJoint == LEFT_ARM && leftArmAngle < 30) {
				leftArmAngle += 2.0;
			}
			else if (activeJoint == RIGHT_ARM && rightArmAngle < 30) {
				rightArmAngle += 2.0;
			}
			else if (activeJoint == HIP && hipAngle < 15) {
				hipAngle += 1.0;
			}
			else if (activeJoint == LEFT_LEG && leftLegAngle < 46) {
				leftLegAngle += 2.0;
			}
			else if (activeJoint == RIGHT_LEG && rightLegAngle < 46) {
				rightLegAngle += 2.0;
			}
			break;
	}
	glutPostRedisplay();   // Trigger a window redisplay
}