/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include "test-environment.h"

#ifdef OS_WINDOWS
	#include "TestEnvironment.h"
#endif

//------------------------------------------------------------------------------
/**
 * \file test-environment.cpp
 * $Author: trappe $
 * $Date: 2004/10/15 20:04:28 $
 * $Revision: 1.75 $
 * \brief cpp Datei der Testumgebung
 *
 * Diese Testumgebung kann genutzt werden um eigene Programmteile auszutesten.
 * Es wird beispielhaft ein Würfel gezeichnet. Dieser kann durch eigene Objekte
 * ersetzt werden.
 */
//------------------------------------------------------------------------------

using namespace std;

#ifdef OS_WINDOWS
	//const TestEnvironment& testEnvironment=testEnvironmentFreefall;
	const TestEnvironment& testEnvironment=testEnvironmentNetwork;
	//const TestEnvironment& testEnvironment=testEnvironmentSimon;
	//const TestEnvironment& testEnvironment=testEnvironmentStress;
	//const TestEnvironment& testEnvironment=testEnvironmentBodies;
	//const TestEnvironment& testEnvironment=testEnvironmentBodies2;
	//const TestEnvironment& testEnvironment=testEnvironmentBodies3;
	//const TestEnvironment& testEnvironment=testEnvironmentClothSimonWish;
	//const TestEnvironment& testEnvironment=testEnvironmentCollision;
	//const TestEnvironment& testEnvironment=testEnvironmentConstraints;
	//const TestEnvironment& testEnvironment=testEnvironmentConstraints2;
	//const TestEnvironment& testEnvironment=testEnvironmentConstraints3;
	//const TestEnvironment& testEnvironment=testEnvironmentCloth;
#endif

// Fenster groesse
GLint windowWidth = 400;
GLint windowHeight = 400;

GLfloat lightPosition1[] = {0.5, 0.5, 0.0, 0.0};//Position der Lichtquelle
GLfloat lightPosition2[] = {0.0, 0.0, 0.0, 1.0};//Position der Lichtquelle

// use some booleans to save the mousestate
GLint leftButton = 0,
  rightButton = 0,
  middleButton = 0;
GLfloat transX = 0.0;
GLfloat transY = 0.0;

// old mouspositions
GLint oldX = windowWidth / 2,
  oldY = windowHeight / 2;

GLdouble sinTheta;   // vorberechneter sin(theta)

GLdouble theta,
	phi;
GLdouble sensibilityX = 0.01;
GLdouble sensibilityY = 0.01;
// ein Meter abstand von der Szene
GLdouble viewRadius = 1000; // in millimeter
GLdouble zoomSensibility = 3;

//Boolsche Variablen
bool autoCam = false;
bool wireframe = false;

bool allErrors = false;
bool simonErrors = false;
bool bodyErrors = false;
bool constraintErrors = false;
bool clothErrors = false;
bool collisionErrors = false;

bool allMessages = false;
bool simonMessages = false;
bool bodyMessages = false;
bool constraintMessages = false;
bool clothMessages = false;
bool collisionMessages = false;


Clock watch;

//! Welche Zeit hatten wir im letzten Display() durchlauf?
float lastTime = 33;


 //! liefert die letzte dauer des display-Loops zurück
float getLastTime(){
	return lastTime;
}



/**
* \brief Display Methode für die Anzeige
*/
void display(void)
{
	watch.start();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Falls automatische Kammera, verändere blickposition
	if (autoCam){
		theta = theta + 0.0001;
		phi =  phi + 0.0001;
	}

	// look at (0,0,0). Use radius and the angles theta and
	// phi to set cam location
	sinTheta = sin (theta);

	gluLookAt(transX + (viewRadius * sinTheta * cos (phi)),      // x - coord of cam
			  transY + (viewRadius * cos (theta)),               // y - coord of cam
			  viewRadius * sinTheta * sin (phi),      // z - coord of cam
			  transX,transY,0,                                  // center of intrest
			  0,1,0);                                 // upvector

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition1);//Lichtquelle definieren
	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition2);//Lichtquelle definieren

    GLfloat color[3] = { 0.9, 0.8, 0.9};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, color);//Lichtquelle definieren
	glLightfv(GL_LIGHT1, GL_DIFFUSE, color);//Lichtquelle definieren

	glLightfv(GL_LIGHT0, GL_AMBIENT, color);//Lichtquelle definieren
	glLightfv(GL_LIGHT1, GL_AMBIENT,color);//Lichtquelle definieren

	if (wireframe){//Falls wireframe = true zeige Gitternetz an
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);//Wireframe eingeschaltet
	}
	else { 
		glPolygonMode(GL_FRONT, GL_FILL);//Polygone werden gefüllt
		glPolygonMode(GL_BACK, GL_LINE);
	}

	// Aufrufen der test eigenen Methoden
#ifdef OS_WINDOWS
	TestEnvironment::DisplayLoopFunctionPointer displayLoop =
		testEnvironment.getDisplayLoop();
#endif
	displayLoop();

	//Ausgabe der Fehlermeldungen
	if (allErrors){
		SimonState::exemplar()->printErrors();
		allErrors = false;
	}
	if (simonErrors){
		SimonState::exemplar()->printErrors("Simon");
		simonErrors = false;
	}
	if (bodyErrors){
		SimonState::exemplar()->printErrors("Bodies");
		SimonState::exemplar()->printErrors("Body");
		bodyErrors = false;
	}
	if (constraintErrors){
		SimonState::exemplar()->printErrors("Constraint");
		constraintErrors = false;
	}
	if (clothErrors){
		SimonState::exemplar()->printErrors("Cloth");
		SimonState::exemplar()->printErrors("Hair");
		clothErrors = false;
	}
	if (collisionErrors){
		SimonState::exemplar()->printErrors("Collision");
		collisionErrors = false;
	}


	//Ausgabe der Meldungen
	if (allMessages){
		SimonState::exemplar()->printMessages();
		allMessages = false;
	}
	if (simonMessages){
		SimonState::exemplar()->printMessages("Simon");
		simonMessages = false;
	}
	if (bodyMessages){
		SimonState::exemplar()->printMessages("Bodies");
		SimonState::exemplar()->printMessages("Body");
		bodyMessages = false;
	}
	if (constraintMessages){
		SimonState::exemplar()->printMessages("Constraint");
		constraintMessages = false;
	}
	if (clothMessages){
		SimonState::exemplar()->printMessages("Cloth");
		SimonState::exemplar()->printMessages("Hair");
		clothErrors = false;
	}
	if (collisionMessages){
		SimonState::exemplar()->printMessages("Collision");
		collisionMessages = false;
	}

 	glutSwapBuffers();

	// speichere lastTime für getLastTime()
	lastTime = watch.fillUp(30);
}

/**
* \brief Init-Methode um Startzustand zu initialisieren
*/

void init(int argc, char** argv){//Initialisierung

	glClearColor (0.5, 0.5, 0.6, 0.0); //Hintergrundfarbe
	glEnable(GL_DEPTH_TEST);//Tiefentest einschalten
	glEnable(GL_NORMALIZE);//Automatisches normalisieren einschalten
	glEnable(GL_LIGHTING);//Licht einschalten
	glEnable(GL_LIGHT0);//Lichtquelle 0 einschalten
	glEnable(GL_LIGHT1);//Lichtquelle 1 einschalten
	glMatrixMode(GL_PROJECTION);

	puts("\n\
Tastaturbefehle:\n\
     \n\
  Grundfunktionen:\n\
     'q': beenden des Programmes\n\
     'r': zuruecksetzen der Parameter\n\
     'a': automatische Kammera fürung ein-/ausschalten\n\
     'w': Wireframe ein-/ausschalten\n\
\n\
  Fehlermeldungen:\n\
     'e': alle Fehlermeldungen ausgeben\n\
     's': Simons Fehlermeldungen ausgeben\n\
     'g': Gelenk Fehlermeldungen ausgeben\n\
     'f': Festkörper Fehlermeldungen ausgeben\n\
     'k': Kollisions Fehlermeldungen ausgeben\n\
     'h': Haare & Kleidung Fehlermeldungen ausgeben\n\
\n\
  Meldungen:\n\
     'E': alle Meldungen ausgeben\n\
     'S': Simons Meldungen ausgeben\n\
     'G': Gelenk Meldungen ausgeben\n\
     'F': Festkörper Meldungen ausgeben\n\
     'K': Kollisions Meldungen ausgeben\n\
     'H': Haare & Kleidung Meldungen ausgeben\n");



	//Anfangseinstellung für Mausbewegung
	phi = 1.4;
	theta = 0.6;

	gluPerspective(45,1,1,10000);


	// Farben und Formen
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(2);

#ifdef OS_WINDOWS
	TestEnvironment::InitializeFunctionPointer initialize=
		testEnvironment.getInitialize();
#endif

	Clock::init();

	// Aufrufen der Test-Eigenen initialisierung
	initialize(argc, argv);
}

/**
* \brief Idle-Methode; um ständiges Zeichnen zu bewirken
*/
void idle()// für permanentes Ausführen der display-Funktion
{
	display();
}

/**
* \brief Keyboard-Methode; Abfrage der Tastatureingabe
*/
void keyboard(unsigned char key, int /*x*/, int /*y*/){

	switch(key){//Tastaturabfrage
	case 'a'://Ein-Ausschalten der Autoanimation
		if (autoCam){
			autoCam = false;
		}
		else {
			autoCam = true;
		}
		glutPostRedisplay();
		break;

	case 'q' :
		puts("Bye Bye");
		exit(1);
		break;

	case 'w'://Ein-Ausschalten Wireframe
		puts("Wireframe Mode");

		wireframe=!wireframe;

		glutPostRedisplay();
		break;

	case 'r'://Zurücksetzen des Parameter
		cout << "Resetting all Values" << endl;
		phi = 45;
		theta = 45;
		viewRadius = 5;
		autoCam = false;
		transX = 0.0;
		transY = 0.0;
		glutPostRedisplay();
		break;

	case 'e'://alle Fehlermeldungen ausgeben
		if (!allErrors)
			allErrors = true;
		glutPostRedisplay();
		break;

	case 's':// Fehlermeldungen für Simon
		if (!simonErrors)
			simonErrors = true;
		glutPostRedisplay();
		break;

	case 'f':// Fehlermeldungen für Festkörper
		if (!bodyErrors)
			bodyErrors = true;
		glutPostRedisplay();
		break;
	case 'g':// Fehlermeldungen für Gelenke
		if (!constraintErrors)
			constraintErrors = true;
		glutPostRedisplay();
		break;
	case 'h':// Fehlermeldungen für Haare und Kleidung
		if (!clothErrors)
			clothErrors = true;
		glutPostRedisplay();
		break;
	case 'k':// Fehlermeldungen für Kollision
		if (!collisionErrors)
			collisionErrors = true;
		glutPostRedisplay();
		break;

	case 'E'://alle Meldungen ausgeben
		if (!allMessages)
			allMessages = true;
		glutPostRedisplay();
		break;

	case 'S':// Meldungen für Simon
		if (!simonMessages)
			simonMessages = true;
		glutPostRedisplay();
		break;

	case 'F':// Meldungen für Festkörper
		if (!bodyMessages)
			bodyMessages = true;
		glutPostRedisplay();
		break;
	case 'G':// Meldungen für Gelenke
		if (!constraintMessages)
			constraintMessages = true;
		glutPostRedisplay();
		break;
	case 'H':// Meldungen für Haare und Kleidung
		if (!clothMessages)
			clothMessages = true;
		glutPostRedisplay();
		break;
	case 'K':// Meldungen für Kollision
		if (!collisionMessages)
			collisionMessages = true;
		glutPostRedisplay();
		break;
	}

	// Aufrufen der test-eigenen
#ifdef OS_WINDOWS
	TestEnvironment::KeyHandlerFunctionPointer keyHandler=
		testEnvironment.getKeyHandler();
	assert(keyHandler);
#endif
	keyHandler(key);
}

/**
* \brief empfängt die Bewegung der Maus bei gedrückter Maustaste
 *
 * \param x      the x-koordinate (from left to right)
 * \param y      the y-koordinate (from top to bottom)
 */
void motionTracker(int x, int y) {
  GLdouble xVariation = sensibilityX * (x - oldX),
    yVariation = sensibilityY * (y - oldY);

  if (leftButton) {
    if (xVariation > 0) {
      if (phi < 2 * M_PI)
	phi += xVariation;
      else
	phi = 0.001;
    }
    else {
      if (phi > 0)
	phi += xVariation;
      else
	phi = 2 * M_PI + 0.001;
    }
    if (yVariation > 0) {
      if (theta < M_PI - 0.1)
	theta += yVariation;
    }
    else {
      if (theta > 0.1)
	theta += yVariation;
    }

    oldX = x;
    oldY = y;
  }

  if (rightButton) {
    viewRadius += yVariation * zoomSensibility;
  }

  if (middleButton) {
	transX = 10 * sensibilityX * (x - oldX);
	transY = 10 * sensibilityY * (oldY - y);
  }

}

/**
* \brief Abfrage welche Maustaste gedrückt wurde.
 *
 * \param button tells the type of button (GLUT_LEFT_BUTTON, GLUT_RIGHT_BUTTON, ...)
 * \param state  tells about the action (GLUT_DOWN or GLUT_UP).
 * \param x      the x-koordinate (from left to right)
 * \param y      the y-koordinate (from top to bottom)
*/
void clickTracker(int button, int /*state*/, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) {
    leftButton = 1;
    oldX = x;
    oldY = y;
  }
  else {
    leftButton = 0;
  }
  if (button == GLUT_RIGHT_BUTTON) {
    rightButton = 1;
    oldX = x;
    oldY = y;
  }
  else {
    rightButton = 0;
  }
  if (button == GLUT_MIDDLE_BUTTON) {
	middleButton = 1;
	oldX = x;
	oldY = y;
  }
  else {
    middleButton = 0;
  }
}

#ifdef OS_WINDOWS
#define MAIN main
#else
#define MAIN main
#endif


int MAIN(int argc, char** argv){

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize (windowHeight, windowWidth);//Größe des Fensters angeben
	glutInitWindowPosition (100, 100);//Position des Fensters angeben
	glutCreateWindow ("Physik-Test");//Titel des Fensters
	init(argc, argv);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);//Keyboard Funktion
	glutMouseFunc(clickTracker);//Maus-Funktion
	glutMotionFunc(motionTracker);//Maus-Bewegung
	glutDisplayFunc(display);
	glutMainLoop();

	return 0;


}
