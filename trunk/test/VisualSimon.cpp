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


/**
 * \class VisualSimon
 * \author trappe
 * \brief Visual interface to test and show the capabilitys of SIMON.
 *
 */


#include "VisualSimon.h"
#include "GlutApplication.h"
#include "SphereDescription.h"
#include "BoxDescription.h"
#include "PlaneDescription.h"
#include "GraphicsGL.h"
#include "GraphicsRIB.h"


using namespace std;

extern bool parse(const char* filename, SceneDescription *scene);

SceneDescription scene;

VisualSimon::VisualSimon(const char* fileName) :
	GlutWindow("Visual Simon") {

	// set the graphic which shoud be used to draw the scene
	mGraphicContext = new GraphicsGL;
	// set the graphic which shoud be used to draw the scene
	mRIBContext = new GraphicsRIB;


	// no mousebutton pressed at start
	mLeftButton = mRightButton = mMiddleButton = false;

	// initial old mouse positions at (0,0)
	mOldX = mOldY = 0;

	// set mouse sensibility
	mMouseSensibilityX = mMouseSensibilityY = 0.01;

	//Anfangseinstellung für Mausbewegung
	mPhi = 1.4;
	mTheta = 0.6;

	//one meter distance to the objects
	mViewRadius = 1000; // millimeters
	mZoomSensibility = 3;

	mTranslateX = 0.0;
	mTranslateY = 0.0;

	Clock::init();
	
	parse(fileName, &scene);

	ConstraintSystemPtr cs = SimonState::exemplar()->getConstraintSystem();
 	cs->buildGraphs();
 	cs->printGraphs();
 	cs->setComputationAlgorithm(4);
 	cs->setTau(60);
}

VisualSimon::~VisualSimon(){

	delete mGraphicContext;
	delete mRIBContext;
}

/**
* \brief To view the scene
*/
void VisualSimon::display(void){

	Clock watch; // for timing and framerate
	watch.start();


	GLdouble sinMTheta = sin (mTheta);

	mGraphicContext->
		beginFrame(mTranslateX + 
				   (mViewRadius * sinMTheta * cos (mPhi)),   // x - coord of cam
				   mTranslateY + 
				   (mViewRadius * cos (mTheta)),             // y - coord of cam
				   mViewRadius * sinMTheta * sin (mPhi),     // z - coord of cam
				   mTranslateX, mTranslateY, 0               // center of intrest
			);

	mRIBContext->
		beginFrame(mTranslateX + 
				   (mViewRadius * sinMTheta * cos (mPhi)),   // x - coord of cam
				   mTranslateY + 
				   (mViewRadius * cos (mTheta)),             // y - coord of cam
				   mViewRadius * sinMTheta * sin (mPhi),     // z - coord of cam
				   mTranslateX, mTranslateY, 0               // center of intrest
			);

	for (int step = 0; step < scene.getIntegrationsPerFrame(); ++step){
		float intervall = SimonState::exemplar()->getIntegrationIntervall();

		SimonState::exemplar()->getBodySystem()->addGravity();

		SimonState::exemplar()->getConstraintSystem()->step();

		if (scene.testCollisions())
			// Collision test shoud be done about five times (Fedkiw)
			for (int i = 0; i < 5; ++i)
				SimonState::exemplar()->getGeometrySystem()->resolveCollisions(intervall);

		SimonState::exemplar()->getBodySystem()->integrateVelocities(intervall);

		if (scene.testCollisions())
			SimonState::exemplar()->getGeometrySystem()->resolveContactsFast(intervall);

		SimonState::exemplar()->getBodySystem()->integratePositions(intervall);
	}
	
 	drawContacts(SimonState::exemplar()->getContactInformationContainer());
	
	processSceneDescription();

	mGraphicContext->finishFrame();
	mRIBContext->finishFrame();

	watch.fillUp(1000/scene.getFramerate());

}

/**
* \brief Keyboard-Methode; Abfrage der Tastatureingabe
*/
bool VisualSimon::keyboard(unsigned char key, int /*x*/, int /*y*/){

	switch(key){

	case 'q' :
		puts("Bye Bye");
		exit(1);
		break;

	case 'w'://Ein-Ausschalten MWireframe
		mGraphicContext->useWireframe(!mGraphicContext->useWireframe());

		glutPostRedisplay();
		break;

	case 'r'://Zurücksetzen des Parameter
		cout << "Resetting all Values" << endl;
		mPhi = 45;
		mTheta = 45;
		mViewRadius = 5;
		mTranslateX = 0.0;
		mTranslateY = 0.0;
		glutPostRedisplay();
		break;
	}
	
	return true;
}


void VisualSimon::mouseMotion(int x, int y){

	GLdouble xVariation = mMouseSensibilityX * (x - mOldX),
		yVariation = mMouseSensibilityY * (y - mOldY);
                                                                                
	if (mLeftButton) {
		if (xVariation > 0) {
			if (mPhi < 2 * M_PI)
				mPhi += xVariation;
			else
				mPhi = 0.001;
		}
		else {
			if (mPhi > 0)
				mPhi += xVariation;
			else
				mPhi = 2 * M_PI + 0.001;
		}
		if (yVariation > 0) {
			if (mTheta < M_PI - 0.1)
				mTheta += yVariation;
		}else {
			if (mTheta > 0.1)
				mTheta += yVariation;
		}
                                                                                
		mOldX = x;
		mOldY = y;
	}
                                                                                
	if (mRightButton) {
		mViewRadius += yVariation * mZoomSensibility;
	}
                                                                                
	if (mMiddleButton) {
		mTranslateX = 10 * mMouseSensibilityX * (x - mOldX);
		mTranslateY = 10 * mMouseSensibilityY * (mOldY - y);
	}
}


void VisualSimon::mouse(int button, int /*state*/, int x, int y){

	if (button == GLUT_LEFT_BUTTON) {
		mLeftButton = 1;
		mOldX = x;
		mOldY = y;
	}
	else {
		mLeftButton = 0;
	}
	if (button == GLUT_RIGHT_BUTTON) {
		mRightButton = 1;
		mOldX = x;
		mOldY = y;
	}
	else {
		mRightButton = 0;
	}
	if (button == GLUT_MIDDLE_BUTTON) {
        mMiddleButton = 1;
        mOldX = x;
        mOldY = y;
	}
	else {
		mMiddleButton = 0;
	}
}

void VisualSimon::drawContacts(ContactInformationContainerPtr cic){

	for (int i = 0; i < cic->getNumOfContactInformations(); ++i){
		
		ContactInformationContainer::ContactInformation cp
			= cic->getContactInformation(i);

		mGraphicContext->drawSphere(cp.position, 10, GraphicsBase::materialGlass);

//  		cout << "CONTACT " 
//  			 << "pos: " << cp.position 
// 			 << "  obA: " << it->objectA
// 			 << "  obB: " << it->objectB
// 			 << "  impulse: " << it->impulse
// 			 << "  grind: " << it->grinding
//  			 << endl;
	}

	SimonState::exemplar()->getContactInformationContainer()->clear();
}

void VisualSimon::processSceneDescription(){
	
	int length = scene.objectVector.size();
	for (int i = 0; i < length; ++i){
		scene.objectVector[i]->getRigidBody()
			->addForce(scene.objectVector[i]->getSteadyForce());
		scene.objectVector[i]->draw(mGraphicContext);
		scene.objectVector[i]->draw(mRIBContext);
	}

/* How drawing is nice!

	for_each(scene.objectVector.begin(), 
			 scene.objectVector.end(),
			 bind2nd(mem_fun(&ObjectDescription::draw), mGraphicContext));
*/
}


int main(int argc, char** argv){

	GlutApplication::init(&argc, argv);

	if (argc != 2){
		cout << "Please specify a scene file name." << endl;
		exit(1);
	}
 
	GlutWindow* mainWin = new VisualSimon(argv[1]);
	mainWin->setSize(500,500);
 
	GlutApplication::run();

	return 0;


}
