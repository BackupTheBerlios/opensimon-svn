%{
#include <iostream>
#include <cstdlib>
#include <cstdio>

#include <map>
#include <simon/simon.h>
#include "SceneDescription.h"

using namespace std;

extern FILE* yyin;
extern unsigned int lineNo;

extern SceneDescription *scene;
extern bool parse(const char* filename, SceneDescription *scene);
extern int yylex();

void yyerror(const char* msg);

unsigned int numSphere;
unsigned int numPlane;
unsigned int numBox;

unsigned int numBallJoint;

RigidBodyPtr rigidBody;
GeometryPtr geometry;
BallAndSocketConstraintPtr ballJoint;

map<unsigned int, Id> idConvertMap;

SceneDescription *sceneDescription;

%}

%union {
	float real;
	int   integer;
}


%token GLOBAL
%token SPHERE PLANE BOX CAPSULE
%token BALLJOINT
%token GRAVITY CONTACTTHRESHOLD NOCOLLISION 
%token INTEGRATIONINTERVALL INTEGRATIONSPERFRAME FRAMERATE
%token ANGULARVISCOSITYSLOWDOWN LINEARVISCOSITYSLOWDOWN 
%token CENTEROFINTEREST CAMERAPOSITION
%token STEADYFORCE
%token CONNECT ANCHOR_A ANCHOR_B
%token POSITION MASS IDNUM ORIENTATION FIXED FORCE
%token RADIUS SCALE BOUNCINESS FRICTION HEIGHT
%token <real> REAL
%token <integer> INTEGER

%type <real> NUMBER


%%

File			: ObjectList
				;

ObjectList		: /* Epsilon */
				| ObjectList Object
				;

Object			: Global
				| Sphere
				| Plane
				| Box
				| Capsule
				| Balljoint
				;

Global				: GLOBAL '{'
					GlobalAttributeList '}'
					;

GlobalAttributeList : /* Epsilon */
					| GlobalAttributeList GlobalAttribute
					;

GlobalAttribute		: GRAVITY NUMBER NUMBER NUMBER
					{
						SimonState::exemplar()->setGravityVector(Vec3($2,$3,$4));
					}
					| CONTACTTHRESHOLD NUMBER
					{
						SimonState::exemplar()->setContactInformationThreshold($2);
					}
					| ANGULARVISCOSITYSLOWDOWN NUMBER
					{
						SimonState::exemplar()->setViscositySlowdownAngular($2);
					}
					| LINEARVISCOSITYSLOWDOWN NUMBER
					{
						SimonState::exemplar()->setViscositySlowdownLinear($2);
					}
					| NOCOLLISION
					{
						sceneDescription->testCollisions(false);
					}
					| INTEGRATIONINTERVALL NUMBER
					{
						SimonState::exemplar()->setIntegrationIntervall($2);
					}
					| INTEGRATIONSPERFRAME INTEGER
					{
						sceneDescription->setIntegrationsPerFrame($2);
					}
					| FRAMERATE INTEGER
					{
						sceneDescription->setFramerate($2);
					}
					| CAMERAPOSITION NUMBER NUMBER NUMBER
					{
						sceneDescription->setCameraPosition(Vec3($2,$3,$4));
					}
					| CENTEROFINTEREST NUMBER NUMBER NUMBER
					{
						sceneDescription->setCenterOfInterest(Vec3($2,$3,$4));
					}
					;

RigidBodyAttribute	: POSITION NUMBER NUMBER NUMBER
					{
						rigidBody->setPosition(Vec3($2, $3, $4));
					}
					| MASS NUMBER
					{
						rigidBody->setMass($2);
					}
					| IDNUM INTEGER
					{
						idConvertMap.insert(std::pair<unsigned int, Id>(static_cast<unsigned int>($2), rigidBody->getId()));
					}
					| ORIENTATION NUMBER NUMBER NUMBER NUMBER
					{
						rigidBody->setOrientation(Quaternion(DEGTORAD($2), Vec3($3, $4, $5)));
					}
					| FIXED
					{
						rigidBody->setIsDynamicFlag(false);
					}
					| FORCE NUMBER NUMBER NUMBER
					{
						rigidBody->addForce(Vec3($2, $3, $4));
					}
					| STEADYFORCE NUMBER NUMBER NUMBER
					{
						sceneDescription->objectVector.back()->addSteadyForce(Vec3($2,$3,$4));
					}	
					;


GeometryAttribute   : BOUNCINESS NUMBER
					{
						geometry->setBounciness($2);
					}
					| FRICTION NUMBER
					{
						geometry->setFriction($2);
					}
					;

Sphere			: SPHERE '{'
				{
					rigidBody = SimonState::exemplar()->getBodySystem()
						->create(Id(Id::typeSphere,numSphere++));

			  		geometry = SimonState::exemplar()->getGeometrySystem()
						->createSphere(rigidBody,100);

					SphereDescription* sphere = new SphereDescription;
					sphere->setGeometry(boost::static_pointer_cast<Sphere>(geometry));
					sceneDescription->objectVector.push_back(sphere);

				}
				SphereAttributeList '}'
				;

SphereAttributeList	: /* Epsilon */
					| SphereAttributeList SphereAttribute
					;

SphereAttribute		: RigidBodyAttribute
					| GeometryAttribute
					| RADIUS NUMBER
					{
						boost::static_pointer_cast<Sphere>(geometry)->setRadius($2);
					}
					;

Capsule			: CAPSULE '{'
				{
					rigidBody = SimonState::exemplar()->getBodySystem()
						->create(Id(Id::typeCapsule,numSphere++));

			  		geometry = SimonState::exemplar()->getGeometrySystem()
						->createCapsule(rigidBody,100,200);

					CapsuleDescription* caps = new CapsuleDescription;
					caps->setGeometry(boost::static_pointer_cast<Capsule>(geometry));
					sceneDescription->objectVector.push_back(caps);

				}
				CapsuleAttributeList '}'
				;

CapsuleAttributeList : /* Epsilon */
					| CapsuleAttributeList CapsuleAttribute
					;

CapsuleAttribute	: RigidBodyAttribute
					| GeometryAttribute
					| RADIUS NUMBER
					{
						boost::static_pointer_cast<Capsule>(geometry)->setRadius($2);
					}
					| HEIGHT NUMBER
					{
						boost::static_pointer_cast<Capsule>(geometry)->setHeight($2);
					}
					;

Plane			: PLANE '{'
				{

					rigidBody = SimonState::exemplar()->getBodySystem()
						->create(Id(Id::typePlane,numPlane++));

			  		geometry = SimonState::exemplar()->getGeometrySystem()
						->createPlane(rigidBody);

					PlaneDescription* plane = new PlaneDescription;
					plane->setGeometry(boost::static_pointer_cast<Plane>(geometry));
					sceneDescription->objectVector.push_back(plane);
				}
				PlaneAttributeList '}'
				;

PlaneAttributeList	: /* Epsilon */
					| PlaneAttributeList PlaneAttribute
					;

PlaneAttribute		: RigidBodyAttribute
					| GeometryAttribute
					;

Box				: BOX '{'
				{

					rigidBody = SimonState::exemplar()->getBodySystem()
						->create(Id(Id::typeBox,numBox++));

			  		geometry = SimonState::exemplar()->getGeometrySystem()
						->createBox(rigidBody, Vec3(1,1,1));

					BoxDescription* box = new BoxDescription;
					box->setGeometry(boost::static_pointer_cast<Box>(geometry));
					sceneDescription->objectVector.push_back(box);

				}
				BoxAttributeList '}'
				;

BoxAttributeList	: /* Epsilon */
					| BoxAttributeList BoxAttribute
					;

BoxAttribute		: RigidBodyAttribute
					| GeometryAttribute
					| SCALE NUMBER NUMBER NUMBER
					{
						boost::static_pointer_cast<Box>(geometry)->setScale(Vec3($2,$3,$4));
					}
					;

ConnectionAttribute : /* Epsilon */
					;

Balljoint			: BALLJOINT '{' CONNECT NUMBER NUMBER ANCHOR_A NUMBER NUMBER NUMBER ANCHOR_B NUMBER NUMBER NUMBER
					{
						SimonState::exemplar()->getConstraintSystem()
						 ->createBallAndSocketConstraint(
							Id(Id::typeBallJoint, numBallJoint++),
							SimonState::exemplar()->getBodySystem()
							->getRigidBody(idConvertMap[$4]),
							SimonState::exemplar()->getBodySystem()
							->getRigidBody(idConvertMap[$5]),
							Vec3($7,$8,$9), Vec3($11,$12,$13)
						);	
					}
					BalljointAttributeList '}'
					;

BalljointAttributeList	: /* Epsilon */
						| BalljointAttributeList BalljointAttribute
						;

BalljointAttribute		: ConnectionAttribute
						;

NUMBER				: INTEGER
					{
						$$ = $1;
					}
					| REAL
					{
						$$ = $1;
					}
					;

%%

bool parse(const char* filename, SceneDescription *scene)
{

	sceneDescription = scene;
	
	yyin = fopen(filename, "rb");
	if (!yyin)
		yyerror("Error opening file!");

	lineNo = 1;
	numSphere = 0;
	numPlane = 0;
	numBox = 0;
	numBallJoint = 0;

	int result = yyparse();

	fclose(yyin);

	return (result == 0);
}

void yyerror(const char* msg)
{
	cerr << "Error in line " << lineNo << endl << "   " << msg << endl;
	exit(1);
}
