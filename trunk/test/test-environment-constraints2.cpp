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


// $Id: test-environment-constraints2.cpp,v 1.29 2004/12/15 11:52:38 alangs Exp $

#include "test-environment.h"
#include "TestEnvironment.h"
#include <stdio.h>
//template <unsigned int N> bool choleskyFactorizationColumns (Matrix <float, N, N> & a);
//template <unsigned int N> bool choleskyFactorizationRows (Matrix <float, N, N> & a);
//
//template <unsigned int N, unsigned int M> bool choldc (Matrix <float, N, M> & a,
//    Matrix <float, N, 1> & p);

using namespace std;

/**
 * \brief Loop-Funktion der Testumgebung
 *
 * Diese Funktion wird direkt von display() aufgerufen. Hier
 * sollte alles reingeschrieben werden, was für die einzelnen
 * Tests nötig ist.
 */
TEFUNC void displayLoop() {

  // Beispiel: Zeichne eine Kiste
  //glutSolidCube(0.7);

}


/**
 * \brief Start-Funktion der Testumgebung
 *
 * Diese Funktion wird einmal beim Starten der Testumgebung 
 * aufgerufen. Hier sollte alles reingeschrieben werden, 
 * was für die initialisierung der einzelnen Tests nötig ist.
 */
TEFUNC void initialize(int /*argc*/, char** /*argv*/) {


  // Nils Hornung, Tests fuer Cholesky
  Matrix2x1 p;
  Clock performance;
  Matrix2x2 a;
  bool successful;
  performance.start ();
  for (int i=0; i<1000000; ++i) {
    a [0][0] = 2.0f;
    a [0][1] = -2.0f;
    a [1][0] = -2.0f;
    a [1][1] = 5.0f;
    successful =  choleskyFactorizationColumns (a);
  }
  performance.stop ();
  cout << a << endl;
  cout << performance << endl << endl;
  performance.start ();
  for (int i=0; i<1000000; ++i) {
    a [0][0] = 2.0f;
    a [0][1] = -2.0f;
    a [1][0] = -2.0f;
    a [1][1] = 5.0f;
    successful = choldc (a, p);
  }
  performance.stop ();
  cout << a << endl << p << endl;
  cout << performance << endl << endl;

  Matrix2x2 inv;
  a [0][0] = 2.0f;
  a [0][1] = -2.0f;
  a [1][0] = -2.0f;
  a [1][1] = 5.0f;
  cout << "bis hierher gekommen...\n";
  performance.start ();
  for (int i=0; i<1000000; ++i) {
    inv = choleskyInverse (a);
  }
  performance.stop ();
  cout << inv  << endl;
  cout << performance << endl << endl;
  performance.start ();
  for (int i=0; i<1000000; ++i) {
    inv = choleskyInverse (a);
  }
  performance.stop ();
  cout << inv  << endl;
  cout << performance << endl << endl;
  
    
  
  // Nils Hornung, Tests fuer Primary-Constraint bzw. BallAndSocketConstraint

/*
  RigidBodyPtr body1(new RigidBody());
  RigidBodyPtr body2(new RigidBody());
  
  body1->setPosition (Vector3 <float> (1, 10, 0));
  body2->setPosition (Vector3 <float> (-1, 10, 0));

  body1->setOrientation (Quaternion (0, 0, 0));
  body2->setOrientation (Quaternion (0, 0, 0));

  body1->setVelocity (Vector3 <float> (0, 0, 0));
  body2->setVelocity (Vector3 <float> (0, 0, 0));
  body1->setAngularVelocity (Vector3 <float> (0, 0, 0));
  body2->setAngularVelocity (Vector3 <float> (0, 0, 0));
  body1->setForce (Vector3 <float> (0, 0, 1));
  body2->setForce (Vector3 <float> (0, 0, 1));
  body1->setTorque (Vector3 <float> (1, 0, 0));
  body2->setTorque (Vector3 <float> (1, 0, 0));
  body1->setMass (2);
  body2->setMass (4);

  cout << "Universitaet Koblenz, FB4, AG Computergraphik\n\n";
  cout << "Projektpraktikum ---MARIONETTE---\n";
  cout << "Autor: Nils Hornung\nDatum: 26. Mai 04\n\n\n";
  cout << "TEST FUER PRIMARY_CONSTRAINT UND BALL_AND_SOCKET_CONSTRAINT\n\n\n";
  cout << "Es wurden zwei RogidBody-Objekte angelegt\n";
  cout << "  mit den folgenden Eigenschaften:\n\n";
  cout << "Position:        (1, 10, 0), (-1, 10, 0)\n";
  cout << "Orientierung:    (0, 0,  0),  (0, 0, 0)\n";
  cout << "Geschwindigkeit: (0, 0,  0),  (0, 0, 0)\n";
  cout << "Winkelgeschw.:   (0, 0,  0),  (0, 0, 0)\n";
  cout << "Kraft:           (0, 0,  1),  (0, 0, 1)\n";
  cout << "Drehmoment:      (1, 0,  0),  (1, 0, 0)\n\n";

  cout << "body1:\n";
  cout << &body1 << endl;
  cout << endl;
  cout << "body2:\n";
  cout << &body2 << endl;
  cout << endl;

  cout << "Es wird ein BAC-Constraint-Objekt zwischen diesen RigidBodys";
  cout << "angelegt\n";

  //BallAndSocketConstraint bac (&body1, &body2,
  //                        Matrix <float> (Vector3 <float> (-1, 0, 0)),
  //                        Matrix <float> (Vector3 <float> (1, 0, 0)));

  ConstraintSystem constraintSystem;
  constraintSystem.createBallAndSocketConstraint (Id (Id::typeBallJoint, 1), body1, body2,
                            Vector3 <float> (-1, 0, 0),
                            Vector3 <float> (1, 0, 0));
  BallAndSocketConstraint bac =
	  * constraintSystem.getBallAndSocketConstraint (Id (Id::typeBallJoint, 1));

  cout << "Es wurde ein BAC-Constraint-Objekt zwischen diesen RigidBodys";
  cout << "angelegt\n";
  cout << "  mit folgender Eigenschaft\n\n";
  cout << "lok. Position:   (-1, 0, 0),   (1, 0, 0)\n\n";

  cout << "StepSize fuer die partielle Ableitung wird gesetzt";
  cout << " (setStepSize (0.1))\n";
  bac.setStepSize (0.1);
  cout << "StepSize fuer die partielle Ableitung wurde auf 0.1 gesetzt\n\n";

  cout << "Constraint-Vektor wird berechnet (computeConstraint ())\n\n";
  cout << "  R1 lokalePos1 + x1 - (R2 lokalePos2 + x2)\n\n";
  (&bac)->computeConstraint ();
  cout << "Constraint-Vektor wurde berechnet\n\n";
  cout << "Constraint:\n";
  bac.getConstraint ().show ();
  cout << endl;
  cout << "ConstraintDot-Vektor wurde berechnet:\n\n";
  bac.getConstraint ().show ();
  cout << endl;
  
  cout << "Jacobian (1 und 2) und JacobianDot (1 und 2) werden berechnet\n";
  cout << "  (computeJacobian ())\n\n";
  cout << "  Partielle Ableitung von Constraint[Dot] nach spacePos\n\n";
  (&bac)->computeJacobian ();
  cout << "Jacobian (1 und 2) und JacobianDot (1 und 2) wurden berechnet\n\n";
  cout << "Jacobian1\n";
  bac.getJacobian1 ().show ();
  cout << endl;
  cout << "Jacobian2:\n";
  bac.getJacobian2 ().show ();
  cout << endl;
  cout << "Jacobian1Dot:\n";
  bac.getJacobian1Dot ().show ();
  cout << endl;
  cout << "Jacobian2Dot:\n";
  bac.getJacobian2Dot ().show ();
  cout << endl;

  cout << "RightHandSide (1 und 2) werden berechnet\n";
  cout << "  (computeRightHandSide ())\n\n";
  cout << "  RightHandSide = - JDot*spaceVel - J*spaceM^(-1)*spaceF\n\n";
  (&bac)->computeRightHandSide ();
  cout << "RightHandSide (1 und 2) wurden berechnet\n\n";
  cout << "RightHandSide1\n";
  bac.getRightHandSide1 ().show ();
  cout << endl;
  cout << "RightHandSide2\n";
  bac.getRightHandSide2 ().show ();
  cout << endl;
*/
 /* 
  constraintSystem.buildGraphs();
  constraintSystem.printGraphs ();*/

  cout << "END-OF-TESTS----------------------------------------------------\n";

/*
	//Create test matrixs/vectors
	Matrix<float> ma(1,1);
	Vector3<float> ve(1.0,2.0,3.0);
	Vector3<float> ve2(4.0,5.0,6.0);
	Vector3<float> ve3(0.0,0.0,0.0);
	Vector3<float> ve4(0.0,0.0,0.0);
	//create Id Values
	Id Ident1;
	Id Ident2;
	Id Ident3;
	Id Ident4;
	Id Ident5;
	Id Ident6;
	Id Ident7;
	Id Ident8;
	Id Ident9;
	Id Ident10;
	Id Ident11;
	Ident1.setType(1);
	Ident2.setType(1);
	Ident3.setType(1);
	Ident4.setType(0);
	Ident5.setType(0);
	Ident6.setType(0);
	Ident7.setType(0);
	Ident8.setType(0);
	Ident9.setType(0);
	Ident10.setType(1);
	Ident11.setType(1);
	Ident1.setNumber(1);
	Ident2.setNumber(2);
	Ident3.setNumber(3);
	Ident4.setNumber(1);
	Ident5.setNumber(2);
	Ident6.setNumber(3);
	Ident7.setNumber(4);
	Ident8.setNumber(5);
	Ident9.setNumber(6);
	Ident10.setNumber(4);
	Ident11.setNumber(5);
	
	//Create Rigid Bodys
	RigidBody myBody1;
	RigidBody myBody2;
	RigidBody myBody3;
	RigidBody myBody4;
	RigidBody myBody5;
	RigidBody myBody6;
	myBody1.setId(Ident4);
	myBody2.setId(Ident5);
	myBody3.setId(Ident6);
	myBody4.setId(Ident7);
	myBody5.setId(Ident8);
	myBody6.setId(Ident9);
	//set body values
	myBody1.setPosition(ve);
	myBody2.setPosition(ve2);
	//create constraint systemst
	ConstraintSystem myConstraintSystem;
	//create constraints
	myConstraintSystem.createBallAndSocketConstraint(Ident1, &myBody1, &myBody2, ma, ma);
	myConstraintSystem.createBallAndSocketConstraint(Ident2, &myBody2, &myBody3, ma, ma);
	//myConstraintSystem.createBallAndSocketConstraint(Ident3, &myBody2, &myBody4, ma, ma);
	
	myConstraintSystem.createBallAndSocketConstraint(Ident10, &myBody4, &myBody5, ma, ma);
	myConstraintSystem.createBallAndSocketConstraint(Ident11, &myBody5, &myBody6, ma, ma);
	
	myConstraintSystem.buildGraphs();
	myConstraintSystem.printGraphs();
*/
	
	
	
	/*BallAndSocketConstraint* myBASC1 = myConstraintSystem.getBallAndSocketConstraint(Ident1);
	ve3 = myBASC1->getObjectA()->getPosition();
	ve4 = myBASC1->getObjectB()->getPosition();
	printf("Vector A: %f %f %f\n",ve3[0],ve3[1],ve3[2]);
	printf("Vector B: %f %f %f\n",ve4[0],ve4[1],ve4[2]);*/
/*
  cout << "\n----------------NEW-SECTION----------------\n";
  cout << "Gausssche Elimination, Test1 nach Baraff\n\n";

  Matrix <float> aMatrix (3,3);
  Matrix <float> xVector (3,1);
  Matrix <float> bVector (3,1);
*/
/*aMatrix [0][0] = 10;
  aMatrix [0][1] = 20;
  aMatrix [0][2] = 30;
  aMatrix [1][0] = 20;
  aMatrix [1][1] = 45;
  aMatrix [1][2] = 80;
  aMatrix [2][0] = 30;
  aMatrix [2][1] = 80;
  aMatrix [2][2] = 171; */
/*
  aMatrix [0][0] = 10;
  aMatrix [0][1] = 0;
  aMatrix [0][2] = 30;
  aMatrix [1][0] = 0;
  aMatrix [1][1] = 5;
  aMatrix [1][2] = 20;
  aMatrix [2][0] = 30;
  aMatrix [2][1] = 20;
  aMatrix [2][2] = 0;

  cout << "Diese Matrix soll faktorisiert werden (LDL^t):\n";
  aMatrix.show ();

  // O(n^3) factor nach Baraff 1996 auf reinen Zahlen!
  for (int i=0; i<3; i++) {
    for (int k=i-1; k>=0; k--) {
      aMatrix[i][i] = aMatrix[i][i] - 
                      aMatrix[k][i] * aMatrix[k][k] * aMatrix [k][i];
      
    }
    for (int j=i+1; j<3; j++) {
      for (int k=i-1; k>=0; k--) {
        aMatrix[i][j] = aMatrix[i][j] -
                        aMatrix[k][i] * aMatrix[k][k] * aMatrix [k][j];
      }
      aMatrix[i][j] = (1.0f / aMatrix[i][i]) * aMatrix[i][j];
    }
  }
*/
/*
  // O(n^3) factor nach Baraff 1996 auf reinen Zahlen!
  for (int i=0; i<3; i++) {
    for (int k=i-1; k>=0; k--) {
      aMatrix[i][i] = aMatrix[i][i] -
                      aMatrix[i][k] * aMatrix[k][k] * aMatrix [i][k];
    }
    for (int j=i+1; j<3; j++) {
      for (int k=i-1; k>=0; k--) {
        aMatrix[i][j] = aMatrix[i][j] -
                        aMatrix[k][i] * aMatrix[k][k] * aMatrix [k][j];;
      }
      aMatrix[j][i] = (1.0f / aMatrix[i][i]) * aMatrix[j][i];
    }
  }
*/

/*  cout << "...und das ist das Ergebnis\n";
  aMatrix.show ();

  Matrix <float> uMatrix (3,3);
  Matrix <float> dMatrix (3,3);

  uMatrix [0][0] = 1.0;
  uMatrix [1][1] = 1.0;
  uMatrix [2][2] = 1.0;
  uMatrix [0][1] = aMatrix [0][1];
  uMatrix [0][2] = aMatrix [0][2];
  uMatrix [1][2] = aMatrix [1][2];

  dMatrix [0][0] = aMatrix [0][0];
  dMatrix [1][1] = aMatrix [1][1];
  dMatrix [2][2] = aMatrix [2][2];

  Matrix <float> lMatrix = uMatrix.T();

  Matrix <float> testMatrix = lMatrix * dMatrix * lMatrix.T();

  cout << "Und das sollte wieder die urspruengliche Matrix sein,";
  cout << " berechnet aus LDL^t\n";
  testMatrix.show ();

  cout << "\n----------------------------------\n";
  cout << "Loesung nach Gulob... inhaerent Baraffs Methode!\n\n";

  xVector [0][0] = -3;
  xVector [1][0] = -4;
  xVector [2][0] = 1;

  bVector = testMatrix * xVector;
  //bVector [0][0] = -3;
  //bVector [1][0] = -4;
  //bVector [2][0] = 1;
  xVector = 0;

  cout << "bVector:\n";
  bVector.show ();
  cout << "lMatrix:\n";
  lMatrix.show ();
  cout << "dMatrix:\n";
  dMatrix.show ();

  // Loesung nach Gulob und van Loan
  for (int i=0; i<3; i++) {
    xVector [i][0] = bVector [i][0];
    for (int j=0; j<i; j++) {
      xVector [i][0] -= aMatrix [j][i] * xVector [j][0];
    }
  }
  for (int i=0; i<3; i++) {
    xVector [i][0] /= aMatrix [i][i];
  }
  for (int i=2; i>=0; i--) {
    for (int j=i+1; j<3; j++) {
      xVector [i][0] -= aMatrix [i][j] * xVector [j][0];
    }
  }

  cout << "Ergebnis (sollte sein: -3, -4, 1):\n";
  xVector.show ();

  cout << "\n-------------------------------\n";
  cout << "Gausssche Elimination, Test2 nach Golub und van Loan\n\n";

  // Just to remind you...
  //Matrix <float> aMatrix (3,3);
  //Matrix <float> xVector (3,1);
  //Matrix <float> bVector (3,1);

  aMatrix [0][0] = 10;
  aMatrix [0][1] = 20;
  aMatrix [0][2] = 30;
  aMatrix [1][0] = 20;
  aMatrix [1][1] = 45;
  aMatrix [1][2] = 80;
  aMatrix [2][0] = 30;
  aMatrix [2][1] = 80;
  aMatrix [2][2] = 171;

  cout << "Diese Matrix soll faktorisiert werden (LDL^t):\n";
  aMatrix.show ();

  // O(n^3) factor nach Gulob und van Loan "Matrix Computations", S. 139!
  for (int i=0; i<3; i++) {
    for (int j=0; j<i; j++) {
      aMatrix [i][i] -= aMatrix [i][j] * aMatrix [j][j] * aMatrix [i][j];
    }
    for (int j=i+1; j<3; j++) {
      for (int k=0; k<i; k++) {
        aMatrix [j][i] -= aMatrix [j][k] * aMatrix [k][k] * aMatrix [i][k];
      }
      aMatrix [j][i] *= (1.0 / aMatrix [i][i]);
    }
  }

  cout << "...und das ist das Ergebnis\n";
  aMatrix.show ();

  //Matrix <float> lMatrix (3,3);
  //Matrix <float> dMatrix (3,3);

  lMatrix [0][0] = 1.0;
  lMatrix [1][1] = 1.0;
  lMatrix [2][2] = 1.0;
  lMatrix [1][0] = aMatrix [1][0];
  lMatrix [2][0] = aMatrix [2][0];
  lMatrix [2][1] = aMatrix [2][1];

  dMatrix [0][0] = aMatrix [0][0];
  dMatrix [1][1] = aMatrix [1][1];
  dMatrix [2][2] = aMatrix [2][2];

  //lMatrix = lMatrix.T();
  testMatrix = lMatrix * dMatrix * lMatrix.T();

  cout << "Und das sollte wieder die urspruengliche Matrix sein,";
  cout << " berechnet aus LDL^t\n";
  testMatrix.show ();

  cout << "\n----------------------------------\n";
  cout << "Loesung nach Gulob...\n\n";

  uMatrix = lMatrix.T ();

  xVector [0][0] = 1;
  xVector [1][0] = 5;
  xVector [2][0] = 3;

  bVector = testMatrix * xVector;
  xVector = 0;

  cout << "bVector:\n";
  bVector.show ();
  cout << "lMatrix:\n";
  lMatrix.show ();
  cout << "dMatrix:\n";
  dMatrix.show ();

  // Loesung nach Gulob und van Loan
  for (int i=0; i<3; i++) {
    xVector [i][0] = bVector [i][0];
    for (int j=0; j<i; j++) {
      xVector [i][0] -= aMatrix [i][j] * xVector [j][0];
    }
  }
  for (int i=0; i<3; i++) {
    xVector [i][0] /= aMatrix [i][i];
  }
  for (int i=2; i>=0; i--) {
    for (int j=i+1; j<3; j++) {
      xVector [i][0] -= aMatrix [j][i] * xVector [j][0];
    }
  }

  cout << "Ergebnis (sollte sein: 1, 5, 3):\n";
  xVector.show ();
*/
}

TEFUNC void keyHandler(unsigned char /*key*/) {
	;
}

#ifdef OS_WINDOWS
const TestEnvironment testEnvironmentConstraints2(initialize,displayLoop,keyHandler);
#endif
