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


//------------------------------------------------------------------------------
/**
 * $Author: trappe $
 * $Date: 2004/12/15 17:04:50 $
 * $Revision: 1.5 $
 * \brief Dieses Programm stellt Testfunktionen für die Quaternionen bereit.
 *
 * Funktion test_rotation_euler() testet den Euler-Konstruktor, Matrix-Konstruktor, getEulerRotation, 
 * getRotationMatrix auf Konsistenz
 * 
 */
//------------------------------------------------------------------------------

#include <simon/config.h>
#include <simon/Quaternion.h>
#include <simon/Vector3.h>
#include <simon/Matrix.h>
#include <iostream>
#include <math.h>
#include <assert.h>

using namespace std;

/*
//check if two matrixes - of the same size - are equal or nearly equal
bool isDifferent(Matrix<float> one, Matrix<float> two) {


	assert (one.getSizeM() == two.getSizeM());
	assert (one.getSizeN() == two.getSizeN());
	Matrix<float> diffmatrix = one - two;
	float epsilon = 0.01;//Maximale Differenz für die Einzelwerte
	for (unsigned int i = 0; i < diffmatrix.getSizeM();i++) {
		for (unsigned int j = 0; j < diffmatrix.getSizeN();j++) {
			if (abs(diffmatrix[i][j]) > epsilon) {
				return true;
			}
		}
	}
	return false;

}

*/
/*
//Berechnet eine Rotationsmatrix Rz*Ry*Rx, wobei R? eine Rotationsmatrix für
//Rotation um x oder y oder z ist.
Matrix<float> computeRotationMatrix(float radx, float rady, float radz) {
	Matrix<float> Rx(3,3);
			Matrix<float> Ry(3,3);
				Matrix<float> Rz(3,3);
				
				Rx[0][0] = 1.0;
				Rx[1][1] = cos(radx);
				Rx[1][2] = -1.0*sin(radx);
				Rx[2][1] = sin(radx);
				Rx[2][2] = cos(radx);
				
				Ry[1][1] = 1.0;
				Ry[0][0] = cos(rady);
				Ry[2][0] = -1.0*sin(rady);
				Ry[0][2] = sin(rady);
				Ry[2][2] = cos(rady);
				
				Rz[2][2] = 1.0;
				Rz[0][0] = cos(radz);
				Rz[1][0] = sin(radz);
				Rz[0][1] = -1.0*sin(radz);
				Rz[1][1] = cos(radz);
				
	return Rz * Ry * Rx;
}
*/
/*
Testet folgende Funktionen auf Konsistenz:
-Euler-Konstruktor
-RotationsMatrix-Konstruktor
-getRotationMatrix
-getEulerRotation
-getAxisAngle

Dabei wird folgendes gemacht:
-Schleife über alle drei Achsen, von -360 bis + 360 Grad
-Pro Durchlauf:
--Lege Quaternion aus aktuellen Winkelwerten an
--Hole RotationsMatrix, Euler-Winkel und Axis-Angle aus dem Quaternion
--Berechne Quaternion aus obiger RotationsMatrix und rechne das ganze wieder zurück
--Berechne Rotationsmatrix für Original- und Ergebnis-Euler-Werte "per Hand"
--Vergleiche die vier Matrizen
*/
/*
void test_rotation_euler()	
{
	float radx, rady, radz;//Winkel im Bogenmass
	Vector3<float> axis; //Drehachse
	float qx,qy,qz,qw,q2x,q2y,q2z,q2w, angle;//Quaternion-Elemente und Drehwinkel
	int count = 0;//Anzahl der zu großen Abweichungen
	int max = 0;//Gesamtanzahl der Durchläufe
	bool showall = true;//Immer alles ausgeben ?
	int stepsize = 10; //Inkrement für das durchgehen der Winkel
	
	for (int i = -360; i < 360; i+=stepsize) {
		for (int j = -360; j < 360; j+=stepsize) {
			for (int k = -360; k < 360; k+=stepsize) {
				
				//DEG TO RAD
				radz = ((float)k/180.0)*M_PI;
				radx = ((float)i/180.0)*M_PI;
				rady = ((float)j/180.0)*M_PI;
				
				//Quaternionen anlegen
				Quaternion q(radx,rady,radz);
				
				//Werte aus Quaternion holen
				Vector3<float> qeuler = q.getEulerRotation();
				Matrix<float> qmatrix(qeuler);
				Matrix<float> qrotation = q.getRotationMatrix();
				q.getValues(qw,qx,qy,qz);

				//Test des Matrix-Konstruktors
				Quaternion q2(qrotation);
				q2.getValues(q2w, q2x, q2y, q2z);
				Matrix<float> test = q2.getRotationMatrix();

				//Umrechnung in Axis-Angle
				q.getAxisAngle(axis, angle);

				
				//RAD TO DEG
				Matrix<float> qematrix(3,1);
				qematrix[0][0] = (qmatrix[0][0] / M_PI) * 180.0;
				qematrix[1][0] = (qmatrix[1][0] / M_PI) * 180.0;
				qematrix[2][0] = (qmatrix[2][0] / M_PI) * 180.0;
				angle = (angle / M_PI) * 180.0;
				
				//Rotationsmatrizen nachrechnen "per Hand"
				Matrix <float> control = computeRotationMatrix(radx, rady, radz);
				Matrix<float> result = computeRotationMatrix(qmatrix[0][0], qmatrix[1][0], qmatrix[2][0]);
				
				
				//falls Ausgabe aktiviert oder eine Abweichung: Ausgeben
				if (isDifferent(qrotation, control) || isDifferent(result, control) || isDifferent(test, control) ||showall) {
					cout << "Drehung um " << i << " " << j << " " << k << endl;
					cout << "Axis: " << axis[0] << " " << axis[1] << " " << axis[2] << "\n" << "Angle: " << angle << endl;
					cout << "Quaternionenwerte: "  << qw << " " <<  qx << " " << qy << " " << qz << endl;
					cout << "Quat.-werte aus Matrix-Konstruktor: "  << q2w << " " <<  q2x << " " << q2y << " " << q2z << endl;
					cout << "Bogenwerte input " << radx << " " << rady << " " << radz << endl << endl;
					cout << "Bogenwerte Output " << endl;
					qmatrix.show();
					cout << "Gradwerte Output: " << endl;
					qematrix.show();
					cout << "Quaternion Matrix: " << endl;
					qrotation.show();
					cout << "Kontroll Matrix: " << endl;
					control.show();
					cout << "Matrix aus Euler-Winkeln: " << endl;
					result.show();
					cout << "Aus Matrix-Konstruktor zurueckgerechnet: " << endl;
					test.show();
					cout << "----------------------------" << endl;
					count ++;
				}
				max ++;
			}
			//cout << "Zwischenstand \"Anzahl der Berechnungen\" " << max << endl;
		}
	}
	cout << "Counter  \"Anzahl der Fehler\": " << count << " / " << max << endl;
}
*/
/*
-rotiert einen beliebigen Ortsvektor (Rotation als axis-angle)
-testet qRotate (und damit auch Quaternionen-Mutiplikation und -Invertierung)
-zum Verwenden entsprechend in main() umkommentieren
*/
/*
void test_rotation(){

	
	Vector3<float> v, axis, ergebnis;
	float angle;
	char repeat;

	while (repeat != 'n'){

	cout << "Den zu rotierenden Ortsvektor eingeben :\n";
	cout << " X= ";
	cin >> v[0];
	cout << " Y= ";
	cin >> v[1];
	cout << " Z= ";
	cin >> v[2];
	cout << "\nRotationsachse als Vektor :\n";
	cout << " X= ";
	cin >> axis[0];
	cout << " Y= ";
	cin >> axis[1];
	cout << " Z= ";
	cin >> axis[2];
	axis.normalize();
	cout << "\nWinkel in Grad :\n";
	cin >> angle;
	angle = (float)((angle/180.0) * M_PI);


	Quaternion q1(angle, axis);
	cout << "\nDas zugehoerige Rotationsquaternion: \n\n";
	q1.print();

	Matrix<float> rotMat = q1.getRotationMatrix();
	cout << "\nDie zugehoerige RotationsMatrix: \n\n";
	rotMat.show();


	ergebnis = qRotate(v, q1);
	cout <<"\nDer rotierte Ortsvektor: \n\n";
	cout << ergebnis[0] << "\n" << ergebnis[1] << "\n" << ergebnis[2] << "\n";

	

	cout << "\nnochmal (Y/N) ?\n";
	cin >>	repeat;
	}

}
*/

int main(int /*argc*/, char ** /*argv*/)
{
//test_rotation_euler();
	//test_rotation();
}

