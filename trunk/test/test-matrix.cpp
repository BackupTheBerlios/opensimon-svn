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



//! Dieses Kleine Programm Testet die Matrix Klasse auf 
//! Herz und Nieren (noch Zukunftsmusik)

#include "Matrix.h"
#include <iostream>

using namespace std;

//static bool successfull = false;

int main() {

	cout << "Lege Matrix 1 an!" << endl;
	Matrix3x3 mA;
	cout << mA << endl;

	cout << "Überschreibe Matrix 1 mit initalisierungs Array!" << endl;
	float values1[] = {1,2,3,4,5,6,7,8,9};
	mA = Matrix3x3(values1);
	cout << mA << endl;

	Vec3 vectorA(1.0,-2,3);
	cout << "Erstelle aus einem Vector eine Matrix!" << endl;
 	Matrix3x1 mB(vectorA);
	cout << mB << endl;
	float values2[] = {1.0, -2, 3};
	if (mB != Matrix3x1(values2)) 
		exit(1);

	cout << "Fülle Matrix 1 mit 4en!" << endl;
	mA = 4;
	cout << mA << endl;
	float values3[] = {4,4,4,4,4,4,4,4,4};
	if (mA != Matrix3x3(values3)) 
		exit(1);

	cout << "Ersetze einige Zahlen Matrix 1!" << endl;
	mA[2][1] = 1;
	mA[1][1] = 2;
	mA[0][0] = 0;
	mA[0][2] = 0;
	mA[2][2] = 0;
	cout << mA << endl;
	float values4[] = {0,4,0,4,2,4,4,1,0};
	if (mA != Matrix3x3(values4)) 
		exit(1);

	
	cout << "Multipliziere Matrix 1 und 2!" << endl;
	Matrix3x1 mC;
	mC = mA * mB;
	cout << mC << endl;
	float values5[] = {-8,12,2};
	if (mC != Matrix3x1(values5)) 
		exit(1);

	cout << "Kopiere Ergebniss mit copy-consturctor in neues Objekt!" << endl;
 	Matrix3x1 mD(mC);
	cout << mD << endl;
	if (mD != Matrix3x1(values5)) 
		exit(1);

	cout << "Multipliziere Matrix 1 mit dem Vector " << Vec3(2,0,0) << " und speichere das Ergebniss in Matrix 2!" << endl;
	mB = mA * Vec3(2,0,0);
	cout << mB << endl;
	float values6[] = {0,8,8};
	if (mB != Matrix3x1(values6)) 
		exit(1);

	cout << "Greife auf das Element 2,1 zu!" << endl;
	cout << mB[1][0] << endl;

	cout << endl << endl << endl << "Alles hat funktionert!" << endl;
	exit(0);
}
