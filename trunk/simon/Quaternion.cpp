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



#include <simon/Quaternion.h>
#include <simon/config.h>

#include <math.h>
#include <iostream>

// f�r die FLT_EPSILON konstante
#include <float.h>

using namespace std;

//------------------------------------------------------------------------------
/**
 *  \class Quaternion
 *  \author Achilles, Baierl, Koehler
 *
 * \brief Quaternion-Klasse
 *
* Man benutzt Einheits(!!)-Quaternionen, um Rotationen zu berechnen. Ein Quaternion hat die Form q=(w, v),
* wobei w ein Skalar und v=(x, y, z) ein Vektor ist.
* Um z.B. einen Punkt P=(a, b, c) um einen Winkel angle und eine Achse axis zu rotieren, erstellt man sich
* aus dem Vektor P ein pures Quaternion p und aus axis und angle ein RotationsQuaternion q.
* Den Punkt P rotiert man nun indem man rechnet: qpq* (q* ist die Inverse von q).
*
* Um die Umrechnung bei Quaternionen (Eulerwinkel, Matrizen) konsistent zu halten, 
* muss man festlegen, welche Reihenfolge bei den Eulerwinkeln zu Grunde liegt.
* Daher _Konvention_ hier: Rz(phiz)*Ry(phiy)*Rx(phix), d.h. es wird erst um x rotiert, dann um y, dann um z. Und zwar
* jeweils um die * entsprechenden Winkel phix, phiy, phiz.
*
* Anmerkungen: Zu m�glichen Konventionen siehe "Ken Shoemake: Euler Angle Conversion", GraphicsGems IV, Kapitel III.5,
* Seiten 222-229. 
*
* Getestete Funktionen:
* -Euler-Konstruktor
* -getEulerRotation
* -getRotationMatrix
*
* \todo Andere Funktionen testen (mit Testprogramm)
*/

const Quaternion Quaternion::mIdentity;

/**
* \brief Default-Konstruktor
*/
Quaternion::Quaternion(){
	mqElement[0]	= 0.0;
	mqElement[1]	= 0.0;
	mqElement[2]	= 1.0;
	mqElement[3]	= 0.0;
}

/**
* \brief Konstruktor, der als Input die vier einzelnen Komponenten eines Quaternions hat
* \
*/
Quaternion::Quaternion(float w, float x, float y, float z){
	mqElement[0] = w;
	mqElement[1] = x;
	mqElement[2] = y;
	mqElement[3] = z;
}

/**
* \brief Konstruktor, der aus einem Winkel (angle im Bogenma�) und der Rotationsachse (axis) ein RotationsQuaternion erstellt. Die RotationsAchse wird normalisiert
*
* \param float angle Bogenma� der Drehung.
* \param Vec3 axis
*/
Quaternion::Quaternion(float angle, Vec3 axis){
	float tempSin = sinf(angle / 2.f);
	axis.normalize();
	mqElement[0] = cos(angle / 2.f);
	mqElement[1] = axis[0] * tempSin;
	mqElement[2] = axis[1] * tempSin;
	mqElement[3] = axis[2] * tempSin;

}

/**
* \brief Konstruktor, der als Input die Eulerwinkel hat (im Bogenma�)
* \param float phix, phiy, phiz (Eulerwinkel im Bogenma�)
*
* Konvention: Rz*Ry*Rx !!! (s.o.)
* Angepasster Code aus OpenSG Quaternionen... 
*
* Idee: Ein Quaternion repr�sentiert eine Rotation durch
* [cos(phi/2),sin(phi/2)*Rotationsachse]. Daher kann man die Eulerwinkel in Quaternionen umrechnen, indem man f�r
* jede Euler-Rotationsachse ein eigenes Quaternion erstellt, und diese Quaternionen multipliziert.
* Dabei ist f�r die Reihenfolge der Rotationen zu beachten: Q1*Q2*Q3 bedeutet, das zun�chst die Q3-Rotation angewendet 
* wird, danach Q2, danach Q1.
* Der Code dieser Funktion ist die ausformulierte Muliplikation der Quaternionen.
*/
Quaternion::Quaternion(float phix, float phiy, float phiz){
	
	float sx = sin(phix * 0.5f);
	float cx = cos(phix * 0.5f);

	float sy = sin(phiy  * 0.5f);
 	float cy = cos(phiy  * 0.5f);
	
	float sz = sin(phiz * 0.5f);
	float cz = cos(phiz * 0.5f);

	mqElement[0] = (cx * cy * cz) + (sx * sy * sz);
	mqElement[1] = (sx * cy * cz) - (cx * sy * sz);
	mqElement[2] = (cx * sy * cz) + (sx * cy * sz);
	mqElement[3] = (cx * cy * sz) - (sx * sy * cz);
    
}

/**
* \brief Konstruktor, der ein aus einem Vektor ein pures Quaternion erstellt
* \param Vec3 v (Vektor, z.B. ein Punkt, der rotiert werden soll)
* \todo Ahhhrg. Was mach das ding und wieso? bitte besser kommentieren oder entfernen.
*/
Quaternion::Quaternion(Vec3 v){
    mqElement[0] = 0;
	mqElement[1] = v[0];
	mqElement[2] = v[1];
	mqElement[3] = v[2];

}


/**
* \brief Konstruktor, der als Input eine Rotationsmatrix hat
* \todo Was macht der Kram. Bitte Einheiten und Vorgehen kurz kommentieren.
*/
Quaternion::Quaternion(Matrix3x3& m){

	float tr, s;

	tr = m[0][0] + m[1][1] + m[2][2];

	if (tr >= 0){
		s = sqrt(tr + 1);
		mqElement[0] = 0.5f * s;
		s = 0.5f / s;
		mqElement[1] = (m[2][1] - m[1][2]) * s;
		mqElement[2] = (m[0][2] - m[2][0]) * s;
		mqElement[3] = (m[1][0] - m[0][1]) * s;
	}

	else {
		int i = 0;
			
		if(m[1][1] > m[0][0])
			i = 1;
		if(m[2][2] > m[i][i])
			i = 2;

		switch (i){

		case 0:
			s = sqrt((m[0][0] - (m[1][1] + m[2][2])) + 1);
			mqElement[1] = 0.5f * s;
			s= 0.5f / s;
			mqElement[2] = (m[0][1] + m[1][0]) * s;
			mqElement[3] = (m[2][0] + m[0][2]) * s;
			mqElement[0] = (m[2][1] - m[1][2]) * s;
			break;

		case 1:
			s = sqrt((m[1][1] - (m[2][2] + m[0][0])) + 1);
			mqElement[2] = 0.5f * s;
			s = 0.5f / s;
			mqElement[3] = (m[1][2] + m[2][1]) * s;
			mqElement[1] = (m[0][1] + m[1][0]) * s;
			mqElement[0] = (m[0][2] - m[2][0]) * s;
			break;

		case 2:
			s = sqrt((m[2][2] - (m[0][0] + m[1][1])) + 1);
			mqElement[3] = 0.5f * s;
			s = 0.5f / s;
			mqElement[1] = (m[2][0] + m[0][2]) * s;
			mqElement[2] = (m[1][2] + m[2][1]) * s;
			mqElement[0] = (m[1][0] - m[0][1]) * s;
		}
	}		

}


/**
* \brief Destruktor
*/
Quaternion::~Quaternion()
{
	
}

/**
* \brief Gibt gew�nschtes Element des Quaternions zur�ck
*/
float& Quaternion::operator [](unsigned int index){
   return mqElement[index];
}

/**
* \brief Gibt gew�nschtes Element des Quaternions zur�ck
*/
const float& Quaternion::operator [](unsigned int index) const{
   return mqElement[index];
}


/**
* \brief elementweise Addition zweier Quaternionen 
* \param Quaternion
* \return Quaternion
*/
Quaternion& Quaternion::operator +=(const Quaternion& rhs){
	mqElement[0] += rhs.mqElement[0];
	mqElement[1] += rhs.mqElement[1];
	mqElement[2] += rhs.mqElement[2];
	mqElement[3] += rhs.mqElement[3];

   return *this;
}

/**
* \brief Multiplikation zweier Quaternionen
* \param Quaternion
* \return Quaternion
*/
Quaternion& Quaternion::operator*=(const Quaternion& rhs){
	
	//! \todo Das geht bestimmt besser, oder?
	Quaternion temp = *this * rhs;
	*this = temp;
	
	return *this;
}




/**
* \brief Identit�ts-Quaternion f�r die Multiplikation
*/
void Quaternion::identity(){
	mqElement[0]	= 1.0;
	mqElement[1]	= 0.0;
	mqElement[2]	= 0.0;
	mqElement[3]	= 0.0;
}

/**
* \brief Gibt das inverse Quaternion zur�ck
* \return Invertiertes Quaternion
*/
Quaternion Quaternion::inverse(){
	return Quaternion (mqElement[0], -mqElement[1], -mqElement[2], -mqElement[3]);
}

/**
* \brief Invertiert das Quaternion, setz es jedoch nicht
*/
void Quaternion::invert(){
	mqElement[1] = -mqElement[1];
	mqElement[2] = -mqElement[2];
	mqElement[3] = -mqElement[3];
}

/**
* \brief Normalisiert das Quaternion
* \return Gibt die Magnitude (Norm) zur�ck (float)
*/
float Quaternion::normalize() {
	if (mqElement[0] == 0.0 && mqElement[1] == 0.0 && mqElement[2] == 0.0 && mqElement[3] == 0.0)
		return 0.0;

	const float norm = sqrt(mqElement[0] * mqElement[0] + mqElement[1] * mqElement[1] + mqElement[2] * mqElement[2] + mqElement[3] * mqElement[3]);
	
	//assert(norm == 0.0);

	for (int i=0; i<4; ++i)
		mqElement[i] /= norm;
	return norm;
}

/**
* \brief Normalisiert die Rotationsachse eines Orientierungs-Quaternions (noch nicht sicher ob n�tig)
*/

void Quaternion::normalizeAxis() {

    float denom = sqrt(1 - (mqElement[0]*mqElement[0]));
    float x = mqElement[1]/denom;
    float y = mqElement[2]/denom;
    float z = mqElement[3]/denom;
    Vec3 axis = Vec3(x, y, z);
	axis.normalize();
	mqElement[1] = axis[0]*denom;
	mqElement[2] = axis[1]*denom;
	mqElement[3] = axis[2]*denom;
}


/**
* \brief Setzt die Werte des Quaternions, hat als Input die vier einzelnen Komponenten eines Quaternions
*/
void Quaternion::setValues(float w, float x, float y, float z){
	mqElement[0] = w;
	mqElement[1] = x;
	mqElement[2] = y;
	mqElement[3] = z;
}

/**
* \brief Setzt die WErte des RotationsQuaternion aus Winkel (angle im Bogenma�) und der Rotationsachse (axis) neu
*
* \param float angle Bogenma� der Drehung.
* \param Vec3 axis
*/
void Quaternion::setValues(float angle, Vec3& axis){
	float tempSin = sinf(angle / 2);

	mqElement[0] = cosf(angle / 2);
	mqElement[1] = axis[0] * tempSin;
	mqElement[2] = axis[1] * tempSin;
	mqElement[3] = axis[2] * tempSin;
}


/**
* \brief Setzt die Werte des Quaternions, hat als Input die Eulerwinkel
* \param float x, y, z die Eulerwinkel in Bogenma�
*/
void Quaternion::setValues(float x, float y, float z) {
	Quaternion(x, y, z);
}

/**
* \brief setzt die Werte des Vektorteils im Quaternion neu
* \return Setzt die Werte in den �bergebenen Referenzen
*/
void Quaternion::setVector(const Vec3& v) {
	mqElement[1] = v[0];
	mqElement[2] = v[1];
	mqElement[3] = v[2];
}

/**
* \brief Gibt die Werte des Quaternion in der Achsen- und Winkel-Form zur�ck
* \return Setzt die Werte in den �bergebenen Referenzen
*/
void Quaternion::getAxisAngle(Vec3& axis, float& angle) const {
	float s = sqrt((mqElement[1] * mqElement[1]) + (mqElement[2] * mqElement[2]) + (mqElement[3] * mqElement[3]));

	if (s == 0) {
	 	axis[0] = 1;
		axis[1] = 0;
		axis[2] = 0;

		angle = 0;

	} else {
		axis[0] = mqElement[1] / s;
		axis[1] = mqElement[2] / s;
		axis[2] = mqElement[3] / s;

		angle = 2 * acos(mqElement[0]);
	}
}

//! \return The axis of rotation
Vec3 Quaternion::getAxis() const {
	float s = sqrt((mqElement[1] * mqElement[1]) + (mqElement[2] * mqElement[2]) + (mqElement[3] * mqElement[3]));

	Vec3 axis;
		
	if (s == 0) {
	 	axis[0] = 1;
		axis[1] = 0;
		axis[2] = 0;
	} else {
		axis[0] = mqElement[1] / s;
		axis[1] = mqElement[2] / s;
		axis[2] = mqElement[3] / s;
	}

	return axis;
}

//! \return The angle of rotation in radian
float Quaternion::getAngle() const {
	float s = sqrt((mqElement[1] * mqElement[1]) + (mqElement[2] * mqElement[2]) + (mqElement[3] * mqElement[3]));
	float angle;
	if (s == 0) {
		angle = 0;
	} else {
		angle = 2 * acos(mqElement[0]);
	}
	return angle;
}

/**
* \brief Gibt die einzelnen 4 Komponenten des Quaternions zur�ck
* \return Setzt die Werte in die �bergebenen Referenzen
*/
void Quaternion::getValues(float &w, float &x, float &y, float &z) const
{
	w = mqElement[0];
	x = mqElement[1];
	y = mqElement[2];
	z = mqElement[3];
}

float Quaternion::w() const {
	return mqElement[0];
}

float Quaternion::x() const {
	return mqElement[1];
}

float Quaternion::y() const {
	return mqElement[2];
}
float Quaternion::z() const {
	return mqElement[3];
}

/**
* \brief Gibt den Vektor des Quaternions zur�ck
* \return Vec3
*/
Vec3 Quaternion::getVector () const{
	return Vec3(mqElement[1], mqElement[2], mqElement[3]);
}
/**
* \brief Gibt die Werte des Quaternions als Euler Winkel zur�ck (im Bogenma�)
* \return Vec3 (Euler-Winkel des Quaternions)
*
* Konvention: Rz*Ry*Rx !!! (s.o.)
* Angepasster Code aus GraphicsGems IV, Kapitel III.5 (Euler-Angle-Conversion)
*
* Idee:
* getRotationMatrix liefert eine Matrix. Diese Matrix ist eben jene, die man erh�lt, wenn man drei
* Rotationsmatrizen (jeweils um z, y, x-Achse) in der Reihenfolge Rz*Ry*Rx multipliziert. atan2(b,a) rechnet
* intern atan(b/a). Dementsprechend werden aus der Matrix Werte so an atan2 �bergeben, das b/a im Endeffekt 
* sinx/cosx=tanx (bzw. tany, tanz) ergibt.
* a und b werden so aus der Matrix gew�hlt, dass sich cos(phiy) immer wegk�rzt. Daher muss der cos(phiy)=0 abgefangen 
* werden (else-Zweig). Dies ist f�r rotationY nicht m�glich, daher muss hier cos(phiy) anders berechnet werden. Hierzu wird 
* die Formel cos^2(phi)+sin^2(phi)=1 angewandt.
* 
*/
void Quaternion::getEulerRotation(Vec3 &rotation) const {

	Matrix3x3 m;
	getRotationMatrix(m);
	float cosY = sqrt(m[0][0]*m[0][0]+m[1][0]*m[1][0]);
	
	if (cosY > 16 * FLT_EPSILON) {
		rotation[X] = atan2(1.0f*m[2][1], m[2][2]);
		rotation[Y] = atan2(-1.0f*m[2][0], cosY);
		rotation[Z] = atan2(1.0f*m[1][0], m[0][0]);
	} else {
		rotation[X] = atan2(-1.0f*m[1][2], m[1][1]);
		rotation[Y] = atan2(-1.0f*m[2][0], cosY);
		rotation[Z] = 0.0;
	}
	
	assert(!ISNAN(rotation[X]));
	assert(!ISNAN(rotation[Y]));
	assert(!ISNAN(rotation[Z]));
}

Vec3 Quaternion::getEulerRotation() const {

  Vec3 rotation;
  getEulerRotation(rotation);
  return rotation;
}

/**
* \brief Formt das Quaternion in eine Rotationsmatrix um
* \param Matrix3x3 (3x3Rotationsmatrix)
* Konvention: Rz*Ry*Rx !!! (s.o.)
*
* Die Werte im Quaternion entsprechen Multiplikationen der verschiedenen Cosinus- und Sinuswerte. Daher kann man durch
* geschicktes Vorgehen diese Sinus-/Cosinuswerte r�ckrechnen. Dabei werden die Quaternionenwerte so geschickt zusammen-
* gerechnet, das sich aufgrund der Trigonometrischen Gesetzm�ssigkeiten im Endeffekt die Eintr�ge der Rotationsmatrix 
* Rz*Ry*Rx ergeben.
* 
* Diesen Code haben wir nicht bis ins letzte Detail nachvollzogen, ihn allerdings durch ausf�hrliche Vergleichstests best�tigt.
*
* Dieser Code wurde aus OpenSG �bernommen. OpenSG rechnet anscheinend mit transponierten Matrizen, daher wird die
* Ergebnismatrix "zur�ck"-transponiert.
*/
void Quaternion::getRotationMatrix(Matrix3x3 &matrix) const{

	assert(matrix.getSizeM () == 3 && matrix.getSizeN () == 3);

    matrix[0][0] = 1.0f - 2.0f * (mqElement[2] * mqElement[2] +mqElement[3] * mqElement[3]);
    matrix[0][1] = 2.0f * (mqElement[1] * mqElement[2] +mqElement[3] * mqElement[0]);
    matrix[0][2] = 2.0f * (mqElement[3] * mqElement[1] -mqElement[2] * mqElement[0]);

    matrix[1][0] = 2.0f * (mqElement[1] * mqElement[2] -mqElement[3] * mqElement[0]);
    matrix[1][1] = 1.0f - 2.0f * (mqElement[3] * mqElement[3] +mqElement[1] * mqElement[1]);
    matrix[1][2] = 2.0f * (mqElement[2] * mqElement[3] +mqElement[1] * mqElement[0]);

    matrix[2][0] = 2.0f * (mqElement[3] * mqElement[1] +mqElement[2] * mqElement[0]);
    matrix[2][1] = 2.0f * (mqElement[2] * mqElement[3] -mqElement[1] * mqElement[0]);
    matrix[2][2] = 1.0f - 2.0f * (mqElement[2] * mqElement[2] +mqElement[1] * mqElement[1]);

	matrix = matrix.T();

}

//! \see getRotationMatrix(Matrix3x3)
Matrix3x3 Quaternion::getRotationMatrix() const{
	
	Matrix3x3 matrix;
	getRotationMatrix(matrix);
    return matrix;
}

/**
* \brief Gibt die Werte des Quaternions aus
*/
void Quaternion::print() {
	cout << "Quaternion: " << mqElement[0] << " (" << mqElement[1] << " " << mqElement[2] << " " << mqElement[3] << ") "<< endl;
}


//--- Related functions --------------------------------------------------------

/**
* \brief Multiplikation zweier Quaternionen
* \return neues Quaterion
*/
const Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs){

	return Quaternion (	lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
						lhs[1] * rhs[0] + lhs[0] * rhs[1] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
						lhs[2] * rhs[0] + lhs[0] * rhs[2] + lhs[3] * rhs[1] - lhs[1] * rhs[3],
						lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1]);

}

/**
* \brief Addition zweier Quaternionen
* \return neues Quaterion
*/
const Quaternion operator +(const Quaternion& lhs, const Quaternion& rhs){
	return Quaternion(
		lhs[0] + rhs[0],
		lhs[1] + rhs[1],
		lhs[2] + rhs[2],
		lhs[3] + rhs[3]);
}


/**
* \brief Multiplikation von Skalar und Quaternion
* \return neues Quaterion
*/
const Quaternion operator *(const float f, const Quaternion& q){
	return Quaternion(
		f * q[0],
		f * q[1],
		f * q[2],
		f * q[3]);
}

/**
* \brief Multiplikation von Quaternion und Skalar
* \return neues Quaterion
*/
const Quaternion operator *(const Quaternion& q, const float f){
	return Quaternion(
		f * q[0],
		f * q[1],
		f * q[2],
		f * q[3]);
}

/**
* \brief POunktprodukt von Quaternionen
* \return neues float
*/

float dot(const Quaternion& lhs, const Quaternion& rhs){
	return lhs[0] * rhs[0] +lhs[1] * rhs[1] +lhs[2] * rhs[2] +lhs[3] * rhs[3];
}


/**
* \brief Rotation von Vektor um Orientierungs-Quaternion 
* \param vector ein Vektor
* \param quat das Orientierungs-Quaternion, um welches gedreht wird 
* \return um Quaternion rotierter Vektor
*/
const Vec3 qRotate(const Vec3& vector, const Quaternion& qRot)
{
    Quaternion qInvRot = qRot;
    qInvRot.invert();
	Quaternion qVector(vector);
    qVector = qRot * qVector * qInvRot;
    
    return Vec3(qVector[1], qVector[2], qVector[3]);
} 

std::ostream& operator <<(std::ostream& os, const Quaternion& quaternion) {
	//Werte des QUaternions pur ausgeben
	os << "W: " << quaternion[0] << ", ";
	os << "X: " << quaternion[1] << ", ";
	os << "Y: " << quaternion[2] << ", ";
	os << "Z: " << quaternion[3] << "    oder:    ";

	Vec3 vec;
	float ang;
	quaternion.getAxisAngle(vec, ang);

	os << "Winkel: " << ang << " Achse: " << vec;
	
	return os;
}
