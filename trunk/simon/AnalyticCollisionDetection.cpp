/**
 * \class AnalyticCollisionDetection
 */


#include <simon/AnalyticCollisionDetection.h>
#include <simon/Id.h>

using namespace std;



void AnalyticCollisionDetection::add(GeometryPtr geometry){
	if (geometry)
		mGeometry.push_back(geometry);
}

/**
 * \param interferenceVector The storage place of all
 * interferences. Objects will just be appended.
 */
void
AnalyticCollisionDetection::findInterferences(
	vector<InterferencePtr> &interferenceVector){

	int length = mGeometry.size();

	for (int i = 0; i < length - 1; ++i){

		for (int j = i + 1; j < length; ++j){

		 	InterferencePtr interference(new Interference);
			if (check(mGeometry[i], mGeometry[j], *interference)){
				assert(interference->getObjectA());
				assert(interference->getObjectB());
				interferenceVector.push_back(interference);
			}
        }
    }
}


/**
 * \param geometry The geometry which should be checked
 * \param interferenceVector The storage place of all
 * interferences. Objects will just be appended
 */
void
AnalyticCollisionDetection::findInterferences(
	GeometryPtr geometry,
	vector<InterferencePtr>& interferenceVector){

	assert(geometry);

	int length = mGeometry.size();

	for (int i = 0; i < length; ++i){

		if (geometry == mGeometry[i])
			continue;

		InterferencePtr interference(new Interference);
		if (check(geometry, mGeometry[i], *interference)){
			assert(interference->getObjectA());
			assert(interference->getObjectB());
			interferenceVector.push_back(interference);
		}
	}
}


bool AnalyticCollisionDetection::check(GeometryPtr objectA,
  									   GeometryPtr objectB,
  									   Interference &interference){

	// objectA == plane
	if (objectA->getRigidBody()->getId().getType() == Id::typePlane){

		// objectB == sphere
		if(objectB->getRigidBody()->getId().getType() == Id::typeSphere)
			return check(boost::static_pointer_cast<Sphere>(objectB),
						 boost::static_pointer_cast<Plane>(objectA),
						 interference);

		// objectB == box
		if(objectB->getRigidBody()->getId().getType() == Id::typeBox)
			return check(boost::static_pointer_cast<Box>(objectB),
						 boost::static_pointer_cast<Plane>(objectA),
						 interference);

		// objectB == capsule
		if(objectB->getRigidBody()->getId().getType() == Id::typeCapsule)
			return check(boost::static_pointer_cast<Capsule>(objectB),
						 boost::static_pointer_cast<Plane>(objectA),
						 interference);

	}

	// objectA == sphere
	if (objectA->getRigidBody()->getId().getType() == Id::typeSphere){

		// objectB == sphere
		if (objectB->getRigidBody()->getId().getType() == Id::typeSphere)
			return check(boost::static_pointer_cast<Sphere>(objectA),
						 boost::static_pointer_cast<Sphere>(objectB),
						 interference);

		// objectB == plane
		if (objectB->getRigidBody()->getId().getType() == Id::typePlane)
			return check(boost::static_pointer_cast<Sphere>(objectA),
						 boost::static_pointer_cast<Plane>(objectB),
						 interference);

		// objectB == capsule
		if (objectB->getRigidBody()->getId().getType() == Id::typeCapsule)
			return check(boost::static_pointer_cast<Capsule>(objectB),
						 boost::static_pointer_cast<Sphere>(objectA),
						 interference);

		// objectB == box
		if (objectB->getRigidBody()->getId().getType() == Id::typeBox)
			return check(boost::static_pointer_cast<Box>(objectB),
						 boost::static_pointer_cast<Sphere>(objectA),
						 interference);


	}

	// objectA == box
	if (objectA->getRigidBody()->getId().getType() == Id::typeBox){

		// objectB == Box
		if (objectB->getRigidBody()->getId().getType() == Id::typeBox)
			return check(boost::static_pointer_cast<Box>(objectA),
						 boost::static_pointer_cast<Box>(objectB),
						 interference);

		// objectB == sphere
		if (objectB->getRigidBody()->getId().getType() == Id::typeSphere)
			return check(boost::static_pointer_cast<Box>(objectA),
						 boost::static_pointer_cast<Sphere>(objectB),
						 interference);

		// objectB == plane
		if (objectB->getRigidBody()->getId().getType() == Id::typePlane)
			return check(boost::static_pointer_cast<Box>(objectA),
						 boost::static_pointer_cast<Plane>(objectB),
						 interference);

		// objectB == capsule
		if (objectB->getRigidBody()->getId().getType() == Id::typePlane)
			return check(boost::static_pointer_cast<Capsule>(objectB),
						 boost::static_pointer_cast<Box>(objectA),
						 interference);
	}

	// objectA == capsule
	if (objectA->getRigidBody()->getId().getType() == Id::typeCapsule){

		// objectB == capsule
		if (objectB->getRigidBody()->getId().getType() == Id::typeCapsule)
			return check(boost::static_pointer_cast<Capsule>(objectA),
						 boost::static_pointer_cast<Capsule>(objectB),
						 interference);

		// objectB == sphere
		if (objectB->getRigidBody()->getId().getType() == Id::typeSphere)
			return check(boost::static_pointer_cast<Capsule>(objectA),
						 boost::static_pointer_cast<Sphere>(objectB),
						 interference);

		// objectB == plane
		if (objectB->getRigidBody()->getId().getType() == Id::typePlane)
			return check(boost::static_pointer_cast<Capsule>(objectA),
						 boost::static_pointer_cast<Plane>(objectB),
						 interference);

		// objectB == box
		if (objectB->getRigidBody()->getId().getType() == Id::typeBox)
			return check(boost::static_pointer_cast<Capsule>(objectA),
						 boost::static_pointer_cast<Box>(objectB),
						 interference);
	}

	return false;
}

bool AnalyticCollisionDetection::check(SpherePtr objectA,
									   SpherePtr objectB,
									   Interference &interference){


	Vec3 distance =
		objectA->getRigidBody()->getPosition() -
		objectB->getRigidBody()->getPosition();

	assert(distance.length() != 0.0);

	float radiusSum = objectA->getRadius() + objectB->getRadius();

	if (distance.sqrLength() < radiusSum * radiusSum){

		//compute normalB and contactPoint
		Vec3 normalB = distance;
		normalB.normalize();

		const Vec3& contactPoint =
			objectA->getRigidBody()->getPosition() -
			(objectA->getRadius() * normalB);
		float difference = radiusSum - distance.length();

		interference.setObjectA(objectA);
		interference.setObjectB(objectB);
		interference.addContactPoint(contactPoint, normalB, difference);

		return true;
	}

	return false;
}

bool AnalyticCollisionDetection::check(SpherePtr objectA,
									   PlanePtr objectB,
									   Interference &interference){

	float distance = dot(objectA->getRigidBody()->getPosition() -
						 objectB->getRigidBody()->getPosition(),
						 objectB->getNormal());

    // check collision
    if ( distance <= objectA->getRadius()){

		interference.setObjectA(objectA);
		interference.setObjectB(objectB);
		interference.addContactPoint(
			objectA->getRigidBody()->getPosition() - (objectA->getRadius() *
													  objectB->getNormal()),
			objectB->getNormal(), objectA->getRadius() - distance);
		return true;
	} else
		return false;
}


bool AnalyticCollisionDetection::check(BoxPtr objectA,
									   PlanePtr objectB,
									   Interference &interference){

	assert(objectB);
   	Vec3 vertices[8];
	Vec3 normals[3];
	Vec3 contactPoints[8];
	Vec3 contactPoint;
	unsigned int i = 0;
	unsigned int numOfContacts = 0;
	normals[0] = qRotate(Vec3(1,0,0), objectA->getRigidBody()->getOrientation());
	normals[1] = qRotate(Vec3(0,1,0), objectA->getRigidBody()->getOrientation());
	normals[2] = qRotate(Vec3(0,0,1), objectA->getRigidBody()->getOrientation());

	const Vec3& normalB = objectB->getNormal();

	vertices[0] =  objectA->getRigidBody()->getPosition() +
		objectA->getScale()[X] * normals[X] +
		objectA->getScale()[Y] * normals[Y] +
		objectA->getScale()[Z] * normals[Z];

   	vertices[1] =  objectA->getRigidBody()->getPosition() -
		objectA->getScale()[X] * normals[X] +
		objectA->getScale()[Y] * normals[Y] +
		objectA->getScale()[Z] * normals[Z];

   	vertices[2] =  objectA->getRigidBody()->getPosition() +
		objectA->getScale()[X] * normals[X] -
		objectA->getScale()[Y] * normals[Y] +
		objectA->getScale()[Z] * normals[Z];

	vertices[3] =  objectA->getRigidBody()->getPosition() -
		objectA->getScale()[X] * normals[X] -
		objectA->getScale()[Y] * normals[Y] +
		objectA->getScale()[Z] * normals[Z];

	vertices[4] =  objectA->getRigidBody()->getPosition() +
		objectA->getScale()[X] * normals[X] +
		objectA->getScale()[Y] * normals[Y] -
		objectA->getScale()[Z] * normals[Z];

	vertices[5] =  objectA->getRigidBody()->getPosition() -
		objectA->getScale()[X] * normals[X] +
		objectA->getScale()[Y] * normals[Y] -
		objectA->getScale()[Z] * normals[Z];

	vertices[6] =  objectA->getRigidBody()->getPosition() +
		objectA->getScale()[X] * normals[X] -
		objectA->getScale()[Y] * normals[Y] -
		objectA->getScale()[Z] * normals[Z];

	vertices[7] =  objectA->getRigidBody()->getPosition() -
		objectA->getScale()[X] * normals[X] -
		objectA->getScale()[Y] * normals[Y] -
		objectA->getScale()[Z] * normals[Z];

   	for (i = 0; i < 8; i++){

		if (dot((vertices[i] -
				 objectB->getRigidBody()->getPosition()), normalB) < 0 &&
			numOfContacts < 8){

			contactPoints[numOfContacts] = vertices[i];
			numOfContacts ++;
		}
	}


   	if (numOfContacts == 0)
		return false;

	interference.setObjectA(objectA);
	interference.setObjectB(objectB);

	for (i = 0; i < numOfContacts; i++)
	{
		contactPoint += contactPoints[i];
		interference.addContactPoint(contactPoints[i],
									 normalB,
									 dot(normalB,objectB->getRigidBody()->getPosition())-
									 dot(normalB,contactPoints[i])
			);
	}

	contactPoint *= 1.0f/numOfContacts;
	interference.addContactPoint(contactPoint,
								 normalB,
								 dot(normalB,objectB->getRigidBody()->getPosition()) -
								 dot(normalB, contactPoint)
		);


	return true;

}

bool AnalyticCollisionDetection::check(BoxPtr objectA,
									   BoxPtr objectB,
									   Interference &interference){

	return false;
}

bool AnalyticCollisionDetection::check(BoxPtr objectA,
									   SpherePtr objectB,
									   Interference &interference){


	//sphere in das lokale koordinatensystem der box bringen
	Vec3 spherePos = objectB->getRigidBody()->getPosition() -
		objectA->getRigidBody()->getPosition();
	Quaternion boxOri = objectA->getRigidBody()->getOrientation();
	const Quaternion& boxInvOri = boxOri.inverse();
    spherePos = qRotate(spherePos, boxInvOri);

    unsigned int cnt = 0;
    unsigned int cntEdges = 0;

	Vec3 halfDimensions(objectA->getWidth()/2,
						objectA->getHeight()/2,
						objectA->getDepth()/2);

	float rad = objectB->getRadius();

	const float& sphereX = dot(spherePos, Vec3(1,0,0));
	const float& sphereY = dot(spherePos, Vec3(0,1,0));
	const float& sphereZ = dot(spherePos, Vec3(0,0,1));
	Vec3 sphere(sphereX, sphereY, sphereZ);

	// to find out the distance, how far the sphere is in the box.
	Vec3 distanceVector(0.0, 0.0, 0.0);

    for (unsigned int i=0; i<3; i++)
    {
		if (halfDimensions[i]+rad > fabs(sphere[i]))
		{
			if ((sphere[i] >= 0))// && (halfDimensions[i]+rad > sphere[i]))
			{
				cnt++;
				if (sphere[i]>halfDimensions[i])
				{
					spherePos[i] = halfDimensions[i];
					distanceVector[i] = halfDimensions[i] + rad -sphere[i];
					cntEdges++;
				}
				else
				{
					spherePos[i] = sphere[i];
					distanceVector[i] = 0.0;
				}
			}
			else
//		if ((sphere[i] < 0))// && (halfDimensions[i]+rad > -sphere[i]))
			{
				cnt++;
				if (-sphere[i]>halfDimensions[i])
				{
					spherePos[i] = -halfDimensions[i];
					distanceVector[i] = halfDimensions[i] + rad + sphere[i];
					cntEdges++;
				}
				else
				{
					spherePos[i] = sphere[i];
					distanceVector[i] = 0.0;
				}
			}
		}
    }

	//test auf collision
    if (cnt >= 3)
	{
		//berechnung normalB und contactPoint
		const Vec3& contactPoint = qRotate(spherePos, boxOri) +
			objectA->getRigidBody()->getPosition();
		Vec3 normalB = objectB->getRigidBody()->getPosition() -
			contactPoint;
		//assert zu debugzwecken
		assert (normalB.length() != 0.0);
		normalB.normalize();

		float distanceFromBox=0.0;
		//berechnung der eindringtiefe.
		switch (cntEdges)
		{
		case 0:
			// die Kugel ist komplett in der Box
			distanceFromBox = rad;
			break;
		case 1:
			// die Kugel steckt in einem Face
			distanceFromBox = distanceVector.max();
			break;
		case 2:
			// die Kugel steckt in einer Kante;
			distanceFromBox = distanceVector.min();
			break;
		case 3:
			// die Kugel steckt in einer Ecke;
			distanceFromBox = distanceVector.length();
		}

		interference.setObjectA(objectA);
		interference.setObjectB(objectB);
		interference.addContactPoint(contactPoint, normalB, distanceFromBox);
		return true;
	}
	else
	{
		return false;
	}

}

bool AnalyticCollisionDetection::check(CapsulePtr objectA,
									   CapsulePtr objectB,
									   Interference &interference){

    // teile des codes Copyright 2001, softSurfer (www.softsurfer.com)
	Vec3 unitV(0,1,0);
    Vec3 raV(qRotate(unitV, objectA->getRigidBody()->getOrientation()));
    Vec3 rbV(qRotate(unitV, objectB->getRigidBody()->getOrientation()));

    Vec3 A1 = objectA->getRigidBody()->getPosition() + (raV*(objectA->getHeight()/2));
    Vec3 A2 = objectA->getRigidBody()->getPosition() - (raV*(objectA->getHeight()/2));

    Vec3 B1 = objectB->getRigidBody()->getPosition() + (rbV*(objectB->getHeight()/2));
    Vec3 B2 = objectB->getRigidBody()->getPosition() - (rbV*(objectB->getHeight()/2));

    Vec3   u = A2 - A1;
    Vec3   v = B2 - B1;
    Vec3   w = A1 - B1;
    float    a = dot(u,u);        // always >= 0
    float    b = dot(u,v);
    float    c = dot(v,v);        // always >= 0
    float    d = dot(u,w);
    float    e = dot(v,w);
    float    D = a*c - b*b;       // always >= 0
    float    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    float    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < 0.0000000001) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }
    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    if(fabs(sN) < 0.000000001)
        sc = 0.0;
    else
        sc = sN/sD;

    if(fabs(tN) < 0.0000000001)
        tc = 0.0;
    else
        tc = tN/tD;

    Vec3  dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)


    float dist = dP.length();   // return the closest distance
    if (dist==0) cout << "CAPS-DISTANCE =0!\n";

    //test auf collision
    if (dist < (objectA->getRadius() + objectB->getRadius()))
    {

		//assert zu debugzwecken
		assert(dP.length() != 0.0);

        //berechnung von normalB und contactPoint
        Vec3 normalB = dP;
		normalB.normalize();
        Vec3 contactPoint = (B1 + (tc*v)) + (objectB->getRadius() * normalB);
        Vec3 posBToContact = contactPoint - (objectB->getRigidBody()->getPosition());

        //überprüfung der normalen, falls die capsule weiter als radius eingedrungen ist, umdrehen
        if (dot(posBToContact, normalB)<0){

//            cout << "Normale " << normalB << " von B=" << caps->getId() << " zeigt in die falsche Richtung !\n";
			normalB = -normalB;
        } else {
//            cout << "Normale " << normalB << " von B=" << caps->getId() << " zeigt in die richtige Richtung !\n";
        }

		// der contaktpunkt auf B ist
		float distanceFromB = (objectA->getRadius() + objectB->getRadius()) - dP.length();
		interference.setObjectA(objectA);
		interference.setObjectB(objectB);
		interference.addContactPoint(contactPoint, normalB, distanceFromB);
		return true;
	}
	else
	{
		return false;
	}

}

bool AnalyticCollisionDetection::check(CapsulePtr objectA,
									   PlanePtr objectB,
									   Interference &interference){
    Vec3 unitV(0,1,0);
    Vec3 rV(qRotate(unitV, objectA->getRigidBody()->getOrientation()));

    Vec3 A1 = objectA->getRigidBody()->getPosition()+(rV*(objectA->getHeight()/2));
    Vec3 A2 = objectA->getRigidBody()->getPosition()-(rV*(objectA->getHeight()/2));

    const Vec3& normalB = objectB->getNormal();

    float dist1 = dot((A1 - objectB->getRigidBody()->getPosition()), normalB);
    float dist2 = dot((A2 - objectB->getRigidBody()->getPosition()), normalB);

    //test auf collision
	if(dist1 < objectA->getRadius() || dist2 < objectA->getRadius()) {

		interference.setObjectA(objectA);
		interference.setObjectB(objectB);


		//berechnung contactPoint und normalB
		Vec3 contactPoint;

		if (abs(dist1 - dist2) < 1){

			contactPoint =
				objectA->getRigidBody()->getPosition() -
				(normalB * objectA->getRadius());
	        interference.addContactPoint(contactPoint, normalB,
										  objectA->getRadius() - (dist1+dist2) / 2.f);
		} else {
			if (dist1 < objectA->getRadius())
			{
				contactPoint = A1 - (normalB * objectA->getRadius());
				interference.addContactPoint(contactPoint, normalB,
											 objectA->getRadius() - dist1);
			}
			if (dist2 < objectA->getRadius())
			{
				contactPoint = A2 - (normalB * objectA->getRadius());
				interference.addContactPoint(contactPoint, normalB,
											  objectA->getRadius() - dist2);
			}
		}

		return true;
	} else
		return false;

}

bool AnalyticCollisionDetection::check(CapsulePtr objectA,
									   SpherePtr objectB,
									   Interference &interference){

	Vec3 unitV(0,1,0);
	Vec3 rV(qRotate(unitV, objectA->getRigidBody()->getOrientation()));

	float height = objectA->getHeight();
	Vec3 position = objectA->getRigidBody()->getPosition();

	Vec3 A1 = position + (rV*(height/2));
	Vec3 A2 = position - (rV*(height/2));

	rV = qRotate(unitV, objectB->getRigidBody()->getOrientation());

	// teile des codes Copyright 2001, softSurfer (www.softsurfer.com)

	// alles geschummelt. Kugelzentrum als infinitisimal kleines Segment interpretiert.
	Vec3 smallNumberVector(0.0001f,0.0001f,0.0001f);

	Vec3 B1 = objectB->getRigidBody()->getPosition() + smallNumberVector;
	Vec3 B2 = objectB->getRigidBody()->getPosition() - smallNumberVector;

	Vec3   u = A2 - A1;
	Vec3   v = B2 - B1;
	Vec3   w = A1 - B1;
	float    a = dot(u,u);        // always >= 0
	float    b = dot(u,v);
	float    c = dot(v,v);        // always >= 0
	float    d = dot(u,w);
	float    e = dot(v,w);
	float    D = a*c - b*b;       // always >= 0
	float    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
	float    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < 0.0000000001) { // the lines are almonnnnnnnst parallel
		sN = 0.0;        // force using point P0 on segment S1
		sD = 1.0;        // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {                // get the closest points on the infinite lines
		sN = (b*e - c*d);
		tN = (a*e - b*d);
		if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}
	if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	if(fabs(sN) < 0.000000001)
		sc = 0.0;
	else
		sc = sN/sD;

	if(fabs(tN) < 0.0000000001)
		tc = 0.0;
	else
		tc = tN/tD;

	// get the difference of the two closest points
	Vec3  dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

	Vec3 axis;
	float angle;
	objectA->getRigidBody()->getOrientation().getAxisAngle(axis, angle);

	float dist = dP.length();   // return the closest distance
    if (dist==0)
		cout << "CAPS-SPHERE DISTANCE =0!\n";

	//test auf collision
	if (dist < (objectA->getRadius() + objectB->getRadius()))
	{
		//assert zu debugzwecken
		assert(dP.length() != 0.0);
		dP.normalize();
		const Vec3& normalB = dP ;
		const Vec3& contactPoint = objectB->getRigidBody()->getPosition() +
			(objectB->getRadius() * dP);

		float distanceFromB = (objectA->getRadius() + objectB->getRadius()) - dist;

		interference.setObjectA(objectA);
		interference.setObjectB(objectB);
		interference.addContactPoint(contactPoint, normalB, distanceFromB);
		return true;
	}
	else
	{
		return false;
	}

}

bool AnalyticCollisionDetection::check(CapsulePtr objectA,
									   BoxPtr objectB,
									   Interference &interference){

	assert(objectB);

	unsigned int numOfSpheres;
	float stepSize;

	Vec3 axis = qRotate(Vec3 (0,1,0), objectA->getRigidBody()->getOrientation());
	float length = objectA->getHeight();
	float scale = length / 2.f;
	Vec3 halfAxis = axis * scale;
	const Vec3& endPoint1 = objectA->getRigidBody()->getPosition() - halfAxis;

	numOfSpheres = static_cast<unsigned int> (length / objectA->getRadius());
	stepSize = length / static_cast<float> (numOfSpheres);

	Vec3 startPosition = endPoint1;
	Vec3 contactPoint(0,0,0);
	Vec3 averageContactPoint(0,0,0);
	Vec3 normal(0,0,0);
	Vec3 averageNormal(0,0,0);
	float distanceFromBox=0.0;

	//scheint beides nicht so richtig zu funktionieren...
	float averageDistanceFromBox=0.0;
	float maxDistanceFromBox=0.0;

	const Vec3& step = axis * stepSize;
	unsigned int numOfContacts = 0;

	for (unsigned int i = 0; i < numOfSpheres+1; i++)
	{

		if (collideSphereBox(startPosition,
							 objectA->getRadius(),
							 contactPoint,
							 normal,
							 distanceFromBox,
							 objectB)){

			if (numOfContacts==0){
				interference.setObjectA(objectA);
				interference.setObjectB(objectB);
			}

			interference.addContactPoint(contactPoint, normal, distanceFromBox);
			averageContactPoint += contactPoint;
			averageNormal += normal;
			averageDistanceFromBox += distanceFromBox;
			if (distanceFromBox > maxDistanceFromBox)
				maxDistanceFromBox = distanceFromBox;
			numOfContacts++;
		}

		startPosition += step;
	}


	if (numOfContacts > 0)
	{
		averageNormal /= (float)numOfContacts;

		//assert (averageNormal.length() != 0.0);
		if (averageNormal.length() != 0.0)
			averageNormal.normalize();
		averageContactPoint /= (float)numOfContacts;
		averageDistanceFromBox /= (float)numOfContacts;

		interference.addContactPoint(averageContactPoint,
									 averageNormal,
									 maxDistanceFromBox);
		return true;
	}
	else
		return false;
}

/**
* \brief sphere-box kollision speziell fuer caps-box
* \param position und radius der sphere, referenzen auf kontakt-punkt und normale
*/
bool
AnalyticCollisionDetection::collideSphereBox(Vec3 spherePosition,
											 float radius,
											 Vec3& contactPoint,
											 Vec3& normalB,
											 float distanceFromBox,
											 BoxPtr boxGeo){

    //sphere in das lokale koordinatensystem der box bringen
    Vec3 spherePos = spherePosition - boxGeo->getRigidBody()->getPosition();
    Quaternion boxOri = boxGeo->getRigidBody()->getOrientation();
    const Quaternion& boxInvOri = boxOri.inverse();
    spherePos = qRotate(spherePos, boxInvOri);

    unsigned int cnt = 0;
    unsigned int cntEdges = 0;

	Vec3 halfDimensions(boxGeo->getWidth()/2,
						boxGeo->getHeight()/2,
						boxGeo->getDepth()/2);

	const float& sphereX = dot(spherePos, Vec3(1,0,0));
	const float& sphereY = dot(spherePos, Vec3(0,1,0));
	const float& sphereZ = dot(spherePos, Vec3(0,0,1));
	Vec3 sphere(sphereX, sphereY, sphereZ);

	// to find out the distance, how far the sphere is in the box.
	Vec3 distanceVector(0.0, 0.0, 0.0);

    for (unsigned int i=0; i<3; i++)
    {
		if (halfDimensions[i]+radius > fabs(sphere[i]))
		{
			if ((sphere[i] >= 0))// && (halfDimensions[i]+rad > sphere[i]))
			{
				cnt++;
				if (sphere[i]>halfDimensions[i])
				{
					spherePos[i] = halfDimensions[i];
					distanceVector[i] = halfDimensions[i] + radius -sphere[i];
					cntEdges++;
				}
				else
				{
					spherePos[i] = sphere[i];
					distanceVector[i] = 0.0;
				}
			}
			else
				//if ((sphere[i] < 0))// && (halfDimensions[i]+rad > -sphere[i]))
			{
				cnt++;
				if (-sphere[i]>halfDimensions[i])
				{
					spherePos[i] = -halfDimensions[i];
					distanceVector[i] = halfDimensions[i] + radius + sphere[i];
					cntEdges++;
				}
				else
				{
					spherePos[i] = sphere[i];
					distanceVector[i] = 0.0;
				}
			}
		}
    }

	//test auf collision
    if (cnt >= 3){

		//berechnung normalB und contactPoint
		contactPoint = qRotate(spherePos, boxOri) +
			boxGeo->getRigidBody()->getPosition();
		normalB = spherePosition - contactPoint;

		if (!normalB.length())
		{
			//! \todo wieder rein machen
			/*
			  SimonState::exemplar()->errors
			  << "Collision: Box-Sphere Kollision erhielt eine unmögliche Konfiguration."
			  << endl << "Kann es sein, das falsch Positioniert wurde?"
			  << SimonState::endm;
			  normalB = Vec3(0,1,0);
			*/
		}
		else
			normalB.normalize();
		//assert zu debugzwecken
		//assert (normalB.length() != 0.0);

		//berechnung der eindringtiefe.
		switch (cntEdges)
		{
		case 0:
			// die Kugel ist komplett in der Box
			distanceFromBox = radius;
			break;
		case 1:
			// die Kugel steckt in einem Face
			distanceFromBox = distanceVector.max();
			break;
		case 2:
			// die Kugel steckt in einer Kante;
			distanceFromBox = distanceVector.min();
			break;
		case 3:
			// die Kugel steckt in einer Ecke;
			distanceFromBox = distanceVector.length();
		}
		return true;
	}
	else
	{
		return false;
	}

}
