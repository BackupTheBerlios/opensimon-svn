#ifndef SPHERE_DESCRIPTION_H
#define SPHERE_DESCRIPTION_H

#include <simon/simon.h>
#include "ObjectDescription.h"

class SphereDescription : public ObjectDescription{

public:

	SphereDescription();
	virtual ~SphereDescription(){};

	virtual void draw(GraphicsBase*);
private:

};

#endif   // SPHERE_DESCRIPTION_H
