#ifndef PLANE_DESCRIPTION_H
#define PLANE_DESCRIPTION_H

#include <simon/simon.h>
#include "ObjectDescription.h"

class PlaneDescription : public ObjectDescription{

public:

	PlaneDescription();
	virtual ~PlaneDescription(){};

	virtual void draw(GraphicsBase*);

private:

};

#endif   // PLANE_DESCRIPTION_H
