#ifndef CAPSULE_DESCRIPTION_H
#define CAPSULE_DESCRIPTION_H

#include <simon/simon.h>
#include "ObjectDescription.h"

class CapsuleDescription : public ObjectDescription{

public:

	CapsuleDescription();
	virtual ~CapsuleDescription(){};

	virtual void draw(GraphicsBase*);
private:

};

#endif   // CAPSULE_DESCRIPTION_H
