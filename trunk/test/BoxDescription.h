#ifndef BOX_DESCRIPTION_H
#define BOX_DESCRIPTION_H

#include <simon/simon.h>
#include "ObjectDescription.h"

class BoxDescription : public ObjectDescription{

public:

	BoxDescription();
	virtual ~BoxDescription(){};

	virtual void draw(GraphicsBase*);
private:

};

#endif   // BOX_DESCRIPTION_H
