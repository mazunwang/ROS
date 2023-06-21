#ifndef _QUADOG_H_
#define _QUADOG_H_

#include "Types.h"
#include "Quadruped.h"

static Quadruped buildQuadog(){
    Quadruped quadog;
    quadog.bodyLength = 0.635;
    quadog.bodyWidth = 0.272;
    quadog.bodyMass = 0;

    quadog.HipXLength = 0.037;
    quadog.HipYLength = 0.3;
    quadog.KneeLength = 0.3;

    quadog.HipXMass = 0;
    quadog.HipYMass = 0;
    quadog.KneeMass = 0;

    return quadog;
}


#endif
