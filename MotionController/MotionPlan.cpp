#include "MotionPlan.h"

InverseKinematics::TargetSet *MotionPlan::next()
{
	InverseKinematics::TargetSet* nextTrajectory = motions[currentMotion]->next();
	
	if(nextTrajectory)
		return nextTrajectory;
	
	currentMotion++;

	if(currentMotion >= motions.size())
	{
		if(cyclic)
			currentMotion = 0;
		// Not cyclic, and we finished the last motion; it's over.
		return NULL;
	}

	motions[currentMotion]->reset();

	return motions[currentMotion]->next();
}