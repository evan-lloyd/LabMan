#include "Motion.h"

Motion::Motion()
{
	trajectory.resize(TRAJECTORY_SIZE);
	reset();
}