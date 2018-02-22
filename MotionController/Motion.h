#ifndef MOTION_H
#define MOTION_H

#include "InverseKinematics.h"

class MotionPlan;
class Motion
{
public:
	typedef std::vector<InverseKinematics::TargetSet> Trajectory;
private:
	friend class MotionPlan;
	size_t curPos;
	Trajectory trajectory;
public:
	static const int TRAJECTORY_SIZE = 20;
	Motion();
	operator Trajectory& () { return trajectory; }
	InverseKinematics::TargetSet &operator [] (size_t i) { return trajectory[i]; }
	InverseKinematics::TargetSet *next()
	{
		if(curPos >= TRAJECTORY_SIZE)
			return NULL;

		return &trajectory.at(curPos++);
	}
	bool hasNext() { return curPos < TRAJECTORY_SIZE; }
	void reset() { curPos = 0; }
	size_t currentTarget() const { return curPos; }
};

#endif