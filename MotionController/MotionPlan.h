#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include <vector>
#include "Motion.h"

class MotionPlan
{
public:
	typedef std::vector<Motion*> MotionList;
private:
	bool       cyclic;
	MotionList motions;
	size_t     currentMotion;
	
public:
	MotionPlan(bool c = false) : currentMotion(0), cyclic(c) {}

	void addMotion(Motion *m) { motions.push_back(m); }
	Motion &operator[] (size_t i) { return *motions[i]; }
	void reset() { currentMotion = 0; if(motions.size() > 0) motions[0]->reset(); }
	bool hasNext() const { return cyclic || (currentMotion < motions.size() && motions[currentMotion]->hasNext()); }


	size_t curMotion() const { if(hasNext()) return motions[currentMotion]->currentTarget(); return -1;}

	InverseKinematics::TargetSet *next();
};

#endif