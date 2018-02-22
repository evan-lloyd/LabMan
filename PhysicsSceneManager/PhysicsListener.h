#ifndef PHYSICS_LISTENER_H
#define PHYSICS_LISTENER_H

#include "btBulletDynamicsCommon.h"

class PhysicsListener
{
public:
	PhysicsListener(void);
	~PhysicsListener(void);

	virtual void physicsCallback(btDynamicsWorld *w, btScalar time) = 0;
};

#endif