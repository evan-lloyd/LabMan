#ifndef POSE_CONTROLLER_H
#define POSE_CONTROLLER_H

#include <vector>
#include "InverseKinematics.h"

namespace Ogre
{
class Bone;
class PhysicsResource;
}

class btRigidBody;
class btQuaternion;
class btVector3;
class InverseKinematics;

class PoseController
{
public:
	typedef std::vector<Ogre::Bone *> BoneList;
	typedef std::vector<btRigidBody *> BodyList;
	typedef std::vector<btQuaternion *> TargetList;
	typedef std::vector<btGeneric6DofConstraint *> ConstraintList;
	typedef std::vector<btVector3 *> VectorList;

	typedef std::vector<double> ScalarList;

	float totalError;
	float errorThreshold;

private:
	Ogre::PhysicsResource *obj;
	BoneList bones;
	BodyList bodies;
	ConstraintList constraints;

	ScalarList maxTorques;
	ScalarList torqueScales;

	VectorList lastErrors;
	VectorList errorSums;

	TargetList targets;

	// TODO: per-joint max torques
	static float maxImpulse;
	static float impulseScale;
	static float torqueScale;
	static float maxVelocity;
	InverseKinematics *ik;
	// velocities / other parameters
public:
	PoseController(Ogre::PhysicsResource *r, InverseKinematics::ConstraintMap &c);

	void setTargets(const InverseKinematics::TargetSet &t, const InverseKinematics::NameList &dontCare);

	void controlStep(double time);
};

#endif