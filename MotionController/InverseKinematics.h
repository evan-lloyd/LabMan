#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#define _USE_MATH_DEFINES
#include <math.h>
#include <OpenTissue/kinematics/inverse/inverse.h>
#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
#include <OpenTissue/kinematics/animation/animation_naive_blend_scheduler.h>
#include <OpenTissue/kinematics/inverse/inverse_utility_sample_motion.h>
#include <OpenTissue/kinematics/animation/animation_keyframe_animation.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/big/big_types.h>

#include "btBulletDynamicsCommon.h"
#include "PhysicsResource.h"

#include "OgreSkeleton.h"

#include <map>
#include <vector>

typedef float															 real_type;
typedef OpenTissue::math::default_math_types		  				     math_types;
typedef OpenTissue::skeleton::DefaultBoneTraits<math_types>              bone_traits;
typedef OpenTissue::kinematics::inverse::BoneTraits< bone_traits >       ik_bone_traits;
typedef OpenTissue::skeleton::Types<math_types,ik_bone_traits>           skeleton_types;
typedef skeleton_types::skeleton_type                                    skeleton_type;
typedef math_types::coordsys_type										 coordsys_type;
typedef skeleton_type::bone_type										 bone_type;
typedef bone_traits::transform_type										 transform_type;
typedef transform_type::vector3_type                                     vector_type;
typedef transform_type::quaternion_type                                  quaternion_type;
typedef transform_type::matrix3x3_type                                   matrix_type;

typedef OpenTissue::kinematics::inverse::NonlinearSolver< skeleton_type >   solver_type;
typedef OpenTissue::animation::NaiveBlendScheduler <skeleton_type>          blend_scheduler_type;
typedef solver_type::chain_type                                             chain_type;
typedef OpenTissue::animation::KeyframeAnimation <skeleton_type>            animation_type;
typedef OpenTissue::animation::KeyframeJointChannels <skeleton_type>        channels_type;

typedef          ublas::vector<double>                uvector_type;
typedef          ublas::vector_range<vector_type>        uvector_range;

class InverseKinematics
{
public:
	typedef std::map<const std::string, btTransform *> TargetSet;
	typedef std::map<const std::string, vector_type *> VectorSet;
	typedef std::map<const std::string, btQuaternion *> RotationSet;
	typedef std::map<const std::string, btVector3 *> PositionSet;
	typedef std::map<const std::string, chain_type *> ChainMap;
	typedef std::map<const std::string, Ogre::Bone*> OgreBoneMap;
	typedef std::map<const std::string, bone_type*> OpenBoneMap;
	typedef std::map<const std::string, btRigidBody*> BodyMap;
	typedef std::map<const std::string, btGeneric6DofConstraint*> ConstraintMap;
	typedef std::set<const std::string> NameList;
private:
	Ogre::Skeleton *ogreSkeleton;
	Ogre::PhysicsResource *physics;
	Ogre::Bone *rootBone;
	blend_scheduler_type scheduler;
	solver_type solver;
	animation_type animation;
	ChainMap chains;
	std::vector<channels_type*> channels;

	OgreBoneMap   ogreBones;
	OpenBoneMap   openBones;
	BodyMap       bodies;
	ConstraintMap constraints;
	TargetSet     pivots;
	PositionSet     pivotOffsets;
public:
	skeleton_type *openSkeleton;
	NameList boneNames;
	InverseKinematics(Ogre::Skeleton *s, Ogre::PhysicsResource *r);

	btRigidBody *getBody(const std::string &name) { return bodies[name]; }
	btQuaternion rotationFromEuler(const std::string &name, TargetSet &targ, double *eAngles);
	void processBone(Ogre::Bone *b);
	void getTrajectory(const RotationSet &rotationTargets, const PositionSet &positionTargets, const NameList &dontCare, std::vector<TargetSet> &trajectory/*, std::vector<PositionSet> &eulerAngles*/);

	void resetAnimation();
	void addAnimationFrame(float time);
	void updateOpenSkeleton();
	void updateBone(Ogre::Bone *b);
	void bindPose();
	void setBoneTransform(bone_type *b, const btTransform &tr);
	void setChainTransform(const std::string &name, const btTransform &tr);
	void setChainPosition(const std::string &name, const btVector3 &v);
	void setChainRotation(const std::string &name, const btQuaternion &q);
};

#endif