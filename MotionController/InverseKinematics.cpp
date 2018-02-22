#include "btBulletDynamicsCommon.h"
#include "LinearMath/btTransformUtil.h"
#include "OgreBone.h"

#include "OgreLogManager.h"
#include "InverseKinematics.h"

btQuaternion InverseKinematics::rotationFromEuler(const std::string &name, TargetSet &targ, double *eAngles)
{
	btQuaternion bq;
	if(constraints.find(name) != constraints.end())
	{
		// Project onto allowed values, since the neural net may output bad ones
		eAngles[0] = std::max(constraints[name]->getRotationalLimitMotor(0)->m_loLimit, btScalar(eAngles[0]));
		eAngles[1] = std::max(constraints[name]->getRotationalLimitMotor(1)->m_loLimit, btScalar(eAngles[1]));
		eAngles[2] = std::max(constraints[name]->getRotationalLimitMotor(2)->m_loLimit, btScalar(eAngles[2]));

		eAngles[0] = std::min(constraints[name]->getRotationalLimitMotor(0)->m_hiLimit, btScalar(eAngles[0]));
		eAngles[1] = std::min(constraints[name]->getRotationalLimitMotor(1)->m_hiLimit, btScalar(eAngles[1]));
		eAngles[2] = std::min(constraints[name]->getRotationalLimitMotor(2)->m_hiLimit, btScalar(eAngles[2]));


		btMatrix3x3 Fp = constraints[name]->getFrameOffsetA().getBasis();
		btMatrix3x3 FBinv = constraints[name]->getFrameOffsetB().getBasis().inverse();
		Ogre::Matrix3 om; om.FromEulerAnglesXYZ(Ogre::Radian(eAngles[0]), Ogre::Radian(eAngles[1]), Ogre::Radian(eAngles[2]));
		Ogre::Quaternion ot; ot.FromRotationMatrix(om); ot.normalise();
		btMatrix3x3 Rtheta; Rtheta.setRotation(btQuaternion(ot.x, ot.y, ot.z, ot.w));
		(targ[ogreSkeleton->getBone(name)->getParent()->getName()]->getBasis() * Fp * Rtheta.inverse() * FBinv).getRotation(bq);
		bq.normalize();
	}
	else
	{
		Ogre::Matrix3 om; om.FromEulerAnglesXYZ(Ogre::Radian(eAngles[0]), Ogre::Radian(eAngles[1]), Ogre::Radian(eAngles[2]));
		Ogre::Quaternion ot; ot.FromRotationMatrix(om); ot.normalise();
		btMatrix3x3 Rtheta; Rtheta.setRotation(btQuaternion(ot.x, ot.y, ot.z, ot.w));
		Rtheta.inverse().getRotation(bq);
		bq.normalize();
	}

	return bq;
}

inline std::ostream &operator<<(std::ostream &str, const btTransform &tr)
{
	str << "[[" << tr.getOrigin().x() << "," << tr.getOrigin().y() << "," << tr.getOrigin().z()
	    << "][" << tr.getRotation().w() << "," << tr.getRotation().x() << "," << tr.getRotation().y()
		<< "," << tr.getRotation().z() << "]]";
	return str;
}

inline btQuaternion bulletQuaternion(const Ogre::Quaternion &q)
{
	return btQuaternion(q.x, q.y, q.z, q.w);
}

inline btVector3 bulletVector(const Ogre::Vector3 &v)
{
	return btVector3(v.x, v.y, v.z);
}

inline btQuaternion bulletQuaternion(const quaternion_type &q)
{
	return btQuaternion(q.v()[0], q.v()[1], q.v()[2], q.s());
}

inline btVector3 bulletVector(const vector_type &v)
{
	return btVector3(v[0], v[1], v[2]);
}

inline vector_type openVector(const btVector3 &v)
{
	return vector_type(v.x(), v.y(), v.z());
}

inline quaternion_type openQuaternion(const btQuaternion &q)
{
	quaternion_type ret;
	ret.s() = q.w();
	ret.v()[0] = q.x();
	ret.v()[1] = q.y();
	ret.v()[2] = q.z();

	return ret;
}

inline btTransform bulletTransform(const Ogre::Vector3 &v, const Ogre::Quaternion &q)
{
	return btTransform(bulletQuaternion(q), bulletVector(v));
}

Ogre::Quaternion ogreQuaternion(const btQuaternion &q)
{
	return Ogre::Quaternion(q.w(), q.x(), q.y(), q.z());
}

InverseKinematics::InverseKinematics(Ogre::Skeleton *s, Ogre::PhysicsResource *r) : ogreSkeleton(s), physics(r)
{
	openSkeleton = new skeleton_type();
	Ogre::Skeleton::BoneIterator it = ogreSkeleton->getRootBoneIterator();

	while(it.hasMoreElements())
		processBone(rootBone = it.getNext());

	solver.init(*openSkeleton);
}

inline transform_type absoluteToRelative(const transform_type &absolute, const transform_type &parent)
{
	transform_type relative;

	relative.T() = OpenTissue::math::inverse(parent).Q().rotate(absolute.T() - parent.T());
	relative.Q() = prod(OpenTissue::math::inverse(parent).Q(), absolute.Q());

	return relative;
}

transform_type relativeToAbsolute(const bone_type *b, const transform_type &tr)
{
	if(!b->is_root())
		return bone_type::compute_absolute_pose_transform(b->parent()->absolute(), tr);

	return transform_type(tr);
}

inline btTransform bulletTransform(const transform_type &t)
{
	transform_type tr = t;
	return btTransform(bulletQuaternion(tr.Q()), bulletVector(tr.T()));
}

transform_type bulletToOpen(const btTransform &tr)
{
	transform_type t;
	t.T() = openVector(tr.getOrigin());
	t.Q() = openQuaternion(tr.getRotation());

	return t;
}

static btScalar btGetMatrixElem(const btMatrix3x3& mat, int index)
{
         int i = index%3;
         int j = index/3;
         return mat[i][j];
}

void getXYZ(const btQuaternion &q, double angles[3])
{
	btMatrix3x3 mat(q);

	if (btGetMatrixElem(mat,2) < btScalar(1.0))
     {
             if (btGetMatrixElem(mat,2) > btScalar(-1.0))
             {
                     angles[0] = btAtan2(-btGetMatrixElem(mat,5),btGetMatrixElem(mat,8));
                     angles[1] = btAsin(btGetMatrixElem(mat,2));
                     angles[2] = btAtan2(-btGetMatrixElem(mat,1),btGetMatrixElem(mat,0));
             }
             else
             {
                     // WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
                     angles[0] = -btAtan2(btGetMatrixElem(mat,3),btGetMatrixElem(mat,4));
                     angles[1] = -SIMD_HALF_PI;
                     angles[2] = btScalar(0.0);
            }
     }
     else
     {
             // WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
             angles[0] = btAtan2(btGetMatrixElem(mat,3),btGetMatrixElem(mat,4));
             angles[1] = SIMD_HALF_PI;
             angles[2] = 0.0;

     }
}

void getXYZ(const quaternion_type &q, double angles[3])
{
//      // rot =  cy*cz          -cy*sz           sy
//      //        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
//      //       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
//
 
	getXYZ(bulletQuaternion(q), angles);
}

void InverseKinematics::processBone(Ogre::Bone *b)
{
	const Ogre::String &name = b->getName();
	bone_type *openBone;
	if(b->getParent())
		openBone = openSkeleton->create_bone(openBones[b->getParent()->getName()]);
	else
		openBone = openSkeleton->create_bone();
	openBone->set_name(name);

	bodies[name] = btRigidBody::upcast(physics->getPart(name)->getBody(0)->getBody());

	ogreBones[name] = b;
	openBones[name] = openBone;
	

	if(!openBone->is_root())
	{
		const Ogre::String &pName = b->getParent()->getName();
		Ogre::Part *p = physics->getPart(name), *pp = physics->getPart(pName);

		if(p && pp)
		{
			for(size_t i = 0; i < physics->getConstraints().size(); i++)
			{
				if(physics->getConstraints()[i]->getBody(0)->getPartName() != pName)
					continue;
				if(physics->getConstraints()[i]->getBody(1)->getPartName() != name)
					continue;

				openBone->type() = bone_type::ball_type;
				btGeneric6DofConstraint *c = static_cast<btGeneric6DofConstraint*>(physics->getConstraints()[i]->getConstraint(0));

				constraints[name] = c;
				
				openBone->min_joint_limit(0) = c->getRotationalLimitMotor(0)->m_loLimit;
				openBone->min_joint_limit(1) = c->getRotationalLimitMotor(1)->m_loLimit;
				openBone->min_joint_limit(2) = c->getRotationalLimitMotor(2)->m_loLimit;
 
				openBone->max_joint_limit(0) = c->getRotationalLimitMotor(0)->m_hiLimit;
				openBone->max_joint_limit(1) = c->getRotationalLimitMotor(1)->m_hiLimit;
				openBone->max_joint_limit(2) = c->getRotationalLimitMotor(2)->m_hiLimit;

				c->calculateTransforms();

				pivots[name] = new btTransform(bodies[name]->getWorldTransform().getRotation(), c->getCalculatedTransformB().getOrigin());
				pivotOffsets[name] = new btVector3(bodies[name]->getWorldTransform().getBasis().inverse() * -(pivots[name]->getOrigin() - bodies[name]->getWorldTransform().getOrigin()));
			}
		}
	}
	else
	{
		openBone->min_joint_limit(0) = -0.5;
		openBone->min_joint_limit(1) = -.3;
		openBone->min_joint_limit(2) = 0.0;
		openBone->max_joint_limit(0) = 0.5;
		openBone->min_joint_limit(1) = 0.3;
		openBone->min_joint_limit(2) = 0.0;
		pivotOffsets[name] = new btVector3(0, 0, 0);
	}

	if(bodies[name]->isStaticOrKinematicObject())
	{
		return; // don't include fixed bones
	}

	boneNames.insert(name);
	transform_type bindPose;

	if(!openBone->is_root())
		bindPose = bulletToOpen(*pivots[name]);
	else
		bindPose = bulletToOpen(bodies[name]->getWorldTransform());
	openBone->absolute() = bindPose;

	if(!openBone->is_root())
	{
		openBone->pivot = bulletToOpen(btTransform(constraints[name]->getFrameOffsetB().getRotation(), pivots[name]->getOrigin() - bodies[name]->getWorldTransform().getOrigin()));
		openBone->parent_pivot = bulletToOpen(btTransform(constraints[name]->getFrameOffsetA().getRotation(), pivots[name]->getOrigin() - bodies[openBone->parent()->get_name()]->getWorldTransform().getOrigin()));
	}

	openBone->inv_pivot = OpenTissue::math::inverse(openBone->pivot);

	if(!openBone->is_root())
		bindPose = absoluteToRelative(bindPose, openBone->parent()->absolute());

	openBone->bind_pose() = bindPose;
	openBone->relative() = bindPose;

	Ogre::Node::ChildNodeIterator it = b->getChildIterator();
	while(it.hasMoreElements())
		processBone(static_cast<Ogre::Bone*>(it.getNext()));

}

void InverseKinematics::updateBone(Ogre::Bone *b)
{
	if(b->getParent())
	{
		constraints[b->getName()]->calculateTransforms();

		setBoneTransform(openBones[b->getName()], constraints[b->getName()]->getCalculatedTransformB());
	}
	else
		setBoneTransform(openBones[b->getName()], bodies[b->getName()]->getWorldTransform());
}

// Set the *relative* transform of the bone to the *world* Bullet transform
void InverseKinematics::setBoneTransform(bone_type *b, const btTransform &tr)
{
	double angles[3];

	if(!b->is_root())
	{
		angles[0] = constraints[b->get_name()]->getAngle(0);
		angles[1] = constraints[b->get_name()]->getAngle(1);
		angles[2] = constraints[b->get_name()]->getAngle(2);
	}
	else
		getXYZ(bulletToOpen(tr).Q() /*% OpenTissue::math::conj(b->bind_pose().Q())*/, angles);

	uvector_type v(3);
	v(0) = angles[0];
	v(1) = angles[1];
	v(2) = angles[2];

	bone_type::set_theta(*b, subrange(v, 0, 3));
}

void InverseKinematics::setChainRotation(const std::string &name, const btQuaternion &q)
{
	matrix_type m = matrix_type(openQuaternion(q));
	chains[name]->x_global() = m.column(0);
	chains[name]->y_global() = m.column(1);
}

void InverseKinematics::setChainPosition(const std::string &name, const btVector3 &v)
{
	chains[name]->p_global() = openVector(v);
}

void InverseKinematics::updateOpenSkeleton()
{
	skeleton_type::bone_iterator it = openSkeleton->begin();

	for(; it != openSkeleton->end(); it++)
	{
		if(boneNames.find(it->get_name()) != boneNames.end())
			updateBone(ogreSkeleton->getBone(it->get_name()));
	}

	openSkeleton->compute_pose();
}

void InverseKinematics::addAnimationFrame(float time)
{
	for(size_t i = 0; i < channels.size(); i++)
	{
		bone_type *bone = openSkeleton->get_bone(channels[i]->get_bone_number());

		channels[i]->add_key(time, bone->relative());
	}
}

void InverseKinematics::bindPose()
{
	for(skeleton_type::bone_iterator bone = openSkeleton->begin(); bone != openSkeleton->end(); bone++)
	{
		if(!bone->is_root())
			bone_type::set_theta(*bone, 0, 0, 0);
		else
			bone->relative() = bone->absolute() = bone->bind_pose();
	}
	openSkeleton->compute_pose();

	for(skeleton_type::bone_iterator bone = openSkeleton->begin(); bone != openSkeleton->end(); bone++)
	{
		if(bodies[bone->get_name()]->isKinematicObject())
			continue;
		btTransform transform = bulletTransform(bone->absolute());
	
		if(!bone->is_root())
		{
			// Location of pivot in absolute coordinates
			btVector3 pivot = bodies[bone->parent()->get_name()]->getWorldTransform() * constraints[bone->get_name()]->getFrameOffsetA().getOrigin();
			// Relative position of the pivot to center of mass
			btVector3 framePos = transform.getBasis() * constraints[bone->get_name()]->getFrameOffsetB().getOrigin();
			// Absolute position of the center of mass
			transform.setOrigin(pivot - framePos);
		}
		bodies[bone->get_name()]->setWorldTransform(transform);
		bodies[bone->get_name()]->getMotionState()->setWorldTransform(transform);
	}
}

void InverseKinematics::getTrajectory(const RotationSet &rotationTargets, const PositionSet &positionTargets, const NameList &dontCare, std::vector<TargetSet> &trajectory/*, std::vector<PositionSet> &eulerAngles*/)
{
	resetAnimation();

	updateOpenSkeleton();

	addAnimationFrame(0.0f);

	// Set inverse kinematics targets; if the target is not defined, just try to keep the bone still
	solver.clear_chains();
	for(NameList::iterator name = boneNames.begin(); name != boneNames.end(); name++)
	{
		if(dontCare.find(*name) != dontCare.end())
		{
			openBones[*name]->update_theta = false;
			continue;
		}

		openBones[*name]->update_theta = true;

		chain_type chain;
		chain.init(openSkeleton->root(), openSkeleton->get_bone(*name));
		solver.add_chain(chain);
		chains[*name] = &(*--solver.chain_end());
		chains[*name]->p_local() = openVector(*pivotOffsets[*name]);

		bool foundPos, foundRot;
		if(foundPos = (positionTargets.find(*name) != positionTargets.end()))
  		{
  			setChainPosition(*name, *positionTargets.find(*name)->second);
 			chains[*name]->set_weight_p(1);
 			chains[*name]->set_weight_x(0);
 			chains[*name]->set_weight_y(0);
 			chains[*name]->only_position() = true;
  		}
 		if(foundRot = (rotationTargets.find(*name) != rotationTargets.end()))
		{
			setChainRotation(*name, *rotationTargets.find(*name)->second);
			if(!foundPos)
 				chains[*name]->set_weight_p(0);
 			chains[*name]->set_weight_x(1);
 			chains[*name]->set_weight_y(1);
 			chains[*name]->only_position() = false;
  		}
		if(!foundPos && !foundRot)		
		{
 			// Set chain target to current configuration so that it won't move
  			setChainRotation(*name, bodies[*name]->getWorldTransform().getRotation());
 			setChainPosition(*name, bodies[*name]->getWorldTransform().getOrigin());
 			chains[*name]->only_position() = false;
 			chains[*name]->set_weight_p(1);
 			chains[*name]->set_weight_x(1);
 			chains[*name]->set_weight_y(1);
  		}
	}

	static solver_type::Output out;
	out.status() = OpenTissue::math::optimization::OK;
	static solver_type::Settings settings = solver_type::default_SDF_settings();
	settings.max_iterations() = 1000;
	//settings.tau() = 0.0005;
	settings.relative_tolerance() = 1e-6;
	while(out.status() == OpenTissue::math::optimization::OK)
		solver.solve(&settings, &out);
	if(out.status() == OpenTissue::math::optimization::OK)
		Ogre::LogManager::getSingleton().logMessage("still iterating");
	else if(out.status() == OpenTissue::math::optimization::RELATIVE_CONVERGENCE)
		Ogre::LogManager::getSingleton().logMessage("relative convergance");
	else if(out.status() == OpenTissue::math::optimization::ABSOLUTE_CONVERGENCE)
		Ogre::LogManager::getSingleton().logMessage("absolute convergance");
	else if(out.status() == OpenTissue::math::optimization::STAGNATION)
		Ogre::LogManager::getSingleton().logMessage("stagnation");
	else if(out.status() == OpenTissue::math::optimization::BACKTRACKING_FAILED)
		Ogre::LogManager::getSingleton().logMessage("backtracking failed");
	else if(out.status() == OpenTissue::math::optimization::ITERATING)
		Ogre::LogManager::getSingleton().logMessage("still iterating");
	else
	{
		char st[256];
		sprintf(st, "Unknown status: %d", out.status());
		Ogre::LogManager::getSingleton().logMessage(st);
	}

	addAnimationFrame(1.0f);

	scheduler.add(&animation);
	
	std::vector<std::vector<transform_type>> samples = sample_motion(scheduler, *openSkeleton, trajectory.size());
	// assert(samples[i].size() == trajectory.size());
	// set samples
	for(size_t i = 0; i < samples.size(); i++)
	{
		bone_type *bone = openSkeleton->get_bone(i);
		const std::string &name = bone->get_name();
		for(size_t j = 0; j < samples[i].size(); j++)
		{
			//if(!openBones[name]->update_theta) // not updated => don't care, and not in a chain we do care about
				//break;
			if(boneNames.find(name) == boneNames.end())
			{
				constraints[name]->calculateTransforms();
				btTransform transform = bodies[name]->getWorldTransform();
				btTransform parentTrans;
				if(boneNames.find(bone->parent()->get_name()) != boneNames.end())
					parentTrans = *trajectory[samples[i].size()-1][bone->parent()->get_name()];
				else
					parentTrans = bodies[bone->parent()->get_name()]->getWorldTransform();

				btVector3 pivot = parentTrans * constraints[name]->getFrameOffsetA().getOrigin();
				btVector3 framePos = transform.getBasis() * constraints[name]->getFrameOffsetB().getOrigin();
				transform.setOrigin(pivot - framePos);
				////
				//bodies[name]->setWorldTransform(transform);
				//bodies[name]->getMotionState()->setWorldTransform(transform);
				break;
			}

			btTransform transform = bulletTransform(samples[i][j]);
			
			if(!bone->is_root())
			{
				// Location of pivot in absolute coordinates
				btVector3 pivot = *trajectory[j][bone->parent()->get_name()] * constraints[name]->getFrameOffsetA().getOrigin();
				// Relative position of the pivot to center of mass
				btVector3 framePos = transform.getBasis() * constraints[name]->getFrameOffsetB().getOrigin();
				// Absolute position of the center of mass
				transform.setOrigin(pivot - framePos);

				double angles[3];
				btQuaternion q = ((*trajectory[j][bone->parent()->get_name()] * constraints[name]->getFrameOffsetA()).inverse() * transform * constraints[name]->getFrameOffsetB()).getRotation();
				getXYZ(q, angles);
				trajectory[j][name]->setRotation(rotationFromEuler(name, trajectory[j], angles));
				//*eulerAngles[j][name] = btVector3(angles[0], angles[1], angles[2]);
			}
			else
			{
				double angles[3];
				getXYZ(transform.getRotation(), angles);
				trajectory[j][name]->setRotation(rotationFromEuler(name, trajectory[j], angles));
				//*eulerAngles[j][name] = btVector3(angles[0], angles[1], angles[2]);
			}

			*(trajectory[j][name]) = transform;
	
			////
			if(j == samples[i].size() - 1)
			{
				//bodies[name]->setCenterOfMassTransform(transform);
				//bodies[name]->getMotionState()->setWorldTransform(transform);
			}

			////
			if(j == samples[i].size() - 1 && (rotationTargets.find(name) != rotationTargets.end() || positionTargets.find(name) != positionTargets.end()))
			{
				std::stringstream str;
				str << name << ": " << bone_type::transform_point(  chains[name]->get_end_effector()->absolute(),  chains[name]->p_local() ) << " -> " << chains[name]->p_global();
				Ogre::LogManager::getSingleton().logMessage(str.str());
			}
		}
	}
}

void InverseKinematics::resetAnimation()
{
	animation.clear();
	channels.clear();
	scheduler.clear();

	skeleton_type::bone_iterator it = openSkeleton->begin();
	for(; it != openSkeleton->end(); it++)
	{
		channels_type *channel = animation.create_joint_channels();
		channel->set_bone_number(it->get_number());
		channels.push_back(channel);
	}
}