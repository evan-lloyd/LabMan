#include "PoseController.h"
#include "PhysicsResource.h"

#include "btBulletDynamicsCommon.h"
//#include "CEGUI/CEGUI.h"

using namespace Ogre;

//float PoseController::maxImpulse = 80000;
float PoseController::torqueScale = 400;
float PoseController::impulseScale = 20000;
float PoseController::maxVelocity = 80000;

PoseController::PoseController(PhysicsResource *r, InverseKinematics::ConstraintMap &c) : obj(r)
{
	errorThreshold = 0.001f;
	totalError = 0.0f;

	PhysicsResource::PartList &parts = r->getParts();

	// TODO: follow complete hierarchy
	for(size_t i = 0; i < parts.size(); i++)
	{
		//if(parts[i]->getName() != "LowerBack") continue;
		Part::BodyList &b = parts[i]->getBodies();
		for(size_t j = 0; j < b.size(); j++)
		{
			if(c.find(parts[i]->getName()) != c.end())
				constraints.push_back(c[parts[i]->getName()]);
			else
				constraints.push_back(NULL);

			bodies.push_back(btRigidBody::upcast(b[j]->getBody()));
			bones.push_back(b[j]->getAttachments()[0]->getBone());
			targets.push_back(new btQuaternion(0, 0, 0, 1));
			maxTorques.push_back(b[j]->maxControllerTorque);
			torqueScales.push_back(b[j]->controllerTorqueScale);
			lastErrors.push_back(new btVector3(0, 0, 0));
			errorSums.push_back(new btVector3(0, 0, 0));
		}
	}

}

void PoseController::setTargets(const InverseKinematics::TargetSet &t, const InverseKinematics::NameList &dontCare)
{
	static float epsilon = 0.0001f;
	for(size_t i = 0; i < targets.size(); i++)
	{
		InverseKinematics::TargetSet::const_iterator it;
		if((it = t.find(bones[i]->getName())) == t.end())
			continue;

		btQuaternion target = ((*it).second)->getRotation();

		// convert to body reference frame?
		/*if(skeletonFrame)
		{
			Quaternion q = bones[i]->getInitialOrientation().UnitInverse();

			target = target * btQuaternion(q.x, q.y, q.z, q.w);
		}*/

		//target.normalize();

		//if(target.getW() > 1) // clamp W to one (don't rescale since this is likely just floating point error)
		//	target.setW(1.0f);
		//if(target.getW() < -1)
			//target.setW(-1.0f);

		*targets[i] = target;
	}
}

void PoseController::controlStep(double time)
{
	//static CEGUI::String defaultBoneName = "LowerBack";
	static float epsilon = 1e-4;

	totalError = 0.0f;

	for(size_t i = 0; i < bodies.size(); i++)
	{
		if(bodies[i]->isKinematicObject())
			continue;
		btQuaternion d = *targets[i] * bodies[i]->getCenterOfMassTransform().getRotation().inverse();
		float angle = fabs(2 * asin(sqrt(d.getX()*d.getX()+d.getY()*d.getY()+d.getZ()*d.getZ())));

		//if(d.getW() < 0)
			//angle = angle - Ogre::Math::PI;
		if(angle < epsilon)
			continue;

		//bodies[i]->applyCentralForce(-bodies[i]->getGravity() / bodies[i]->getInvMass());
		//continue;

		Quaternion q(d.getW(), d.getX(), d.getY(), d.getZ());

		btMatrix3x3 frame = bodies[i]->getInvInertiaTensorWorld();

		//btVector3 estimatedGravityError(0,0,0);
		//if(constraints[i])
		//	estimatedGravityError = (constraints[i]->getCalculatedTransformA().getOrigin() - bodies[i]->getWorldTransform().getOrigin()).cross(bodies[i]->getGravity() / bodies[i]->getInvMass());

		btVector3 targetDirection(q.getPitch().valueRadians(), q.getYaw().valueRadians(), q.getRoll().valueRadians());
		btVector3 targetInLocalFrame = (frame * targetDirection.normalized());

		btVector3 error(0,0,0);

		float integralScale = 0.125;
		float derivScale = 0.0;
		float velocityScale = 200;//maxTorques[i] == 0.0 ? impulseScale / bodies[i]->getInvMass() : maxTorques[i] / 4;
		float errorScale = 5;
		float velocity = angle * velocityScale > maxVelocity ? maxVelocity : angle * velocityScale;

		btVector3 targetVelocity = velocity * targetInLocalFrame;
		
		if(bodies[i]->getAngularVelocity().length2() > epsilon)
		{
			error = frame.inverse() * (targetVelocity - bodies[i]->getAngularVelocity());
				//(targetVelocity - (bodies[i]->getAngularVelocity() - (targetInLocalFrame * bodies[i]->getAngularVelocity().dot(targetInLocalFrame))));
		}
		else if(targetDirection.length2() > epsilon)
			error = frame.inverse() * targetVelocity;

		btVector3 deriv = (error - *lastErrors[i]) / time;

		float impulse;

		error = errorScale * error + integralScale * *errorSums[i] + (derivScale) * deriv;

		if(error.length2() > epsilon)
		{
			float maxT = maxTorques[i] == 0.0 ? maxVelocity / bodies[i]->getInvMass() : maxTorques[i] / bodies[i]->getInvMass();
			//maxT = maxT > 10 * bodies[i]->getAngularVelocity().length() / time ? 10 * bodies[i]->getAngularVelocity().length() / time : maxT;
			btVector3 impulseDirection = error.normalized();

			impulse = error.length() / bodies[i]->getInvMass();
			//impulse = impulse > maxT ? maxT : impulse;

			//if(frame * (impulse * time * impulseDirection)
			bodies[i]->applyTorqueImpulse(impulse * time * impulseDirection);
			*lastErrors[i] = error;
			*errorSums[i] += error * time;
			*errorSums[i] *= 0.9;

			totalError += error.length();
		}
		else
		{
			impulse = 0.0;
			
			if(targetVelocity.length2() > epsilon)
				bodies[i]->setAngularVelocity(targetVelocity);
			else
				bodies[i]->setAngularVelocity(btVector3(0,0,0));

			*lastErrors[i] = btVector3(0, 0, 0);
	
		}

		/*CEGUI::Window *td = static_cast<CEGUI::Window *>(CEGUI::WindowManager::getSingleton().getWindow("Impulse"));
		CEGUI::Window *ad = static_cast<CEGUI::Window *>(CEGUI::WindowManager::getSingleton().getWindow("Angle"));

		CEGUI::Combobox *selectedBone = static_cast<CEGUI::Combobox *>(CEGUI::WindowManager::getSingleton().getWindow("SelectedBone"));
		const CEGUI::String *bonename = &defaultBoneName;
		if(selectedBone->getSelectedItem())
			bonename = &selectedBone->getSelectedItem()->getText();

		if(bones[i]->getName() == *bonename)
		{
			std::stringstream newtext;
			newtext << "Impulse: " << impulse;
			td->setText(newtext.str());
			newtext.str("");
			if(targetDirection.length2() > epsilon)
				newtext << "Angle: " << angle;
			else
				newtext << "Angle: 0";
			ad->setText(newtext.str());
		}*/
	}
}