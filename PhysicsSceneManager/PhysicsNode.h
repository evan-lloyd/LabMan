#ifndef PHYSICSNODE_H
#define PHYSICSNODE_H

#include "btBulletDynamicsCommon.h"
#include "OgreSceneNode.h"
#include "OgreSceneManager.h"
#include "OgreEntity.h"
#include "OgreBone.h"
#include <vector>

using namespace Ogre;
using namespace std;

class PhysicsNode : public SceneNode, public btMotionState
{
protected:
	Entity *mesh;
public:
	PhysicsNode(SceneManager *s) : mesh(NULL), SceneNode(s) {}
	PhysicsNode(SceneManager *s, Entity *ent) : mesh(ent), SceneNode(s) { attachObject(mesh); }

	void setMesh(Entity *ent)
	{
		if(mesh)
			detachObject(mesh);
		mesh = ent;
		attachObject(mesh);
	}

	//void getWorldTransform(btTransform &tr) const
	//{
	//	tr = trans;
	//}

	//void setWorldTransform(const btTransform &tr)
	//{
	//	trans = tr;
	//	update();
	//}

	//virtual void update() = 0;
};

class RigidBodyNode : public PhysicsNode
{
protected:
	btRigidBody *body;
public:
//	RigidBodyNode(SceneManager *s) : body(NULL), PhysicsNode(s) {}
//	RigidBodyNode(SceneManager *s, btRigidBody *b) : body(b), PhysicsNode(s) {}
	RigidBodyNode(SceneManager *s, Entity *ent, btRigidBody::btRigidBodyConstructionInfo &i, btTransform &tr) : PhysicsNode(s, ent)
	{
		body = new btRigidBody(i);
		body->setMotionState(this);
		body->setWorldTransform(tr);
		body->setActivationState(DISABLE_DEACTIVATION);
	}

	void setWorldTransform(const btTransform &trans)
	{
		setPosition(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
		setOrientation(trans.getRotation().getW(), trans.getRotation().getX(), trans.getRotation().getY(), trans.getRotation().getZ());
	}

	void getWorldTransform(btTransform &tr) const
	{
		tr = body->getWorldTransform();
	}

	btRigidBody *getBody() { return body; }

};

class SkeletonNode : public SceneNode, public btMotionState
{
public:
	typedef std::vector<btTypedConstraint*> ConstraintList;
	typedef std::vector<btRigidBody*> BodyList;

protected:
	btRigidBody *body;
	SceneNode *parent;
	Bone *bone;
	Entity *entity;

	btConeTwistConstraint *constraint;

	ConstraintList constraints;
	BodyList       bodies;

	btTransform boundOffset;
public:
	SkeletonNode(SceneManager *sm, Entity *e, SceneNode *p, Bone *b, double mass, btVector3 &s1, double r1, btVector3 &s2, double r2) : SceneNode(sm), bone(b)
	{
		//entity = e;

		//p->addChild(this);
		sm->getRootSceneNode()->addChild(this);
		
		double x1 = min<double>(s1.getX() - r1, s2.getX() - r2);
		double x2 = max<double>(s1.getX() + r1, s2.getX() + r2);
		double y1 = min<double>(s1.getY() - r1, s2.getY() - r2);
		double y2 = max<double>(s1.getY() + r1, s2.getY() + r2);
		double z1 = min<double>(s1.getZ() - r1, s2.getZ() - r2);
		double z2 = max<double>(s1.getZ() + r1, s2.getZ() + r2);

		btVector3 boundingHalfWidths((x2 - x1) / 2,
		                             (y2 - y1) / 2,
									 (z2 - z1) / 2);

		entity = sm->createEntity(getName() + "_bounds", SceneManager::PrefabType::PT_CUBE);
		attachObject(entity);
		setScale(boundingHalfWidths.getX() / 50, boundingHalfWidths.getY() / 50, boundingHalfWidths.getZ() / 50);

		boundOffset = btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
		                //btVector3(x1 + boundingHalfWidths.getX(), y1 + boundingHalfWidths.getY(), z1 + boundingHalfWidths.getZ()));

		s1 += btVector3(p->getPosition().x, p->getPosition().y, p->getPosition().z);
		s2 += btVector3(p->getPosition().x, p->getPosition().y, p->getPosition().z);

		btVector3 worldPos = s2 + 0.5 * (s1 - s2);

		btVector3 s[2] = {s1 - worldPos, s2 - worldPos};
		btScalar r[2] = {r1, r2};
		btMultiSphereShape *shape = new btMultiSphereShape(/*boundingHalfWidths,*/ s, r, 2);
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(mass, inertia);
		
		//btVector3 centerOfMass = 0.5 * (s1 - s2);
		btRigidBody::btRigidBodyConstructionInfo CI(mass, NULL, shape, inertia);
		body = new btRigidBody(CI);
		body->setMotionState(this);
		body->setActivationState(DISABLE_DEACTIVATION);
		body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), worldPos));
		//body->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), centerOfMass));

		// delete below
		btSphereShape *pivotShape = new btSphereShape(0.01);
		btRigidBody::btRigidBodyConstructionInfo pivotCI(0, new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), s1)), pivotShape);
		btRigidBody *pivot = new btRigidBody(pivotCI);

		constraint = new btConeTwistConstraint(*body, *pivot, btTransform(btQuaternion(0, 0, 0, 1), (0.55 + r1) * (s1 - s2) / (s1 - s2).length()), btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
		constraint->setLimit(Math::HALF_PI, Math::HALF_PI, 0);
		constraints.push_back(constraint);

		bodies.push_back(body);
		bodies.push_back(pivot);
	}

	void setWorldTransform(const btTransform &trans)
	{
		bone->setOrientation(trans.getRotation().getW(), trans.getRotation().getX(), trans.getRotation().getY(), trans.getRotation().getZ());
		//bone->setPosition(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
		//bone->roll(Radian(0.1));

		//btTransform disp = trans * boundOffset;

		setPosition(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
		setOrientation(trans.getRotation().getW(), trans.getRotation().getX(), trans.getRotation().getY(), trans.getRotation().getZ());
	}

	void getWorldTransform(btTransform &trans) const
	{
		trans = body->getWorldTransform();
	}

	btRigidBody *getBody() { return body; }
	BodyList &getBodies () { return bodies; }
	ConstraintList &getConstraints() { return constraints; }
};

#endif