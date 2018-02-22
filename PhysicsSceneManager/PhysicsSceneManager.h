#ifndef PHYSICS_SCENE_MANAGER_H
#define PHYSICS_SCENE_MANAGER_H

#define LOG(m) LogManager::getSingleton().logMessage(m)

#include "OgreSceneManager.h"
#include "OgreResourceGroupManager.h"
#include "DebugDrawer.h"
#include "PhysicsNode.h"
#include "PhysicsResource.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "PhysicsListener.h"
#include <list>

namespace Ogre
{

	class PhysicsSceneManagerFactory;
	class PhysicsSceneManager;
	class PhysicsResource;

class PhysicsSceneManagerFactory : public SceneManagerFactory
{
protected:
	void initMetaData() const;
public:
	PhysicsSceneManagerFactory() {}
	~PhysicsSceneManagerFactory() {}

	static const String FACTORY_TYPE_NAME;

	SceneManager *createInstance(const String &instanceName);
	void destroyInstance(SceneManager *instance);
};

class Part;

class PhysicsFilterCallback : public btOverlapFilterCallback
{
public:
	enum CollisionType { NoSelfCollision = 1 };
private:
	virtual bool	needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
	{
		if((proxy0->m_collisionFilterMask & NoSelfCollision && proxy1->m_collisionFilterMask & NoSelfCollision) && proxy0->m_collisionFilterGroup == proxy1->m_collisionFilterGroup)
		{
			return false; // disable self collision
		}

		return true;
	}
};

class PhysicsSceneManager : public SceneManager
{
public:
	typedef std::map<Ogre::String, PhysicsNode *> PhysicsNodeList;
	typedef std::map<Ogre::String, SkeletonNode *> SkeletonNodeList;
	typedef std::list<PhysicsListener *> PhysicsListeners;

	PhysicsSceneManager(const String &name);

	void toggleGravity()
	{
		gravity = !gravity;
		if(gravity)
			world->setGravity(btVector3(0,-85.7, 0));
		else
			world->setGravity(btVector3(0,0,0));
	}

	void addPhysicsListener(PhysicsListener *l)
	{
		mPhysicsListeners.push_back(l);
	}

	void debugRay(const Ogre::Ray &r)
	{
		debugDrawer->drawLine(cvt(r.getOrigin()), cvt(r.getOrigin() + 100.0f * r.getDirection()), btVector3(1.0, 1.0, 1.0));
	}

	void applyImpulseAlongRay(const Ray &r, float mag)
	{
		static float maxDist = 1000.0f;
		btVector3 from(r.getOrigin().x, r.getOrigin().y, r.getOrigin().z);
		btVector3 to(r.getDirection().x, r.getDirection().y, r.getDirection().z);
		to = to.normalize();
		to = from + (to * maxDist);
		btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);
		world->rayTest(from, to, rayCallback);
		if(rayCallback.hasHit())
		{
			btRigidBody *body = btRigidBody::upcast(rayCallback.m_collisionObject);
			if(body->getInvMass() != 0.0)
				mag /= body->getInvMass();
			if(body)
			{
				btVector3 rel = rayCallback.m_hitPointWorld - body->getCenterOfMassPosition();
				btVector3 impulse(r.getDirection().x, r.getDirection().y, r.getDirection().z);
				impulse = impulse.normalize();
				body->applyForce(impulse * mag, rel);
			}
		}
	}

	void setDebugDrawer(OgreDebugDrawer *d)
	{
		world->setDebugDrawer(d);
		debugDrawer = d;
	}

	const String &PhysicsSceneManager::getTypeName() const
	{
		return PhysicsSceneManagerFactory::FACTORY_TYPE_NAME;
	}

	RigidBodyNode *createRigidBodyNode(Entity *e, btRigidBody::btRigidBodyConstructionInfo &b, btTransform &tr, bool dynamic = true)
	{
		RigidBodyNode *n = new RigidBodyNode(this, e, b, tr);
		mSceneNodes[n->getName()] = n;

		if(dynamic)
			mDynamicNodes[n->getName()] = n;
		else
			mStaticNodes[n->getName()] = n;

		world->addRigidBody(n->getBody(), 0, 0);
		return n;
	}

	SkeletonNode *createSkeletonNode(Entity *e, SceneNode *p, Bone *b, double mass, btVector3 &s1, double r1, btVector3 &s2, double r2)
	{
		SkeletonNode *n = new SkeletonNode(this, e, p, b, mass, s1, r1, s2, r2);
		mSceneNodes[n->getName()] = n;
		mSkeletonNodes[n->getName()] = n;

		for(SkeletonNode::BodyList::iterator i = n->getBodies().begin(); i != n->getBodies().end(); i++)
		{
			world->addRigidBody(*i);
		}

		for(SkeletonNode::ConstraintList::iterator i = n->getConstraints().begin(); i != n->getConstraints().end(); i++)
		{
			world->addConstraint(*i, true);
		}

		return n;
		
	}

	void debugDisplay(bool show)
	{
		if(!debugDrawer)
			return;
		if(show)
			debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		else
			debugDrawer->setDebugMode(btIDebugDraw::DBG_NoDebug);
	}

	void sceneUpdates(bool up)
	{
		updates = up;
	}

	void updateScene(Real timeSinceLastFrame)
	{
		if(updates)
		{
			world->stepSimulation(timeSinceLastFrame,20, btScalar(1.0) / btScalar(120.0));
		}
		world->debugDrawWorld();

		/*for(PhysicsNodeList::iterator i = mDynamicNodes.begin(); i != mDynamicNodes.end(); i++)
		{
			(*i).second->update();
		}*/
	}

	PhysicsResource *createObject(const String &filename, const btVector3 &trans, const String &group = ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	PhysicsResource *createObject(const String &filename, const btTransform &trans = btTransform::getIdentity(), const String &group = ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SceneNode *createPartNode(Part *p, const String &parentName, const btTransform &trans, short objectNum);

	btDiscreteDynamicsWorld *getWorld() { return world; }
private:
	btDiscreteDynamicsWorld* world;
	OgreDebugDrawer *debugDrawer;

	PhysicsNodeList mDynamicNodes;
	PhysicsNodeList mStaticNodes;
	SkeletonNodeList mSkeletonNodes;
	PhysicsListeners mPhysicsListeners;

	friend void callback(btDynamicsWorld *w, btScalar time);

	bool gravity;
	bool updates;
};

}

#endif