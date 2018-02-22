#include "PhysicsSceneManager.h"
#include "PhysicsResourceManager.h"
#include "PhysicsResource.h"
#include "OgreLogManager.h"

namespace Ogre
{

const String PhysicsSceneManagerFactory::FACTORY_TYPE_NAME = "PhysicsSceneManager";

void callback(btDynamicsWorld *w, btScalar time)
{
	PhysicsSceneManager *m = (PhysicsSceneManager*)w->getWorldUserInfo();
	PhysicsSceneManager::PhysicsListeners::iterator i = m->mPhysicsListeners.begin();
	for(; i != m->mPhysicsListeners.end(); i++)
	{
		(*i)->physicsCallback(w, time);
	}
}

PhysicsSceneManager::PhysicsSceneManager(const String &name) : SceneManager(name), debugDrawer(0), gravity(false), updates(false)
	{
		static int maxProxies = 1024;

		btVector3 worldAabbMin(-10000,-10000,-10000);
        btVector3 worldAabbMax(10000,10000,10000);
		btAxisSweep3* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

		PhysicsFilterCallback *filterCallback = new PhysicsFilterCallback(); // To allow objects that disallow self-collision

		// -85.7
		world->setGravity(btVector3(0,0,0));
		
		world->getPairCache()->setOverlapFilterCallback(filterCallback);

		btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

		world->setInternalTickCallback(callback, this);
	}

void PhysicsSceneManagerFactory::initMetaData() const
{
	mMetaData.typeName = FACTORY_TYPE_NAME;
	mMetaData.description = "Scene manager for creating physics scenes based on bullet";
	mMetaData.sceneTypeMask = Ogre::ST_GENERIC;
	mMetaData.worldGeometrySupported = true;
}

SceneManager *PhysicsSceneManagerFactory::createInstance(const String &instanceName)
{
	return OGRE_NEW PhysicsSceneManager(instanceName);
}

void PhysicsSceneManagerFactory::destroyInstance(SceneManager *instance)
{
	OGRE_DELETE instance;
}

SceneNode *PhysicsSceneManager::createPartNode(Part *part, const String &parentName, const btTransform &trans, short objectNum)
{
	std::stringstream name(ios_base::out);

	name << parentName << ":" << part->getID();

	SceneNode *node = createSceneNode(name.str());
	for(size_t i = 0; i < part->getSubParts().size(); i++)
	{
		node->addChild(createPartNode(part->getSubParts()[i], name.str(), trans, objectNum));
	}

	for(size_t i = 0; i < part->getBodies().size(); i++)
	{
		Body *b = part->getBodies()[i];

		std::stringstream bodyName(ios_base::out);
		bodyName << name.str() << ";" << b->getID();
		SceneNode *bnode = createSceneNode(bodyName.str());
		node->addChild(bnode);
		b->attachToNode(bnode);
		b->applyTransform(trans);

		if(b->type() == Body::Rigid)
		{
			// TODO: respect body's self-collision flags!
			//world->addRigidBody(static_cast<btRigidBody*>(b->getBody()));
			//world->addRigidBody(static_cast<btRigidBody*>(b->getBody()), objectNum, PhysicsFilterCallback::NoSelfCollision);
			if(b->nonCollideGroup > -1)
			{
				world->addRigidBody(static_cast<btRigidBody*>(b->getBody()), b->nonCollideGroup, PhysicsFilterCallback::NoSelfCollision);
			}
			else
				world->addRigidBody(static_cast<btRigidBody*>(b->getBody()), -1, 0);
		}
		//else if(b->type() == Body::Soft)
		//	world->addSoftBody(static_cast<btSoftBody*>(b->getBody()));
	}

	return node;
}

// Convenience method to create with identity orientation
PhysicsResource *PhysicsSceneManager::createObject(const String &filename, const btVector3 &trans, const String &groupname)
{
	return createObject(filename, btTransform(btQuaternion(0,0,0,1), trans), groupname);
}

PhysicsResource *PhysicsSceneManager::createObject(const String &filename, const btTransform &trans, const String &groupname)
{
	static short objectNum = 0; // TODO: support more than 64K objects!

	PhysicsResourcePtr res = static_cast<PhysicsResourcePtr>(PhysicsResourceManager::getSingleton().load(filename, groupname));
	PhysicsResource *inst = res->createInstance();

	SceneNode *objectNode = createSceneNode(inst->getName());
	getRootSceneNode()->addChild(objectNode);

	for(size_t i = 0; i < inst->getDisplayMeshes().size(); i++)
	{
		MeshReference *ref = inst->getDisplayMeshes()[i];
		std::stringstream meshname(ios_base::out);
		meshname << inst->getName() << "_mesh_" << i;
		Entity *ent = createEntity(meshname.str(), ref->getName());
		ref->attachEntity(ent);

		if(ref->isControlled())
		{
			// TODO: transforms!
			SceneNode *n = objectNode->createChildSceneNode();
			ref->attachToNode(n);

			//n->setPosition(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
			//n->setOrientation(trans.getRotation().getW(), trans.getRotation().getX(), trans.getRotation().getY(), trans.getRotation().getZ());
		}
	}

	for(size_t i = 0; i < inst->getParts().size(); i++)
	{
		objectNode->addChild(createPartNode(inst->getParts()[i], inst->getName(), trans, objectNum));
	}

	for(size_t i = 0; i < inst->getConstraints().size(); i++)
	{
		std::vector<btTypedConstraint*> &l = inst->getConstraints()[i]->getConstraints();
		for(size_t j = 0; j < l.size(); j++)
			world->addConstraint(l[j], true);
	}

	objectNum++;
	return inst;
}

}