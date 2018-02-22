#include "PhysicsSceneManagerPlugin.h"
#include "OgreRoot.h"
#include "OgreLogManager.h"

namespace Ogre
{

const String sPluginName = "Physics Scene Manager";

PhysicsSceneManagerPlugin::PhysicsSceneManagerPlugin(void) : mPhysicsFactory(0), mPhysicsResourceManager(0)
{
}

const String &PhysicsSceneManagerPlugin::getName() const
{
	return sPluginName;
}

void PhysicsSceneManagerPlugin::install()
{
	mPhysicsFactory = OGRE_NEW PhysicsSceneManagerFactory();
}

void PhysicsSceneManagerPlugin::initialise()
{
	LogManager::getSingleton().logMessage("Physics: Loaded scene manager");
	Root::getSingleton().addSceneManagerFactory(mPhysicsFactory);
	mPhysicsResourceManager = OGRE_NEW PhysicsResourceManager();
}

void PhysicsSceneManagerPlugin::shutdown()
{
	Root::getSingleton().removeSceneManagerFactory(mPhysicsFactory);
	OGRE_DELETE mPhysicsResourceManager;
	mPhysicsResourceManager = 0;
}

void PhysicsSceneManagerPlugin::uninstall()
{
	OGRE_DELETE mPhysicsFactory;
	mPhysicsFactory = 0;
}

}