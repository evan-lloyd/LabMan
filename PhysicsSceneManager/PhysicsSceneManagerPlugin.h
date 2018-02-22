#ifndef PHYSICS_SCENE_MANAGER_PLUGIN_H
#define PHYSICS_SCENE_MANAGER_PLUGIN_H

#include "OgrePlugin.h"
#include "PhysicsSceneManager.h"
#include "PhysicsResourceManager.h"

namespace Ogre
{
	class PhysicsSceneManagerPlugin : public Plugin
	{
	public:
		PhysicsSceneManagerPlugin(void);
		
		const String& getName() const;
		void install();
		void initialise();
		void shutdown();
		void uninstall();

	protected:
		PhysicsSceneManagerFactory *mPhysicsFactory;
		PhysicsResourceManager *mPhysicsResourceManager;

	};
}

#endif