#ifndef PHYSICS_RESOURCE_MANAGER_H
#define PHYSICS_RESOURCE_MANAGER_H

#include "OgreResourceManager.h"

namespace Ogre
{

class PhysicsResourceManager : public ResourceManager, public Singleton<PhysicsResourceManager>
{
public:
	PhysicsResourceManager();
	~PhysicsResourceManager();

	ResourcePtr load(const String &name, const String &group, bool isManual = false, ManualResourceLoader *loader = 0, const NameValuePairList *loadParams = 0);

	ResourcePtr load(DataStreamPtr &stream, const String &group);

	static PhysicsResourceManager &getSingleton(void);
	static PhysicsResourceManager *getSingletonPtr(void);

protected:
	Resource *createImpl(const String &name, ResourceHandle handle, const String &group, bool isManual, ManualResourceLoader *loader, const NameValuePairList *createParams);
};

}

#endif