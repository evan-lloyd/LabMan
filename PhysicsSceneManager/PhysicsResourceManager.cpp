#include "PhysicsResourceManager.h"
#include "PhysicsResource.h"
#include "OgreLogManager.h"
#include "tinyxml.h"

namespace Ogre
{

template<> PhysicsResourceManager *Singleton<PhysicsResourceManager>::ms_Singleton = 0;

PhysicsResourceManager *PhysicsResourceManager::getSingletonPtr()
{
	return ms_Singleton;
}
PhysicsResourceManager &PhysicsResourceManager::getSingleton()
{
	return *ms_Singleton;
}

PhysicsResourceManager::PhysicsResourceManager()
{
	LogManager::getSingleton().logMessage("Physics: Loaded resource manager");
	mResourceType = "PhysicsData";

	ResourceGroupManager::getSingleton()._registerResourceManager(mResourceType, this);
}

PhysicsResourceManager::~PhysicsResourceManager()
{
	LogManager::getSingleton().logMessage("Physics: Unloaded resource manager");
	ResourceGroupManager::getSingleton()._unregisterResourceManager(mResourceType);
}

ResourcePtr PhysicsResourceManager::load(const String &name, const String &group, bool isManual, ManualResourceLoader *loader, const NameValuePairList *loadParams)
{
	return ResourceManager::load(name, group, isManual, loader, loadParams);
}

ResourcePtr PhysicsResourceManager::load(DataStreamPtr &stream, const String &group)
{
	//MessageBoxA(NULL, "", "", 0);
	ResourcePtr ret = create("PhysicsData", group, true, 0);
	PhysicsResourcePtr physics = ret;
	physics->load(stream);

	return ret;
}

Resource *PhysicsResourceManager::createImpl(const String &name, ResourceHandle handle, const String &group, bool isManual, ManualResourceLoader *loader, const NameValuePairList *createParams)
{
	return OGRE_NEW PhysicsResource(this, name, handle, group, isManual, loader);
}

}