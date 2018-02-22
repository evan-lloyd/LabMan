#ifndef MeshStrider_h__
#define MeshStrider_h__

#include "btBulletDynamicsCommon.h"
#include "OgreMesh.h"
#include "OgreSubMesh.h"
#include "OgreEntity.h"

#define ASSERT(m)

/// Shares vertices/indexes between Ogre and Bullet
class MeshStrider : public btStridingMeshInterface{

public:
	MeshStrider( Ogre::Entity *e, int s ):mMesh(&*e->getMesh()), mEntity(e), sub(s) {}
	MeshStrider( Ogre::Mesh *m, int s):mMesh(m), mEntity(0), sub(s) {}

	void set( Ogre::Mesh * m ) { ASSERT(m); mMesh = m; }
	// inherited interface
	virtual int		getNumSubParts() const;

	virtual void	getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0);
	virtual void	getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart=0) const;

	virtual void	unLockVertexBase(int subpart);
	virtual void	unLockReadOnlyVertexBase(int subpart) const;

	virtual void	preallocateVertices(int numverts);
	virtual void	preallocateIndices(int numindices);
private:
	Ogre::Mesh * mMesh;
	Ogre::Entity *mEntity;
	int sub;
};


#endif // MeshStrider_h__