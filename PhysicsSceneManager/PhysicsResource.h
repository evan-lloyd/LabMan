#ifndef PHYSICS_RESOURCE_H
#define PHYSICS_RESOURCE_H

//#include "PhysicsSceneManager.h"
#include "OgreResource.h"
#include "OgreSceneNode.h"
#include "OgreMesh.h"
#include "OgreBone.h"
#include "OgreResourceGroupManager.h"
#include "OgreMeshManager.h"
#include "tinyxml.h"
#include "btBulletDynamicsCommon.h"
#include <vector>
#include <limits>
#include "GIMPACTUtils/btGImpactConvexDecompositionShape.h"

class PhysicsResourceManager;

// TODO: break this up into more files

namespace Ogre
{

class Part;
class Constraint;
class MeshAttachment;
struct PhysicsInstance;
class PhysicsResource;

class MeshReference
{
private:
	int id;
	String name;
	String groupName;
	MeshPtr mesh;
	Vector3 scale;
	bool controlled;

	Entity *entity;

	std::vector<MeshAttachment *> attachments;
public:
	MeshReference(int i, String n, bool c, const Vector3 &s, String g) : id(i), name(n), groupName(g), scale(s), controlled(c), entity(0)
	{
		mesh = MeshManager::getSingleton().load(name, groupName);
		/*Skeleton *skel = const_cast<Skeleton*>(getSkeleton());
		Skeleton::BoneIterator it = skel->getBoneIterator();
		while(it.hasMoreElements())
		{
			Bone *b = it.getNext();
			b->setManuallyControlled(true);
			b->setInheritOrientation(false);
		}*/
	}

	MeshPtr getMesh() { return mesh; }

	int getID() const { return id; }
	const String &getName() const { return name; }
	const String &getGroup() const { return groupName; }
	const Vector3 &getScale() const { return scale; }
	std::vector<MeshAttachment *> &getAttachments() { return attachments; }

	bool isControlled() const { return controlled; }

	void attach(MeshAttachment *a) { attachments.push_back(a); }
	void attachToNode(SceneNode *n);

	Entity *getEntity() { return entity; }
	void attachEntity(Entity *e);

	const Skeleton *getSkeleton() { return &*mesh->getSkeleton(); } 

	MeshReference *createInstance() const;
	void construct(PhysicsResource *r);

	static MeshReference *fromXML(TiXmlElement *el);
};

class PhysicsResource : public Resource
{
	friend class PhysicsResourceManager;
public:
	typedef std::vector<MeshReference *> MeshList;
	typedef std::vector<Part*> PartList;
	typedef std::vector<Constraint *> ConstraintList;
	typedef std::map<String, int> IndexMap;

	btTypedConstraint *findConstraint(const String &parent, const String &child);

	void load(DataStreamPtr &stream);

	PhysicsResource(ResourceManager *creator, const String &name, ResourceHandle handle, const String &group, bool isManual = false, ManualResourceLoader *loader = 0);
	~PhysicsResource();

	PartList *getPartList() { return &parts; }
	Part *getPart(const String &n);

	static btVector3 parseVector(const char *v);
	static btScalar parseScalar(const char *v);
	static btTransform parseTransform(TiXmlElement *el);
	static btTransform getCumulativeTransform(TiXmlElement *el, const btTransform &trans);

	MeshReference *getCollisionMesh(int i) { return collisionMeshes[i]; }
	MeshReference *getDisplayMesh(int i) { return displayMeshes[i]; }

	MeshList &getDisplayMeshes() { return displayMeshes; }
	ConstraintList &getConstraints() { return constraints; }
	PartList &getParts() { return parts; }

	PhysicsResource *createInstance();

protected:
	PhysicsResource(const PhysicsResource &orig);
	int nInstances;

	bool validName(const char *str);
	void getDisplayMeshReferences(TiXmlElement *root);
	void getCollisionMeshReferences(TiXmlElement *root);
	void getParts(TiXmlElement *root);
	void getConstraints(TiXmlElement *root);

	PartList *getPartInstances(PhysicsResource *r) const;
	MeshList *getDisplayMeshInstances(PhysicsResource *r) const;
	MeshList *getCollisionMeshInstances(PhysicsResource *r) const;
	ConstraintList *getConstraintInstances(PhysicsResource *r) const;

	void loadImpl();
	void unloadImpl();
	size_t calculateSize() const;

	MeshList displayMeshes;
	MeshList collisionMeshes;
	
	PartList parts;
	IndexMap partIndex;

	ConstraintList constraints;
};

class Shape
{
protected:
	btTransform transform;
public:
	Shape(const btTransform &tr) : transform(tr) {}

	const btTransform &getTransform() const { return transform; }
	virtual btCollisionShape *getShape() = 0;

	virtual void construct(PhysicsResource *r) {} // Perform additional construction after the file has been parsed, if necessary

	void applyRotation(const btQuaternion &r) { transform.setRotation(r);}

	static Shape *fromXML(TiXmlElement *el, const btTransform &currentTransform);
};

class SphereShape : public Shape
{
private:
	btSphereShape *sphere;
public:
	SphereShape(const btTransform &tr, double radius) : Shape(tr)
	{
		sphere = new btSphereShape(radius);
	}
	~SphereShape() { if(sphere) OGRE_DELETE sphere; }

	btCollisionShape *getShape() { return sphere; }

	static SphereShape *fromXML(TiXmlElement *el, const btTransform &currentTransform);
};

class MultiSphereShape : public Shape
{
private:
	btMultiSphereShape *multiSphere;
public:
	MultiSphereShape(const btTransform &tr, btVector3 *p, btScalar *r, int ns) : Shape(tr)
	{
		double maxX = -std::numeric_limits<double>::infinity();
		double maxY = -std::numeric_limits<double>::infinity();
		double maxZ = -std::numeric_limits<double>::infinity();

		double minX = std::numeric_limits<double>::infinity();
		double minY = std::numeric_limits<double>::infinity();
		double minZ = std::numeric_limits<double>::infinity();

		// TODO: maybe we should calculate the REAL inertia instead of relying on bullet to do it?
		for(int i = 0; i < ns; i++)
		{
			if(p[i].getX() + r[i] > maxX)
				maxX = p[i].getX() + r[i];
			if(p[i].getY() + r[i] > maxY)
				maxY = p[i].getY() + r[i];
			if(p[i].getZ() + r[i] > maxZ)
				maxZ = p[i].getZ() + r[i];

			if(p[i].getX() - r[i] < minX)
				minX = p[i].getX() - r[i];
			if(p[i].getY() - r[i] < minY)
				minY = p[i].getY() - r[i];
			if(p[i].getZ() - r[i] < minZ)
				minZ = p[i].getZ() - r[i];
		}

		btVector3 halfExt(maxX - minX, maxY - minY, maxZ - minZ);
		multiSphere = new btMultiSphereShape(/*halfExt,*/ p, r, ns);
	}
	~MultiSphereShape() { if(multiSphere) OGRE_DELETE multiSphere; }

	btCollisionShape *getShape() { return multiSphere; }

	static MultiSphereShape *fromXML(TiXmlElement *el, const btTransform &currentTransform);
};

class BoxShape : public Shape
{
private:
	btBoxShape *box;
public:
	BoxShape(const btTransform &tr, const btVector3 &hExt) : Shape(tr)
	{
		box = new btBoxShape(hExt);
	}
	~BoxShape() { if(box) OGRE_DELETE box; }

	btCollisionShape *getShape() { return box; }

	static BoxShape *fromXML(TiXmlElement *el, const btTransform &currentTransform);
};

/*class CylinderShape : public Shape
{
private:
	btCylinderShape *cylinder;
public:
	CylinderShape(const btTransform &tr, float radius, float length) : Shape(tr)
	{
		cylinder = new btCylinderShape(
	}
};*/

class MeshShape : public Shape
{
private:
	//btGImpactConvexDecompositionShape *shape;
	btConvexHullShape *hull;
	std::vector<int> vertices;
	int meshID;
	bool refersToSubmesh;
	bool constructed;
	String submeshName;
public:
	MeshShape(const btTransform &tr, int mid) : Shape(tr), meshID(mid), refersToSubmesh(false), constructed(false) {}
	MeshShape(const btTransform &tr, int mid, const String &sName) : Shape(tr), meshID(mid), refersToSubmesh(true), submeshName(sName), constructed(false) {}
	~MeshShape() { if(hull) delete hull; }
	void construct(PhysicsResource *r);

	void addVertex(int i) { vertices.push_back(i); }

	btCollisionShape *getShape() { return hull; }

	static MeshShape *fromXML(TiXmlElement *el, const btTransform &currentTransform);
};

class RigidMotionState : public btMotionState
{
public:
	typedef std::vector<const btTransform *> TransformList;
private:
	std::vector<SceneNode*> nodes;
	std::vector<Bone*> bones;
	TransformList transforms;
	TransformList nodeTrans;
	TransformList invTransforms;
	SceneNode *rootNode; // node for the root of a skeleton

	btTransform transform;

	btRigidBody *body;
	btRigidBody *parent;
	Bone *parentBone;
public:
	RigidMotionState(btRigidBody *b, SceneNode *n, const btTransform &tr) : body(b), parent(0), rootNode(n), btMotionState(), transform(tr) {}

	void getWorldTransform(btTransform &tr) const;

	void setWorldTransform(const btTransform &worldTrans);

	void addNode(SceneNode *n, const btTransform *tr)
	{
		nodes.push_back(n);
		nodeTrans.push_back(tr);
	}

	void addBone(Bone *b, const btTransform *tr);

	void setParent(btRigidBody *p) { parent = p; }
	void setRootNode(SceneNode *n) { rootNode = n; }
	SceneNode *getRootNode() { return rootNode; }

	btRigidBody *getBody() { return body; }
};

class Body;

class MeshAttachment
{
private:
	int meshid;
	String boneName;

	btTransform transform;
	bool controlsBone;

	MeshReference *mesh;

	SceneNode *node;
	Bone *bone;

	btRigidBody *body;
	btRigidBody *parent;

public:
	MeshAttachment(const btTransform &tr, int mid) : transform(tr), meshid(mid), controlsBone(false), mesh(0), bone(0), node(0), body(0), parent(0) {}
	MeshAttachment(const btTransform &tr, int mid, String bn) : transform(tr), meshid(mid), boneName(bn), controlsBone(true), mesh(0), bone(0), node(0), body(0), parent(0) {}

	int getMeshId() const { return meshid; }
	const String &getBoneName() const { return boneName; }
	bool isBoneController() const { return controlsBone; }
	const btTransform &getTransform() const { return transform; }

	MeshReference *getMesh() { return mesh; }
	Bone   *getBone() { return bone; }
	SceneNode *getNode() { return node; }

	void attachToNode(SceneNode *n);

	void construct(PhysicsResource *r);

	void attachEntity(Entity *e);

	MeshAttachment *createInstance() const;

	void setBody(btRigidBody *b) { body = b;}
	void setParent(btRigidBody *p) { parent = p;}
	btRigidBody *getBody() { return body; }
	btRigidBody *getParent() { return parent; }

	static MeshAttachment *fromXML(TiXmlElement *el, const btTransform &currentTransform);
};

class BodyReference;

class Body
{
public:
	typedef std::vector<Shape*> ShapeList;
	typedef std::vector<MeshAttachment*> AttachmentList;
	enum BodyType { Rigid = 0, Soft };

	double maxControllerTorque;
	double controllerTorqueScale;

	int nonCollideGroup;

protected:
	int id;
	ShapeList shapes;
	AttachmentList attachments;
	btTransform transform;
	String name;

	double mass;

	btCompoundShape *collisionShape;

	BodyReference *parentReference; // TODO: make skeleton a primitive type!!!

	void addAttachment(MeshAttachment *a) { attachments.push_back(a); }
	void addShape(Shape *s)
	{
		shapes.push_back(s);
	}


public:
	Body(const btTransform &tr, int i, double m) : transform(tr), id(i), mass(m), parentReference(0), nonCollideGroup(-1)
	{
		collisionShape = new btCompoundShape();
	}

	btCompoundShape *getCollisionShape() { return collisionShape; }
	int getID() const { return id; }
	const btTransform &getTransform() const { return transform; }

	static Body* fromXML(TiXmlElement *el, const btTransform &currentTransform);

	AttachmentList &getAttachments() { return attachments; }

	virtual btCollisionObject *getBody() = 0;
	virtual Body *createInstance() const = 0;
	virtual BodyType type() const = 0;

	virtual void attachToNode(SceneNode *node) = 0;
	virtual void createBody() = 0;
	virtual void applyTransform(const btTransform &trans) = 0;

	virtual void construct(PhysicsResource *r);
	double getMass() const { return mass; }

	void setPartName(const String &n) { name = n; }
	const String &getPartName() const { return name; }

	~Body() { if(collisionShape) delete collisionShape; }
};

class RigidBody : public Body
{
private:
	btRigidBody *body;
	RigidMotionState *state;
	double restitution;
	double angularDamping;
	double friction;
	double linearDamping;
public:
	RigidBody(const btTransform &tr, int i, double m, double r, double ad, double ld, double f) : Body(tr, i, m), body(0), state(0), restitution(r), angularDamping(ad), linearDamping(ld), friction(f){}
	static RigidBody *fromXML(TiXmlElement *el, const btTransform &currentTransform);

	btCollisionObject *getBody() { return body; }

	BodyType type() const { return BodyType::Rigid; }

	void attachToNode(SceneNode *node);
	void createBody();
	virtual void applyTransform(const btTransform &trans);

	Body *createInstance() const;
};

class SoftBody : public Body
{
private:
	//btSoftBody *body;
public:
	SoftBody(const btTransform &tr, int i, double m) : Body(tr, i, m) {};
	static SoftBody *fromXML(TiXmlElement *el, const btTransform &currentTransform);

	btCollisionObject *getBody() { return 0; }

	BodyType type() const { return BodyType::Soft; }

	void attachToNode(SceneNode *node);
	void createBody();
	virtual void applyTransform(const btTransform &trans);

	Body *createInstance() const;
};

class Part
{
public:
	typedef std::vector<Body*> BodyList;
private:
	String name;
	int id;
	bool hasName;
	PhysicsResource::PartList subParts;
	PhysicsResource::IndexMap subPartIndex;
	BodyList bodies;

	btTransform transform;
public:
	Part(const btTransform &tr, int i) : transform(tr), id(i), hasName(false) {}
	Part(const btTransform &tr, int i, String n) : transform(tr), id(i), name(n), hasName(true) {}
	~Part();

	int getID() const { return id; }
	const String &getName() { return name; }

	PhysicsResource::PartList &getSubParts() { return subParts; }
	BodyList &getBodies() { return bodies; }
	Body *getBody(int id) { return bodies[id]; }

	bool isNamed() const { return hasName; }
	Part *getSubPart(String &n);

	void construct(PhysicsResource *r);
	Part *createInstance() const;

	static Part *fromXML(TiXmlElement *el, const btTransform &currentTransform);

};

class BodyReference
{
protected:
	std::vector<String> partIDs;
	int bodyID;

	Body *body;
public:
	BodyReference(const std::vector<String> &partIDs, int bid);
	BodyReference(const String &refstr);

	Body *getBody() { return body; }
	const String &getPartName() const { return partIDs[0]; }

	void construct(PhysicsResource *r);
	BodyReference *createInstance() const;
};

class Constraint
{
public:
	enum Type { Cone = 0, Joint };
protected:
	BodyReference *bodies[2];
	btTransform jointFrames[2];

	std::vector<btTypedConstraint *> constraints;

public:

	std::vector<btTypedConstraint *> &getConstraints() { return constraints; }
	btTypedConstraint * getConstraint(size_t index) { return constraints[index]; }
	virtual Type getType() const = 0;

	void construct(PhysicsResource *r);
	virtual void createConstraints() = 0;

	const BodyReference *getBody(int i) { return bodies[i]; }
	const btTransform &getJointFrame(int i) { return jointFrames[i]; }

	virtual Constraint *createInstance() const = 0;

	static Constraint *fromXML(TiXmlElement *el);
};

class JointConstraint : public Constraint
{
private:
	btVector3 minAngles, maxAngles;
	btTransform frame1, frame2;
	double damping, restitution, softness, maxForce;

public:
	JointConstraint(const btVector3 &minA, const btVector3 &maxA, double d, double r, double s, double m) :
	  minAngles(minA), maxAngles(maxA), damping(d), restitution(r), softness(s), maxForce(m) {}

	  void createConstraints();

	  Type getType() const { return Joint; }

	  Constraint *createInstance() const;

	  const btTransform &getFrame(int frame) const { return frame ? frame2 : frame1; }

	  static JointConstraint *fromXML(TiXmlElement *el);
};

class ConeConstraint : public Constraint
{
private:
	double angle1, angle2, twist;
public:
	ConeConstraint(double a1, double a2, double t)
		: angle1(a1), angle2(a2), twist(t) {}

	void createConstraints();

	Type getType() const { return Cone; }

	Constraint *createInstance() const;

	static ConeConstraint *fromXML(TiXmlElement *el);
};

/*struct PhysicsInstance
{
public:
	PhysicsInstance(String &n, PhysicsResource::PartList &p, PhysicsResource::MeshList &m, PhysicsResource::ConstraintList &c) : name(n), parts(p), constraints(c), meshes(m) {}
	String name;
	PhysicsResource::PartList parts;
	PhysicsResource::MeshList meshes;
	PhysicsResource::ConstraintList constraints;
};*/

class PhysicsResourcePtr : public SharedPtr<PhysicsResource>
{
public:
	PhysicsResourcePtr() : SharedPtr<PhysicsResource>() {}
	explicit PhysicsResourcePtr(PhysicsResource *rep) : SharedPtr<PhysicsResource>(rep) {}
	PhysicsResourcePtr(const PhysicsResourcePtr &r) : SharedPtr<PhysicsResource>(r) {}
	PhysicsResourcePtr(const ResourcePtr & r) : SharedPtr<PhysicsResource>()
	{
		OGRE_LOCK_MUTEX(*r.OGRE_AUTO_MUTEX_NAME)
		OGRE_COPY_AUTO_SHARED_MUTEX(r.OGRE_AUTO_MUTEX_NAME)
		pRep = static_cast<PhysicsResource*>(r.getPointer());
		pUseCount = r.useCountPointer();
		if(pUseCount)
		{
			++(*pUseCount);
		}
	}

	PhysicsResourcePtr &operator=(const ResourcePtr &r)
	{
		if(pRep == static_cast<PhysicsResource*>(r.getPointer()))
			return *this;
		release();
		OGRE_LOCK_MUTEX(*r.OGRE_AUTO_MUTEX_NAME)
		OGRE_COPY_AUTO_SHARED_MUTEX(r.OGRE_AUTO_MUTEX_NAME)
		pRep = static_cast<PhysicsResource*>(r.getPointer());
		pUseCount = r.useCountPointer();
		if(pUseCount)
		{
			++(*pUseCount);
		}
		return *this;
	}
};

}

#endif