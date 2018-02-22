#include "PhysicsResource.h"
#include "PhysicsNode.h"
#include "PhysicsSceneManager.h"
#include "tinyxml.h"
#include "OgreMesh.h"
#include "OgreSubMesh.h"
#include "OgreVertexIndexData.h"
#include "OgreLogManager.h"

#include "MeshStrider.h"

// TODO: fine-grained control about what bodies can collide

PhysicsResource::PhysicsResource(Ogre::ResourceManager *creator, const Ogre::String &name, ResourceHandle handle, const Ogre::String &group, bool isManual, Ogre::ManualResourceLoader *loader) : Resource(creator, name, handle, group, isManual, loader), nInstances(0)
{
}

PhysicsResource::~PhysicsResource()
{
	unload();
}

void PhysicsResource::loadImpl()
{
	DataStreamPtr stream = ResourceGroupManager::getSingleton().openResource(mName, mGroup);
	load(stream);
}

void PhysicsResource::unloadImpl()
{

}

bool PhysicsResource::validName(const char *n)
{
	// TODO: cannot contain :;
	return true;
}

void Constraint::construct(PhysicsResource *r)
{
	bodies[0]->construct(r);
	bodies[1]->construct(r);
	createConstraints();
}

Constraint *Constraint::fromXML(TiXmlElement *el)
{
	const char *t = el->Attribute("type");
	if(!t) {} // ERROR: undefined type!

	Constraint *c = 0;

	if(!strcmp(t, "cone"))
		c = ConeConstraint::fromXML(el);
	else if(!strcmp(t, "joint"))
		c = JointConstraint::fromXML(el);

	if(!c) {} // ERROR: unknown constraint type!
	c->bodies[0] = new BodyReference(String(el->Attribute("body_1")));
	c->bodies[1] = new BodyReference(String(el->Attribute("body_2")));

	TiXmlNode *jf1 = el->FirstChildElement("jointframe_1");
	if(!jf1) {} // ERROR: must define joint frame!

	c->jointFrames[0] = PhysicsResource::parseTransform(jf1->ToElement());

	TiXmlNode *jf2 = el->FirstChildElement("jointframe_2");
	if(!jf2) { c->jointFrames[1] = c->jointFrames[0]; } // if second joint frame not defined, use the first one as default
	else
		c->jointFrames[1] = PhysicsResource::parseTransform(jf1->ToElement());

	return c;
}

void Part::construct(PhysicsResource *r)
{
	for(size_t i = 0; i < subParts.size(); i++)
		subParts[i]->construct(r);
	
	for(size_t i = 0; i < bodies.size(); i++)
	{
		bodies[i]->setPartName(name);
		bodies[i]->construct(r);
	}
}

Part *Part::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);
	int id = -1;
	el->Attribute("id", &id);
	if(id < 0) {} // ERROR: invalid id!

	const char *name = el->Attribute("name");

	Part *p;

	// TODO: importing parts!

	if(name)
		p = new Part(transform, id, String(name));
	else
		p = new Part(transform, id);

	TiXmlNode *node = NULL;
	while(node = el->IterateChildren("part", node))
	{
		Part *sp = Part::fromXML(node->ToElement(), transform);

		if(sp->isNamed())
			p->subPartIndex[sp->getName()] = sp->getID();

		p->subParts.push_back(sp);
	}

	node = NULL;
	while(node = el->IterateChildren("body", node))
	{
		Body *b = Body::fromXML(node->ToElement(), transform);
		p->bodies.push_back(b);
	}

	// Validate part indices
	for(size_t i = 0; i < p->subParts.size(); i++)
	{
		if(p->subParts[i]->getID() != i) {} // ERROR: out of order ID!
	}

	return p;
}

SphereShape *SphereShape::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	double radius = -1.0;
	el->Attribute("radius", &radius);
	if(radius <= 0.0) {} // ERROR: invalid radius!

	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);

	return new SphereShape(transform, radius);
}

MultiSphereShape *MultiSphereShape::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);

	int nSpheres = 0;

	TiXmlNode *node = NULL;
	while(node = el->IterateChildren("sphere", node)) nSpheres++;

	btVector3 *centers = new btVector3[nSpheres];
	btScalar *radii = new btScalar[nSpheres];

	int i = 0;
	node = NULL;
	while(node = el->IterateChildren("sphere", node))
	{
		centers[i] = PhysicsResource::parseVector(node->ToElement()->Attribute("position"));

		radii[i] = PhysicsResource::parseScalar(node->ToElement()->Attribute("radius"));
		if(radii[i] <= 0.0) {} // ERROR: invalid radius!

		i++;
	}

	MultiSphereShape *shape = new MultiSphereShape(transform, centers, radii, nSpheres);

	OGRE_DELETE [] centers;
	OGRE_DELETE [] radii;

	return shape;
}

BoxShape *BoxShape::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);

	btVector3 lengths = PhysicsResource::parseVector(el->Attribute("lengths"));

	return new BoxShape(transform, lengths);
}

void Body::construct(PhysicsResource *r)
{
	for(size_t i = 0; i < attachments.size(); i++)
	{
		attachments[i]->construct(r);

		if(attachments[i]->isBoneController())
		{
			if(attachments[i]->getMesh()->getSkeleton()->getBone(name)->getParent())
			{
				attachments[i]->setParent(btRigidBody::upcast(r->getPart(attachments[i]->getMesh()->getSkeleton()->getBone(name)->getParent()->getName())->getBody(0)->getBody()));
			}
		}
	}

	for(size_t i = 0; i < shapes.size(); i++)
	{
		if(attachments.size() > 0 && attachments[0]->isBoneController())
		{
			Bone *b = const_cast<Bone*>(attachments[0]->getMesh()->getSkeleton()->getBone(attachments[0]->getBoneName()));
			//b->setInheritOrientation(false);
			btQuaternion q(b->_getDerivedOrientation().x,
			               b->_getDerivedOrientation().y,
						   b->_getDerivedOrientation().z,
						   b->_getDerivedOrientation().w);
			shapes[i]->applyRotation(q);

			transform.setRotation(q);
		}
		shapes[i]->construct(r);
		//collisionShape->addChildShape(shapes[i]->getTransform(), shapes[i]->getShape());
		//collisionShape->addChildShape(shapes[i]->getTransform().inverse(), shapes[i]->getShape());
		collisionShape->addChildShape(btTransform::getIdentity(), shapes[i]->getShape());
	}

	if(parentReference)
	{
		parentReference->construct(r);
	}

	createBody();

	if(attachments.size() > 0)
	attachments[0]->setBody(btRigidBody::upcast(getBody()));
}

void RigidMotionState::addBone(Bone *b, const btTransform *tr)
{
	//assert(b->getParent()); // shouldn't call this method on root bones

	if(b->getParent())
		parentBone = static_cast<Bone*>(b->getParent());

	bones.push_back(b);
	//b->_getDerivedOrientation
	Quaternion q = b->_getBindingPoseInverseOrientation();
	Vector3 v = b->_getBindingPoseInversePosition();

	btTransform invOrig = btTransform(btQuaternion(q.x, q.y, q.z, q.w), btVector3(v.x, v.y, v.z));
	
	// TODO: put the offset transform in the right place!
	btTransform *invTrans = new btTransform(invOrig.inverse());// * *tr);

	btTransform *trans = new btTransform(invOrig);

	//q = b->getInitialOrientation();
	//v = b->getInitialPosition();
	//btTransform *trans = new btTransform(invTrans->inverse() * btTransform(btQuaternion(q.x, q.y, q.z, q.w), btVector3(v.x, v.y, v.z)));

	transforms.push_back(trans);
	invTransforms.push_back(invTrans);

}

void RigidMotionState::getWorldTransform(btTransform &worldTrans) const
{
	worldTrans = body->getCenterOfMassTransform();

	for(size_t i = 0; i < bones.size(); i++)
	{
		if(parent) // convert to relative transform if this is not the root
		{		
			btQuaternion q = parent->getWorldTransform().getRotation().inverse() * worldTrans.getRotation();

			bones[i]->setOrientation(q.w(), q.x(), q.y(), q.z());
		}
		else
		{
			btTransform tr = worldTrans * transform;
			btQuaternion q = tr.getRotation();

			rootNode->setPosition(tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z());
			rootNode->setOrientation(q.w(), q.x(), q.y(), q.z());
		}
	}
}

void RigidMotionState::setWorldTransform(const btTransform &worldTrans)
{
	for(size_t i = 0; i < nodes.size(); i++)
	{
		btTransform tr = worldTrans;// * *nodeTrans[i];
		btQuaternion q = tr.getRotation();
		nodes[i]->setPosition(tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z());
		nodes[i]->setOrientation(q.w(), q.x(), q.y(), q.z());
	}
	for(size_t i = 0; i < bones.size(); i++)
	{
		if(parent) // convert to relative transform if this is not the root
		{		
			btQuaternion q = parent->getWorldTransform().getRotation().inverse() * worldTrans.getRotation();

			bones[i]->setOrientation(q.w(), q.x(), q.y(), q.z());
		}
		else
		{
			btTransform tr = worldTrans * transform;
			btQuaternion q = tr.getRotation();

			rootNode->setPosition(tr.getOrigin().x(), tr.getOrigin().y(), tr.getOrigin().z());
			rootNode->setOrientation(q.w(), q.x(), q.y(), q.z());
		}
	}
}

void RigidBody::createBody()
{
	btVector3 inertia(0, 0, 0);
	collisionShape->calculateLocalInertia(mass, inertia);
	body = new btRigidBody(mass, 0, collisionShape, inertia);
	body->setCenterOfMassTransform(transform);
	body->setActivationState(DISABLE_DEACTIVATION);
	body->setFriction(friction);
	body->setRestitution(restitution);
	body->setDamping(linearDamping, angularDamping); // TODO: parameterize!
}

void RigidBody::applyTransform(const btTransform &trans)
{
	bool wasStatic = false;
	if (body->getCollisionFlags() & btCollisionObject::CF_STATIC_OBJECT)
	{
		body->setCollisionFlags(body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
		wasStatic = true;
	}

	btTransform tr = body->getCenterOfMassTransform();
	tr.setOrigin(tr.getOrigin() + trans.getOrigin());
	// TODO: rotations
	body->setCenterOfMassTransform(tr);
	body->getMotionState()->setWorldTransform(tr);

	//stringstream str;
	//str << "Applying transform: " << trans.getOrigin().x() << "," << trans.getOrigin().y() << "," << trans.getOrigin().z();
	//str << " final transfarm: " << tr.getOrigin().x() << "," << tr.getOrigin().y() << "," << tr.getOrigin().z();
	//LOG(str.str());
	//body->set

	if(wasStatic)
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
}

void SoftBody::applyTransform(const btTransform &trans)
{
}

void SoftBody::createBody()
{
}

void MeshShape::construct(PhysicsResource *r)
{
	MeshPtr mesh = r->getCollisionMesh(meshID)->getMesh();

	/*
	int sid = 0;
	Mesh::SubMeshIterator itr = mesh->getSubMeshIterator();
	while(itr.hasMoreElements())
	{
		if(itr.getNext()->getMaterialName() == submeshName)
			break;
		sid++;
	}

	if(sid >= mesh->getNumSubMeshes())
		LOG(String("Warning - mesh does not have a submesh named ") + submeshName);

	MeshStrider *meshInterface = new MeshStrider(&*mesh, sid);
	shape = new btGImpactConvexDecompositionShape(meshInterface, btVector3(1.0, 1.0, 1.0));
	*/

	SubMesh *submesh;
	if(refersToSubmesh)
	{
		int sid = 0;
		Mesh::SubMeshIterator itr = mesh->getSubMeshIterator();
		while(itr.hasMoreElements())
		{
			if(itr.getNext()->getMaterialName() == submeshName)
				break;
			sid++;
		}

		if(sid >= mesh->getNumSubMeshes())
			LOG(String("Warning - mesh does not have a submesh named ") + submeshName);

		submesh = mesh->getSubMesh(sid);
	}
	else
		submesh = mesh->getSubMesh(0);

	btTransform invTrans = transform.inverse();

	VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

	const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

    Ogre::HardwareVertexBufferSharedPtr vbuf =
        vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

    unsigned char* vertex =
        static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

	float *p;

	hull = new btConvexHullShape();

	// TODO: use actual indices?
	for(size_t i = 0; i < vertex_data->vertexCount; i++)
	{
		posElem->baseVertexPointerToElement(vertex, &p);

		btVector3 point(p[0], p[1], p[2]);

		point = invTrans * point;

		hull->addPoint(point);

		vertex += vbuf->getVertexSize();
	}

	vbuf->unlock();

	hull->setMargin(0.01);

	constructed = true;
}

MeshShape *MeshShape::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);
	int meshID = -1;
	el->Attribute("meshid", &meshID);
	if(meshID < 0) {} // ERROR: invalid ID!

	const char *sName = el->Attribute("submesh");
	MeshShape *shape;

	if(sName)
		shape = new MeshShape(transform, meshID, String(sName));
	else
		shape = new MeshShape(transform, meshID);

	TiXmlNode *node = NULL;
	while(node = el->IterateChildren("vertex", node))
	{
		int id = -1;
		node->ToElement()->Attribute("id", &id);

		if(id < 0) {} // ERROR: invalid id!
		shape->addVertex(id);
	}

	return shape;
}

Shape *Shape::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	const char *t = el->Attribute("type");
	if(!t) {} // ERROR: undefined type!
	if(!strcmp(t, "sphere"))
		return SphereShape::fromXML(el, currentTransform);
	if(!strcmp(t, "multisphere"))
		return MultiSphereShape::fromXML(el, currentTransform);
	if(!strcmp(t, "box"))
		return BoxShape::fromXML(el, currentTransform);
	if(!strcmp(t, "mesh"))
		return MeshShape::fromXML(el, currentTransform);

	// ERROR: unknown type
}

MeshAttachment *MeshAttachment::createInstance() const
{
	MeshAttachment *ma;

	if(controlsBone)
		ma = new MeshAttachment(transform, meshid, boneName);
	else
		ma = new MeshAttachment(transform, meshid);

	return ma;
}

void MeshAttachment::attachToNode(SceneNode *n)
{
	node = n;
}

void MeshAttachment::construct(PhysicsResource *r)
{
	mesh = r->getDisplayMesh(meshid);
	mesh->attach(this);
}

void MeshReference::attachEntity(Entity *e)
{
	entity = e;

	if(controlled)
	{
		for(size_t i = 0; i < attachments.size(); i++)
			attachments[i]->attachEntity(entity);
	}
}

void MeshReference::attachToNode(SceneNode *n)
{
	for(size_t i = 0; i < attachments.size(); i++)
		attachments[i]->attachToNode(n);

	n->attachObject(entity);
}

void MeshAttachment::attachEntity(Entity *e)
{
	if(controlsBone)
		bone = e->getSkeleton()->getBone(boneName);
}

MeshAttachment *MeshAttachment::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	int id = -1;
	el->Attribute("meshid", &id);
	if(id < 0) {} // ERROR: invalid id
	
	btTransform trans = PhysicsResource::parseTransform(el);

	MeshAttachment *ma;

	const char *bn = el->Attribute("bone");
	if(bn)
		ma = new MeshAttachment(trans, id, String(bn));
	else
		ma = new MeshAttachment(trans, id);

	return ma;
}

Body *Body::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	const char *t = el->Attribute("type");
	Body *body = 0;
	if(!t) {} // ERROR: undefined type!
	if(!strcmp(t, "rigid"))
		body = RigidBody::fromXML(el, currentTransform);
	if(!strcmp(t, "soft"))
		body = SoftBody::fromXML(el, currentTransform);

	const char *p = el->Attribute("parent");
	if(p)
	{
		body->parentReference = new BodyReference(String(p));
	}

	const char *mct = el->Attribute("max_controller_torque");
	if(mct)
	{
		body->maxControllerTorque = PhysicsResource::parseScalar(mct);
	}
	else
		body->maxControllerTorque = 0;

	const char *cts = el->Attribute("controller_torque_scale");
	if(cts)
	{
		body->controllerTorqueScale = PhysicsResource::parseScalar(cts);
	}
	else
		body->controllerTorqueScale = 0;

	const char *ncg = el->Attribute("noncollision_group");
	if(ncg)
		body->nonCollideGroup = PhysicsResource::parseScalar(ncg);

	return body;
}

Constraint *ConeConstraint::createInstance() const
{
	ConeConstraint *cc = new ConeConstraint(angle1, angle2, twist);
	cc->bodies[0] = bodies[0]->createInstance();
	cc->bodies[1] = bodies[1]->createInstance();
	cc->jointFrames[0] = jointFrames[0];
	cc->jointFrames[1] = jointFrames[1];

	return cc;
}

void ConeConstraint::createConstraints()
{
	btConeTwistConstraint *cone = new btConeTwistConstraint(*btRigidBody::upcast(bodies[0]->getBody()->getBody()), *btRigidBody::upcast(bodies[1]->getBody()->getBody()), jointFrames[0], jointFrames[1]);
	cone->setLimit(angle1, angle2, twist);
	cone->setAngularOnly(false);

	constraints.push_back(cone);
}

ConeConstraint *ConeConstraint::fromXML(TiXmlElement *el)
{
	TiXmlNode *limits = el->FirstChildElement("limits");
	if(!limits) {} // ERROR: must define limits for constraint!

	double angle1, angle2, twist;
	angle1 = PhysicsResource::parseScalar(limits->ToElement()->Attribute("angle_1"));
	angle2 = PhysicsResource::parseScalar(limits->ToElement()->Attribute("angle_2"));
	twist = PhysicsResource::parseScalar(limits->ToElement()->Attribute("twist"));

	return new ConeConstraint(angle1, angle2, twist);
}

Constraint *JointConstraint::createInstance() const
{
	JointConstraint *jc = new JointConstraint(minAngles, maxAngles, damping, restitution, softness, maxForce);
	jc->bodies[0] = bodies[0]->createInstance();
	jc->bodies[1] = bodies[1]->createInstance();
	jc->jointFrames[0] = jointFrames[0];
	jc->jointFrames[1] = jointFrames[1];

	return jc;
}

void JointConstraint::createConstraints()
{
	static float linearRange = 1e-05;
	frame1 = bodies[0]->getBody()->getTransform().inverse() * jointFrames[0];
	frame2 = bodies[1]->getBody()->getTransform().inverse() * jointFrames[1];
	btRigidBody *body1 = btRigidBody::upcast(bodies[0]->getBody()->getBody());
	btRigidBody *body2 = btRigidBody::upcast(bodies[1]->getBody()->getBody());

	btGeneric6DofConstraint *constraint =
		new btGeneric6DofConstraint(*body1,
		                            *body2,
		                             frame1,
		                             frame2,
		                             true);
	constraint->setLinearLowerLimit(btVector3(-linearRange, -linearRange, -linearRange));
	constraint->setLinearUpperLimit(btVector3(linearRange, linearRange, linearRange));

	constraint->setAngularLowerLimit(minAngles);
	constraint->setAngularUpperLimit(maxAngles);

	constraint->getRotationalLimitMotor(0)->m_damping = damping;
	constraint->getRotationalLimitMotor(0)->m_limitSoftness = softness;
	constraint->getRotationalLimitMotor(0)->m_bounce = restitution;
	constraint->getRotationalLimitMotor(0)->m_maxLimitForce = maxForce;
	constraint->getRotationalLimitMotor(0)->m_maxMotorForce = maxForce;

	constraint->getRotationalLimitMotor(1)->m_damping = damping;
	constraint->getRotationalLimitMotor(1)->m_limitSoftness = softness;
	constraint->getRotationalLimitMotor(1)->m_bounce = restitution;
	constraint->getRotationalLimitMotor(1)->m_maxLimitForce = maxForce;
	constraint->getRotationalLimitMotor(1)->m_maxMotorForce = maxForce;

	constraint->getRotationalLimitMotor(2)->m_damping = damping;
	constraint->getRotationalLimitMotor(2)->m_limitSoftness = softness;
	constraint->getRotationalLimitMotor(2)->m_bounce = restitution;
	constraint->getRotationalLimitMotor(2)->m_maxLimitForce = maxForce;
	constraint->getRotationalLimitMotor(2)->m_maxMotorForce = maxForce;

	if(bodies[1]->getBody()->getMass() > 0.0)
	{
		constraint->getRotationalLimitMotor(0)->m_enableMotor = true;
		constraint->getRotationalLimitMotor(1)->m_enableMotor = true;
		constraint->getRotationalLimitMotor(2)->m_enableMotor = true;
	}
	constraint->getTranslationalLimitMotor()->m_enableMotor[0] = false;
	constraint->getTranslationalLimitMotor()->m_enableMotor[1] = false;
	constraint->getTranslationalLimitMotor()->m_enableMotor[2] = false;
	constraint->getTranslationalLimitMotor()->m_damping = damping;

	constraints.push_back(constraint);

	//constraints.push_back(new btPoint2PointConstraint(*btRigidBody::upcast(bodies[0]->getBody()->getBody()), *btRigidBody::upcast(bodies[1]->getBody()->getBody()), bodies[0]->getBody()->getTransform().inverse() * jointFrames[0].getOrigin(), bodies[1]->getBody()->getTransform().inverse() * jointFrames[1].getOrigin()));
}

JointConstraint *JointConstraint::fromXML(TiXmlElement *el)
{
	TiXmlNode *limits = el->FirstChildElement("limits");
	if(!limits) {} // ERROR: must define limits for constraint!
	
	btVector3 minAngles = PhysicsResource::parseVector(limits->ToElement()->Attribute("min_angles"));
	btVector3 maxAngles = PhysicsResource::parseVector(limits->ToElement()->Attribute("max_angles"));

	TiXmlNode *properties = el->FirstChildElement("properties");
	double damping = 1.0, // default = 1.0
		   restitution = 0.1,
		   softness = 0.5, // default = 0.5
		   maxForce = 20; // default = 300

	if(properties)
	{
		damping = PhysicsResource::parseScalar(properties->ToElement()->Attribute("damping"));
		restitution = PhysicsResource::parseScalar(properties->ToElement()->Attribute("restitution"));
		softness = PhysicsResource::parseScalar(properties->ToElement()->Attribute("softness"));
		maxForce = PhysicsResource::parseScalar(properties->ToElement()->Attribute("max_force"));
	}

	return new JointConstraint(minAngles, maxAngles, damping, restitution, softness, maxForce);
}

void RigidBody::attachToNode(SceneNode *n)
{
	// TODO: does this work with offsets?
	state = new RigidMotionState(body, n, transform.inverse());
	body->setMotionState(state);

	// TODO: who am I kidding? this only works with single attachments.
	for(size_t i = 0; i < attachments.size(); i++)
	{
		if(attachments[i]->isBoneController())
		{
			Bone *b = attachments[i]->getBone();

			b->setManuallyControlled(true);
			state->addBone(b, &attachments[i]->getTransform());
			state->setParent(attachments[i]->getParent());

			if(!b->getParent())
				state->setRootNode(attachments[i]->getNode());
		}
		else
		{
			SceneNode *node = n->createChildSceneNode();
			state->addNode(node, &attachments[i]->getTransform());

			node->attachObject(attachments[i]->getMesh()->getEntity());

			node->scale(attachments[i]->getMesh()->getScale());
		}
	}
}

void SoftBody::attachToNode(SceneNode *n)
{
}


RigidBody *RigidBody::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	int id = -1;
	el->Attribute("id", &id);

	if(id < 0) {} // ERROR: invalid id

	double mass = -1.0;
	el->Attribute("mass", &mass);

	if(mass < 0) {} // ERROR: invalid mass

	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);
	//btTransform transform = PhysicsResource::parseTransform(el);

	double restitution = 0.0,
		   angularDamping = 0.85,
		   linearDamping = 0.05,
		   friction = 0.5;

	if(el->Attribute("restitution"))
		el->Attribute("restitution", &restitution);
	if(el->Attribute("angular_damping"))
		el->Attribute("angular_damping", &angularDamping);
	if(el->Attribute("linear_damping"))
		el->Attribute("linear_damping", &linearDamping);
	if(el->Attribute("friction"))
		el->Attribute("friction", &friction);

	RigidBody *body = new RigidBody(transform, id, mass, restitution, angularDamping, linearDamping, friction);
	TiXmlNode *node = NULL;
	while(node = el->IterateChildren("shape", node))
	{
		// TODO: exceptions
		body->addShape(Shape::fromXML(node->ToElement(), currentTransform));
	}

	node = NULL;
	while(node = el->IterateChildren("meshattachment", node))
	{
		// TODO: make sure this mesh isn't attached to any other body!
		body->attachments.push_back(MeshAttachment::fromXML(node->ToElement(), currentTransform));
	}

	return body;
}

SoftBody *SoftBody::fromXML(TiXmlElement *el, const btTransform &currentTransform)
{
	int id = -1;
	el->Attribute("id", &id);

	if(id < 0) {} // ERROR: invalid id

	double mass = -1.0;
	el->Attribute("mass", &mass);

	if(mass < 0) {} // ERROR: invalid mass

	btTransform transform = PhysicsResource::getCumulativeTransform(el, currentTransform);

	SoftBody *body = new SoftBody(transform, id, mass);

	// TODO: set up soft body!

	return body;
}

BodyReference *BodyReference::createInstance() const
{
	return new BodyReference(partIDs, bodyID);
}

BodyReference::BodyReference(const std::vector<String> &p, int b) : partIDs(p), bodyID(b) {}
BodyReference::BodyReference(const String &refstr)
{
	size_t start, stop;
	start = 0;
	while((start >= 0) && (start < refstr.length()))
	{
		stop = refstr.find_first_of(":;", start);
		if(stop < 0 || stop > refstr.length())
			break;

		String sub = refstr.substr(start, stop - start);

		partIDs.push_back(sub);

		start = stop + 1;
	}

	stringstream parseID(refstr.substr(start));
	parseID >> bodyID;
}

void BodyReference::construct(PhysicsResource *r)
{
	Part *part = r->getPart(partIDs[0]);

	for(size_t i = 1; i < partIDs.size(); i++)
	{
		part = part->getSubPart(partIDs[i]);
	}

	body = part->getBodies()[bodyID];
}

MeshReference *MeshReference::createInstance() const
{
	return new MeshReference(id, name, controlled, scale, groupName);
}

MeshReference *MeshReference::fromXML(TiXmlElement *el)
{
	int id = -1;
	el->Attribute("id", &id);

	if(id < 0) {} // ERROR: invalid id
	const char *name = el->Attribute("mesh");
	if(name == 0) {} // ERROR: no mesh name!
	const char *group = el->Attribute("group");

	const char *sc = el->Attribute("scale");
	btVector3 btScale;
	if(sc)
		btScale = PhysicsResource::parseVector(sc);
	else
		btScale = btVector3(1.0, 1.0, 1.0);

	Vector3 scale(btScale.getX(), btScale.getY(), btScale.getZ());

	bool controlled = (el->Attribute("controlled") != 0);

	String mName(name);
	String gName;
	if(group)
		gName = String(group);
	else
		gName = ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;

	return(new MeshReference(id, mName, controlled, scale, gName));
}

btVector3 PhysicsResource::parseVector(const char *v)
{
	if(!v)
		return btVector3(0, 0, 0);

	double x, y, z;

	std::stringstream s(String(v), std::ios_base::in);
	s >> x >> y >> z;

	return btVector3(x, y, z);
}

btScalar PhysicsResource::parseScalar(const char *v)
{
	if(!v) // TODO: throw exception?
		return btScalar(-1.0);

	double val;

	std::stringstream s(String(v), std::ios_base::in);
	s >> val;
	return val;
}

btTransform PhysicsResource::getCumulativeTransform(TiXmlElement *el, const btTransform &curTransform)
{
	btTransform tr = parseTransform(el);
	return curTransform * tr;
}

btTransform PhysicsResource::parseTransform(TiXmlElement *el)
{
	btVector3 offset = parseVector(el->Attribute("offset"));
	btVector3 rotation = parseVector(el->Attribute("rotation"));

	btQuaternion quat;
	quat.setEulerZYX(rotation.getZ(), rotation.getY(), rotation.getX());

	return btTransform(quat, offset);
}

void PhysicsResource::getDisplayMeshReferences(TiXmlElement *root)
{
	TiXmlNode *meshNode = NULL;
	while(meshNode = root->IterateChildren("displaymesh", meshNode))
	{
		// TODO: exceptions
		displayMeshes.push_back(MeshReference::fromXML(meshNode->ToElement()));
	}

	// validate ids
	for(size_t i = 0; i < displayMeshes.size(); i++)
	{
		if(displayMeshes[i]->getID() >= displayMeshes.size()) {} // ERROR: invalid id
	}

}

void PhysicsResource::getCollisionMeshReferences(TiXmlElement *root)
{
	TiXmlNode *meshNode = NULL;
	while(meshNode = root->IterateChildren("collisionmesh", meshNode))
	{
		// TODO: exceptions
		collisionMeshes.push_back(MeshReference::fromXML(meshNode->ToElement()));
	}

	// validate ids
	for(size_t i = 0; i < collisionMeshes.size(); i++)
	{
		if(collisionMeshes[i]->getID() >= collisionMeshes.size()) {} // ERROR: invalid id
	}
}

void PhysicsResource::getParts(TiXmlElement *root)
{
	TiXmlNode *partNode = NULL;
	while(partNode = root->IterateChildren("part", partNode))
	{
		Part *p = Part::fromXML(partNode->ToElement(), btTransform::getIdentity());
		// TODO: exceptions
		parts.push_back(p);
		if(p->isNamed())
			partIndex[p->getName()] = p->getID();
	}

	// TODO: validation
	for(size_t i = 0; i < parts.size(); i++)
	{
		parts[i]->construct(this);
	}
}

void PhysicsResource::getConstraints(TiXmlElement *root)
{
	TiXmlNode *constraintNode = NULL;
	while(constraintNode = root->IterateChildren("constraint", constraintNode))
	{
		constraints.push_back(Constraint::fromXML(constraintNode->ToElement()));
	}

	// TODO: do we really need to construct non-instantiated versions??
	for(size_t i = 0; i < constraints.size(); i++)
		constraints[i]->construct(this);
	// TODO: validation
}

void PhysicsResource::load(DataStreamPtr &stream)
{
	String rawData = stream->getAsString();

	TiXmlDocument parseData;
	parseData.Parse(rawData.c_str());

	if(strcmp(parseData.RootElement()->Value(), "physicsobject"))
	{
		// Error: incorrect document type
		return;
	}

	const char *name = 0;
	if(name = parseData.RootElement()->Attribute("name"))
	{
		if(validName(name))
			this->mName = String(name);
		else {} // Warning: invalid name
	}

	getDisplayMeshReferences(parseData.RootElement());
	getCollisionMeshReferences(parseData.RootElement());
	getParts(parseData.RootElement());
	getConstraints(parseData.RootElement());
}

Body *SoftBody::createInstance() const
{
	// TODO: soft body!
	return 0;
}

Body *RigidBody::createInstance() const
{
	RigidBody *b = new RigidBody(transform, id, mass, restitution, angularDamping, linearDamping, friction);

	for(size_t i = 0; i < shapes.size(); i++)
	{
		// TODO: make copies instead? yes, if we want them to be deformable
		b->shapes.push_back(shapes[i]);
	}

	for(size_t i = 0; i < attachments.size(); i++)
	{
		b->attachments.push_back(attachments[i]->createInstance());
	}

	if(parentReference)
		b->parentReference = parentReference->createInstance();

	b->maxControllerTorque = maxControllerTorque;
	b->controllerTorqueScale = controllerTorqueScale;
	b->nonCollideGroup = nonCollideGroup;

	return b;
}

Part *Part::createInstance() const
{
	Part *p;
	if(hasName)
		p = new Part(transform, id, name);
	else
		p = new Part(transform, id);

	for(size_t i = 0; i < subParts.size(); i++)
	{
		Part *sp = subParts[i]->createInstance();
		p->subParts.push_back(sp);
		if(sp->isNamed())
			p->subPartIndex[sp->getName()] = sp->getID();
	}

	for(size_t i = 0; i < bodies.size(); i++)
		p->bodies.push_back(bodies[i]->createInstance());

	return p;
}

PhysicsResource::MeshList *PhysicsResource::getDisplayMeshInstances(PhysicsResource *copier) const
{
	MeshList *l = new MeshList();
	for(size_t i = 0; i < displayMeshes.size(); i++)
		l->push_back(displayMeshes[i]->createInstance());
	
	return l;
}

PhysicsResource::MeshList *PhysicsResource::getCollisionMeshInstances(PhysicsResource *copier) const
{
	MeshList *l = new MeshList();
	for(size_t i = 0; i < collisionMeshes.size(); i++)
		l->push_back(collisionMeshes[i]->createInstance());

	return l;
}

PhysicsResource *PhysicsResource::createInstance()
{
	// TODO: this more elegantly
	PhysicsResource *r = new PhysicsResource(*this);
	nInstances++;
	return r;
}

PhysicsResource::PhysicsResource(const PhysicsResource &orig) : nInstances(0), partIndex(orig.partIndex)
{
	std::stringstream instancename;
	instancename << orig.mName << "_instance_" << orig.nInstances;
	this->mName = instancename.str();

	this->displayMeshes = *orig.getDisplayMeshInstances(this);
	this->collisionMeshes = *orig.getCollisionMeshInstances(this);
	orig.getPartInstances(this);
	this->constraints = *orig.getConstraintInstances(this);
}

PhysicsResource::PartList *PhysicsResource::getPartInstances(PhysicsResource *copier) const
{
	PartList *l = new PartList();

	for(size_t i = 0; i < parts.size(); i++)
		l->push_back(parts[i]->createInstance());

	copier->parts = *l;

	for(size_t i = 0; i < l->size(); i++)
		(*l)[i]->construct(copier);

	return l;
}

Part *PhysicsResource::getPart(const String &n)
{
	if(partIndex.find(n) != partIndex.end())
	{
		return parts[partIndex[n]];
	}
	// TODO: verify it's a number at this point
	
	if(n[0] < '0' || n[0] > '9')
		return 0;

	int id;
	std::stringstream number(n);
	number >> id;

	if(id >= parts.size()) {return 0;} // ERROR: invalid part!

	return parts[id];
}

Part *Part::getSubPart(String &n)
{
	if(subPartIndex.find(n) != subPartIndex.end())
		return subParts[subPartIndex[n]];
	
	int id;
	std::stringstream number(n);
	number >> id;
	return subParts[id];
}

PhysicsResource::ConstraintList *PhysicsResource::getConstraintInstances(PhysicsResource *copier) const
{
	ConstraintList *l = new ConstraintList();

	for(size_t i = 0; i < constraints.size(); i++)
	{
		l->push_back(constraints[i]->createInstance());
	}

	for(size_t i = 0; i < l->size(); i++)
		(*l)[i]->construct(copier);

	return l;
}

size_t PhysicsResource::calculateSize() const
{
	return 0; // TODO: calculate it!
}