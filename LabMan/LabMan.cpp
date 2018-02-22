//|||||||||||||||||||||||||||||||||||||||||||||||

#include "LabMan.h"

#include <OgreLight.h>
#include <OgreWindowEventUtilities.h>
#include <OgreOverlayContainer.h>
#include "PhysicsSceneManager.h"

//|||||||||||||||||||||||||||||||||||||||||||||||

LabManApp::LabManApp()
{
	m_pCubeNode			= 0;
	m_pCubeEntity		= 0;
	m_pBall             = 0;

	graspDontCare.insert("Shoulder.l");
	graspDontCare.insert("UpperArm.l");
	graspDontCare.insert("LowerArm.l");
	graspDontCare.insert("Wrist.l");
	graspDontCare.insert("ThumbMeta.l");
	graspDontCare.insert("ThumbInter.l");
	graspDontCare.insert("ThumbDistal.l");
	graspDontCare.insert("IndexMeta.l");
	graspDontCare.insert("IndexInter.l");
	graspDontCare.insert("IndexDistal.l");
	graspDontCare.insert("MiddleMeta.l");
	graspDontCare.insert("MiddleInter.l");
	graspDontCare.insert("MiddleDistal.l");
	graspDontCare.insert("RingMeta.l");
	graspDontCare.insert("RingInter.l");
	graspDontCare.insert("RingDistal.l");
	graspDontCare.insert("PinkyMeta.l");
	graspDontCare.insert("PinkyInter.l");
	graspDontCare.insert("PinkyDistal.l");

	graspDontCare.insert("Head");
	graspDontCare.insert("Neck");
	graspDontCare.insert("ThumbMeta.r");
	graspDontCare.insert("ThumbInter.r");
	graspDontCare.insert("ThumbDistal.r");
	graspDontCare.insert("IndexMeta.r");
	graspDontCare.insert("IndexInter.r");
	graspDontCare.insert("IndexDistal.r");
	graspDontCare.insert("MiddleMeta.r");
	graspDontCare.insert("MiddleInter.r");
	graspDontCare.insert("MiddleDistal.r");
	graspDontCare.insert("RingMeta.r");
	graspDontCare.insert("RingInter.r");
	graspDontCare.insert("RingDistal.r");
	graspDontCare.insert("PinkyMeta.r");
	graspDontCare.insert("PinkyInter.r");
	graspDontCare.insert("PinkyDistal.r");
}

//|||||||||||||||||||||||||||||||||||||||||||||||

LabManApp::~LabManApp()
{
	delete OgreFramework::getSingletonPtr();
}

//|||||||||||||||||||||||||||||||||||||||||||||||

//|||||||||||||||||||||||||||||||||||||||||||||||

void LabManApp::physicsCallback(btDynamicsWorld *w, btScalar time)
{
	controller->controlStep(time);

	if(controller->totalError < controller->errorThreshold && plan.hasNext())
	{
		controller->setTargets(*plan.next(), graspDontCare);
	}
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void LabManApp::start()
{
	new OgreFramework();
	if(!OgreFramework::getSingletonPtr()->initOgre("LabMan", this, this))
		return;
	
	OgreFramework::getSingletonPtr()->m_pSceneMgr->addPhysicsListener(this);

	m_bShutdown = false;

	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Beginning experiment...");

	setupScene();
	run();
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void LabManApp::setupScene()
{
	// Set up GUI
	Ogre::OverlayManager &mgr = OverlayManager::getSingleton();
	m_pCursor = (OverlayContainer*)(mgr.createOverlayElement("Panel", "Cursor"));
	m_pCursor->setMetricsMode(Ogre::GMM_PIXELS);
	m_pCursor->setDimensions(16, 16);
	m_pCursor->setMaterialName("cursor");
	m_pOverlay = mgr.create("Cursor");
	m_pOverlay->add2D(m_pCursor);
	m_pOverlay->setZOrder(-1);
	m_pOverlay->show();
	

	OgreFramework::getSingletonPtr()->m_pSceneMgr->createLight("Light")->setPosition(0,25,10);

	PhysicsResource *labman = OgreFramework::getSingletonPtr()->m_pSceneMgr->createObject("labman.physics", btVector3(0, 8.7, 0));


	InverseKinematics::ConstraintMap constraints;
	// Don't worry, this confusing logic is going away soon (unifying constraint types!)
	for(PhysicsResource::ConstraintList::iterator i = labman->getConstraints().begin(); i != labman->getConstraints().end(); i++)
		constraints[(*i)->getBody(1)->getPartName()] = (btGeneric6DofConstraint*)((*i)->getConstraint(0));
	controller = new PoseController(labman, constraints);
	ik = new InverseKinematics(
		OgreFramework::getSingleton().m_pSceneMgr->getEntity("LabMan_instance_0_mesh_0")->getSkeleton(),
		labman
		);
	plan.addMotion(new Motion());
	for(InverseKinematics::NameList::iterator name = ik->boneNames.begin(); name != ik->boneNames.end(); name++)
	{
		for(int i = 0; i < Motion::TRAJECTORY_SIZE; i++)
			plan[0][i][*name] = new btTransform;
	}

	OgreFramework::getSingletonPtr()->m_pSceneMgr->createObject("table.physics", btVector3(0, 0.01, 6.5));
	m_pBall = OgreFramework::getSingletonPtr()->m_pSceneMgr->createObject("ball.physics", btVector3(0, 8.2, 8));

	// Set up ground
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

	btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0,NULL,groundShape,btVector3(0,0,0));

	Ogre::Plane plane(Vector3::UNIT_Y, 0);
	Ogre::MeshManager::getSingleton().createPlane("ground",
       ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
       150,150,20,20,true,1,5,5,Vector3::UNIT_Z);
	Ogre::Entity *ent = OgreFramework::getSingletonPtr()->m_pSceneMgr->createEntity("GroundEntity", "ground");
	ent->setMaterialName("Examples/GrassFloor");
    ent->setCastShadows(false);

	RigidBodyNode *groundNode = OgreFramework::getSingletonPtr()->m_pSceneMgr->createRigidBodyNode(ent, groundRigidBodyCI, btTransform(btQuaternion(0,0,0,1),btVector3(0,-1,0)), false);
	groundNode->getBody()->setRestitution(0.3);
	OgreFramework::getSingletonPtr()->m_pSceneMgr->getRootSceneNode()->addChild(groundNode);
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void LabManApp::run()
{
	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Start main loop...");
	
	double timeSinceLastFrame = 0;
	double startTime = 0;

	OgreFramework::getSingletonPtr()->m_pRenderWnd->resetStatistics();
	
	while(!m_bShutdown && !OgreFramework::getSingletonPtr()->isOgreToBeShutDown()) 
	{
		if(OgreFramework::getSingletonPtr()->m_pRenderWnd->isClosed())m_bShutdown = true;

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
		Ogre::WindowEventUtilities::messagePump();
#endif	
		if(OgreFramework::getSingletonPtr()->m_pRenderWnd->isActive())
		{
			startTime = OgreFramework::getSingletonPtr()->m_pTimer->getMillisecondsCPU();
					
			OgreFramework::getSingletonPtr()->m_pKeyboard->capture();
			OgreFramework::getSingletonPtr()->m_pMouse->capture();

			update(timeSinceLastFrame);
			OgreFramework::getSingletonPtr()->updateOgre(timeSinceLastFrame);
			
			OgreFramework::getSingletonPtr()->m_pRoot->renderOneFrame();
		
			timeSinceLastFrame = OgreFramework::getSingletonPtr()->m_pTimer->getMillisecondsCPU() - startTime;
		}
		else
		{
			Sleep(1000);
		}
	}

	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Main loop quit");
	OgreFramework::getSingletonPtr()->m_pLog->logMessage("Shutdown OGRE...");
}

//|||||||||||||||||||||||||||||||||||||||||||||||

float randX()
{
	return -6 + rand() * 1.0 / (RAND_MAX + 1.0) * (-2 - -6);
}

float randZ()
{
	return 3 + rand() * 1.0 / (RAND_MAX + 1.0) * (5 - 3);
}

bool LabManApp::keyPressed(const OIS::KeyEvent &keyEventRef)
{
	OgreFramework::getSingletonPtr()->keyPressed(keyEventRef);
	
	if(OgreFramework::getSingletonPtr()->m_pKeyboard->isKeyDown(OIS::KC_Q))
	{
		 // place ball
		 btVector3 pos(randX(), 8.2, randZ());

		 // Sorry about this syntax =/
		 // The minimum to know about PhysicsResources is that at top level they consist of a
		 // hierarchy of "parts". Parts contain one or more (but always one with my usage) Bodies
		 // which themselves reference a btRigidBody.
		 btRigidBody *body = (btRigidBody*)m_pBall->getPart("0")->getBodies()[0]->getBody();
		 body->getWorldTransform().setOrigin(pos);
		 body->getMotionState()->setWorldTransform(body->getWorldTransform());
		 body->setLinearVelocity(btVector3(0, 0, 0));
		 body->setAngularVelocity(btVector3(0, 0, 0));

		 // Reach for ball
		 InverseKinematics::RotationSet rotationTargets;
		 InverseKinematics::PositionSet positionTargets;
		 InverseKinematics::NameList dontCare;
		 InverseKinematics::NameList doCare;

		 ik->bindPose();
		 plan.reset();

		 doCare.insert("Wrist.r");
		 //doCare.insert("LowerBack");

		 set_difference(ik->boneNames.begin(), ik->boneNames.end(), doCare.begin(), doCare.end(), inserter(dontCare, dontCare.begin()));

		 positionTargets["Wrist.r"] = new btVector3(body->getWorldTransform().getOrigin() + btVector3(0, 0.5, 0));

		 ik->getTrajectory(rotationTargets, positionTargets, dontCare, plan[0]);
		 dontCare.clear();

		 controller->setTargets(*plan.next(), graspDontCare);

		 delete positionTargets["Wrist.r"];
	}

	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool LabManApp::keyReleased(const OIS::KeyEvent &keyEventRef)
{
	OgreFramework::getSingletonPtr()->keyReleased(keyEventRef);
	
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool LabManApp::mouseMoved(const OIS::MouseEvent &evt)
{
	if(evt.state.buttonDown(OIS::MB_Right))
		OgreFramework::getSingletonPtr()->mouseMoved(evt);

	m_pCursor->setPosition(evt.state.X.abs, evt.state.Y.abs);

	return true;
}

bool LabManApp::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id)
{
	if(id == OIS::MB_Right)
		m_pCursor->hide();
	return true;
}

bool LabManApp::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id)
{
	if(id == OIS::MB_Right)
		m_pCursor->show();
	return true;
}

void LabManApp::update(double t)
{
	const OIS::MouseState &mouseState = OgreFramework::getSingleton().m_pMouse->getMouseState();

	if(mouseState.buttonDown(OIS::MB_Left))
	{
		Ogre::Ray mouseRay = OgreFramework::getSingletonPtr()->
			m_pCamera->
			getCameraToViewportRay(m_pCursor->getLeft() / mouseState.width,
			m_pCursor->getTop() / mouseState.height);


		OgreFramework::getSingletonPtr()->m_pSceneMgr->applyImpulseAlongRay(mouseRay, 100.0f);
	}
}