//|||||||||||||||||||||||||||||||||||||||||||||||

#ifndef OGRE_DEMO_HPP
#define OGRE_DEMO_HPP

//|||||||||||||||||||||||||||||||||||||||||||||||

#include "OgreFramework.h"
#include "PhysicsListener.h"
#include "PoseController.h"
#include "MotionPlan.h"

//|||||||||||||||||||||||||||||||||||||||||||||||

class LabManApp : public OIS::KeyListener, public OIS::MouseListener, public PhysicsListener
{
public:
	LabManApp();
	~LabManApp();

	void start();
	
	bool keyPressed(const OIS::KeyEvent &keyEventRef);
	bool keyReleased(const OIS::KeyEvent &keyEventRef);

	bool mouseMoved(const OIS::MouseEvent &evt);
	bool mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id);
	bool mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id);

	void physicsCallback(btDynamicsWorld *w, btScalar time);

private:
	void setupScene();
	void run();
	void update(double t);

	InverseKinematics*          ik;
	MotionPlan                  plan;
	PoseController*             controller;
	InverseKinematics::NameList graspDontCare;

	Ogre::SceneNode*			m_pCubeNode;
	Ogre::Entity*				m_pCubeEntity;

	Ogre::Overlay*              m_pOverlay;
	Ogre::OverlayContainer*     m_pCursor;

	Ogre::PhysicsResource*      m_pBall;

	bool						m_bShutdown;
};

//|||||||||||||||||||||||||||||||||||||||||||||||

#endif 

//|||||||||||||||||||||||||||||||||||||||||||||||