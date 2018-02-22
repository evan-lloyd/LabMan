//|||||||||||||||||||||||||||||||||||||||||||||||

#include "OgreFramework.h"
#include "PhysicsSceneManagerPlugin.h"
#include "PhysicsSceneManager.h"
#include "DebugDrawer.h"

//|||||||||||||||||||||||||||||||||||||||||||||||

using namespace Ogre;

//|||||||||||||||||||||||||||||||||||||||||||||||

template<> OgreFramework* Ogre::Singleton<OgreFramework>::ms_Singleton = 0;

//|||||||||||||||||||||||||||||||||||||||||||||||

OgreFramework::OgreFramework()
{
	m_MoveSpeed			= 0.1f;
	m_RotateSpeed		= 0.3f;

	m_bShutDownOgre		= false;
	m_iNumScreenShots	= 0;

	m_pRoot				= 0;
	m_pSceneMgr			= 0;
	m_pRenderWnd		= 0;
	m_pCamera			= 0;
	m_pViewport			= 0;
	m_pLog				= 0;
	m_pTimer			= 0;
	m_pDebugDrawer      = 0;

	m_pInputMgr			= 0;
	m_pKeyboard			= 0;
	m_pMouse			= 0;

	m_pDebugOverlay		= 0;
	m_pInfoOverlay		= 0;

	m_bGravity          = false;
	m_bUpdateSimulation = false;
	m_bShowPhysics      = false;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool OgreFramework::initOgre(Ogre::String wndTitle, OIS::KeyListener *pKeyListener, OIS::MouseListener *pMouseListener)
{
	Ogre::LogManager* logMgr = new Ogre::LogManager();
	
	m_pLog = Ogre::LogManager::getSingleton().createLog("OgreLogfile.log", true, true, false);
	m_pLog->setDebugOutputEnabled(true);
	
	m_pRoot = new Ogre::Root();

	if(!m_pRoot->showConfigDialog())
		return false;
	m_pRenderWnd = m_pRoot->initialise(true, wndTitle);

	PhysicsSceneManagerPlugin *plug = new PhysicsSceneManagerPlugin();
	plug->install();
	plug->initialise();
	m_pSceneMgr = static_cast<Ogre::PhysicsSceneManager*>(m_pRoot->createSceneManager("PhysicsSceneManager"));

	m_pDebugDrawer = new OgreDebugDrawer(m_pSceneMgr);
	m_pDebugDrawer->setDebugMode(btIDebugDraw::DBG_NoDebug);

	m_pSceneMgr->setDebugDrawer(m_pDebugDrawer);
	m_pSceneMgr->debugDisplay(false);

	m_pSceneMgr->setAmbientLight(Ogre::ColourValue(0.0f, 0.0f, 0.0f));
	
	m_pCamera = m_pSceneMgr->createCamera("Camera");
	m_pCamera->setPosition(Vector3(20, 15, 20));
	m_pCamera->lookAt(Vector3(-6, 4.5, -6));
	m_pCamera->setNearClipDistance(1);

	m_pViewport = m_pRenderWnd->addViewport(m_pCamera);
	m_pViewport->setBackgroundColour(ColourValue(0.0f, 0.0f, 0.0f, 1.0f));

	m_pCamera->setAspectRatio(Real(m_pViewport->getActualWidth()) / Real(m_pViewport->getActualHeight()));
	
	m_pViewport->setCamera(m_pCamera);

	unsigned long hWnd = 0;
    OIS::ParamList paramList;
    m_pRenderWnd->getCustomAttribute("WINDOW", &hWnd);

	paramList.insert(OIS::ParamList::value_type("WINDOW", Ogre::StringConverter::toString(hWnd)));

	m_pInputMgr = OIS::InputManager::createInputSystem(paramList);

    m_pKeyboard = static_cast<OIS::Keyboard*>(m_pInputMgr->createInputObject(OIS::OISKeyboard, true));
	m_pMouse = static_cast<OIS::Mouse*>(m_pInputMgr->createInputObject(OIS::OISMouse, true));
    
	m_pMouse->getMouseState().height = m_pRenderWnd->getHeight();
	m_pMouse->getMouseState().width	 = m_pRenderWnd->getWidth();

	if(pKeyListener == 0)
		m_pKeyboard->setEventCallback(this);
	else
		m_pKeyboard->setEventCallback(pKeyListener);

	if(pMouseListener == 0)
		m_pMouse->setEventCallback(this);
	else
		m_pMouse->setEventCallback(pMouseListener);

	Ogre::String secName, typeName, archName;
	Ogre::ConfigFile cf;
    cf.load("resources.cfg");

	Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
        }
    }
	Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

	m_pTimer = new Ogre::Timer();
	m_pTimer->reset();
	
	m_pDebugOverlay = OverlayManager::getSingleton().getByName("Core/DebugOverlay");
	m_pDebugOverlay->show();

	m_pRenderWnd->setActive(true);

	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

OgreFramework::~OgreFramework()
{
	if(m_pInputMgr)
		OIS::InputManager::destroyInputSystem(m_pInputMgr);
	
	delete m_pRoot;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool OgreFramework::keyPressed(const OIS::KeyEvent &keyEventRef)
{
	if(m_pKeyboard->isKeyDown(OIS::KC_ESCAPE))
	{
		m_bShutDownOgre = true;
		return true;
	}

	if(m_pKeyboard->isKeyDown(OIS::KC_SYSRQ))
	{
		m_pRenderWnd->writeContentsToTimestampedFile("BOF_Screenshot_", ".jpg");
		return true;
	}

	if(m_pKeyboard->isKeyDown(OIS::KC_M))
	{
		static int mode = 0;
		
		if(mode == 2)
		{
			m_pCamera->setPolygonMode(PM_SOLID);
			mode = 0;
		}
		else if(mode == 0)
		{
			 m_pCamera->setPolygonMode(PM_WIREFRAME);
			 mode = 1;
		}
		else if(mode == 1)
		{
			m_pCamera->setPolygonMode(PM_POINTS);
			mode = 2;
		}
	}

	if(m_pKeyboard->isKeyDown(OIS::KC_O))
	{
		if(m_pDebugOverlay)
		{
			if(!m_pDebugOverlay->isVisible())
				m_pDebugOverlay->show();
			else
				m_pDebugOverlay->hide();
		}
	}

	if(m_pKeyboard->isKeyDown(OIS::KC_G))
	{
		m_bGravity = !m_bGravity;
		m_pSceneMgr->toggleGravity();
	}

	if(m_pKeyboard->isKeyDown(OIS::KC_B))
	{
		m_bShowPhysics = !m_bShowPhysics;
		m_pSceneMgr->debugDisplay(m_bShowPhysics);
	}

	if(m_pKeyboard->isKeyDown(OIS::KC_U))
	{
		m_bUpdateSimulation = !m_bUpdateSimulation;
		m_pSceneMgr->sceneUpdates(m_bUpdateSimulation);
	}

	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool OgreFramework::keyReleased(const OIS::KeyEvent &keyEventRef)
{
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool OgreFramework::mouseMoved(const OIS::MouseEvent &evt)
{
	m_pCamera->yaw(Degree(evt.state.X.rel * -0.1f));
	m_pCamera->pitch(Degree(evt.state.Y.rel * -0.1f));
	
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool OgreFramework::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id)
{
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

bool OgreFramework::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id)
{
	return true;
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void OgreFramework::updateOgre(double timeSinceLastFrame)
{
	m_MoveScale = m_MoveSpeed   * (float)timeSinceLastFrame;
	m_RotScale  = m_RotateSpeed * (float)timeSinceLastFrame;
		
	m_TranslateVector = Vector3::ZERO;

	getInput();
	moveCamera();
	m_pSceneMgr->updateScene(timeSinceLastFrame);

	updateStats();
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void OgreFramework::updateStats() 
{ 
	static String currFps	= "Current FPS: "; 
    static String avgFps	= "Average FPS: "; 
    static String bestFps	= "Best FPS: "; 
    static String worstFps	= "Worst FPS: "; 
    static String tris		= "Triangle Count: "; 
    static String batches	= "Batch Count: "; 
 
    OverlayElement* guiAvg	= OverlayManager::getSingleton().getOverlayElement("Core/AverageFps"); 
    OverlayElement* guiCurr	= OverlayManager::getSingleton().getOverlayElement("Core/CurrFps"); 
    OverlayElement* guiBest	= OverlayManager::getSingleton().getOverlayElement("Core/BestFps"); 
    OverlayElement* guiWorst = OverlayManager::getSingleton().getOverlayElement("Core/WorstFps"); 

	const RenderTarget::FrameStats& stats = m_pRenderWnd->getStatistics(); 
    guiAvg->setCaption(avgFps + StringConverter::toString(stats.avgFPS)); 
    guiCurr->setCaption(currFps + StringConverter::toString(stats.lastFPS)); 
    guiBest->setCaption(bestFps + StringConverter::toString(stats.bestFPS) 
            +" "+StringConverter::toString(stats.bestFrameTime)+" ms"); 
    guiWorst->setCaption(worstFps + StringConverter::toString(stats.worstFPS) 
            +" "+StringConverter::toString(stats.worstFrameTime)+" ms"); 

    OverlayElement* guiTris = OverlayManager::getSingleton().getOverlayElement("Core/NumTris"); 
    guiTris->setCaption(tris + StringConverter::toString(stats.triangleCount)); 

	OverlayElement* guiBatches = OverlayManager::getSingleton().getOverlayElement("Core/NumBatches"); 
    guiBatches->setCaption(batches + StringConverter::toString(stats.batchCount)); 

	OverlayElement* guiDbg = OverlayManager::getSingleton().getOverlayElement("Core/DebugText"); 
	guiDbg->setCaption("");
} 

//|||||||||||||||||||||||||||||||||||||||||||||||

void OgreFramework::moveCamera()
{
	if(m_pKeyboard->isKeyDown(OIS::KC_LSHIFT)) 
		m_pCamera->moveRelative(m_TranslateVector);
	else
		m_pCamera->moveRelative(m_TranslateVector / 10);
}

//|||||||||||||||||||||||||||||||||||||||||||||||

void OgreFramework::getInput()
{
	if(m_pKeyboard->isKeyDown(OIS::KC_A))
		m_TranslateVector.x = -m_MoveScale;
	
	if(m_pKeyboard->isKeyDown(OIS::KC_D))
		m_TranslateVector.x = m_MoveScale;

	if(m_pKeyboard->isKeyDown(OIS::KC_W))
		m_TranslateVector.z = -m_MoveScale;
	
	if(m_pKeyboard->isKeyDown(OIS::KC_S))
		m_TranslateVector.z = m_MoveScale;
}

//|||||||||||||||||||||||||||||||||||||||||||||||