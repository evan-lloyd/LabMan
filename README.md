## Note (Feb 22 2018)
This project was recovered from my backups; no guarantees that things will compile smoothly. The below notes were written for future students of the course, when Professor Terzopoulos asked for permission to use the project as an example course project.

There is an associated [term paper](./Report.pdf) and [video demonstration](https://www.youtube.com/watch?v=8roCJuddQtc).

---

LabMan Project copyright 2010 by Evan Lloyd.
Free for academic use. All other rights are reserved by the author.

Notes/warnings:
Some code may work improperly as I recently had to restore it from an incomplete and not fully-recent backup,
and there are a few visual bugs with the labman model as I also lost the most recent version of that. Also,
be warned that some of the code is truly horrendous. Don't try to import your own models; setting them
up requires a complicated series of steps that I have not been able to duplicate yet, let alone document.
While the CMake system in theory should allow compilation on any platform, I have only tested it on Windows
using Visual Studio 2008.

Code structure:
The LabManApp class demonstrates a simple application utilizing the motion controller hierarchy to reach
for a tennis ball. I recommend starting there to see how the different parts fit together.
Don't even try to comprehend the physics resource system (PhysicsSceneManager directory); let's just say
that it's confusing and poorly written (and mostly deals with Ogre trivialities). Focus on what is going on in the MotionController directory.
The top level of the controller hierarchy is handled by the MotionPlan class, which concatenates multiple
trajectories. The mid-level controller (Motion class) determines when to advance to each pose.
Finally, the low-level PID controller is implemented in the PoseController class.

How to compile:
1) Download and install the CMake build system (http://www.cmake.org) 
2) Download, compile, and install (build the "install" target) the dependencies (listed below) using CMake
3) Run CMake on the root LabMan directory. Set BULLET_INSTALL_DIR to the directory where Bullet was installed.
   The Ogre installation should be detected automatically; if not, try setting the environment variable OGRE_HOME
   to the directory where Ogre was installed.
4) Compile. The program executable will be placed in the "bin" directory under the project root. Make sure that the
   Ogre libraries are accessible to execution from that directory (for instance in Windows, you must copy
   OgreMain.dll, OIS.dll, and RenderSystem_X.dll [where X is Direct3D or GL] into the bin directory)
   
How to run:
Make sure you run the application from within the bin directory so that paths to media files are correct.
Keys:
'b' - toggle "debug" drawing of rigid bodies (overlays collision wireframes)
'u' - toggle updating the simulation
'g' - toggle gravity
'w' - move forward
's' - move backward
'a' - move left
'd' - move right
'q' - place a ball on the table and reach for it
'esc' - exit program

Mouse:
Left click - hold to apply force to the body under the cursor
Right click - hold and drag the mouse to rotate the camera

Dependencies:
. OGRE 3D rendering library - http://www.ogre3d.org
. Bullet physics library - http://www.bulletphysics.com

Included packages:
See the respective package directories for license and other information.

. OpenTissue - a minimal subset of the OpenTissue meta-library (http://www.opentissue.org)
supporting inverse kinematics and numerical optimization is included
. boost - a minimal subset of the Boost library (http://www.boost.org)
is included as it is a dependency of OpenTissue (you may need to download boost and configure for your system,
overwriting config.hpp)