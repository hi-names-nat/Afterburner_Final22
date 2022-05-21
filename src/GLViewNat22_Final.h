#pragma once

#include "AftrNavMesh.h"
#include "GLView.h"
#include "DetourCrowd.h"

namespace Aftr
{
	class WODetourActor;
	class Camera;

/**
   \class GLViewNat22_Final
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewNat22_Final : public GLView
{
public:
   static GLViewNat22_Final* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewNat22_Final();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

   void SetUpDetourCrowd();

protected:
   GLViewNat22_Final( const std::vector< std::string >& args );
   virtual void onCreate();

   Model* levelMesh;
   WO* levelWO;
   dtCrowd* crowdManger;
   AftrNavMesh afmesh;
   time_t lastFrameTime;
   bool isCrowdInitialized = false;

   WODetourActor* dtA;
};

/** \} */

} //namespace Aftr
