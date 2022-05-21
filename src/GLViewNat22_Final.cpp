#include "GLViewNat22_Final.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "AftrNavMesh.h"
#include "AftrNavMesh.h"
#include "WODetourActor.h"

using namespace Aftr;

GLViewNat22_Final* GLViewNat22_Final::New( const std::vector< std::string >& args )
{
   GLViewNat22_Final* glv = new GLViewNat22_Final( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewNat22_Final::GLViewNat22_Final( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewNat22_Final::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewNat22_Final::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewNat22_Final::onCreate()
{
   //GLViewNat22_Final::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}


GLViewNat22_Final::~GLViewNat22_Final()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewNat22_Final::updateWorld()
{
    time_t deltaTime = time(nullptr) - lastFrameTime;
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   if (!isCrowdInitialized) return;

   dtCrowdAgentDebugInfo dbnfo;
   crowdManger->update(deltaTime,  &dbnfo);

   lastFrameTime = time(nullptr);

    

   std::cout << crowdManger->getAgent(0)->npos[0] << " " << crowdManger->getAgent(0)->npos[1] << " " << crowdManger->getAgent(0)->npos[2] << '\n';
}


void GLViewNat22_Final::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewNat22_Final::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewNat22_Final::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewNat22_Final::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewNat22_Final::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );

    if (key.keysym.sym == SDLK_t)
    {
        float* camPosRC;
        AftrNavMesh::AftrToRC(cam->getPosition(), camPosRC);
        
        rcVcopy(crowdManger->getEditableAgent(0)->npos, camPosRC);
    }

    if (key.keysym.sym == SDLK_g)
    {
        dtA->setDestination(cam->getPosition());
    }
}


void GLViewNat22_Final::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewNat22_Final::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );

   std::string level(ManagerEnvironmentConfiguration::getLMM() + "/models/dungeon.obj");
   std::string aimodel(ManagerEnvironmentConfiguration::getLMM() + "/models/AI test model.dae");
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );

   {
      //Create a light
      float ga = 0.1f; //Global Ambient Light level for this module
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
      //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );
   }

   {
      //Create the SkyBox
      WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }
   
   //Make a Dear Im Gui instance via the WOImGui in the engine... This calls
   //the default Dear ImGui demo that shows all the features... To create your own,
   //inherit from WOImGui and override WOImGui::drawImGui_for_this_frame(...) (among any others you need).
   WOImGui* gui = WOImGui::New( nullptr );
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
      [this, gui]() //this is a lambda, the capture clause is in [], the input argument list is in (), and the body is in {}
      {

      } );
   this->worldLst->push_back( gui );

   

   levelWO = WO::New(level, Vector(1,1,1), MESH_SHADING_TYPE::mstAUTO);
   levelWO->setPosition(0, 0, 0);
   worldLst->push_back(levelWO);
   levelWO->upon_async_model_loaded([&]
       {
		std::string navMesh = (ManagerEnvironmentConfiguration::getLMM() + "models/dungeon.obj");

            levelWO->getModel()->getSkin(0).setAmbient(aftrColor4f(.1f, .1f, .1f, 1));
           levelWO->getModel()->getSkin(0).setColor4f(aftrColor4f(.1f, .1f, .1f, 1));
           levelWO->getModel()->getSkin(0).setSpecular(aftrColor4f(.1f, .1f, .1f, 1));
           levelWO->getModel()->getSkin(0).setDiffuse(aftrColor4f(.1f, .1f, .1f, 1));

           

           //Start navmesh generation
           navData nmData;

           Model* temp = levelWO->getModel();


           nmData.numVerts = this->levelWO->getModel()->getModelDataShared()->getCompositeVertexList().size() * 3;
           nmData.verts = new float[nmData.numVerts];
           nmData.numindeces = this->levelWO->getModel()->getModelDataShared()->getCompositeIndexList().size();
           nmData.indeces = new int[nmData.numindeces];


			//Allocate X Z Y, has to be Y - up.
           for (int i = 0; i < nmData.numVerts /3; i++)
           {
               nmData.verts[i * 3] = levelWO->getModel()->getCompositeVertexList()[i].x;
               nmData.verts[i * 3 + 1] = levelWO->getModel()->getCompositeVertexList()[i].z;
               nmData.verts[i * 3 + 2] = levelWO->getModel()->getCompositeVertexList()[i].y;
           }

           for (int i = 0; i < nmData.numindeces; i++)
           {
               nmData.indeces[i] = levelWO->getModel()->getModelDataShared()->getCompositeIndexList()[i];
           }

			//Custom behavior after nm is built, used to call the setup for crowd, could be replaced for a unique solution.
           afmesh.onNavMeshBuilt = [&] {SetUpDetourCrowd(); };
           AftrNavMesh::AftrToRC(temp->getBoundingBox().getMax(), afmesh.bmax);
           AftrNavMesh::AftrToRC(temp->getBoundingBox().getMin(), afmesh.bmin);

           afmesh.CreateNavSurface(navMesh, levelWO->getPosition());
			

			//ModelMeshRenderData data* = new ModelMeshRenderData()
       });
}
 

void GLViewNat22_Final::SetUpDetourCrowd()
{
    //setup crowd st/ we can use actors on the navmesh

    float radius = .25f;

    crowdManger = new dtCrowd();
    dtNavMesh* nav = afmesh.m_navMesh;

    crowdManger->init(128, radius, nav);

    //crowdManger->getEditableFilter(0)->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);

    dtObstacleAvoidanceParams params;
    memcpy(&params, crowdManger->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

    // Low (11)
    params.velBias = 0.5f;
    params.adaptiveDivs = 5;
    params.adaptiveRings = 2;
    params.adaptiveDepth = 1;
    crowdManger->setObstacleAvoidanceParams(0, &params);

    dtA = WODetourActor::New(crowdManger, Vector(0, 0, 0));
    worldLst->push_back(dtA);

}