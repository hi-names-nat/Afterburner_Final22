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
   crowdManger->getEditableAgent(0)->state = DT_CROWDAGENT_STATE_WALKING;

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
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {

   }

    if (key.keysym.sym == SDLK_t)
    {
        
        rcVcopy(crowdManger->getEditableAgent(0)->npos, new float[3]{ cam->getPosition().x, cam->getPosition().y, cam->getPosition().z });
        //crowdManger->getEditableAgent(0)->
    }

    if (key.keysym.sym == SDLK_g)
    {
        dtPolyRef refToGoal(0);
        dtQueryFilter filter = dtQueryFilter();
        const float* halfExtents = crowdManger->getQueryExtents();
        float tgt[3];
        dtPolyRef ref;
        float center[3]{ cam->getPosition().x, cam->getPosition().y, cam->getPosition().z };
        //crowdManger->getNavMeshQuery()->findNearestPoly(center, new float[3]{ 5,5,5 }, &filter, &refToGoal, tgt);
        //filter is invalid?
        crowdManger->getNavMeshQuery()->findRandomPoint(&filter, []() {return (float)0; }, &ref, tgt);
    	crowdManger->requestMoveTarget(0, ref, tgt);
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
           levelWO->getModel()->getSkin(0).setAmbient(aftrColor4f(.1f, .1f, .1f, 1));
           levelWO->getModel()->getSkin(0).setColor4f(aftrColor4f(.1f, .1f, .1f, 1));
           levelWO->getModel()->getSkin(0).setSpecular(aftrColor4f(.1f, .1f, .1f, 1));
           levelWO->getModel()->getSkin(0).setDiffuse(aftrColor4f(.1f, .1f, .1f, 1));





           //Start navmesh generation
           navData nmData;

           Model* temp = levelWO->getModel();

           nmData.numVerts = this->levelWO->getModel()->getModelDataShared()->getCompositeVertexList().size() * 3;
           nmData.verts = std::shared_ptr<float[]>(new float[nmData.numVerts]);
           nmData.numindeces = this->levelWO->getModel()->getModelDataShared()->getCompositeIndexList().size();
           nmData.indeces = std::shared_ptr<unsigned[]>(new unsigned[nmData.numindeces]);

           std::vector<Vector> t = levelWO->getModel()->getModelDataShared()->getCompositeVertexList();
           for (int i = 0; i < t.size(); i++)
           {
               nmData.verts.get()[i * 3] = t[i].x;
               nmData.verts.get()[i * 3 + 1] = t[i].y;
               nmData.verts.get()[i * 3 + 2] = t[i].z;
           }

           std::vector<unsigned> t2 = levelWO->getModel()->getModelDataShared()->getCompositeIndexList();
           for (int i = 0; i < t2.size(); i++)
           {
               nmData.indeces.get()[i] = t2[i];
           }

			//Custom behavior after nm is built, used to call the setup for crowd, could be replaced for a unique solution.
           afmesh.onNavMeshBuilt = [&] {SetUpDetourCrowd(); };
           afmesh.CreateNavSurface(nmData);

           WO* navMeshVis = WO::New();
           navMeshVis->setPosition(Vector(afmesh.m_navMesh->getParams()->orig[0], afmesh.m_navMesh->getParams()->orig[1], afmesh.m_navMesh->getParams()->orig[2]));

			//ModelMeshRenderData data* = new ModelMeshRenderData()
       });
}


void GLViewNat22_Final::SetUpDetourCrowd()
{
    //setup crowd st/ we can use actors on the navmesh

    crowdManger = new dtCrowd();
    crowdManger->init(30, 3, afmesh.m_navMesh);
    dtObstacleAvoidanceParams p;
    
    crowdManger->setObstacleAvoidanceParams(0, &p); 
    dtCrowdAgentParams agentParams;
    agentParams.maxSpeed = 1.25f;
    agentParams.maxAcceleration = .5f;
    
    WODetourActor *dtA = WODetourActor::New(crowdManger, Vector(0, 0, 0), agentParams);
    worldLst->push_back(dtA);
    isCrowdInitialized = true;
}