#pragma once
#include "PhysicsManager.h"
#include "PhysicsManager.h"
#include "physx/PxPhysicsAPI.h"

using namespace physx;

namespace Aftr
{
	static class PhysicsManager
	{
	public:
		static PxPhysics* physics_;
		static PxScene* scene_;
	private:
		static PxFoundation* foundation_;

		PhysicsManager()
		{
			PxDefaultAllocator* a = new PxDefaultAllocator();
			PxErrorCallback* e = new PxDefaultErrorCallback();
			foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, *a, *e);
			physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(),
				false);
			
			scene_ = physics_->createScene(PxSceneDesc(PxTolerancesScale()));
		}
	};

}


