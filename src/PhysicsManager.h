#pragma once
#include "physx/PxPhysicsAPI.h"
#include "physx/PxPhysics.h"
#include "physx/PxScene.h"
#include "physx/PxFoundation.h"

using namespace physx;

namespace Aftr
{
	static class PhysicsManager
	{
	public:
		physx::PxPhysics* physics_;
		physx::PxScene* scene_;

		PhysicsManager()
		{
			PxDefaultAllocator* a = new PxDefaultAllocator();
			PxErrorCallback* e = new PxDefaultErrorCallback();
			foundation_ = PxCreateFoundation(PX_PHYSICS_VERSION, *a, *e);
			physics_ = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation_, physx::PxTolerancesScale(),
				false);

			scene_ = physics_->createScene(PxSceneDesc(PxTolerancesScale()));
		}
	private:
		static PxFoundation* foundation_;

	};
}


