#pragma once
#include <WO.h>
#include "Camera.h"

namespace physx {
    class PxActor;
    class PxRigidStatic;
    class PxRigidDynamic;
    class PxMaterial;
}

namespace Aftr {

    class PhysicsManager;

    class AftrPawn : public Aftr::WO
    {
    private:
        float currentSpeed = 0;

        physx::PxRigidDynamic* collider;
        physx::PxMaterial* material;

    protected:

        void onUpdateWO() override;
        AftrPawn();
        ~AftrPawn() override;
		
    public:
        //movement vars
        float decel = .75f, maxSpeed = 2, accel = .25f;

        //collider vars
        float radius = .25f, halfHeight = 1;

        AftrPawn* New(PhysicsManager m);
        AftrPawn* New(float radius, float halfHeight, PhysicsManager m);
        AftrPawn* New(float radius, float halfHeight, float maxSpeed, float acceleration, float deceleration, PhysicsManager m);
        



    };
}

