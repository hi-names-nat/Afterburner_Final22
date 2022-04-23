#include "AftrPawn.h"
#include "physx/PxPhysics.h"
#include "PhysicsManager.h"

using namespace physx;
	void Aftr::AftrPawn::onUpdateWO()
	{
		WO::onUpdateWO();
	}

Aftr::AftrPawn::AftrPawn() : WO(), IFace(this)
{
	this->collider = nullptr;
	this->material = nullptr;
}

Aftr::AftrPawn::~AftrPawn()
{
	this->collider->release();
	this->material->release();
}

	Aftr::AftrPawn* Aftr::AftrPawn::New(PhysicsManager m)
	{
		AftrPawn* temp = new AftrPawn();
		PxShape* e = m.physics_->createShape(PxCapsuleGeometry(.25f, 1), *material);
		temp->collider = m.physics_->createRigidDynamic(PxTransform());
		temp->collider->attachShape(*e);
		temp->collider->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		return temp;
	}

	Aftr::AftrPawn* Aftr::AftrPawn::New(float radius, float halfHeight, PhysicsManager m)
	{
		AftrPawn* temp = new AftrPawn();
		PxShape* e = m.physics_->createShape(PxCapsuleGeometry(radius, halfHeight), *material);
		temp->collider = m.physics_->createRigidDynamic(PxTransform());
		temp->collider->attachShape(*e);
		temp->collider->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		temp->radius = radius;
		temp->halfHeight = halfHeight;
		return temp;

	}

	Aftr::AftrPawn* Aftr::AftrPawn::New(float radius, float halfHeight, float maxSpeed, float acceleration,
		float deceleration, PhysicsManager m)
	{
		collider = m.physics_->createRigidDynamic(PxTransform());
		

		AftrPawn* temp = new AftrPawn();
		PxShape *e = m.physics_->createShape(PxCapsuleGeometry(radius, halfHeight), *material);
		temp->collider = m.physics_->createRigidDynamic(PxTransform());
		temp->collider->attachShape(*e);
		temp->collider->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
		temp->accel = acceleration;
		temp->maxSpeed = maxSpeed;
		temp->maxSpeed = maxSpeed;
		temp->halfHeight = halfHeight;
		return temp;
	}
