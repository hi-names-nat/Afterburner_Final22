#include "WODetourActor.h"

#include <Model.h>
#include <WO.h>
#include "IFace.h"
#include "AftrNavMesh.h"

namespace Aftr
{
	WODetourActor::WODetourActor() : WO(), IFace(this)
	{
		std::string aimodel(Aftr::ManagerEnvironmentConfiguration::getLMM() + "/models/AI test model.dae");

		this->model = Model::New(this, aimodel, Vector(1,1,1));
	}

	WODetourActor* WODetourActor::New(dtCrowd* crowd, Vector startPosition)
	{
		WODetourActor* temp = new WODetourActor();
		
		dtCrowdAgentParams ap;
		memset(&ap, 0, sizeof(ap));
		ap.radius = temp->radius;
		ap.height = temp->height;
		ap.maxAcceleration = temp->maxAcceleration;
		ap.maxSpeed = temp->maxSpeed;
		ap.collisionQueryRange = ap.radius * 12.0f;
		ap.pathOptimizationRange = ap.radius * 30.0f;
		ap.updateFlags = 0;
		ap.obstacleAvoidanceType = (unsigned char)0;
		ap.separationWeight = 0;

		float* t2 = new float[3];

		AftrNavMesh::AftrToRC(startPosition, t2);
		temp->agentIndex = crowd->addAgent(t2, &ap);
		temp->myCrowd = crowd;

		return temp;
	}

	void WODetourActor::onUpdateWO()
	{
		float* rcPos = myCrowd->getEditableAgent(agentIndex)->npos;
		Vector newPos = AftrNavMesh::RCToAftr(rcPos);
		setPosition(newPos);
	}

	void WODetourActor::setDestination(Vector position)
	{
		float* pos = new float[3], *refpos = new float[3];
		AftrNavMesh::AftrToRC(getPosition()+Vector(5,0,0), pos);
		dtPolyRef *ref = new dtPolyRef();
		myCrowd->getNavMeshQuery()->findNearestPoly(pos, myCrowd->getQueryExtents(), myCrowd->getFilter(0), ref, refpos);
		myCrowd->getNavMeshQuery()->findRandomPoint(nullptr, []() {return .2f; }, ref, refpos);
		if (!ref)
		{
			printf("invalid ref");
			return;
		}
		myCrowd->requestMoveTarget(agentIndex, *ref, refpos);
	}

}