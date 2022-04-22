#pragma once
#include "Recast.h"
#include "DetourCommon.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"

#include "DetourNavMesh.h"

#include "Model.h"


class AftrNavMesh
{
public:
	AftrNavMesh(Aftr::Model* model);

	

	rcContext NavContext;
	rcConfig NavConfig;
	rcHeightfield *heightfield;
	rcCompactHeightfield *ch;
	rcContourSet *contourset;
	rcPolyMesh *pm;
	rcPolyMeshDetail *pmd;

	dtNavMeshQuery *m_navQuery;
	dtCrowd* m_crowd;
	

	float agentHeight, agentRadius, agentMaxClimb;

	Aftr::Model* inputModel;

	void CreateNavSurface();

};

