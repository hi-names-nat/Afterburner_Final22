#pragma once
#include "RecastNav/Recast/Include/Recast.h"
#include "RecastNav/Detour/Include/DetourCommon.h"
#include "RecastNav/Detour/Include/DetourNavMeshBuilder.h"
#include "RecastNav/Detour/Include/DetourNavMeshQuery.h"
#include "RecastNav/DetourCrowd/Include/DetourCrowd.h"

#include "RecastNav/Detour/Include/DetourNavMesh.h"

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

