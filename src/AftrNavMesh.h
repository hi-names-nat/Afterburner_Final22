#pragma once
#include "Recast.h"
#include "DetourCommon.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"

#include "DetourNavMesh.h"

#include "Model.h"

#include <Recast.h>

#include "physx/PxPhysicsAPI.h"


struct navData
{
	std::shared_ptr<float[]> verts;
	int numVerts;

	std::shared_ptr<unsigned[]> indeces;
	int numindeces;
};

class AftrNavMesh
{

	int numTris;
public:
	float cellSize = .5f, cellHeight = .5f;
	float agentRadius = .25, agentMaxSlope = 70,
	agentHeight = 5, agentMaxClimb = 5, maxEdgeLen = .25f, MaxError = .2f, MinRegionSZ = 0,
	MergeRegionArea = 0, MaxVertsPerPoly = 3, detailSampleDist = .5f;

	AftrNavMesh();
	void CreateNavSurface(navData data);

	

	rcContext NavContext;
	rcConfig NavConfig;
	rcHeightfield *heightfield;
	rcCompactHeightfield *ch;
	rcContourSet *contourset;
	rcPolyMesh *pm;
	rcPolyMeshDetail *pmd;

	dtNavMeshQuery *m_navQuery;
	dtCrowd* m_crowd;
	


	Aftr::Model* inputModel;
	physx::PxTriangleMeshGeometry* geometry;

	int* tris;

	


};

