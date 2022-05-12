// Don't pass planes to me, I will error.

#pragma once
#include <ManagerEnvironmentConfiguration.h>

#include "Recast.h"
#include "DetourCommon.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "DetourDebugDraw.h"

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

	float cellSize = .3f, cellHeight = .2f;
	float agentRadius = .6f, agentMaxSlope = 45,
	agentHeight = 2.0f, agentMaxClimb = .9f, maxEdgeLen = 12, MaxError = 1.3f, MinRegionSZ = 8,
	MergeRegionArea = 20, MaxVertsPerPoly = 6, detailSampleDist = 6, maxSampleError = 1, tilesize = 15;

	static void AftrToRC(Aftr::Vector in, float* &out) { out = new float[3]{ in.x, in.z, in.y }; }
	static Aftr::Vector RCToAftr(const float*& in) { return Aftr::Vector(in[0], in[2], in[1]); }

	AftrNavMesh();
	~AftrNavMesh();

	void CreateNavSurface(navData data);



	rcContext NavContext;
	rcConfig NavConfig;
	rcHeightfield *heightfield;
	rcCompactHeightfield *ch;
	rcContourSet *contourset;
	rcPolyMesh *pm;
	rcPolyMeshDetail *pmd;

	dtNavMeshQuery *m_navQuery;
	dtNavMesh* m_navMesh;
	dtCrowd* m_crowd;

	//Simple callback
	std::function<void(void)> onNavMeshBuilt;

	Aftr::Model* inputModel;
	physx::PxTriangleMeshGeometry* geometry;

	int* tris;
};

