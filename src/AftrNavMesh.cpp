#include "AftrNavMesh.h"

AftrNavMesh::AftrNavMesh()
{
	NavConfig = rcConfig();
	NavContext = rcContext();

}

void AftrNavMesh::CreateNavSurface(navData data)
{
	//OLD
	{
		//std::vector<float> verts;
		//int numVerts;
		//std::vector<float> tris;
		//int numTris;

		//numVerts = level->getNbVertices();
		//numTris = level->getNbTriangles();

		//const physx::PxVec3 *pxVerts = level->getVertices();

		//if ((numVerts <= 0) || (numTris <= 0)) return;

		//for (int i = 0; i < int(level->getNbVertices()); i++)
		//{
		//	physx::PxVec3 pos = pxVerts[i];
		//	verts.push_back(pos.x);
		//	verts.push_back(pos.y);
		//	verts.push_back(pos.z);
		//}

		//

		//if (level->getTriangleMeshFlags() == physx::PxTriangleMeshFlag::e16_BIT_INDICES)
		//	int indecies = level->getTriangles();
	}

	//Something is wrong here I think
	float* bmax = new float[data.numVerts], * bmin = new float[data.numVerts];
	rcCalcBounds(data.verts.get(), data.numVerts / 3, bmin, bmax);

	numTris = data.numindeces / 3;

	if (!bmax || !bmin) return;

	tris = new int[numTris * 3];
	for (int i = 0; i < numTris; i++)
	{
		tris[i * 3] = data.indeces.get()[i * 3];
		tris[i * 3 + 1] = data.indeces.get()[i * 3 + 1];
		tris[i * 3 + 2] = data.indeces.get()[i * 3 + 2];
	}

	//RC Config
	{
		memset(&NavConfig, 0, sizeof(NavConfig));

		rcVcopy(NavConfig.bmax, bmax);
		rcVcopy(NavConfig.bmin, bmin);

		NavConfig.cs = cellSize;
		NavConfig.ch = cellHeight;
		NavConfig.walkableSlopeAngle = agentMaxSlope;
		NavConfig.walkableHeight = (int)std::ceilf(agentHeight / NavConfig.ch);
		NavConfig.walkableClimb = (int)std::ceilf(agentMaxClimb / NavConfig.ch);
		NavConfig.walkableRadius = (int)std::ceilf(agentRadius / NavConfig.cs);
		NavConfig.maxEdgeLen = maxEdgeLen / cellSize;
		NavConfig.maxSimplificationError = MaxError;
		NavConfig.minRegionArea = (int)rcSqr(MinRegionSZ);
		NavConfig.mergeRegionArea = (int)rcSqr(MergeRegionArea);	// Note: area = size*size
		NavConfig.maxVertsPerPoly = (int)MaxVertsPerPoly;
		NavConfig.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
		NavConfig.detailSampleMaxError = cellHeight * MaxError;
		rcCalcGridSize(NavConfig.bmin, NavConfig.bmax, NavConfig.cs, &NavConfig.width, &NavConfig.height);
	}
	NavContext.resetTimers();

	NavContext.startTimer(RC_TIMER_TOTAL);

	NavContext.log(RC_LOG_PROGRESS, "Building navigation:");
	NavContext.log(RC_LOG_PROGRESS, " - %d x %d cells", NavConfig.width, NavConfig.height);
	NavContext.log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", data.numVerts / 1000.0f, numTris / 1000.0f);

	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();


	heightfield = rcAllocHeightfield();
	if (!heightfield) return;

	if (!rcCreateHeightfield(&NavContext, *heightfield, NavConfig.width, NavConfig.height, NavConfig.bmin, NavConfig.bmax, NavConfig.cs, NavConfig.ch))
	{
		printf("failed to create heightfield");
		return;
	}

	auto* triAreas = new unsigned char[numTris];
	if (!triAreas) 
	{
		printf("failed to create compact heightfield");
		return;
	}

	memset(triAreas, 0, numTris * sizeof(unsigned char));
	rcMarkWalkableTriangles(&NavContext, NavConfig.walkableSlopeAngle, data.verts.get(), data.numVerts, tris, numTris, triAreas);
	if (!rcRasterizeTriangles(&NavContext, data.verts.get(), data.numVerts / 3, tris, triAreas, numTris, *heightfield, NavConfig.walkableClimb)) return;
	

	ch = rcAllocCompactHeightfield();
	if (!ch) return;
	if (!rcBuildCompactHeightfield(&NavContext, NavConfig.walkableHeight, NavConfig.walkableClimb, *heightfield, *ch))
	{
		printf("failed to create compact heightfield");
		return;
	}

	rcErodeWalkableArea(&NavContext, NavConfig.walkableRadius, *ch);

	if (!rcBuildDistanceField(&NavContext, *ch))
	{
		NavContext.log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
		return;
	}

	if (!rcBuildRegions(&NavContext, *ch, 0, NavConfig.minRegionArea, NavConfig.mergeRegionArea))
	{
		NavContext.log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
		return;
	}

	contourset = rcAllocContourSet();
	if (!contourset) return;
	if (!rcBuildContours(&NavContext, *ch, NavConfig.maxSimplificationError, NavConfig.maxEdgeLen, *contourset))
	{
		printf("failed to create contours");
		return;
	}

	pm = rcAllocPolyMesh();
	if (!pm) return;
	if (!rcBuildPolyMesh(&NavContext, *contourset, NavConfig.maxVertsPerPoly, *pm))
	{
		printf("failed to create poly mesh");
		return;
	}

	pmd = rcAllocPolyMeshDetail();
	if (!pmd) return;
	if (!rcBuildPolyMeshDetail(&NavContext, *pm, *ch, NavConfig.detailSampleDist, NavConfig.detailSampleMaxError, *pmd))
	{
		printf("failed to create poly mesh detail");
		return;
	}

	dtNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = pm->verts;
	params.vertCount = pm->nverts;
	params.polys = pm->polys;
	params.polyAreas = pm->areas;
	params.polyFlags = pm->flags;
	params.polyCount = pm->npolys;
	params.nvp = pm->nvp;
	params.detailMeshes = pmd->meshes;
	params.detailVerts = pmd->verts;
	params.detailVertsCount = pmd->nverts;
	params.detailTris = pmd->tris;
	params.detailTriCount = pmd->ntris;
	params.offMeshConVerts = nullptr;
	params.offMeshConRad = nullptr;
	params.offMeshConDir = nullptr;
	params.offMeshConAreas = nullptr;
	params.offMeshConFlags = nullptr;
	params.offMeshConUserID = nullptr;
	params.offMeshConCount = 0;
	params.walkableHeight = agentHeight;
	params.walkableRadius = agentRadius;
	params.walkableClimb = agentMaxClimb;
	rcVcopy(params.bmin, pm->bmin);
	rcVcopy(params.bmax, pm->bmax);
	params.cs = NavConfig.cs;
	params.ch = NavConfig.ch;
	params.buildBvTree = true;

	unsigned char *nData = new unsigned char;
	int nDataSz;

	if (!dtCreateNavMeshData(&params, &nData, &nDataSz))
	{
		printf("Could not build Detour navmesh.");
		return;
	}

	dtNavMesh* m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		dtFree(nData);
		printf("Could not create Detour navmesh");
		return;
	}

	dtStatus status;

	status = m_navMesh->init(nData, nDataSz, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		dtFree(nData);
		printf("Could not init Detour navmesh");
		return;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		printf("Could not init Detour navmesh query");
		return;
	}

	return;
}
