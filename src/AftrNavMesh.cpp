#include "AftrNavMesh.h"

AftrNavMesh::AftrNavMesh(Aftr::Model* model)
{
	NavConfig = rcConfig();
	NavContext = rcContext();

	inputModel = model;
	GLfloat* verts = new GLfloat[3];
	inputModel->getBoundingBox().getBottomLeftXEdge(verts);
	float a[3]{ verts[0], verts[1], verts[2] };

	rcVcopy(NavConfig.bmax, a);

	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();
	heightfield = rcAllocHeightfield();
	ch = rcAllocCompactHeightfield();
	contourset = rcAllocContourSet();
	pm = rcAllocPolyMesh();

}

void AftrNavMesh::CreateNavSurface()
{
	unsigned char* navData = 0;
	int navDataSize = 0;




	if (!rcCreateHeightfield(&NavContext, *heightfield, NavConfig.width, NavConfig.height, NavConfig.bmin, NavConfig.bmax, NavConfig.cs, NavConfig.ch))
	{
		printf("failed to create heightfield");
		return;
	}

	if (!rcBuildCompactHeightfield(&NavContext, NavConfig.walkableHeight, NavConfig.walkableClimb, *heightfield, *ch))
	{
		printf("failed to create compact heightfield");
		return;
	}

	if (!rcBuildContours(&NavContext, *ch, NavConfig.detailSampleMaxError, NavConfig.maxEdgeLen, *contourset))
	{
		printf("failed to create contours");
		return;
	}

	if (!rcBuildPolyMesh(&NavContext, *contourset, 1, *pm))
	{
		printf("failed to create poly mesh");
		return;
	}

	if (!rcBuildPolyMeshDetail(&NavContext, *pm, *ch, NavConfig.detailSampleDist, NavConfig.detailSampleMaxError, *pmd));
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

	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
	{
		printf("Could not build Detour navmesh.");
		return;
	}

	dtNavMesh* m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		dtFree(navData);
		printf("Could not create Detour navmesh");
		return;
	}

	dtStatus status;

	status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		dtFree(navData);
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
