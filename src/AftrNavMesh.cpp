#include "AftrNavMesh.h"

#include <DebugDraw.h>
#include <RecastDebugDraw.h>
#include <RecastDump.h>
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"


AftrNavMesh::AftrNavMesh()
{
	NavConfig = rcConfig();
	NavContext = new rcContext();
	m_navQuery = new dtNavMeshQuery();
}

AftrNavMesh::~AftrNavMesh()
{
	rcFreeHeightField(heightfield);
	rcFreeCompactHeightfield(ch);
	rcFreeContourSet(contourset);
	rcFreePolyMesh(pm);
	rcFreePolyMeshDetail(pmd);
}	

bool AftrNavMesh::CreateNavSurface(std::string model, Aftr::Vector position)
{

	InputGeom geo = InputGeom();

	geo.load(NavContext, model);

	const float* bmin = geo.getNavMeshBoundsMin();
	const float* bmax = geo.getNavMeshBoundsMax();
	const float* verts = geo.getMesh()->getVerts();
	const int nverts = geo.getMesh()->getVertCount();
	const int* tris = geo.getMesh()->getTris();
	const int ntris = geo.getMesh()->getTriCount();

	if (!bmax || !bmin) return false;

	memset(&NavConfig, 0, sizeof(NavConfig));
	rcVcopy(NavConfig.bmin, bmin);
	rcVcopy(NavConfig.bmax, bmax);
	NavConfig.cs = m_cellSize;
	NavConfig.ch = m_cellHeight;
	NavConfig.walkableSlopeAngle = m_agentMaxSlope;
	NavConfig.walkableHeight = (int)ceilf(m_agentHeight / NavConfig.ch);
	NavConfig.walkableClimb = (int)floorf(m_agentMaxClimb / NavConfig.ch);
	NavConfig.walkableRadius = (int)ceilf(m_agentRadius / NavConfig.cs);
	NavConfig.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	NavConfig.maxSimplificationError = m_edgeMaxError;
	NavConfig.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	NavConfig.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	NavConfig.maxVertsPerPoly = (int)m_vertsPerPoly;
	NavConfig.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	NavConfig.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	rcCalcGridSize(NavConfig.bmin, NavConfig.bmax, NavConfig.cs, &NavConfig.width, &NavConfig.height);


	



	// Reset build times gathering.
	NavContext->resetTimers();

	// Start the build process.	
	NavContext->startTimer(RC_TIMER_TOTAL);

	NavContext->log(RC_LOG_PROGRESS, "Building navigation:");
	NavContext->log(RC_LOG_PROGRESS, " - %d x %d cells", NavConfig.width, NavConfig.height);
	NavContext->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, numTris / 1000.0f);


	//Rasterize polygon soup
	heightfield = rcAllocHeightfield();
	if (!heightfield)
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(NavContext, *heightfield, NavConfig.width, NavConfig.height, NavConfig.bmin, NavConfig.bmax, NavConfig.cs, NavConfig.ch))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", numTris);
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
// If your input data is multiple meshes, you can transform them here, calculate
// the are type for each of the meshes and rasterize them.
	memset(m_triareas, 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(NavContext, NavConfig.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	if (!rcRasterizeTriangles(NavContext, verts, nverts, tris, m_triareas, ntris, *heightfield, NavConfig.walkableClimb))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(NavContext, NavConfig.walkableClimb, *heightfield);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(NavContext, NavConfig.walkableHeight, NavConfig.walkableClimb, *heightfield);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(NavContext, NavConfig.walkableHeight, *heightfield);
		
	ch = rcAllocCompactHeightfield();
	if (!ch)
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(NavContext, NavConfig.walkableHeight, NavConfig.walkableClimb, *heightfield, *ch))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	if (!rcErodeWalkableArea(NavContext, NavConfig.walkableRadius, *ch))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	const ConvexVolume* vols = geo.getConvexVolumes(); /*would need to get convex volumes*/;
	for (int i = 0; i < -1 /*m_geom->getConvexVolumeCount()*/; ++i) /*would need to get convex volumes num*/
		rcMarkConvexPolyArea(NavContext, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *ch);

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		// Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(NavContext, *ch))
		{
			NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(NavContext, *ch, 0, NavConfig.minRegionArea, NavConfig.mergeRegionArea))
		{
			NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		// Partition the walkable surface into simple regions without holes.
		// Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(NavContext, *ch, 0, NavConfig.minRegionArea, NavConfig.mergeRegionArea))
		{
			NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
	}
	else // SAMPLE_PARTITION_LAYERS
	{
		// Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(NavContext, *ch, 0, NavConfig.minRegionArea))
		{
			NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
	}

	contourset = rcAllocContourSet();
	if (!contourset)
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(NavContext, *ch, NavConfig.maxSimplificationError, NavConfig.maxEdgeLen, *contourset))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours.
	//

	// Build polygon navmesh from the contours.
	pm = rcAllocPolyMesh();
	if (!pm)
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(NavContext, *contourset, NavConfig.maxVertsPerPoly, *pm))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//

	pmd = rcAllocPolyMeshDetail();
	if (!pmd)
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(NavContext, *pm, *ch, NavConfig.detailSampleDist, NavConfig.detailSampleMaxError, *pmd))
	{
		NavContext->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	

	printf("Recast has been built...\n");

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
	params.detailVertsCount = pm->nverts;
	params.detailTris = pmd->tris;
	params.detailTriCount = pmd->ntris;
	params.walkableHeight = m_agentHeight;
	params.walkableRadius = m_agentRadius;
	params.walkableClimb = m_agentMaxClimb;
	rcVcopy(params.bmin, pm->bmin);
	rcVcopy(params.bmax, pm->bmax);
	params.cs = NavConfig.cs;
	params.ch = NavConfig.ch;
	params.buildBvTree = true;

	unsigned char* dtNavMeshData = 0;
	int dtNavMeshDataSize = 0;

	if (!dtCreateNavMeshData(&params, &dtNavMeshData, &dtNavMeshDataSize))
	{
		NavContext->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
		return false;
	}

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		dtFree(dtNavMeshData);
		NavContext->log(RC_LOG_ERROR, "Could not create Detour navmesh");
		return false;
	}

	dtStatus status;

	status = m_navMesh->init(dtNavMeshData, dtNavMeshDataSize, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		dtFree(dtNavMeshData);
		NavContext->log(RC_LOG_ERROR, "Could not init Detour navmesh");
		return false;
	}


	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		NavContext->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
		return false;
	}

	NavContext->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	duLogBuildTimes(*NavContext, NavContext->getAccumulatedTime(RC_TIMER_TOTAL));
	NavContext->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", pm->nverts, pm->npolys);

	float m_totalBuildTimeMs;
	m_totalBuildTimeMs = NavContext->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

	onNavMeshBuilt();

	return true;

}
