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


struct navData
{
	float* verts;
	int numVerts;

	int* indeces;
	int numindeces;
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
};

class AftrNavMesh
{

	int numTris;


public:


	SamplePartitionType m_partitionType = SAMPLE_PARTITION_WATERSHED;


	bool m_filterLowHangingObstacles = true;
	bool m_filterLedgeSpans = true;
	bool m_filterWalkableLowHeightSpans = true;

	float m_cellSize = .30f, m_cellHeight = .20f,
	m_agentMaxSlope = 45.0f, m_agentHeight = 2.0f, m_agentMaxClimb = 0.9f, m_agentRadius = 0.6f,
	m_edgeMaxLen = 12.0f, m_edgeMaxError = 1.3f, m_regionMinSize = 8.0f, m_regionMergeSize = 20.0f,
	m_vertsPerPoly = 6.0f, m_detailSampleDist = 6.0f, m_detailSampleMaxError = 1.0f;

	static void AftrToRC(Aftr::Vector in, float* &out) { out = new float[3]{ in.x, in.z, in.y }; }
	static Aftr::Vector RCToAftr(float*& in) { return Aftr::Vector(in[0], in[2], in[1]); }



	navData ownedData;

	AftrNavMesh();
	~AftrNavMesh();

	bool CreateNavSurface(std::string model, Aftr::Vector position);

	float* bmax = nullptr, *bmin = nullptr;

	rcContext* NavContext;
	rcConfig NavConfig;
	rcHeightfield *heightfield;
	rcCompactHeightfield *ch;
	rcContourSet *contourset;
	rcPolyMesh *pm;
	rcPolyMeshDetail *pmd;

	dtNavMeshQuery *m_navQuery;
	dtNavMesh* m_navMesh;
	dtCrowd* m_crowd;

	unsigned char* m_triareas;

	//Simple callback
	std::function<void(void)> onNavMeshBuilt;

	Aftr::Model* inputModel;

	int* tris;
};

enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};
enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
	SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
	SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
};