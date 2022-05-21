////
//// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
////
//// This software is provided 'as-is', without any express or implied
//// warranty.  In no event will the authors be held liable for any damages
//// arising from the use of this software.
//// Permission is granted to anyone to use this software for any purpose,
//// including commercial applications, and to alter it and redistribute it
//// freely, subject to the following restrictions:
//// 1. The origin of this software must not be misrepresented; you must not
////    claim that you wrote the original software. If you use this software
////    in a product, an acknowledgment in the product documentation would be
////    appreciated but is not required.
//// 2. Altered source versions must be plainly marked as such, and must not be
////    misrepresented as being the original software.
//// 3. This notice may not be removed or altered from any source distribution.
////
//
//#define _USE_MATH_DEFINES
//#include <math.h>
//#include <stdio.h>
//#include <string.h>
//#include <float.h>
//#include "SDL.h"
//#include "imgui.h"
//#include "CrowdTool.h"
//
//#include <Recast.h>
//
//#include "DetourCrowd.h"
//#include "DetourDebugDraw.h"
//#include "DetourObstacleAvoidance.h"
//#include "DetourCommon.h"
//#include "DetourNode.h"
//
//#ifdef WIN32
//#	define snprintf _snprintf
//#endif
//
///// These are just sample areas to use consistent values across the samples.
///// The use should specify these base on his needs.
//enum SamplePolyAreas
//{
//	SAMPLE_POLYAREA_GROUND,
//	SAMPLE_POLYAREA_WATER,
//	SAMPLE_POLYAREA_ROAD,
//	SAMPLE_POLYAREA_DOOR,
//	SAMPLE_POLYAREA_GRASS,
//	SAMPLE_POLYAREA_JUMP,
//};
//enum SamplePolyFlags
//{
//	SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
//	SAMPLE_POLYFLAGS_SWIM = 0x02,		// Ability to swim (water).
//	SAMPLE_POLYFLAGS_DOOR = 0x04,		// Ability to move through doors.
//	SAMPLE_POLYFLAGS_JUMP = 0x08,		// Ability to jump.
//	SAMPLE_POLYFLAGS_DISABLED = 0x10,		// Disabled polygon
//	SAMPLE_POLYFLAGS_ALL = 0xffff	// All abilities.
//};
//
//static bool isectSegAABB(const float* sp, const float* sq,
//						 const float* amin, const float* amax,
//						 float& tmin, float& tmax)
//{
//	static const float EPS = 1e-6f;
//	
//	float d[3];
//	dtVsub(d, sq, sp);
//	tmin = 0;  // set to -FLT_MAX to get first hit on line
//	tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
//	
//	// For all three slabs
//	for (int i = 0; i < 3; i++)
//	{
//		if (fabsf(d[i]) < EPS)
//		{
//			// Ray is parallel to slab. No hit if origin not within slab
//			if (sp[i] < amin[i] || sp[i] > amax[i])
//				return false;
//		}
//		else
//		{
//			// Compute intersection t value of ray with near and far plane of slab
//			const float ood = 1.0f / d[i];
//			float t1 = (amin[i] - sp[i]) * ood;
//			float t2 = (amax[i] - sp[i]) * ood;
//			// Make t1 be intersection with near plane, t2 with far plane
//			if (t1 > t2) dtSwap(t1, t2);
//			// Compute the intersection of slab intersections intervals
//			if (t1 > tmin) tmin = t1;
//			if (t2 < tmax) tmax = t2;
//			// Exit with no collision as soon as slab intersection becomes empty
//			if (tmin > tmax) return false;
//		}
//	}
//	
//	return true;
//}
//
//static void getAgentBounds(const dtCrowdAgent* ag, float* bmin, float* bmax)
//{
//	const float* p = ag->npos;
//	const float r = ag->params.radius;
//	const float h = ag->params.height;
//	bmin[0] = p[0] - r;
//	bmin[1] = p[1];
//	bmin[2] = p[2] - r;
//	bmax[0] = p[0] + r;
//	bmax[1] = p[1] + h;
//	bmax[2] = p[2] + r;
//}
//
//CrowdToolState::CrowdToolState() :
//	m_sample(0),
//	m_nav(0),
//	m_crowd(0),
//	m_targetRef(0),
//	m_run(true)
//{
//	m_toolParams.m_expandSelectedDebugDraw = true;
//	m_toolParams.m_showCorners = false;
//	m_toolParams.m_showCollisionSegments = false;
//	m_toolParams.m_showPath = false;
//	m_toolParams.m_showVO = false;
//	m_toolParams.m_showOpt = false;
//	m_toolParams.m_showNeis = false;
//	m_toolParams.m_expandDebugDraw = false;
//	m_toolParams.m_showLabels = false;
//	m_toolParams.m_showGrid = false;
//	m_toolParams.m_showNodes = false;
//	m_toolParams.m_showPerfGraph = false;
//	m_toolParams.m_showDetailAll = false;
//	m_toolParams.m_expandOptions = true;
//	m_toolParams.m_anticipateTurns = true;
//	m_toolParams.m_optimizeVis = true;
//	m_toolParams.m_optimizeTopo = true;
//	m_toolParams.m_obstacleAvoidance = true;
//	m_toolParams.m_obstacleAvoidanceType = 3.0f;
//	m_toolParams.m_separation = false;
//	m_toolParams.m_separationWeight = 2.0f;
//	
//	memset(m_trails, 0, sizeof(m_trails));
//	
//	m_vod = dtAllocObstacleAvoidanceDebugData();
//	m_vod->init(2048);
//	
//	memset(&m_agentDebug, 0, sizeof(m_agentDebug));
//	m_agentDebug.idx = -1;
//	m_agentDebug.vod = m_vod;
//}
//
//CrowdToolState::~CrowdToolState()
//{
//	dtFreeObstacleAvoidanceDebugData(m_vod);
//}
//
//void CrowdToolState::init(class Sample* sample)
//{
//	if (m_sample != sample)
//	{
//		m_sample = sample;
//	}
//	
//	dtNavMesh* nav = m_sample->getNavMesh();
//	dtCrowd* crowd = m_sample->getCrowd();
//	
//	if (nav && crowd && (m_nav != nav || m_crowd != crowd))
//	{
//		m_nav = nav;
//		m_crowd = crowd;
//	
//		crowd->init(MAX_AGENTS, m_sample->getAgentRadius(), nav);
//		
//		// Make polygons with 'disabled' flag invalid.
//		crowd->getEditableFilter(0)->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);
//		
//		// Setup local avoidance params to different qualities.
//		dtObstacleAvoidanceParams params;
//		// Use mostly default settings, copy from dtCrowd.
//		memcpy(&params, crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));
//		
//		// Low (11)
//		params.velBias = 0.5f;
//		params.adaptiveDivs = 5;
//		params.adaptiveRings = 2;
//		params.adaptiveDepth = 1;
//		crowd->setObstacleAvoidanceParams(0, &params);
//		
//		// Medium (22)
//		params.velBias = 0.5f;
//		params.adaptiveDivs = 5; 
//		params.adaptiveRings = 2;
//		params.adaptiveDepth = 2;
//		crowd->setObstacleAvoidanceParams(1, &params);
//		
//		// Good (45)
//		params.velBias = 0.5f;
//		params.adaptiveDivs = 7;
//		params.adaptiveRings = 2;
//		params.adaptiveDepth = 3;
//		crowd->setObstacleAvoidanceParams(2, &params);
//		
//		// High (66)
//		params.velBias = 0.5f;
//		params.adaptiveDivs = 7;
//		params.adaptiveRings = 3;
//		params.adaptiveDepth = 3;
//		
//		crowd->setObstacleAvoidanceParams(3, &params);
//	}
//}
//
//void CrowdToolState::reset()
//{
//}
//
//void CrowdToolState::handleUpdate(const float dt)
//{
//	if (m_run)
//		updateTick(dt);
//}
//
//void CrowdToolState::addAgent(const float* p)
//{
//	if (!m_sample) return;
//	dtCrowd* crowd = m_sample->getCrowd();
//	
//	dtCrowdAgentParams ap;
//	memset(&ap, 0, sizeof(ap));
//	ap.radius = m_sample->getAgentRadius();
//	ap.height = m_sample->getAgentHeight();
//	ap.maxAcceleration = 8.0f;
//	ap.maxSpeed = 3.5f;
//	ap.collisionQueryRange = ap.radius * 12.0f;
//	ap.pathOptimizationRange = ap.radius * 30.0f;
//	ap.updateFlags = 0; 
//	if (m_toolParams.m_anticipateTurns)
//		ap.updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
//	if (m_toolParams.m_optimizeVis)
//		ap.updateFlags |= DT_CROWD_OPTIMIZE_VIS;
//	if (m_toolParams.m_optimizeTopo)
//		ap.updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
//	if (m_toolParams.m_obstacleAvoidance)
//		ap.updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
//	if (m_toolParams.m_separation)
//		ap.updateFlags |= DT_CROWD_SEPARATION;
//	ap.obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
//	ap.separationWeight = m_toolParams.m_separationWeight;
//	
//	int idx = crowd->addAgent(p, &ap);
//	if (idx != -1)
//	{
//		if (m_targetRef)
//			crowd->requestMoveTarget(idx, m_targetRef, m_targetPos);
//		
//		// Init trail
//		AgentTrail* trail = &m_trails[idx];
//		for (int i = 0; i < AGENT_MAX_TRAIL; ++i)
//			dtVcopy(&trail->trail[i*3], p);
//		trail->htrail = 0;
//	}
//}
//
//void CrowdToolState::removeAgent(const int idx)
//{
//	if (!m_sample) return;
//	dtCrowd* crowd = m_sample->getCrowd();
//
//	crowd->removeAgent(idx);
//	
//	if (idx == m_agentDebug.idx)
//		m_agentDebug.idx = -1;
//}
//
//void CrowdToolState::hilightAgent(const int idx)
//{
//	m_agentDebug.idx = idx;
//}
//
//static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
//{
//	dtVsub(vel, tgt, pos);
//	vel[1] = 0.0;
//	dtVnormalize(vel);
//	dtVscale(vel, vel, speed);
//}
//
//void CrowdToolState::setMoveTarget(const float* p, bool adjust)
//{
//	if (!m_sample) return;
//	
//	// Find nearest point on navmesh and set move request to that location.
//	dtNavMeshQuery* navquery = m_sample->getNavMeshQuery();
//	dtCrowd* crowd = m_sample->getCrowd();
//	const dtQueryFilter* filter = crowd->getFilter(0);
//	const float* halfExtents = crowd->getQueryExtents();
//
//	if (adjust)
//	{
//		float vel[3];
//		// Request velocity
//		if (m_agentDebug.idx != -1)
//		{
//			const dtCrowdAgent* ag = crowd->getAgent(m_agentDebug.idx);
//			if (ag && ag->active)
//			{
//				calcVel(vel, ag->npos, p, ag->params.maxSpeed);
//				crowd->requestMoveVelocity(m_agentDebug.idx, vel);
//			}
//		}
//		else
//		{
//			for (int i = 0; i < crowd->getAgentCount(); ++i)
//			{
//				const dtCrowdAgent* ag = crowd->getAgent(i);
//				if (!ag->active) continue;
//				calcVel(vel, ag->npos, p, ag->params.maxSpeed);
//				crowd->requestMoveVelocity(i, vel);
//			}
//		}
//	}
//	else
//	{
//		navquery->findNearestPoly(p, halfExtents, filter, &m_targetRef, m_targetPos);
//		
//		if (m_agentDebug.idx != -1)
//		{
//			const dtCrowdAgent* ag = crowd->getAgent(m_agentDebug.idx);
//			if (ag && ag->active)
//				crowd->requestMoveTarget(m_agentDebug.idx, m_targetRef, m_targetPos);
//		}
//		else
//		{
//			for (int i = 0; i < crowd->getAgentCount(); ++i)
//			{
//				const dtCrowdAgent* ag = crowd->getAgent(i);
//				if (!ag->active) continue;
//				crowd->requestMoveTarget(i, m_targetRef, m_targetPos);
//			}
//		}
//	}
//}
//
//int CrowdToolState::hitTestAgents(const float* s, const float* p)
//{
//	if (!m_sample) return -1;
//	dtCrowd* crowd = m_sample->getCrowd();
//	
//	int isel = -1;
//	float tsel = FLT_MAX;
//
//	for (int i = 0; i < crowd->getAgentCount(); ++i)
//	{
//		const dtCrowdAgent* ag = crowd->getAgent(i);
//		if (!ag->active) continue;
//		float bmin[3], bmax[3];
//		getAgentBounds(ag, bmin, bmax);
//		float tmin, tmax;
//		if (isectSegAABB(s, p, bmin,bmax, tmin, tmax))
//		{
//			if (tmin > 0 && tmin < tsel)
//			{
//				isel = i;
//				tsel = tmin;
//			} 
//		}
//	}
//
//	return isel;
//}	
//
//void CrowdToolState::updateAgentParams()
//{
//	if (!m_sample) return;
//	dtCrowd* crowd = m_sample->getCrowd();
//	if (!crowd) return;
//	
//	unsigned char updateFlags = 0;
//	unsigned char obstacleAvoidanceType = 0;
//	
//	if (m_toolParams.m_anticipateTurns)
//		updateFlags |= DT_CROWD_ANTICIPATE_TURNS;
//	if (m_toolParams.m_optimizeVis)
//		updateFlags |= DT_CROWD_OPTIMIZE_VIS;
//	if (m_toolParams.m_optimizeTopo)
//		updateFlags |= DT_CROWD_OPTIMIZE_TOPO;
//	if (m_toolParams.m_obstacleAvoidance)
//		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
//	if (m_toolParams.m_obstacleAvoidance)
//		updateFlags |= DT_CROWD_OBSTACLE_AVOIDANCE;
//	if (m_toolParams.m_separation)
//		updateFlags |= DT_CROWD_SEPARATION;
//	
//	obstacleAvoidanceType = (unsigned char)m_toolParams.m_obstacleAvoidanceType;
//	
//	dtCrowdAgentParams params;
//	
//	for (int i = 0; i < crowd->getAgentCount(); ++i)
//	{
//		const dtCrowdAgent* ag = crowd->getAgent(i);
//		if (!ag->active) continue;
//		memcpy(&params, &ag->params, sizeof(dtCrowdAgentParams));
//		params.updateFlags = updateFlags;
//		params.obstacleAvoidanceType = obstacleAvoidanceType;
//		params.separationWeight = m_toolParams.m_separationWeight;
//		crowd->updateAgentParameters(i, &params);
//	}	
//}
//
//void CrowdToolState::updateTick(const float dt)
//{
//	if (!m_sample) return;
//	dtNavMesh* nav = m_sample->getNavMesh();
//	dtCrowd* crowd = m_sample->getCrowd();
//	if (!nav || !crowd) return;
//	
//	TimeVal startTime = getPerfTime();
//	
//	crowd->update(dt, &m_agentDebug);
//	
//	TimeVal endTime = getPerfTime();
//	
//	// Update agent trails
//	for (int i = 0; i < crowd->getAgentCount(); ++i)
//	{
//		const dtCrowdAgent* ag = crowd->getAgent(i);
//		AgentTrail* trail = &m_trails[i];
//		if (!ag->active)
//			continue;
//		// Update agent movement trail.
//		trail->htrail = (trail->htrail + 1) % AGENT_MAX_TRAIL;
//		dtVcopy(&trail->trail[trail->htrail*3], ag->npos);
//	}
//	
//	m_agentDebug.vod->normalizeSamples();
//	
//	m_crowdSampleCount.addSample((float)crowd->getVelocitySampleCount());
//	m_crowdTotalTime.addSample(getPerfTimeUsec(endTime - startTime) / 1000.0f);
//}
//
//
//
//
//CrowdTool::CrowdTool() :
//	m_sample(0),
//	m_state(0),
//	m_mode(TOOLMODE_CREATE)
//{
//}
//
//void CrowdTool::reset()
//{	
//}
//
//void CrowdTool::handleStep()
//{
//	if (!m_state) return;
//	
//	const float dt = 1.0f/20.0f;
//	m_state->updateTick(dt);
//
//	m_state->setRunning(false);
//}
//
//void CrowdTool::handleToggle()
//{
//	if (!m_state) return;
//	m_state->setRunning(!m_state->isRunning());
//}
//
//void CrowdTool::handleUpdate(const float dt)
//{
//	rcIgnoreUnused(dt);
//}
//
//void CrowdTool::handleRender()
//{
//}
//
//
