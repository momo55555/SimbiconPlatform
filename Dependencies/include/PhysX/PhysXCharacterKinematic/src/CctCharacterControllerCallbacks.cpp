// This code contains NVIDIA Confidential Information and is disclosed to you 
// under a form of NVIDIA software license agreement provided separately to you.
//
// Notice
// NVIDIA Corporation and its licensors retain all intellectual property and
// proprietary rights in and to this software and related documentation and 
// any modifications thereto. Any use, reproduction, disclosure, or 
// distribution of this software and related documentation without an express 
// license agreement from NVIDIA Corporation is strictly prohibited.
// 
// ALL NVIDIA DESIGN SPECIFICATIONS, CODE ARE PROVIDED "AS IS.". NVIDIA MAKES
// NO WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ALL IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Information and code furnished is believed to be accurate and reliable.
// However, NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2008-2011 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxTriangleMeshExt.h"

#include "CmPhysXCommon.h"
#include "GuTriangle.h"

#include "PxScene.h"
#include "GuBox.h"
#include "GuCapsule.h"
#include "PxSphereGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxBoxGeometry.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PxConvexMesh.h"
#include "PxTriangleMesh.h"
#include "PxHeightField.h"
#include "GuMeshQuery.h"
#include "GuGeometryQuery.h"
#include "PxExtensionsAPI.h"
#include "CctCharacterController.h"
#include "CctController.h"
#include "CctUtils.h"
#include "PxSweepCache.h"
#include "PxBatchQuery.h"
#include "PxTriangleMeshGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxHeightFieldGeometry.h"
#include "PsUserAllocated.h"

#include "PxPhysics.h"
#include "CmRenderOutput.h"

static const bool gVisualizeTouchedTris = false;
static const float gDebugVisOffset = 0.01f;

using namespace physx;
using namespace Cct;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void outputSphereToStream(PxShape* sphereShape, IntArray& geomStream, const PxExtendedVec3& origin)
{
	PX_ASSERT(sphereShape->getGeometryType() == PxGeometryType::eSPHERE);
	PxExtendedSphere WorldSphere;
	{
		PxSphereGeometry sg;
		sphereShape->getSphereGeometry(sg);

		PxVec3 center = PxShapeExt::getGlobalPose(*sphereShape).p;  // LOSS OF ACCURACY

		WorldSphere.radius = sg.radius;
		WorldSphere.center.x = center.x;
		WorldSphere.center.y = center.y;
		WorldSphere.center.z = center.z;
	}

	TouchedSphere* touchedSphere = (TouchedSphere*)reserve(geomStream, sizeof(TouchedSphere)/sizeof(PxU32));
	touchedSphere->mType		= TouchedGeomType::eSPHERE;
	touchedSphere->mUserData	= sphereShape;
	touchedSphere->mOffset		= origin;
	touchedSphere->mRadius		= WorldSphere.radius;
	touchedSphere->mCenter.x	= float(WorldSphere.center.x - origin.x);
	touchedSphere->mCenter.y	= float(WorldSphere.center.y - origin.y);
	touchedSphere->mCenter.z	= float(WorldSphere.center.z - origin.z);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void outputCapsuleToStream(PxShape* capsuleShape, IntArray& geomStream, const PxExtendedVec3& origin)
{
	PX_ASSERT(capsuleShape->getGeometryType() == PxGeometryType::eCAPSULE);
	PxExtendedCapsule WorldCapsule;
	{
		PxCapsuleGeometry cg;
		capsuleShape->getCapsuleGeometry(cg);

		PxTransform globalPose = PxShapeExt::getGlobalPose(*capsuleShape);
		PxVec3 p0, p1;
		p0 = cg.halfHeight * globalPose.q.getBasisVector0();
		p1 = -p0;
		p0 += globalPose.p;
		p1 += globalPose.p;

		WorldCapsule.radius	= cg.radius;
		WorldCapsule.p0.x	= p0.x;
		WorldCapsule.p0.y	= p0.y;
		WorldCapsule.p0.z	= p0.z;
		WorldCapsule.p1.x	= p1.x;
		WorldCapsule.p1.y	= p1.y;
		WorldCapsule.p1.z	= p1.z;
	}

	TouchedCapsule* touchedCapsule	= (TouchedCapsule*)reserve(geomStream, sizeof(TouchedCapsule)/sizeof(PxU32));
	touchedCapsule->mType		= TouchedGeomType::eCAPSULE;
	touchedCapsule->mUserData	= capsuleShape;
	touchedCapsule->mOffset		= origin;
	touchedCapsule->mRadius		= WorldCapsule.radius;
	touchedCapsule->mP0.x		= float(WorldCapsule.p0.x - origin.x);
	touchedCapsule->mP0.y		= float(WorldCapsule.p0.y - origin.y);
	touchedCapsule->mP0.z		= float(WorldCapsule.p0.z - origin.z);
	touchedCapsule->mP1.x		= float(WorldCapsule.p1.x - origin.x);
	touchedCapsule->mP1.y		= float(WorldCapsule.p1.y - origin.y);
	touchedCapsule->mP1.z		= float(WorldCapsule.p1.z - origin.z);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void outputBoxToStream(PxShape* boxShape, IntArray& geomStream, const PxExtendedVec3& origin)
{
	PX_ASSERT(boxShape->getGeometryType() == PxGeometryType::eBOX);
	PxExtendedBox WorldBox;
	{
		PxBoxGeometry bg;
		boxShape->getBoxGeometry(bg);

		PxTransform globalPose = PxShapeExt::getGlobalPose(*boxShape);  // LOSS OF ACCURACY
		PxVec3 center = globalPose.p;
		PxVec3 extents = bg.halfExtents;
		PxMat33Legacy rot = PxMat33Legacy(globalPose.q);

		WorldBox.rot		= rot;
		WorldBox.extents	= extents;
		WorldBox.center.x	= center.x;
		WorldBox.center.y	= center.y;
		WorldBox.center.z	= center.z;
	}

	TouchedBox* touchedBox	= (TouchedBox*)reserve(geomStream, sizeof(TouchedBox)/sizeof(PxU32));
	touchedBox->mType			= TouchedGeomType::eBOX;
	touchedBox->mUserData		= boxShape;
	touchedBox->mOffset			= origin;

	touchedBox->mExtents		= WorldBox.extents;
	touchedBox->mRot			= WorldBox.rot;
	touchedBox->mCenter.x		= float(WorldBox.center.x - origin.x);
	touchedBox->mCenter.y		= float(WorldBox.center.y - origin.y);
	touchedBox->mCenter.z		= float(WorldBox.center.z - origin.z);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void visualizeTouchedTriangles(PxU32 nbTrisToRender, PxU32 startIndex, const Gu::Triangle* triangles, PxScene* scene, const PxVec3& offset, PxU32 upDirection)
{
	PxVec3 yy = -offset;
	yy[upDirection] += gDebugVisOffset;

	for(PxU32 i=0; i<nbTrisToRender; i++)
	{
		const Gu::Triangle& currentTriangle = triangles[i+startIndex];

		Cm::RenderOutput((Cm::RenderBuffer&)scene->getRenderBuffer())
			<< PxDebugColor::eARGB_GREEN << Cm::RenderOutput::TRIANGLES
			<< currentTriangle.verts[0]+yy << currentTriangle.verts[1]+yy << currentTriangle.verts[2]+yy;
	}
}

static PxU32 createInvisibleWalls(const CCTParams& params, const Gu::Triangle& currentTriangle, TriArray& world_triangles, TriArray* worldEdgeNormalsArray, IntArray& edgeFlagsArray, IntArray& triIndicesArray)
{
	const PxF32 wallHeight = params.mInvisibleWallHeight;
	if(wallHeight==0.0f)
		return 0;

	PxU32 nbNewTris = 0;	// Number of newly created tris

	const PxCCTUpAxis::Enum upDirection = params.mUpDirection;

	PxVec3 normal;
	currentTriangle.normal(normal);
	if(testSlope(normal, upDirection, params.mSlopeLimit))
	{
		PxVec3 v0p = currentTriangle.verts[0];	v0p[upDirection] += wallHeight;
		PxVec3 v1p = currentTriangle.verts[1];	v1p[upDirection] += wallHeight;
		PxVec3 v2p = currentTriangle.verts[2];	v2p[upDirection] += wallHeight;

		// Extrude edge 0-1
		PxVec3 faceNormal01;
		{
			// 0-1-0p
			const Gu::Triangle tri0_1_0p(currentTriangle.verts[0], currentTriangle.verts[1], v0p);
			world_triangles.pushBack(tri0_1_0p);

			// 0p-1-1p
			const Gu::Triangle tri0p_1_1p(v0p, currentTriangle.verts[1], v1p);
			world_triangles.pushBack(tri0p_1_1p);

			tri0p_1_1p.normal(faceNormal01);
		}

		// Extrude edge 1-2
		PxVec3 faceNormal12;
		{
			// 1p-1-2p
			const Gu::Triangle tri1p_1_2p(v1p, currentTriangle.verts[1], v2p);
			world_triangles.pushBack(tri1p_1_2p);

			// 2p-1-2
			const Gu::Triangle tri2p_1_2(v2p, currentTriangle.verts[1], currentTriangle.verts[2]);
			world_triangles.pushBack(tri2p_1_2);

			tri2p_1_2.normal(faceNormal12);
		}

		// Extrude edge 2-0
		PxVec3 faceNormal20;
		{
			// 0p-2-0
			const Gu::Triangle tri0p_2_0(v0p, currentTriangle.verts[2], currentTriangle.verts[0]);
			world_triangles.pushBack(tri0p_2_0);

			// 0p-2p-2
			const Gu::Triangle tri0p_2p_2(v0p, v2p, currentTriangle.verts[2]);
			world_triangles.pushBack(tri0p_2p_2);

			tri0p_2p_2.normal(faceNormal20);
		}

		if(worldEdgeNormalsArray)
		{
			PxVec3 v0_2d = currentTriangle.verts[0];	v0_2d[upDirection] = 0.0f;
			PxVec3 v1_2d = currentTriangle.verts[1];	v1_2d[upDirection] = 0.0f;
			PxVec3 v2_2d = currentTriangle.verts[2];	v2_2d[upDirection] = 0.0f;

			const PxVec3 center = (v0_2d + v1_2d + v2_2d)/3.0f;

			PxVec3 radial0 = v0_2d - center;	radial0.normalize();
			PxVec3 radial1 = v1_2d - center;	radial1.normalize();
			PxVec3 radial2 = v2_2d - center;	radial2.normalize();

			// The edge-triangle contains normals for edges 0-1, 1-2, 2-0

			// 0-1-0p
				// 0-1
				// 1-0p
				// 0p-0
			worldEdgeNormalsArray->pushBack(Gu::Triangle(faceNormal01, faceNormal01, radial0));

			// 0p-1-1p
				// 0p-1
				// 1-1p
				// 1p-0p
			worldEdgeNormalsArray->pushBack(Gu::Triangle(faceNormal01, radial1, faceNormal01));

			// 1p-1-2p
				// 1p-1
				// 1-2p
				// 2p-1p
			worldEdgeNormalsArray->pushBack(Gu::Triangle(radial1, faceNormal12, faceNormal12));

			// 2p-1-2
				// 2p-1
				// 1-2
				// 2-2p
			worldEdgeNormalsArray->pushBack(Gu::Triangle(faceNormal12, faceNormal12, radial2));

			// 0p-2-0
				// 0p-2
				// 2-0
				// 0-0p
			worldEdgeNormalsArray->pushBack(Gu::Triangle(faceNormal20, faceNormal20, radial0));

			// 0p-2p-2
				// 0p-2p
				// 2p-2
				// 2-0p
			worldEdgeNormalsArray->pushBack(Gu::Triangle(faceNormal20, radial2, faceNormal20));
		}

		const PxU32 edgeFlags = Gu::TriangleCollisionFlag::eACTIVE_EDGE01 | Gu::TriangleCollisionFlag::eACTIVE_EDGE12 | Gu::TriangleCollisionFlag::eACTIVE_EDGE20;
		const PxU32 triIndex = PX_INVALID_U32;
		for(PxU32 i=0;i<6;i++)
		{
			edgeFlagsArray.pushBack(edgeFlags);
			triIndicesArray.pushBack(triIndex);
		}

		nbNewTris += 6;
	}
	return nbNewTris;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void outputMeshToStream(
	PxShape* meshShape,
	IntArray& geomStream,
	TriArray& world_triangles,
	TriArray* world_edge_normals,
	IntArray& edgeFlagsArray,
	IntArray& triIndicesArray,
	const PxExtendedVec3& origin,
	const PxBounds3& tmpBounds,
	const CCTParams& params
	)
{
	PX_ASSERT(meshShape->getGeometryType() == PxGeometryType::eTRIANGLEMESH);
	// Do AABB-mesh query

	PxTriangleMeshGeometry triGeom;
	meshShape->getTriangleMeshGeometry(triGeom);
	PxTransform meshPose = PxShapeExt::getGlobalPose(*meshShape);

	PxBoxGeometry boxGeom(tmpBounds.getExtents());
	PxTransform boxPose;
	boxPose.p = tmpBounds.getCenter();
	boxPose.q = PxQuat::createIdentity();

	// Collide AABB against current mesh
	PxFindOverlapTriangleMeshUtil overlapUtil;
	const PxU32 nbTouchedTris = overlapUtil.findOverlap(boxGeom, boxPose, triGeom, meshPose);

	const PxVec3 offset(float(-origin.x), float(-origin.y), float(-origin.z));

	TouchedMesh* touchedMesh			= (TouchedMesh*)reserve(geomStream, sizeof(TouchedMesh)/sizeof(PxU32));
	touchedMesh->mType					= TouchedGeomType::eMESH;
	touchedMesh->mUserData				= meshShape;
	touchedMesh->mOffset				= origin;
	touchedMesh->mNbTris				= nbTouchedTris;
	touchedMesh->mIndexWorldTriangles	= world_triangles.size();
	touchedMesh->mIndexWorldEdgeNormals	= world_edge_normals ? world_edge_normals->size() : 0;
	touchedMesh->mIndexEdgeFlags		= edgeFlagsArray.size();

	const PxU32* PX_RESTRICT indices = overlapUtil.getResults();

	if(params.mSlopeLimit!=0.0f)
	{
		// Reserve memory for incoming triangles
//		Gu::Triangle* TouchedTriangles = reserve(world_triangles, nbTouchedTris);
//		Gu::Triangle* EdgeTriangles = world_edge_normals ? reserve(*world_edge_normals, nbTouchedTris) : NULL;

		// Loop through touched triangles
		PxU32 nbCreatedTris = 0;
		for(PxU32 i=0; i < nbTouchedTris; i++)
		{
			const PxU32 triangleIndex = indices[i];

			// Compute triangle in world space, add to array
			Gu::Triangle currentTriangle;
			Gu::Triangle edgeTri;
			PxU32 edgeFlags;
			Gu::MeshQuery::getTriangle(triGeom, meshPose, triangleIndex, currentTriangle, &edgeTri, &edgeFlags);
			currentTriangle.verts[0] += offset;
			currentTriangle.verts[1] += offset;
			currentTriangle.verts[2] += offset;

			const PxU32 nbNewTris = createInvisibleWalls(params, currentTriangle, world_triangles, world_edge_normals, edgeFlagsArray, triIndicesArray);
			nbCreatedTris += nbNewTris;
			if(!nbNewTris)
			{
				world_triangles.pushBack(currentTriangle);

				if(world_edge_normals)
					world_edge_normals->pushBack(edgeTri);

				edgeFlagsArray.pushBack(edgeFlags);
				triIndicesArray.pushBack(triangleIndex);
				nbCreatedTris++;
			}
		}
		touchedMesh->mNbTris = nbCreatedTris;
	}
	else
	{
		// Reserve memory for incoming triangles
		Gu::Triangle* TouchedTriangles = reserve(world_triangles, nbTouchedTris);
		Gu::Triangle* EdgeTriangles = world_edge_normals ? reserve(*world_edge_normals, nbTouchedTris) : NULL;

		// Loop through touched triangles
		for(PxU32 i=0; i < nbTouchedTris; i++)
		{
			const PxU32 triangleIndex = indices[i];

			// Compute triangle in world space, add to array
			Gu::Triangle& currentTriangle = *TouchedTriangles++;
			Gu::Triangle edgeTri;
			PxU32 edgeFlags;
			Gu::MeshQuery::getTriangle(triGeom, meshPose, triangleIndex, currentTriangle, &edgeTri, &edgeFlags);
			currentTriangle.verts[0] += offset;
			currentTriangle.verts[1] += offset;
			currentTriangle.verts[2] += offset;

			if(EdgeTriangles)
				*EdgeTriangles++ = edgeTri;

			edgeFlagsArray.pushBack(edgeFlags);
			triIndicesArray.pushBack(triangleIndex);
		}
	}

	if(gVisualizeTouchedTris)
		visualizeTouchedTriangles(touchedMesh->mNbTris, touchedMesh->mIndexWorldTriangles, &world_triangles[0], meshShape->getActor().getScene(), offset, params.mUpDirection);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void outputHeightFieldToStream(
	PxShape* hfShape,
	IntArray& geomStream,
	TriArray& world_triangles,
	TriArray* world_edge_normals,
	IntArray& edgeFlagsArray,
	IntArray& triIndicesArray,
	const PxExtendedVec3& origin,
	const PxBounds3& tmpBounds,
	const CCTParams& params
	)
{
	PX_ASSERT(hfShape->getGeometryType() == PxGeometryType::eHEIGHTFIELD);
	// Do AABB-mesh query

	PxHeightFieldGeometry hfGeom;
	hfShape->getHeightFieldGeometry(hfGeom);
	PxTransform heightfieldPose = PxShapeExt::getGlobalPose(*hfShape);

	PxBoxGeometry boxGeom(tmpBounds.getExtents());
	PxTransform boxPose;
	boxPose.p = tmpBounds.getCenter();
	boxPose.q = PxQuat::createIdentity();

	// Collide AABB against current heightfield
	PxFindOverlapTriangleMeshUtil overlapUtil;
	const PxU32 nbTouchedTris = overlapUtil.findOverlap(boxGeom, boxPose, hfGeom, heightfieldPose);

	const PxVec3 offset(float(-origin.x), float(-origin.y), float(-origin.z));

	TouchedMesh* touchedMesh			= (TouchedMesh*)reserve(geomStream, sizeof(TouchedMesh)/sizeof(PxU32));
	touchedMesh->mType					= TouchedGeomType::eMESH; // ptchernev: seems to work
	touchedMesh->mUserData				= hfShape;
	touchedMesh->mOffset				= origin;
	touchedMesh->mNbTris				= nbTouchedTris;
	touchedMesh->mIndexWorldTriangles	= world_triangles.size();
	touchedMesh->mIndexWorldEdgeNormals	= world_edge_normals ? world_edge_normals->size() : 0;
	touchedMesh->mIndexEdgeFlags		= edgeFlagsArray.size();

	const PxU32* PX_RESTRICT indices = overlapUtil.getResults();

	if(params.mSlopeLimit!=0.0f)
	{
		// Reserve memory for incoming triangles
//		Gu::Triangle* TouchedTriangles = reserve(world_triangles, nbTouchedTris);
//		Gu::Triangle* EdgeTriangles = world_edge_normals ? reserve(*world_edge_normals, nbTouchedTris) : NULL;

		// Loop through touched triangles
		PxU32 nbCreatedTris = 0;
		for(PxU32 i=0; i < nbTouchedTris; i++)
		{
			const PxU32 triangleIndex = indices[i];

			// Compute triangle in world space, add to array
			Gu::Triangle currentTriangle;
			Gu::Triangle edgeTri;
			PxU32 edgeFlags;
			Gu::MeshQuery::getTriangle(hfGeom, heightfieldPose, triangleIndex, currentTriangle, &edgeTri, &edgeFlags);
			currentTriangle.verts[0] += offset;
			currentTriangle.verts[1] += offset;
			currentTriangle.verts[2] += offset;

			const PxU32 nbNewTris = createInvisibleWalls(params, currentTriangle, world_triangles, world_edge_normals, edgeFlagsArray, triIndicesArray);
			nbCreatedTris += nbNewTris;
			if(!nbNewTris)
			{
				world_triangles.pushBack(currentTriangle);

				if(world_edge_normals)
					world_edge_normals->pushBack(edgeTri);

				edgeFlagsArray.pushBack(edgeFlags);
				triIndicesArray.pushBack(triangleIndex);
				nbCreatedTris++;
			}
		}
		touchedMesh->mNbTris = nbCreatedTris;
	}
	else
	{
		// Reserve memory for incoming triangles
		Gu::Triangle* TouchedTriangles = reserve(world_triangles, nbTouchedTris);
		Gu::Triangle* EdgeTriangles = world_edge_normals ? reserve(*world_edge_normals, nbTouchedTris) : NULL;

		// Loop through touched triangles
		for(PxU32 i=0; i < nbTouchedTris; i++)
		{
			const PxU32 triangleIndex = indices[i];

			// Compute triangle in world space, add to array
			Gu::Triangle& currentTriangle = *TouchedTriangles++;
			Gu::Triangle edgeTri;
			PxU32 edgeFlags;
			Gu::MeshQuery::getTriangle(hfGeom, heightfieldPose, triangleIndex, currentTriangle, &edgeTri, &edgeFlags);
			currentTriangle.verts[0] += offset;
			currentTriangle.verts[1] += offset;
			currentTriangle.verts[2] += offset;

			if(EdgeTriangles)
				*EdgeTriangles++ = edgeTri;

			edgeFlagsArray.pushBack(edgeFlags);
			triIndicesArray.pushBack(triangleIndex);
		}
	}

	if(gVisualizeTouchedTris)
		visualizeTouchedTriangles(touchedMesh->mNbTris, touchedMesh->mIndexWorldTriangles, &world_triangles[0], hfShape->getActor().getScene(), offset, params.mUpDirection);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void outputConvexToStream(
	PxShape* convexShape,
	IntArray& geomStream,
	TriArray& world_triangles,
	TriArray* world_edge_normals,
	IntArray& edgeFlagsArray,
	IntArray& triIndicesArray,
	const PxExtendedVec3& origin,
	const PxBounds3& tmpBounds
	)
{
	PX_ASSERT(convexShape->getGeometryType() == PxGeometryType::eCONVEXMESH);
	PxConvexMeshGeometry cg;
	convexShape->getConvexMeshGeometry(cg);
	PX_ASSERT(cg.convexMesh);

	// Do AABB-mesh query

	PxU32* TF;

	// Collide AABB against current mesh
	// The overlap function doesn't exist for convexes so let's just dump all tris
	PxConvexMesh& cm = *cg.convexMesh;

	// PT: convex triangles are not exposed anymore so we need to access convex polygons & triangulate them

	// PT: TODO: this is copied from "DrawObjects", move this to a shared place. Actually a helper directly in PxConvexMesh would be useful.
	PxU32 Nb = 0;
	{
		const PxU32 nbPolys = cm.getNbPolygons();
		const PxU8* polygons = cm.getIndexBuffer();

		for(PxU32 i=0;i<nbPolys;i++)
		{
			PxHullPolygon data;
			cm.getPolygonData(i, data);
			Nb += data.mNbVerts - 2;
		}

		// PT: revisit this code. We don't use the polygon offset?
		TF = (PxU32*)PxAlloca(sizeof(PxU32)*Nb*3);
		PxU32* t = TF;
		for(PxU32 i=0;i<nbPolys;i++)
		{
			PxHullPolygon data;
			cm.getPolygonData(i, data);

			const PxU32 nbV = data.mNbVerts;

			const PxU32 nbTris = nbV - 2;
			const PxU8 vref0 = *polygons;
			for(PxU32 j=0;j<nbTris;j++)
			{
				const PxU32 vref1 = polygons[(j+1)%nbV];
				const PxU32 vref2 = polygons[(j+2)%nbV];
				*t++ = vref0;
				*t++ = vref1;
				*t++ = vref2;
			}
			polygons += nbV;
		}
	}

	const PxVec3* verts = (const PxVec3*)cm.getVertices();

	PxTransform absPose = PxShapeExt::getGlobalPose(*convexShape);

	PxVec3 tmp = absPose.p;	// LOSS OF ACCURACY
	PxVec3 MeshOffset;
	MeshOffset.x = float(tmp.x - origin.x);
	MeshOffset.y = float(tmp.y - origin.y);
	MeshOffset.z = float(tmp.z - origin.z);

	TouchedMesh* touchedMesh			= (TouchedMesh*)reserve(geomStream, sizeof(TouchedMesh)/sizeof(PxU32));
	touchedMesh->mType					= TouchedGeomType::eMESH;
	touchedMesh->mUserData				= convexShape;
	touchedMesh->mOffset				= origin;
	touchedMesh->mNbTris				= Nb;
	touchedMesh->mIndexWorldTriangles	= world_triangles.size();
	touchedMesh->mIndexWorldEdgeNormals	= world_edge_normals ? world_edge_normals->size() : 0;
	touchedMesh->mIndexEdgeFlags		= edgeFlagsArray.size();

	// Reserve memory for incoming triangles
	Gu::Triangle* TouchedTriangles = reserve(world_triangles, Nb);
	Gu::Triangle* EdgeTriangles = world_edge_normals ? reserve(*world_edge_normals, Nb) : NULL;

	// Loop through touched triangles
	while(Nb--)
	{
		// Compute triangle in world space, add to array
		Gu::Triangle& CurrentTriangle = *TouchedTriangles++;

		const PxU32 vref0 = *TF++;
		const PxU32 vref1 = *TF++;
		const PxU32 vref2 = *TF++;

		PxVec3 v0 = verts[vref0];
		PxVec3 v1 = verts[vref1];
		PxVec3 v2 = verts[vref2];
		v0 = absPose.q.rotate(v0);
		v1 = absPose.q.rotate(v1);
		v2 = absPose.q.rotate(v2);

		CurrentTriangle.verts[0] = v0;
		CurrentTriangle.verts[1] = v1;
		CurrentTriangle.verts[2] = v2;

		CurrentTriangle.verts[0] += MeshOffset;
		CurrentTriangle.verts[1] += MeshOffset;
		CurrentTriangle.verts[2] += MeshOffset;

		if(EdgeTriangles)
			*EdgeTriangles++ = Gu::Triangle(PxVec3(0), PxVec3(0), PxVec3(0));

		// #### hmmm
		const PxU32 edgeFlags = 7;
		edgeFlagsArray.pushBack(edgeFlags);
		triIndicesArray.pushBack(PX_INVALID_U32);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Cct::findTouchedGeometry(
	void* user_data,
	const PxExtendedBounds3& worldBounds,		// ### we should also accept other volumes

	TriArray& world_triangles,
	TriArray* world_edge_normals,
	IntArray& edgeFlagsArray,
	IntArray& triIndicesArray,
	IntArray& geomStream,
	bool static_shapes, bool dynamic_shapes, const PxFilterData* filterData, PxSceneQueryFilterCallback* filterCallback,
	const CCTParams& params)
{
	PX_ASSERT(user_data);
	PxScene* scene = (PxScene*)user_data;

	PxExtendedVec3 Origin;	// Will be TouchedGeom::mOffset
	getCenter(worldBounds, Origin);

	// Find touched *boxes* i.e. touched objects' AABBs in the world
	// We collide against dynamic shapes too, to get back dynamic boxes/etc
	// TODO: add active groups in interface!
	PxSceneQueryFilterFlags Flags;
	if(static_shapes)	Flags |= PxSceneQueryFilterFlag::eSTATIC;
	if(dynamic_shapes)	Flags |= PxSceneQueryFilterFlag::eDYNAMIC;
	if(filterCallback)	Flags |= PxSceneQueryFilterFlag::ePREFILTER;

	// ### this one is dangerous
	PxBounds3 tmpBounds;	// LOSS OF ACCURACY
	tmpBounds.minimum.x = (float)worldBounds.minimum.x;
	tmpBounds.minimum.y = (float)worldBounds.minimum.y;
	tmpBounds.minimum.z = (float)worldBounds.minimum.z;
	tmpBounds.maximum.x = (float)worldBounds.maximum.x;
	tmpBounds.maximum.y = (float)worldBounds.maximum.y;
	tmpBounds.maximum.z = (float)worldBounds.maximum.z;

	// PT: unfortunate conversion forced by the PxGeometry API
	PxVec3 center = tmpBounds.getCenter(), extents = tmpBounds.getExtents();

	PxShape* hits[100];
	PxU32 size = 100;

	const PxSceneQueryFilterData sceneQueryFilterData = filterData ? PxSceneQueryFilterData(*filterData, Flags) : PxSceneQueryFilterData(Flags);

	PxI32 numberHits = scene->overlapMultiple(PxBoxGeometry(extents), PxTransform(center), hits, size, sceneQueryFilterData, filterCallback);
	
	for(PxI32 i = 0; i < numberHits; i++)
	{
		PxShape* shape = hits[i];
		if(shape == NULL)
			continue;

		// Filtering

		// Discard all CCT shapes, i.e. kinematic actors we created ourselves. We don't need to collide with them since they're surrounded
		// by the real CCT volume - and collisions with those are handled elsewhere. We use the userData field for filtering because that's
		// really our only valid option (filtering groups are already used by clients and we don't have control over them, clients might
		// create other kinematic actors that we may want to keep here, etc, etc)
		if(size_t(shape->userData)==PX_MAKEFOURCC('C','C','T','S'))
			continue;

		// Ubi (EA) : Discarding Triggers :
		if ( shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE )
			continue;

		// PT: here you might want to disable kinematic objects.

		// Output shape to stream
		const PxGeometryType::Enum type = shape->getGeometryType();	// ### VIRTUAL!
		if(type==PxGeometryType::eSPHERE)				outputSphereToStream(shape, geomStream, Origin);
		else	if(type==PxGeometryType::eCAPSULE)		outputCapsuleToStream(shape, geomStream, Origin);
		else	if(type==PxGeometryType::eBOX)			outputBoxToStream(shape, geomStream, Origin);
		else	if(type==PxGeometryType::eTRIANGLEMESH)	outputMeshToStream(shape, geomStream, world_triangles, world_edge_normals, edgeFlagsArray, triIndicesArray, Origin, tmpBounds, params);
		else	if(type==PxGeometryType::eHEIGHTFIELD)	outputHeightFieldToStream(shape, geomStream, world_triangles, world_edge_normals, edgeFlagsArray, triIndicesArray, Origin, tmpBounds, params);
		else	if(type==PxGeometryType::eCONVEXMESH)	outputConvexToStream(shape, geomStream, world_triangles, world_edge_normals, edgeFlagsArray, triIndicesArray, Origin, tmpBounds);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #### hmmm, in the down case, isn't reported length too big ? It contains our artificial up component,
// that might confuse the user

void Cct::shapeHitCallback(void* user_data, const SweptContact& contact, const PxVec3& dir, float length)
{
	Controller* controller = (Controller*)user_data;

	if(controller->mCallback)
	{
		PxControllerShapeHit hit;
		hit.shape			= (PxShape*)contact.mGeom->mUserData;
		hit.worldPos.x		= contact.mWorldPos.x;
		hit.worldPos.y		= contact.mWorldPos.y;
		hit.worldPos.z		= contact.mWorldPos.z;
		hit.worldNormal.x	= contact.mWorldNormal.x;
		hit.worldNormal.y	= contact.mWorldNormal.y;
		hit.worldNormal.z	= contact.mWorldNormal.z;
		hit.dir.x			= dir.x;
		hit.dir.y			= dir.y;
		hit.dir.z			= dir.z;
		hit.length			= length;
		hit.controller		= controller->getNxController();
		hit.triangleIndex	= contact.mTriangleIndex;

		controller->mCallback->onShapeHit(hit);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Cct::userHitCallback(void* user_data, const SweptContact& contact, const PxVec3& dir, float length)
{
	Controller* controller = (Controller*)user_data;

	if(controller->mCallback)
	{
		Controller* other = (Controller*)contact.mGeom->mUserData;

		PxControllersHit hit;
		hit.controller	= controller->getNxController();
		hit.other		= other->getNxController();

		controller->mCallback->onControllerHit(hit);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
