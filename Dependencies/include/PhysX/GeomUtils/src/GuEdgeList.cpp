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


#include "PsIntrinsics.h"
#include "GuEdgeList.h"
#include "GuTriangle.h"
#include "GuPlane.h"
#include "PsMathUtils.h"
#include "./Ice/IceSerialize.h"
#include "./Ice/IceRevisitedRadix2.h"

using namespace physx;
using namespace Ice;

// PT: this file should be moved to cooking lib

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	An edge list. Use this class to compute an edge list for a mesh in an efficient way.
 *
 *	An edge of a triangular mesh is a boundary edge if it belongs to exactly one triangle, an internal edge if is shared by exactly two triangles,
 *	and a singular edge if it is shared by three or more triangles (Note that boundary edges and internal edges are also called regular edges.)
 *	A vertex of a triangular mesh is a regular vertex if the set of vertices of all the triangles of the triangular mesh that contain the vertex,
 *	excluding the given vertex, can be reordered to define a single path. If a vertex is not a regular vertex, then it is a singular vertex.
 *	A triangular mesh has boundary if it has one or more boundary edges. A triangular mesh is a manifold if it has no singular vertices and no
 *	singular edges. If a triangular mesh is not a manifold, it is a non-manifold. 
 *
 *	2.0:
 *	- standard version
 *	2.1:
 *	- EdgesRefs replaced with IndexedTriangle to provide an intuitive edge topology.
 *	2.2:
 *	- active edges refactored into this class. "Active" edges are convex and boundary edges.
 *	2.3:
 *	- active vertex flags added.
 *	2.4:
 *	- active user bits added.
 *
 *	\class		EdgeList
 *	\author		Pierre Terdiman
 *	\version	2.4
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Gu::EdgeList::EdgeList()
{
	mData.mNbEdges = 0;
	mData.mEdgeFaces = NULL;
	mData.mEdges = NULL;
	mData.mEdgeToTriangles = NULL;
	mData.mFacesByEdges = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Gu::EdgeList::~EdgeList()
{
	PX_FREE_AND_RESET(mData.mFacesByEdges);
	PX_FREE_AND_RESET(mData.mEdgeToTriangles);
	PX_FREE_AND_RESET(mData.mEdges);
	PX_DELETE_POD(mData.mEdgeFaces);
}

bool Gu::EdgeList::Load(const PxStream& stream)
{
	// Import header
	PxU32 Version;
	bool Mismatch;
	if(!ReadHeader('E', 'D', 'G', 'E', Version, Mismatch, stream))
		return false;

	// Import edges
	mData.mNbEdges = ReadDword(Mismatch, stream);
	//mEdges = ICE_NEW_MEM(Edge[mNbEdges],Edge);
	mData.mEdges = (Gu::EdgeData*)PX_ALLOC(sizeof(Gu::EdgeData)*mData.mNbEdges);
	stream.readBuffer(mData.mEdges, sizeof(Gu::EdgeData)*mData.mNbEdges);

	mData.mNbFaces = ReadDword(Mismatch, stream);
	//mEdgeFaces	= ICE_NEW_MEM(EdgeTriangle[mNbFaces],EdgeTriangle);
	mData.mEdgeFaces = (Gu::EdgeTriangleData*)PX_ALLOC(sizeof(Gu::EdgeTriangleData)*mData.mNbFaces);
	stream.readBuffer(mData.mEdgeFaces, sizeof(Gu::EdgeTriangleData)*mData.mNbFaces);

	//mEdgeToTriangles = ICE_NEW_MEM(EdgeDesc[mNbEdges],EdgeDesc);
	mData.mEdgeToTriangles = (Gu::EdgeDescData*)PX_ALLOC(sizeof(Gu::EdgeDescData)*mData.mNbEdges);
	stream.readBuffer(mData.mEdgeToTriangles, sizeof(Gu::EdgeDescData)*mData.mNbEdges);
	

	PxU32 LastOffset = mData.mEdgeToTriangles[mData.mNbEdges-1].Offset + mData.mEdgeToTriangles[mData.mNbEdges-1].Count;
	mData.mFacesByEdges = (PxU32*)PX_ALLOC(sizeof(PxU32)*LastOffset);
	stream.readBuffer(mData.mFacesByEdges, sizeof(PxU32)*LastOffset);

	return true;
}

//#ifdef PX_COOKING

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Initializes the edge-list.
 *	\param		create	[in] edge-list creation structure
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Gu::EdgeListBuilder::Init(const EDGELISTCREATE& create)
{
	bool FacesToEdges = create.Verts ? true : create.FacesToEdges;
	bool EdgesToFaces = create.Verts ? true : create.EdgesToFaces;

	// "FacesToEdges" maps each face to three edges.
	if(FacesToEdges && !CreateFacesToEdges(create.NbFaces, create.DFaces, create.WFaces))	return false;

	// "EdgesToFaces" maps each edge to the set of faces sharing this edge
	if(EdgesToFaces && !CreateEdgesToFaces(create.NbFaces, create.DFaces, create.WFaces))	return false;

	// Create active edges
	if(create.Verts && !ComputeActiveEdges(create.NbFaces, create.DFaces, create.WFaces, create.Verts, create.Epsilon))	return false;

	// Get rid of useless data
	if(!create.FacesToEdges)	
	{ 
		PX_FREE_AND_RESET(mData.mEdgeFaces); 
	}
	if(!create.EdgesToFaces)	
	{ 
		PX_FREE_AND_RESET(mData.mEdgeToTriangles); 
		PX_FREE_AND_RESET(mData.mFacesByEdges);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes FacesToEdges.
 *	After the call:
 *	- mNbEdges		is updated with the number of non-redundant edges
 *	- mEdges		is a list of mNbEdges edges (one edge is 2 vertex-references)
 *	- mEdgesRef		is a list of nbfaces structures with 3 indexes in mEdges for each face
 *
 *	\param		nb_faces	[in] a number of triangles
 *	\param		dfaces		[in] list of triangles with PxU32 vertex references (or NULL)
 *	\param		wfaces		[in] list of triangles with PxU16 vertex references (or NULL)
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Gu::EdgeListBuilder::CreateFacesToEdges(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces)
{
	// Checkings
	if(!nb_faces || (!dfaces && !wfaces))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "EdgeList::CreateFacesToEdges: NULL parameter!");
		return false;
	}

	if(mData.mEdgeFaces)	return true;	// Already computed!

	// 1) Get some bytes: I need one EdgesRefs for each face, and some temp buffers
	mData.mEdgeFaces			= PX_NEW(Gu::EdgeTriangleData)[nb_faces];	// Link faces to edges
	PxU32*			VRefs0		= PX_NEW_TEMP(PxU32)[nb_faces*3];			// Temp storage
	PxU32*			VRefs1		= PX_NEW_TEMP(PxU32)[nb_faces*3];			// Temp storage
	Gu::EdgeData*	Buffer		= PX_NEW_TEMP(Gu::EdgeData)[nb_faces*3];	// Temp storage

	// 2) Create a full redundant list of 3 edges / face.
	for(PxU32 i=0;i<nb_faces;i++)
	{
		// Get right vertex-references
		const PxU32 Ref0 = dfaces ? dfaces[i*3+0] : wfaces ? wfaces[i*3+0] : 0;
		const PxU32 Ref1 = dfaces ? dfaces[i*3+1] : wfaces ? wfaces[i*3+1] : 1;
		const PxU32 Ref2 = dfaces ? dfaces[i*3+2] : wfaces ? wfaces[i*3+2] : 2;

		// Pre-Sort vertex-references and put them in the lists
		if(Ref0<Ref1)	{ VRefs0[i*3+0] = Ref0; VRefs1[i*3+0] = Ref1; }		// Edge 0-1 maps (i%3)
		else			{ VRefs0[i*3+0] = Ref1; VRefs1[i*3+0] = Ref0; }		// Edge 0-1 maps (i%3)

		if(Ref1<Ref2)	{ VRefs0[i*3+1] = Ref1; VRefs1[i*3+1] = Ref2; }		// Edge 1-2 maps (i%3)+1
		else			{ VRefs0[i*3+1] = Ref2; VRefs1[i*3+1] = Ref1; }		// Edge 1-2 maps (i%3)+1

		if(Ref2<Ref0)	{ VRefs0[i*3+2] = Ref2; VRefs1[i*3+2] = Ref0; }		// Edge 2-0 maps (i%3)+2
		else			{ VRefs0[i*3+2] = Ref0; VRefs1[i*3+2] = Ref2; }		// Edge 2-0 maps (i%3)+2
	}

	// 3) Sort the list according to both keys (VRefs0 and VRefs1)
	RadixSortBuffered Sorter;
	const PxU32* Sorted = Sorter.Sort(VRefs1, nb_faces*3).Sort(VRefs0, nb_faces*3).GetRanks();

	// 4) Loop through all possible edges
	// - clean edges list by removing redundant edges
	// - create EdgesRef list
	mData.mNbEdges = 0;												// #non-redundant edges
	mData.mNbFaces = nb_faces;
	PxU32 PreviousRef0 = PX_INVALID_U32;
	PxU32 PreviousRef1 = PX_INVALID_U32;
	for(PxU32 i=0;i<nb_faces*3;i++)
	{
		PxU32 Face = Sorted[i];								// Between 0 and nbfaces*3
		PxU32 ID = Face % 3;									// Get edge ID back.
		PxU32 SortedRef0 = VRefs0[Face];						// (SortedRef0, SortedRef1) is the sorted edge
		PxU32 SortedRef1 = VRefs1[Face];

		if(SortedRef0!=PreviousRef0 || SortedRef1!=PreviousRef1)
		{
			// New edge found! => stored in temp buffer
			Buffer[mData.mNbEdges].Ref0	= SortedRef0;
			Buffer[mData.mNbEdges].Ref1	= SortedRef1;
			mData.mNbEdges++;
		}
		PreviousRef0 = SortedRef0;
		PreviousRef1 = SortedRef1;

		// Create mEdgesRef on the fly
		mData.mEdgeFaces[Face/3].mLink[ID] = mData.mNbEdges-1;
	}

	// 5) Here, mNbEdges==#non redundant edges
	mData.mEdges = (Gu::EdgeData*)PX_ALLOC(sizeof(Gu::EdgeData)*mData.mNbEdges);

	// Create real edges-list.
	Ps::memCopy(mData.mEdges, Buffer, mData.mNbEdges*sizeof(Gu::EdgeData));

	// 6) Free ram and exit
	PX_DELETE_POD(Buffer);
	PX_DELETE_POD(VRefs1);
	PX_DELETE_POD(VRefs0);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes EdgesToFaces.
 *	After the call:
 *	- mEdgeToTriangles		is created
 *	- mFacesByEdges			is created
 *
 *	\param		nb_faces	[in] a number of triangles
 *	\param		dfaces		[in] list of triangles with PxU32 vertex references (or NULL)
 *	\param		wfaces		[in] list of triangles with PxU16 vertex references (or NULL)
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Gu::EdgeListBuilder::CreateEdgesToFaces(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces)
{
	// 1) I need FacesToEdges !
	if(!CreateFacesToEdges(nb_faces, dfaces, wfaces)) return false;

	// 2) Get some bytes: one Pair structure / edge
	mData.mEdgeToTriangles = (Gu::EdgeDescData*)PX_ALLOC(sizeof(Gu::EdgeDescData)*mData.mNbEdges);
	Ps::memZero(mData.mEdgeToTriangles, sizeof(Gu::EdgeDescData)*mData.mNbEdges);

	// 3) Create Counters, ie compute the #faces sharing each edge
	for(PxU32 i=0;i<nb_faces;i++)
	{
		mData.mEdgeToTriangles[mData.mEdgeFaces[i].mLink[0]].Count++;
		mData.mEdgeToTriangles[mData.mEdgeFaces[i].mLink[1]].Count++;
		mData.mEdgeToTriangles[mData.mEdgeFaces[i].mLink[2]].Count++;
	}

	// 3) Create Radix-like Offsets
	mData.mEdgeToTriangles[0].Offset=0;
	for(PxU32 i=1;i<mData.mNbEdges;i++)	
		mData.mEdgeToTriangles[i].Offset = mData.mEdgeToTriangles[i-1].Offset + mData.mEdgeToTriangles[i-1].Count;

	PxU32 LastOffset = mData.mEdgeToTriangles[mData.mNbEdges-1].Offset + mData.mEdgeToTriangles[mData.mNbEdges-1].Count;

	// 4) Get some bytes for mFacesByEdges. LastOffset is the number of indices needed.
	mData.mFacesByEdges = (PxU32*)PX_ALLOC(sizeof(PxU32)*LastOffset);

	// 5) Create mFacesByEdges
	for(PxU32 i=0;i<nb_faces;i++)
	{
		mData.mFacesByEdges[mData.mEdgeToTriangles[mData.mEdgeFaces[i].mLink[0]].Offset++] = i;
		mData.mFacesByEdges[mData.mEdgeToTriangles[mData.mEdgeFaces[i].mLink[1]].Offset++] = i;
		mData.mFacesByEdges[mData.mEdgeToTriangles[mData.mEdgeFaces[i].mLink[2]].Offset++] = i;
	}

	// 6) Recompute offsets wasted by 5)
	mData.mEdgeToTriangles[0].Offset=0;
	for(PxU32 i=1;i<mData.mNbEdges;i++)
	{
		mData.mEdgeToTriangles[i].Offset = mData.mEdgeToTriangles[i-1].Offset + mData.mEdgeToTriangles[i-1].Count;
	}

	return true;
}

PX_INLINE PxU32 OppositeVertex(PxU32 r0, PxU32 r1, PxU32 r2, PxU32 vref0, PxU32 vref1)
{
	/*fix TTP 8059,	the followed codes will make a wrong result when compile with \O2  on XBOX360 
	for example,  input 1,0,3,1,3  return 0 not 1.   maybe xbox complier bug.
			if(r0==vref0 && r1==vref1)	return r2;
	else	if(r0==vref1 && r1==vref0)	return r2;
	else	if(r0==vref0 && r2==vref1)	return r1;
	else	if(r0==vref1 && r2==vref0)	return r1;
	else	if(r1==vref0 && r2==vref1)	return r0;
	else	if(r1==vref1 && r2==vref0)	return r0;
	return PX_INVALID_U32;*/

	if(vref0==r0)
	{
	  if (vref1==r1) return r2;
	  else if(vref1==r2) return r1;
	}
	else if(vref0==r1)
	{
	  if (vref1==r0) return r2;
	  else if(vref1==r2) return r0;
	}
	else if(vref0==r2)
	{
	  if (vref1==r1) return r0;
	  else if(vref1==r0) return r1;
	}
	return PX_INVALID_U32;
}

bool Gu::EdgeListBuilder::ComputeActiveEdges(PxU32 nb_faces, const PxU32* dfaces, const PxU16* wfaces, const PxVec3* verts, float epsilon)
{
	// Checkings
	if(!verts || (!dfaces && !wfaces))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "EdgeList::ComputeActiveEdges: NULL parameter!");
		return false;
	}

	PxU32 NbEdges = GetNbEdges();
	if(!NbEdges)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "ActiveEdges::ComputeConvexEdges: no edges in edge list!");
		return false;
	}

	const Gu::EdgeData* Edges = GetEdges();
	if(!Edges)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "ActiveEdges::ComputeConvexEdges: no edge data in edge list!");
		return false;
	}

	const Gu::EdgeDescData* ED = GetEdgeToTriangles();
	if(!ED)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "ActiveEdges::ComputeConvexEdges: no edge-to-triangle in edge list!");
		return false;
	}

	const PxU32* FBE = GetFacesByEdges();
	if(!FBE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "ActiveEdges::ComputeConvexEdges: no faces-by-edges in edge list!");
		return false;
	}

	// We first create active edges in a temporaray buffer. We have one bool / edge.
	bool* ActiveEdges = (bool*)PX_ALLOC_TEMP(sizeof(bool)*NbEdges);

	// Loop through edges and look for convex ones
	bool* CurrentMark = ActiveEdges;
	while(NbEdges--)
	{
		// Get number of triangles sharing current edge
		PxU32 Count = ED->Count;
		// Boundary edges are active => keep them (actually they're silhouette edges directly)
		// Internal edges can be active => test them
		// Singular edges ? => discard them
		bool Active = false;
		if(Count==1)
		{
			Active = true;
		}
		else if(Count==2)
		{
			PxU32 FaceIndex0 = FBE[ED->Offset+0]*3;
			PxU32 FaceIndex1 = FBE[ED->Offset+1]*3;

			PxU32 VRef00, VRef01, VRef02;
			PxU32 VRef10, VRef11, VRef12;

			if(dfaces)
			{
				VRef00 = dfaces[FaceIndex0+0];
				VRef01 = dfaces[FaceIndex0+1];
				VRef02 = dfaces[FaceIndex0+2];
				VRef10 = dfaces[FaceIndex1+0];
				VRef11 = dfaces[FaceIndex1+1];
				VRef12 = dfaces[FaceIndex1+2];
			}
			else if(wfaces)
			{
				VRef00 = wfaces[FaceIndex0+0];
				VRef01 = wfaces[FaceIndex0+1];
				VRef02 = wfaces[FaceIndex0+2];
				VRef10 = wfaces[FaceIndex1+0];
				VRef11 = wfaces[FaceIndex1+1];
				VRef12 = wfaces[FaceIndex1+2];
			}

/*			if(1)
			{
				// We check the opposite vertex against the plane instead of actually computing the angle (faster)

	//			PxU32 Op = faces[FBE[ED->Offset+0]].OppositeVertex(Edges->mRef0, Edges->mRef1);
				PxU32 Op = OppositeVertex(VRef00, VRef01, VRef02, Edges->mRef0, Edges->mRef1);

	//			Plane PL1 = faces[FBE[ED->Offset+1]].PlaneEquation(verts);
				Plane PL1(verts[VRef10], verts[VRef11], verts[VRef12]);

				if(PL1.Distance(verts[Op])<-epsilon)	Active = true;
			}
			else
			{
				// The method above doesn't work well since it depends on where the opposite vertex actually is !
				Triangle T0(verts[VRef00], verts[VRef01], verts[VRef02]);
				Triangle T1(verts[VRef10], verts[VRef11], verts[VRef12]);

				PxVec3 N0, N1;
				T0.Normal(N0);
				T1.Normal(N1);
				float a = Angle(N0, N1);

//				if(fabsf(a)>epsilon)	Active = true;
//				if(fabsf(a)>0.05f)	Active = true;
				if(fabsf(a)>0.1f)	Active = true;	// 
//				if(fabsf(a)>0.2f)	Active = true;	// Works best for Rob Elam's ship, but fails on his scene
			}*/

			if(1)
			{
				// We first check the opposite vertex against the plane

	//			PxU32 Op = faces[FBE[ED->Offset+0]].OppositeVertex(Edges->mRef0, Edges->mRef1);
				PxU32 Op = OppositeVertex(VRef00, VRef01, VRef02, Edges->Ref0, Edges->Ref1);

	//			Plane PL1 = faces[FBE[ED->Offset+1]].PlaneEquation(verts);
				Plane PL1(verts[VRef10], verts[VRef11], verts[VRef12]);

//				if(PL1.Distance(verts[Op])<-epsilon)	Active = true;
				if(PL1.distance(verts[Op])<0.0f)	// If opposite vertex is below the plane, i.e. we discard concave edges
				{
					Gu::Triangle T0(verts[VRef00], verts[VRef01], verts[VRef02]);
					Gu::Triangle T1(verts[VRef10], verts[VRef11], verts[VRef12]);

					PxVec3 N0, N1;
					T0.normal(N0);
					T1.normal(N1);
					const float a = Ps::angle(N0, N1);

					if(fabsf(a)>epsilon)	Active = true;
//					if(fabsf(a)>0.05f)	Active = true;
//					if(fabsf(a)>0.1f)	Active = true;	// Best one
//					if(fabsf(a)>0.2f)	Active = true;	// Works best for Rob Elam's ship, but fails on his scene
				}
			}

		}

		*CurrentMark++ = Active;
		ED++;
		Edges++;
	}

	// Now copy bits back into already existing edge structures
	// - first in edge triangles
	for(PxU32 i=0;i<mData.mNbFaces;i++)
	{
		Gu::EdgeTriangleData& ET = mData.mEdgeFaces[i];
		for(PxU32 j=0;j<3;j++)
		{
			PxU32 Link = ET.mLink[j];
			if(!(Link & MSH_ACTIVE_EDGE_MASK))	// else already active
			{
				if(ActiveEdges[Link & MSH_EDGE_LINK_MASK])	ET.mLink[j] |= MSH_ACTIVE_EDGE_MASK;	// Mark as active
			}
		}
	}

	// - then in edge-to-faces
	for(PxU32 i=0;i<mData.mNbEdges;i++)
	{
		if(ActiveEdges[i])	mData.mEdgeToTriangles[i].Flags |= PX_EDGE_ACTIVE;
	}

	// Free & exit
	PX_FREE_AND_RESET(ActiveEdges);

	if(1)
	{
		//initially all vertices are flagged to ignore them. (we assume them to be flat)
		//for all NONFLAT edges, incl boundary
		//unflag 2 vertices in up to 2 trigs as perhaps interesting
		//for all CONCAVE edges
		//flag 2 vertices in up to 2 trigs to ignore them.

		// Handle active vertices
		PxU32 MaxIndex = 0;
		for(PxU32 i=0;i<nb_faces;i++)
		{
			PxU32 VRef0, VRef1, VRef2;
			if(dfaces)
			{
				VRef0 = dfaces[i*3+0];
				VRef1 = dfaces[i*3+1];
				VRef2 = dfaces[i*3+2];
			}
			else if(wfaces)
			{
				VRef0 = wfaces[i*3+0];
				VRef1 = wfaces[i*3+1];
				VRef2 = wfaces[i*3+2];
			}
			if(VRef0>MaxIndex)	MaxIndex = VRef0;
			if(VRef1>MaxIndex)	MaxIndex = VRef1;
			if(VRef2>MaxIndex)	MaxIndex = VRef2;
		}

		MaxIndex++;
		bool* ActiveVerts = (bool*)PX_ALLOC_TEMP(sizeof(bool)*MaxIndex);
		Ps::memZero(ActiveVerts, MaxIndex*sizeof(bool));

		PX_ASSERT(dfaces || wfaces);
		for(PxU32 i=0;i<mData.mNbFaces;i++)
		{
			PxU32 VRef[3];
			if(dfaces)
			{
				VRef[0] = dfaces[i*3+0];
				VRef[1] = dfaces[i*3+1];
				VRef[2] = dfaces[i*3+2];
			}
			else if(wfaces)
			{
				VRef[0] = wfaces[i*3+0];
				VRef[1] = wfaces[i*3+1];
				VRef[2] = wfaces[i*3+2];
			}

			const Gu::EdgeTriangleData& ET = mData.mEdgeFaces[i];
			for(PxU32 j=0;j<3;j++)
			{
				PxU32 Link = ET.mLink[j];
				if(Link & MSH_ACTIVE_EDGE_MASK)
				{
					// Active edge => mark edge vertices as active
					PxU32 r0, r1;
					if(j==0)	{ r0=0;	r1=1; }
					if(j==1)	{ r0=1;	r1=2; }
					if(j==2)	{ r0=0;	r1=2; }
					ActiveVerts[VRef[r0]] = ActiveVerts[VRef[r1]] = true;
				}
			}
		}

/*		for(PxU32 i=0;i<mNbFaces;i++)
		{
			PxU32 VRef[3];
			if(dfaces)
			{
				VRef[0] = dfaces[i*3+0];
				VRef[1] = dfaces[i*3+1];
				VRef[2] = dfaces[i*3+2];
			}
			else if(wfaces)
			{
				VRef[0] = wfaces[i*3+0];
				VRef[1] = wfaces[i*3+1];
				VRef[2] = wfaces[i*3+2];
			}

			const EdgeTriangle& ET = mEdgeFaces[i];
			for(PxU32 j=0;j<3;j++)
			{
				PxU32 Link = ET.mLink[j];
				if(!(Link & MSH_ACTIVE_EDGE_MASK))
				{
					// Inactive edge => mark edge vertices as inactive
					PxU32 r0, r1;
					if(j==0)	{ r0=0;	r1=1; }
					if(j==1)	{ r0=1;	r1=2; }
					if(j==2)	{ r0=0;	r1=2; }
					ActiveVerts[VRef[r0]] = ActiveVerts[VRef[r1]] = false;
				}
			}
		}*/

		// Now stuff this into the structure
		for(PxU32 i=0;i<mData.mNbFaces;i++)
		{
			PxU32 VRef[3];
			if(dfaces)
			{
				VRef[0] = dfaces[i*3+0];
				VRef[1] = dfaces[i*3+1];
				VRef[2] = dfaces[i*3+2];
			}
			else if(wfaces)
			{
				VRef[0] = wfaces[i*3+0];
				VRef[1] = wfaces[i*3+1];
				VRef[2] = wfaces[i*3+2];
			}

			Gu::EdgeTriangleData& ET = mData.mEdgeFaces[i];
			for(PxU32 j=0;j<3;j++)
			{
				PxU32 Link = ET.mLink[j];
				if(!(Link & MSH_ACTIVE_VERTEX_MASK))	// else already active
				{
					if(ActiveVerts[VRef[j]])	ET.mLink[j] |= MSH_ACTIVE_VERTEX_MASK;	// Mark as active
				}
			}
		}

		PX_FREE_AND_RESET(ActiveVerts);
	}

	return true;
}

Gu::EdgeListBuilder::EdgeListBuilder()
{
}

Gu::EdgeListBuilder::~EdgeListBuilder()
{
}
/*
bool EdgeListBuilder::Save(Stream& stream) const
{
	bool PlatformMismatch = PxPlatformMismatch();

	// Export header
	if(!WriteHeader('E', 'D', 'G', 'E', gVersion, PlatformMismatch, stream))
		return false;

	// Export edges
//	stream.StoreDword(mNbEdges);
	WriteDword(mNbEdges, PlatformMismatch, stream);

//	stream.StoreBuffer(mEdges, sizeof(Edge)*mNbEdges);
	for(PxU32 i=0;i<mNbEdges;i++)
	{
		Edge TmpCopy = mEdges[i];
		if(PlatformMismatch)
		{
			Flip(TmpCopy.mRef0);
			Flip(TmpCopy.mRef1);
		}
		stream.StoreBuffer(&TmpCopy, sizeof(Edge));
	}

//	stream.StoreDword(mNbFaces);
	WriteDword(mNbFaces, PlatformMismatch, stream);

//	stream.StoreBuffer(mEdgeFaces, sizeof(EdgeTriangle)*mNbFaces);
	WriteDwordBuffer((const PxU32*)mEdgeFaces, mNbFaces*3, PlatformMismatch, stream);

//	stream.StoreBuffer(mEdgeToTriangles, sizeof(EdgeDesc)*mNbEdges);
	for(PxU32 i=0;i<mNbEdges;i++)
	{
		EdgeDesc TmpCopy = mEdgeToTriangles[i];
		if(PlatformMismatch)
		{
			Flip(TmpCopy.Flags);
			Flip(TmpCopy.Count);
			Flip(TmpCopy.Offset);
		}
		stream.StoreBuffer(&TmpCopy, sizeof(EdgeDesc));
	}

	PxU32 LastOffset = mEdgeToTriangles[mNbEdges-1].Offset + mEdgeToTriangles[mNbEdges-1].Count;
//	stream.StoreBuffer(mFacesByEdges, sizeof(PxU32)*LastOffset);
	WriteDwordBuffer(mFacesByEdges, LastOffset, PlatformMismatch, stream);

	return true;
}*/
//#endif




#ifndef ICE_EDGELIST_V22

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ActiveEdges::ActiveEdges() : mActiveEdges(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ActiveEdges::~ActiveEdges()
{
	PX_FREE_AND_RESET(mActiveEdges);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes active (a.k.a. convex) edges.
 *	\param		edges		[in] edge-list built from source data
 *	\param		faces		[in] source topology
 *	\param		verts		[in] source geometry
 *	\param		epsilon		[in] threshold value used for distance computation
 *	\return		true if success
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ActiveEdges::Compute(const EdgeList& edges, const IndexedTriangle* faces, const PxVec3* verts, float epsilon)
{
	// Checkings
	if(!faces || !verts)	return SetIceError("ActiveEdges::ComputeConvexEdges: NULL parameter!");

	PxU32 NbEdges = edges.GetNbEdges();
	if(!NbEdges)			return SetIceError("ActiveEdges::ComputeConvexEdges: no edges in edge list!");

	const Edge* Edges = edges.GetEdges();
	if(!Edges)				return SetIceError("ActiveEdges::ComputeConvexEdges: no edge data in edge list!");

	const EdgeDesc* ED = edges.GetEdgeToTriangles();
	if(!ED)					return SetIceError("ActiveEdges::ComputeConvexEdges: no edge-to-triangle in edge list!");

	const PxU32* FBE = edges.GetFacesByEdges();
	if(!FBE)				return SetIceError("ActiveEdges::ComputeConvexEdges: no faces-by-edges in edge list!");

	PX_FREE_AND_RESET(mActiveEdges);
	mActiveEdges = (bool*)ICE_ALLOC_MEM(sizeof(bool)*NbEdges,EdgeList_ActiveEdges);

	// Loop through edges and look for convex ones
	bool* CurrentMark = mActiveEdges;
	while(NbEdges--)
	{
		// Get number of triangles sharing current edge
		PxU32 Count = ED->Count;
		// Boundary edges are active => keep them (actually they're silhouette edges directly)
		// Internal edges can be active => test them
		// Singular edges ? => discard them
		bool Active = false;
		if(Count==1)
		{
			Active = true;
		}
		else if(Count==2)
		{
			PxU32 Op = faces[FBE[ED->Offset+0]].OppositeVertex(Edges->mRef0, Edges->mRef1);
			Plane PL1 = faces[FBE[ED->Offset+1]].PlaneEquation(verts);
			if(PL1.Distance(verts[Op])<-epsilon)	Active = true;
		}

		*CurrentMark++ = Active;
		ED++;
		Edges++;
	}
	return true;
}
#endif



