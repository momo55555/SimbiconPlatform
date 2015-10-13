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


#include "DeformableMesh.h"
#include "PsSort.h"
#include "Serialize.h"
#include "PxBounds3.h"

#define DEFORMABLE_MESH_VERSION 10

using namespace physx;

//----------------------------------------------------------------------------//

struct DeformableTriEdge 
{
	void set(PxU32 i0, PxU32 i1, PxU32 i2, PxU32 t) 
	{
		if (i0 < i1) 
		{ 
			particleId[0] = i0; 
			particleId[1] = i1; 
		}
		else 
		{ 
			particleId[0] = i1; 
			particleId[1] = i0; 
		}
		third = i2;
		triangle = t;
	}

	bool operator<(const DeformableTriEdge& other) const 
	{
		return particleId[0] == other.particleId[0] ?
			particleId[1] < other.particleId[1] :
			particleId[0] < other.particleId[0] ;
	}

	bool operator==(const DeformableTriEdge& other) const 
	{
		if (particleId[0] != other.particleId[0]) 
			return false;
		if (particleId[1] != other.particleId[1]) 
			return false;
		return true;
	}

	PxU32 particleId[2];
	PxU32 third;
	PxU32 triangle;
};

//----------------------------------------------------------------------------//

struct DeformableTetraEdge 
{
	void set(PxU32 i0, PxU32 i1, PxU32 t) 
	{
		if (i0 < i1) 
		{ 
			particleId[0] = i0; particleId[1] = i1; 
		}
		else 
		{ 
			particleId[0] = i1; particleId[1] = i0; 
		}
		tetrahedron = t;
	}

	bool operator<(const DeformableTetraEdge& other) 
	{
		if (particleId[0] < other.particleId[0]) 
			return true;
		if (particleId[0] > other.particleId[0]) 
			return false;
		if (particleId[1] < other.particleId[1]) 
			return true;
		if (particleId[1] > other.particleId[1]) 
			return false;		
		return tetrahedron < other.tetrahedron;
	}
	
	bool operator==(const DeformableTetraEdge& other) 
	{
		return particleId[0] == other.particleId[0] && particleId[1] == other.particleId[1] && tetrahedron == other.tetrahedron;
	}

	PxU32 particleId[2];
	PxU32 tetrahedron;
};

//----------------------------------------------------------------------------//

struct DeformableTriangle 
{
	void set(PxU32 i0, PxU32 i1, PxU32 i2, PxU32 t) 
	{
		if (i0 < i1 && i0 < i2) 
		{
			if (i1 < i2) 
			{ 
				particleId[0] = i0; particleId[1] = i1; particleId[2] = i2; 
			}
			else         
			{ 
				particleId[0] = i0; particleId[1] = i2; particleId[2] = i1; 
			}
		}
		else if (i1 < i2) 
		{
			if (i0 < i2) 
			{ 
				particleId[0] = i1; particleId[1] = i0; particleId[2] = i2; 
			}
			else         
			{ 
				particleId[0] = i1; particleId[1] = i2; particleId[2] = i0; 
			}
		}
		else 
		{
			if (i0 < i1) 
			{ 
				particleId[0] = i2; particleId[1] = i0; particleId[2] = i1; 
			}
			else         
			{ 
				particleId[0] = i2; particleId[1] = i1; particleId[2] = i0; 
			}
		}
		triangle = t;
	}

	bool operator<(const DeformableTriangle& other) 
	{
		if (particleId[0] < other.particleId[0]) 
			return true;
		if (particleId[0] > other.particleId[0]) 
			return false;
		if (particleId[1] < other.particleId[1]) 
			return true;
		if (particleId[1] > other.particleId[1]) 
			return false;
		return particleId[2] < other.particleId[2];
	}

	bool operator==(const DeformableTriangle& other) 
	{
		if (particleId[0] != other.particleId[0]) 
			return false;
		if (particleId[1] != other.particleId[1]) 
			return false;
		if (particleId[2] != other.particleId[2]) 
			return false;
		return true;
	}

	PxU32 particleId[3];
	PxU32 triangle;
};

//----------------------------------------------------------------------------//

static void quickSortEdges(Ps::Array<DeformableTriEdge>& edges, int l, int r)
{
	int i,j, mi;
	DeformableTriEdge k, m;

	i = l; j = r; mi = (l + r)/2;
	m = edges[mi];
	while (i <= j) {
		while(edges[i] < m) i++;
		while(m < edges[j]) j--;
		if (i <= j) {
			k = edges[i]; edges[i] = edges[j]; edges[j] = k;
			i++; j--;
		}
	}
	if (l < j) quickSortEdges(edges, l, j);
	if (i < r) quickSortEdges(edges, i, r);
}

//----------------------------------------------------------------------------//

static void quickSortTetraEdges(Ps::Array<DeformableTetraEdge>& edges, int l, int r)
{
	int i,j, mi;
	DeformableTetraEdge k, m;

	i = l; j = r; mi = (l + r)/2;
	m = edges[mi];
	while (i <= j) {
		while(edges[i] < m) i++;
		while(m < edges[j]) j--;
		if (i <= j) {
			k = edges[i]; edges[i] = edges[j]; edges[j] = k;
			i++; j--;
		}
	}
	if (l < j) quickSortTetraEdges(edges, l, j);
	if (i < r) quickSortTetraEdges(edges, i, r);
}

//----------------------------------------------------------------------------//

static void quickSortTriangles(Ps::Array<DeformableTriangle>& triangles, int l, int r)
{
	int i,j, mi;
	DeformableTriangle k, m;

	i = l; j = r; mi = (l + r)/2;
	m = triangles[mi];
	while (i <= j) {
		while(triangles[i] < m) i++;
		while(m < triangles[j]) j--;
		if (i <= j) {
			k = triangles[i]; triangles[i] = triangles[j]; triangles[j] = k;
			i++; j--;
		}
	}
	if (l < j) quickSortTriangles(triangles, l, j);
	if (i < r) quickSortTriangles(triangles, i, r);
}


//----------------------------------------------------------------------------//

DeformableMesh::DeformableMesh() :
mVertexPositions(PX_DEBUG_EXP("dmVertexPositions")),
mPrimitives(PX_DEBUG_EXP("dmVertexIndices")),
mVertexMasses(PX_DEBUG_EXP("dmVertexMasses")),
mVertexFlags(PX_DEBUG_EXP("dmVertexFlags")),
mWeldedVertices(PX_DEBUG_EXP("dmWeldedVertices")),
mVertexToParticleMap(PX_DEBUG_EXP("dmVertexToParticleMap")),
mConstraints(PX_DEBUG_EXP("dmVertexConstraints"))
{
// PX_SERIALIZATION
	mOwnsMemory = true;
//~PX_SERIALIZATION
}

// PX_SERIALIZATION
void DeformableMesh::onRefCountZero()
{
	if(ownsMemory())
		delete this;
	else
		this->~DeformableMesh();
}
//~PX_SERIALIZATION

void DeformableMesh::clear()
{
	mPrimitiveType = PxDeformablePrimitiveType::eNONE;
	
	mVertexPositions.clear();
	mPrimitives.clear();
	mVertexMasses.clear();
	mVertexFlags.clear();
	
	mNumWeldedVertices = 0;
	mVertexToParticleMap.clear();
	mWeldedVertices.clear();
	mConstraints.clear();	

	mFlags = 0;
	mMaxHierarchyLevels = 0;
	mWeldingDistance = 0.0f;
}

//----------------------------------------------------------------------------//

void DeformableMesh::saveToDesc(PxDeformableMeshDesc& desc) const
{
	desc.primitiveType			= mPrimitiveType;
	desc.numVertices			= mVertexPositions.size();
	desc.vertexStrideBytes		= sizeof(PxVec3);
	desc.vertices				= mVertexPositions.begin();
	desc.primitives				= mPrimitives.begin();
	desc.vertexMassStrideBytes	= sizeof(PxReal);
	desc.vertexFlagStrideBytes	= sizeof(PxU32);
	desc.vertexMasses			= mVertexMasses.begin();
	desc.vertexFlags			= mVertexFlags.begin();
	desc.weldingDistance        = mWeldingDistance; 
	desc.maxHierarchyLevels		= mMaxHierarchyLevels;
	desc.flags					= mFlags;

	if(mPrimitiveType ==  PxDeformablePrimitiveType::eTRIANGLE)
	{
		desc.numPrimitives = mPrimitives.size() / 3;
		desc.primitiveStrideBytes = sizeof(PxU32) * 3;
	}
	else if (mPrimitiveType ==  PxDeformablePrimitiveType::eTETRAHEDRON)
	{
		desc.numPrimitives = mPrimitives.size() / 4;
		desc.primitiveStrideBytes = sizeof(PxU32) * 4;
	}
}

//----------------------------------------------------------------------------//

bool DeformableMesh::loadMesh(const PxDeformableMeshDesc& meshDesc)
{
	if (meshDesc.numVertices == 0 || meshDesc.numPrimitives == 0)
		return false;

	// vertices
	PxStrideIterator<const PxVec3> positionIt(static_cast<const PxVec3*>(meshDesc.vertices), meshDesc.vertexStrideBytes);
	PxStrideIterator<const PxReal> massIt(static_cast<const PxReal*>(meshDesc.vertexMasses), meshDesc.vertexMassStrideBytes);
	PxStrideIterator<const PxU32> flagsIt(static_cast<const PxU32*>(meshDesc.vertexFlags), meshDesc.vertexFlagStrideBytes);

	if (positionIt.ptr())
		mVertexPositions.resize(meshDesc.numVertices);
	if (massIt.ptr())		
		mVertexMasses.resize(meshDesc.numVertices);
	if (flagsIt.ptr())
		mVertexFlags.resize(meshDesc.numVertices);

	for (PxU32 i = 0; i < meshDesc.numVertices; i++)
	{
		if (positionIt.ptr())
			mVertexPositions[i] = *positionIt++;
		if (massIt.ptr())		
			mVertexMasses[i] = *massIt++;
		if (flagsIt.ptr())
			mVertexFlags[i] = *flagsIt++;
	}

	// indices
	PxU32 tupleSize = 0;
	if (mPrimitiveType == PxDeformablePrimitiveType::eTRIANGLE)
		tupleSize = 3;
	else if (mPrimitiveType == PxDeformablePrimitiveType::eTETRAHEDRON)
		tupleSize = 4;
	mPrimitives.reserve(meshDesc.numPrimitives * tupleSize);	

	if ((meshDesc.flags & PxDeformableMeshFlag::e16_BIT_INDICES) != 0)
	{
		PxStrideIterator<const PxU16> indexIt(static_cast<const PxU16*>(meshDesc.primitives), meshDesc.primitiveStrideBytes);
		for (PxU32 i = 0; i != meshDesc.numPrimitives; ++i, ++indexIt)
			for (PxU32 j = 0; j < tupleSize; j++)
				mPrimitives.pushBack(indexIt.ptr()[j]);
	}
	else
	{
		PxStrideIterator<const PxU32> indexIt(static_cast<const PxU32*>(meshDesc.primitives), meshDesc.primitiveStrideBytes);
		for (PxU32 i = 0; i != meshDesc.numPrimitives; ++i, ++indexIt)
			for (PxU32 j = 0; j < tupleSize; j++)
				mPrimitives.pushBack(indexIt.ptr()[j]);
	}

	return true;
}

//----------------------------------------------------------------------------//

struct WeldComparator
{
	WeldComparator(const Ps::Array<PxVec3>& pos) : mPos(pos) {}
	bool operator()(const int& a, const int& b) const { return mPos[a].x < mPos[b].x; }
	const Ps::Array<PxVec3>& mPos;
};

void DeformableMesh::weldMesh()
{
	mVertexToParticleMap.resize(mVertexPositions.size());

	if (mPrimitiveType == PxDeformablePrimitiveType::eTRIANGLE && (mFlags & PxDeformableMeshFlag::eWELD_VERTICES) != 0)
	{
		Ps::Array<int> order PX_DEBUG_EXP("defoMeshOrder");
		order.resize(mVertexPositions.size());
		for (PxU32 i = 0; i < order.size(); i++)
			order[i] = i;

		// sort vertices by their x coordinate
		Ps::sort(order.begin(), order.size(), WeldComparator(mVertexPositions));

		// generate permutation table which welds similar vertices
		for (PxU32 i = 0; i < mVertexPositions.size(); i++)
			mVertexToParticleMap[i] = PX_MAX_U32;

		int newNr = 0;
		PxReal mWeldingDistanceSq = mWeldingDistance * mWeldingDistance;
		for (PxU32 i = 0; i < mVertexPositions.size(); i++) 
		{
			int oldNr = order[i];
			if (mVertexToParticleMap[oldNr] != PX_MAX_U32)
				continue;
			
			mVertexToParticleMap[oldNr] = newNr;
			PxVec3 vi = mVertexPositions[oldNr];
			PxU32 j = i+1;
			while (j < mVertexPositions.size() && PxAbs(vi.x - mVertexPositions[order[j]].x) <= mWeldingDistance) 
			{
				oldNr = order[j];
				if ((vi-mVertexPositions[oldNr]).magnitudeSquared() <= mWeldingDistanceSq) 
				{
					if (mVertexToParticleMap[oldNr] == PX_MAX_U32)
						mVertexToParticleMap[oldNr] = newNr;
				}
				j++;
			}
			newNr++;
		}
		mNumWeldedVertices = newNr;
	}
	else
	{
		// Welding not enabled. We use an identity permutation.
		for (PxU32 i = 0; i < mVertexToParticleMap.size(); i++)
			mVertexToParticleMap[i] = i;

#if 0
		Ps::Array<int> order;
		order.resize(mVertexPositions.size());
		for (PxU32 i = 0; i < order.size(); i++)
			order[i] = i;
		Ps::sort(order.begin(), order.size(), WeldComparator(mVertexPositions));
		for (PxU32 i = 0; i < mVertexToParticleMap.size(); i++)
			mVertexToParticleMap[i] = order[i];
#endif

		mNumWeldedVertices = mVertexPositions.size();
	}

	mWeldedVertices.resize(mNumWeldedVertices);
	for (PxU32 i = 0; i < mVertexPositions.size(); i++)
		mWeldedVertices[mVertexToParticleMap[i]] = mVertexPositions[i];
}

//----------------------------------------------------------------------------//

bool DeformableMesh::loadFromDesc(const PxDeformableMeshDesc& desc)
{
	if(!desc.isValid())	return false;

	clear();

	mPrimitiveType = desc.primitiveType;
	mWeldingDistance = desc.weldingDistance;
	mMaxHierarchyLevels = desc.maxHierarchyLevels;
	mFlags = desc.flags;

	if (!loadMesh(desc))
		return false;

	weldMesh();

	if (mPrimitiveType == PxDeformablePrimitiveType::eTRIANGLE) 
		generateConstraintsFromTriangles();
	else if (mPrimitiveType == PxDeformablePrimitiveType::eTETRAHEDRON) 
		generateConstraintsFromTetrahedra();
	else 
		return false;

	//DeformableCooker cooker(mPrimitiveType, mVertexPositions, mConstraints);

	return true;
}

//----------------------------------------------------------------------------//

bool DeformableMesh::load(const PxStream& stream)
{
	// import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('D', 'F', 'M', 'B', version, mismatch, stream))
		return false;

	clear();

	mPrimitiveType = static_cast<PxDeformablePrimitiveType::Enum>(readDword(mismatch, stream));
	mFlags = readDword(mismatch, stream);
	mWeldingDistance = readFloat(mismatch, stream);
	mMaxHierarchyLevels = readDword(mismatch, stream);

	PX_ASSERT(mPrimitiveType == PxDeformablePrimitiveType::eTRIANGLE || mPrimitiveType == PxDeformablePrimitiveType::eTETRAHEDRON);

	// import mesh	
	PxU32 numVertices = readDword(mismatch, stream);
	for (PxU32 i = 0; i < numVertices; i++) 
	{
		PxVec3 p;
		p.x = readFloat(mismatch, stream);
		p.y = readFloat(mismatch, stream);
		p.z = readFloat(mismatch, stream);
		mVertexPositions.pushBack(p);
	}

	// import indices
	PxU32 numIndices = readDword(mismatch, stream);
	for (PxU32 i = 0; i < numIndices; i++) 
		mPrimitives.pushBack(readDword(mismatch, stream));

	// import vertex masses
	PxU32 numVertexMasses = readDword(mismatch, stream);
	for (PxU32 i = 0; i < numVertexMasses; i++)
		mVertexMasses.pushBack(readFloat(mismatch, stream));

	// import vertex flags
	PxU32 numVertexFlags = readDword(mismatch, stream);
	for (PxU32 i = 0; i < numVertexFlags; i++)
		mVertexFlags.pushBack(readDword(mismatch, stream));

	// import number of welded vertices
	mNumWeldedVertices = readDword(mismatch, stream);

	// import vertex to particle map
	PxU32 mapSize = readDword(mismatch, stream);
	for (PxU32 i = 0; i < mapSize; i++)
		mVertexToParticleMap.pushBack(readDword(mismatch, stream));

	// import constraints
	PxU32 numConstraints = readDword(mismatch, stream);
	if (mPrimitiveType == PxDeformablePrimitiveType::eTRIANGLE) 
	{	
		for (PxU32 i = 0; i < numConstraints; i++) 
		{
			DeformableConstraint constraint;			
			constraint.particleId[0] = readDword(mismatch, stream);
			constraint.particleId[1] = readDword(mismatch, stream);
			constraint.particleId[2] = readDword(mismatch, stream);
			constraint.particleId[3] = readDword(mismatch, stream);
			constraint.particleId[4] = -1;
			constraint.particleId[5] = -1;
			constraint.stretchingRestLength = readFloat(mismatch, stream);
			constraint.bendingRestLength = readFloat(mismatch, stream);
			constraint.flags = readWord(mismatch, stream);
			mConstraints.pushBack(constraint); 
		}
	}
	else if (mPrimitiveType == PxDeformablePrimitiveType::eTETRAHEDRON) 
	{
		for (PxU32 i = 0; i < numConstraints; i++) 
		{
			DeformableConstraint constraint;
			for(PxU8 k = 0; k < 4; k++)
				constraint.particleId[k] = readDword(mismatch, stream);
			constraint.restVolume = readFloat(mismatch, stream);
			for(PxU8 k = 0; k < 6; k++)
				constraint.restEdgeLengths[k] = readFloat(mismatch, stream);
			constraint.flags = readWord(mismatch, stream);
			mConstraints.pushBack(constraint); 
		}
	}
	else 
		return false;

	return true;
}

//----------------------------------------------------------------------------//

bool DeformableMesh::save(PxStream& stream, bool platformMismatch) const
{
	// export header
	if(!writeHeader('D', 'F', 'M', 'B', DEFORMABLE_MESH_VERSION, platformMismatch, stream))
		return false;

	writeDword(mPrimitiveType, platformMismatch, stream); 
	writeDword(mFlags, platformMismatch, stream);
	writeFloat(mWeldingDistance, platformMismatch, stream);
	writeDword(mMaxHierarchyLevels, platformMismatch, stream);

	// export mesh
	writeDword(mVertexPositions.size(), platformMismatch, stream);
	for (PxU32 i = 0; i < mVertexPositions.size(); i++) 
	{
		writeFloat(mVertexPositions[i].x, platformMismatch, stream);
		writeFloat(mVertexPositions[i].y, platformMismatch, stream);
		writeFloat(mVertexPositions[i].z, platformMismatch, stream);
	}

	// export indices
	writeDword(mPrimitives.size(), platformMismatch, stream);
	for (PxU32 i = 0; i < mPrimitives.size(); i++)
		writeDword(mPrimitives[i], platformMismatch, stream);
	
	// export vertex masses
	writeDword(mVertexMasses.size(), platformMismatch, stream);
	for (PxU32 i = 0; i < mVertexMasses.size(); i++)
		writeFloat(mVertexMasses[i], platformMismatch, stream);

	// export vertex flags
	writeDword(mVertexFlags.size(), platformMismatch, stream);
	for (PxU32 i = 0; i < mVertexFlags.size(); i++)
		writeDword(mVertexFlags[i], platformMismatch, stream);

	// export number of welded vertices
	writeDword(mNumWeldedVertices, platformMismatch, stream);

	// export vertex to particle map
	writeDword(mVertexToParticleMap.size(), platformMismatch, stream);
	for (PxU32 i = 0; i < mVertexToParticleMap.size(); i++)
		writeDword(mVertexToParticleMap[i], platformMismatch, stream);

	// export constraints
	writeDword(mConstraints.size(), platformMismatch, stream);
	if (mPrimitiveType == PxDeformablePrimitiveType::eTRIANGLE) 
	{
		for (PxU32 i = 0; i < mConstraints.size(); i++) 
		{
			const DeformableConstraint& constraint = mConstraints[i];
			writeDword(constraint.particleId[0], platformMismatch, stream);
			writeDword(constraint.particleId[1], platformMismatch, stream);
			writeDword(constraint.particleId[2], platformMismatch, stream);
			writeDword(constraint.particleId[3], platformMismatch, stream);
			writeFloat(constraint.stretchingRestLength, platformMismatch, stream);
			writeFloat(constraint.bendingRestLength, platformMismatch, stream);
			writeWord(constraint.flags, platformMismatch, stream);
		}
	} 
	else if (mPrimitiveType == PxDeformablePrimitiveType::eTETRAHEDRON) 
	{
		for (PxU32 i = 0; i < mConstraints.size(); i++) 
		{
			const DeformableConstraint& constraint = mConstraints[i];
			for(PxU8 j = 0; j < 4; j++)
				writeDword(constraint.particleId[j], platformMismatch, stream);			
			writeFloat(constraint.restVolume, platformMismatch, stream);
			for(PxU8 j = 0; j < 6; j++)
				writeFloat(constraint.restEdgeLengths[j], platformMismatch, stream);
			writeWord(constraint.flags, platformMismatch, stream);
		}
	}
	else 
		return false;

	return true;
}

//----------------------------------------------------------------------------//

void DeformableMesh::generateConstraintsFromTriangles()
{
	PxU32 numTriangles = mPrimitives.size() / 3;

	DeformableTriEdge e;
	Ps::Array<DeformableTriEdge> edges PX_DEBUG_EXP("defoMeshEdges");
	edges.reserve(numTriangles * 3);
	const PxU32 *i0 = mPrimitives.begin();

	for (PxU32 i = 0; i < numTriangles; i++, i0 += 3) 
	{
		const PxU32 *i1 = i0+1;
		const PxU32 *i2 = i0+2;
		e.set(mVertexToParticleMap[*i0], mVertexToParticleMap[*i1], mVertexToParticleMap[*i2], i); 
		edges.pushBack(e);
		e.set(mVertexToParticleMap[*i1], mVertexToParticleMap[*i2], mVertexToParticleMap[*i0], i); 
		edges.pushBack(e);
		e.set(mVertexToParticleMap[*i2], mVertexToParticleMap[*i0], mVertexToParticleMap[*i1], i); 
		edges.pushBack(e);
	}
	quickSortEdges(edges, 0, edges.size()-1);

	DeformableConstraint constraint;
	constraint.particleId[4] = -1;	// only used when torn
	constraint.particleId[5] = -1;	// only used when torn

	for(PxU32 i=0; i<edges.size(); )
	{
		const DeformableTriEdge &e0 = edges[i];
		PxU32 p0 = constraint.particleId[0] = e0.particleId[0];
		PxU32 p1 = constraint.particleId[1] = e0.particleId[1];
		PxVec3 d0 = mWeldedVertices[p1] - mWeldedVertices[p0]; 

		constraint.stretchingRestLength = d0.magnitude();
		constraint.particleId[2] = e0.third;
		constraint.particleId[3] = -1;			// for border edges -> no bending possible
		constraint.bendingRestLength = 0.0f;
		constraint.flags = 0;
		if (++i < edges.size()) {
			const DeformableTriEdge &e1 = edges[i];
			if (e0 == e1) {
				constraint.particleId[2] = e0.third;
				constraint.particleId[3] = e1.third;
				PxVec3 d1 = mWeldedVertices[e1.third] - mWeldedVertices[e0.third]; 
				constraint.bendingRestLength = d1.magnitude();
			}
			while (i < edges.size() && edges[i] == e0)
				++i;
		}
		mConstraints.pushBack(constraint);
	}
}

//----------------------------------------------------------------------------//

// returns 0 if the edge found is not the one with a minimal tetrahedron index,
// i.e., if the edge actually belongs to a tetrahedron other that the one given in goalEdge
static DeformableTetraEdge* binarySearchTetraEdge(Ps::Array<DeformableTetraEdge>& edges, DeformableTetraEdge& goalEdge)
{
	PxU32 l = 0;
	PxU32 r = edges.size() - 1;

	while (l <= r) {
		PxU32 m = (l + r) / 2;
		if (edges[m] == goalEdge) {
			if(m == 0 || edges[m-1].particleId[0] != edges[m].particleId[0] || edges[m-1].particleId[1] != edges[m].particleId[1])
				return edges.begin() + m;
			else
				return 0;
		}
		else if(goalEdge < edges[m])
			r = m - 1;
		else
			l = m + 1;		
	}
	// not found, bad
	PX_ASSERT(0);
	return 0;
}

//----------------------------------------------------------------------------//

void DeformableMesh::generateConstraintsFromTetrahedra()
{
	PxU32 edgeIndices[6][2] = { {0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3} };
	PxU32 *tetIndices;

	// - tetrahedra are assumed to be unique (thus no parent code here)

	PxU32 numTetrahedra = mPrimitives.size() / 4;

	DeformableTetraEdge e;
	Ps::Array<DeformableTetraEdge> edges PX_DEBUG_EXP("defoMeshEdges2");
	edges.reserve(numTetrahedra * 6);
	tetIndices = mPrimitives.begin();
	PxU32 i, j;
	for (i = 0; i < numTetrahedra; i++, tetIndices += 4) 
	{
		for(j = 0; j < 6; j++) 
		{
			PxU32 e0 = mVertexToParticleMap[tetIndices[edgeIndices[j][0]]];
			PxU32 e1 = mVertexToParticleMap[tetIndices[edgeIndices[j][1]]];
			e.set(e0, e1, i); edges.pushBack(e);	
		}
	}
	
	quickSortTetraEdges(edges, 0, edges.size()-1);

	mConstraints.resize(numTetrahedra);

	DeformableConstraint constraint;
	DeformableTetraEdge *tetEdges[6];
	tetIndices = mPrimitives.begin();
	
	bool warningIssued = false;
	for (i = 0; i < numTetrahedra; i++, tetIndices += 4) 
	{
		for (j = 0; j < 6; j++) 
		{
			PxU32 e0 = mVertexToParticleMap[tetIndices[edgeIndices[j][0]]];
			PxU32 e1 = mVertexToParticleMap[tetIndices[edgeIndices[j][1]]];
			DeformableTetraEdge goalEdge;
			goalEdge.set(e0, e1, i);
			tetEdges[j] = binarySearchTetraEdge(edges, goalEdge);
		}

		for (j = 0; j < 4; j++)
			constraint.particleId[j] = mVertexToParticleMap[tetIndices[j]];
		
		PxVec3 groundArea = (mWeldedVertices[tetIndices[1]] - mWeldedVertices[tetIndices[0]]).cross(mWeldedVertices[tetIndices[2]] - mWeldedVertices[tetIndices[0]]);
		constraint.restVolume = groundArea.dot(mWeldedVertices[tetIndices[3]] - mWeldedVertices[tetIndices[0]]);
		constraint.flags = 0;

		if (!warningIssued && constraint.restVolume < 0.0f)
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Soft body mesh tetrahedron %d has illegal winding order.", i);
			warningIssued = true;
		}

		for (j = 0; j < 6; j++) 
		{
			PxU32 e0 = mVertexToParticleMap[tetIndices[edgeIndices[j][0]]];
			PxU32 e1 = mVertexToParticleMap[tetIndices[edgeIndices[j][1]]];
			PxVec3 edgeVec = mWeldedVertices[e1] - mWeldedVertices[e0];
			if(tetEdges[j]) 
			{
				PX_ASSERT(tetEdges[j]->tetrahedron == i);
				constraint.restEdgeLengths[j] = edgeVec.magnitude();
			} 
			else 
				constraint.restEdgeLengths[j] = -edgeVec.magnitude();
		}

		mConstraints[i] = constraint;
	}
}

//----------------------------------------------------------------------------//

void DeformableMesh::calculateInvMasses(Ps::Array<PxReal>& invMasses, PxReal mass) const
{
	PX_ASSERT(mVertexMasses.empty());
	PX_ASSERT(invMasses.size() == mVertexPositions.size());

	// clear output array
	for (PxU32 i = 0; i < invMasses.size(); i++)
		invMasses[i] = 0.0f;
	
	switch (mPrimitiveType)
	{
	case PxDeformablePrimitiveType::eTRIANGLE:
		{
			// first, we compute area associated with each vertex and temporarily store it in invMasses array
			PxReal totalArea = 0.0f;
			const PxU32* i0 = mPrimitives.begin();
			for (PxU32 i = 0; i < mPrimitives.size() / 3; ++i)
			{
				const PxU32* i1 = i0 + 1;
				const PxU32* i2 = i0 + 2;
				PxVec3 d0(mVertexPositions[*i0].x, mVertexPositions[*i0].y, mVertexPositions[*i0].z);
				PxVec3 d1(mVertexPositions[*i1].x, mVertexPositions[*i1].y, mVertexPositions[*i1].z);
				PxVec3 d2(mVertexPositions[*i2].x, mVertexPositions[*i2].y, mVertexPositions[*i2].z);
				d1 = d1 - d0;
				d2 = d2 - d0;
				PxVec3 n = d1.cross(d2);
				PxReal area = 0.5f * n.magnitude();
				invMasses[*i0] += area / 3.0f;
				invMasses[*i1] += area / 3.0f;
				invMasses[*i2] += area / 3.0f;
				totalArea += area;
				i0 += 3;
			}
			// distribute overall mass by associated area
			for (PxU32 i = 0; i < invMasses.size(); i++)
			{
				PxReal area = invMasses[i];
				invMasses[i] = (1.0f / mass) * (totalArea / area);
			}
		}
		break;
	case PxDeformablePrimitiveType::eTETRAHEDRON:
		{
			// first, we compute volume associated with each vertex and temporarily store it in invMasses array
			PxReal totalVolume = 0.0f;
			const PxU32* i0 = mPrimitives.begin();
			for (PxU32 i = 0; i < mPrimitives.size() / 4; i++)
			{
				const PxU32 *i1 = i0 + 1;
				const PxU32 *i2 = i0 + 2;
				const PxU32 *i3 = i0 + 3;
				PxVec3 d0(mVertexPositions[*i0].x, mVertexPositions[*i0].y, mVertexPositions[*i0].z);
				PxVec3 d1(mVertexPositions[*i1].x, mVertexPositions[*i1].y, mVertexPositions[*i1].z);
				PxVec3 d2(mVertexPositions[*i2].x, mVertexPositions[*i2].y, mVertexPositions[*i2].z);
				PxVec3 d3(mVertexPositions[*i3].x, mVertexPositions[*i3].y, mVertexPositions[*i3].z);
				d1 = d1 - d0;
				d2 = d2 - d0;
				d3 = d3 - d0;
				PxVec3 n = d1.cross(d2);
				PxReal volume = n.dot(d3) / 6.0f;
				invMasses[*i0] += volume / 4.0f;
				invMasses[*i1] += volume / 4.0f;
				invMasses[*i2] += volume / 4.0f;
				invMasses[*i3] += volume / 4.0f;
				totalVolume += volume;
				i0 += 4;
			}
			// distribute overall mass by associated volume
			for (PxU32 i = 0; i < invMasses.size(); i++)
			{
				PxReal volume = invMasses[i];
				invMasses[i] = (1.0f / mass) * (totalVolume / volume);
			}

		}
		break;
	default:
		PX_ASSERT(0);
		break;
	}	
}

//----------------------------------------------------------------------------//
// Raycast queries.
//----------------------------------------------------------------------------//

// PT: TODO: use Gu::rayAABBIntersect or the faster version using SAT (see CTC_RayAABBOverlap.h. You just need a boolean result here)
static bool raycastBounds(const PxVec3& rayOrigin, const PxVec3& rayDir, const PxBounds3& bounds, PxVec3& hit, PxReal& t)
{
	// Fast Ray-Box Intersection by Andrew Woo from "Graphics Gems", Academic Press, 1990
	// adapted by MMF

	#define NUMDIM	3
	#define RIGHT	0
	#define LEFT	1
	#define MIDDLE	2
	#define RAYAABB_EPSILON 0.00001f

	bool inside = true;
	char quadrant[NUMDIM];
	int i;
	int whichPlane;
	float maxT[NUMDIM];
	float candidatePlane[NUMDIM];

	// cast into the format Andrew likes
	const float *minB   = reinterpret_cast<const float *>(&bounds.minimum);
	const float *maxB   = reinterpret_cast<const float *>(&bounds.maximum);
	const float *origin = reinterpret_cast<const float *>(&rayOrigin);
	const float *dir    = reinterpret_cast<const float *>(&rayDir);
	float *coord = reinterpret_cast<float *>(&hit);
	
	/* Find candidate planes; this loop can be avoided if
   	rays cast all from the eye(assume perpsective view) */
	for (i = 0; i < NUMDIM; i++)
		if (origin[i] < minB[i])
		{
			quadrant[i] = LEFT;
			candidatePlane[i] = minB[i];
			inside = false;
		}
		else if (origin[i] > maxB[i])
		{
			quadrant[i] = RIGHT;
			candidatePlane[i] = maxB[i];
			inside = false;
		}
		else
		{
			quadrant[i] = MIDDLE;
		}

	/* Ray origin inside bounding box */
	if (inside)
	{
		hit = rayOrigin;
		t = 0.0f;
		return true;
	}

	/* Calculate T distances to candidate planes */
	for (i = 0; i < NUMDIM; i++)
		if (quadrant[i] != MIDDLE && dir[i] != 0.0f)
			maxT[i] = (candidatePlane[i]-origin[i]) / dir[i];
		else
			maxT[i] = -1.0f;

	/* Get largest of the maxT's for final choice of intersection */
	whichPlane = 0;
	for (i = 1; i < NUMDIM; i++)
		if (maxT[whichPlane] < maxT[i])
			whichPlane = i;

	/* Check final candidate actually inside box */
	t = maxT[whichPlane];
	if (t < 0.0f)
		return false;
	for (i = 0; i < NUMDIM; i++)
		if (whichPlane != i)
		{
			coord[i] = origin[i] + t *dir[i];
			if (coord[i] < minB[i] - RAYAABB_EPSILON || coord[i] > maxB[i] + RAYAABB_EPSILON)
				return false;
		}
		else
			coord[i] = candidatePlane[i];
	return true;		
}

//----------------------------------------------------------------------------//

// PT: TODO: use Gu::intersectLineTriangle
static bool raycastTriangle(
	PxVec3& hit, 
	PxReal& t,
	const PxVec3& rayOrigin,
	const PxVec3& rayDir,
	const PxVec3& p0, 
	const PxVec3& p1, 
	const PxVec3& p2)
{
	PxVec3 d1 = p1 - p0;
	PxVec3 d2 = p2 - p0;
	PxVec3 n = d1.cross(d2);
	t = rayDir.dot(n);
	if (t == 0.0f)
		return false;
	t = n.dot(p0 - rayOrigin) / t;
	if (t < 0.0f)
		return false;
	hit = rayOrigin + rayDir * t;
	PxVec3 d0 = p0 - hit;
	d1 = p1-hit;
	d2 = p2-hit;
	PxVec3 c = d0.cross(d1);
	if (c.dot(n) < 0.0f)
		return false;
	c = d1.cross(d2);
	if (c.dot(n) < 0.0f)
		return false;
	c = d2.cross(d0);
	if (c.dot(n) < 0.0f)
		return false;
	return true;
}

//----------------------------------------------------------------------------//

bool DeformableMesh::raycast(
	PxU32& vertexId,
	PxVec3& hit, 
	const PxDeformablePrimitiveType::Enum primitiveType,
	const PxVec3& rayOrigin,
	const PxVec3& rayDir,
	const PxStrideIterator<const PxVec3>& positionIt,
	const PxU32* indexPermutation,
	const PxU32* indices,
	const PxU32 numIndices,
	const PxBounds3& bounds)
{
	PxReal t;
	if (!raycastBounds(rayOrigin, rayDir, bounds, hit, t)) 
		return false;

	bool bHit = false;
	PxReal minDistance = FLT_MAX;

	if (primitiveType == PxDeformablePrimitiveType::eTRIANGLE)
	{
		PxU32 numTriangles = numIndices / 3;

		for (PxU32 i = 0; i < numTriangles; i++)
		{
			PxU32 o[3], v[3];
			for (PxU32 j = 0; j < 3; j++)
			{
				o[j] = indices[3*i+j];
				v[j] = indexPermutation ? indexPermutation[o[j]] : o[j];
			}		

			const PxVec3& p0 = positionIt[v[0]];
			const PxVec3& p1 = positionIt[v[1]];
			const PxVec3& p2 = positionIt[v[2]];

			PxVec3 triHit;
			if (raycastTriangle(triHit, t, rayOrigin, rayDir, p0, p1, p2))
			{
				PxReal d0 = (triHit - p0).magnitudeSquared();
				PxReal d1 = (triHit - p1).magnitudeSquared();
				PxReal d2 = (triHit - p2).magnitudeSquared();
				if ((d0 < d1) && (d0 < d2))
				{
					d0 = (triHit - rayOrigin).magnitudeSquared();
					if (d0 < minDistance)
					{
						minDistance = d0;
						vertexId = o[0];
						hit = triHit;
						bHit = true;
					}
				}
				else if (d1 < d2)
				{
					d1 = (triHit - rayOrigin).magnitudeSquared();
					if (d1 < minDistance)
					{
						minDistance = d1;
						vertexId = o[1];
						hit = triHit;
						bHit = true;
					}
				}
				else
				{
					d2 = (triHit - rayOrigin).magnitudeSquared();
					if (d2 < minDistance)
					{
						minDistance = d2;
						vertexId = o[2];
						hit = triHit;
						bHit = true;
					}
				}
			}
		}
	}
	else if (primitiveType == PxDeformablePrimitiveType::eTETRAHEDRON)
	{
		static PxU8 sideIndices[4][3] = {{2, 1, 0}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}};

		PxU32 numTetras = numIndices / 4;
		
		for (PxU32 i = 0; i < numTetras; i++)
		{
			PxU32 o[4], v[4];
			for (PxU32 j = 0; j < 4; j++)
			{
				o[j] = indices[4*i+j];
				v[j] = indexPermutation ? indexPermutation[o[j]] : o[j];
			}	

			for (PxU32 j = 0; j < 4; j++)
			{
				PxU32 s0 = sideIndices[j][0];
				PxU32 s1 = sideIndices[j][1];
				PxU32 s2 = sideIndices[j][2];

				const PxVec3& p0 = positionIt[v[s0]];
				const PxVec3& p1 = positionIt[v[s1]];
				const PxVec3& p2 = positionIt[v[s2]];

				PxVec3 tetHit;
				if (raycastTriangle(tetHit, t, rayOrigin, rayDir, p0, p1, p2))
				{
					PxReal d0 = (tetHit - p0).magnitudeSquared();
					PxReal d1 = (tetHit - p1).magnitudeSquared();
					PxReal d2 = (tetHit - p2).magnitudeSquared();

					if ((d0 < d1) && (d0 < d2))
					{
						d0 = (tetHit - rayOrigin).magnitudeSquared();
						if (d0 < minDistance)
						{
							minDistance = d0;
							vertexId = o[s0];
							hit = tetHit;
							bHit = true;
						}
					}
					else if (d1 < d2)
					{
						d1 = (tetHit - rayOrigin).magnitudeSquared();
						if (d1 < minDistance)
						{
							minDistance = d1;
							vertexId = o[s1];
							hit = tetHit;
							bHit = true;
						}
					}
					else
					{
						d2 = (tetHit - rayOrigin).magnitudeSquared();
						if (d2 < minDistance)
						{
							minDistance = d2;
							vertexId = o[s2];
							hit = tetHit;
							bHit = true;
						}
					}
				}
			}
		}
	}
	else
	{
		PX_ASSERT(0);
	}

	return bHit;
}

//----------------------------------------------------------------------------//