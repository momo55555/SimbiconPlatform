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
#include "GuTriangleMesh.h"
#include "PsFoundation.h"
#include "Serialize.h"
#include "GuMeshFactory.h"
#include "CmRenderOutput.h"
#include "PxVisualizationParameter.h"
#include "GuConvexEdgeFlags.h"
#include "GuDebug.h"

// PX_SERIALIZATION
#include "CmSerialFramework.h"

using namespace physx;

bool PxOpcodeError(const char* message, const char* file, unsigned line)
{
	//error hook for opcode
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, file, line, message);
	return false;
}

namespace physx
{

BEGIN_FIELDS(Gu::TriangleMesh)
//	DEFINE_FIELD(TriangleMesh, mesh,	PxField::eSERIAL_EMBEDDED,	0),
END_FIELDS(Gu::TriangleMesh)

// PT: used to be automatic but making it manual saves bytes in the internal mesh

void Gu::TriangleMesh::exportExtraData(PxSerialStream& stream)
{
	mesh.exportExtraData(stream);
}

char* Gu::TriangleMesh::importExtraData(char* address, PxU32& totalPadding)
{
	return mesh.importExtraData(address, totalPadding);
}
//~PX_SERIALIZATION

Gu::TriangleMesh::TriangleMesh()
{
	mesh.mData.mAABB = PxBounds3::empty();
// PX_SERIALIZATION
	setType(PxSerialType::eTRIANGLE_MESH);
//~PX_SERIALIZATION
}

Gu::TriangleMesh::~TriangleMesh()
{
	mesh.release();
}

// PX_SERIALIZATION
void Gu::TriangleMesh::onRefCountZero()
{
	if(mMeshFactory->removeTriangleMesh(*this))
	{
		deleteSerializedObject(this);
		return;
	}

	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Gu::TriangleMesh::release: double deletion detected!");
}
//~PX_SERIALIZATION

bool Gu::TriangleMesh::load(const PxStream& stream)
{
	mesh.release();

	// Import header
	PxU32 version;
	bool mismatch;
	if(!readHeader('M', 'E', 'S', 'H', version, mismatch, stream))
		return false;

	// Check if old (incompatible) mesh format is loaded
	if (version < 1)
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Loading triangle mesh failed: "
			"Deprecated mesh cooking format. Please install and run the mesh converter tool to convert your mesh to the new cooking format.");
		return false;
	}

	// Import serialization flags
	PxU32 serialFlags	= readDword(mismatch, stream);

	// Import misc values
	mesh.mConvexEdgeThreshold	= readFloat(mismatch, stream);

	if (version < 2)
	{
		readDword(mismatch, stream);	// Used to be heightFieldVerticalAxis
		readFloat(mismatch, stream);	// Used to be heightFieldVerticalExtent
	}

	// Import mesh

	PxVec3* verts = mesh.allocateVertices(readDword(mismatch, stream));
	void* tris = mesh.allocateTriangles(readDword(mismatch, stream));

	stream.readBuffer(verts, sizeof(PxVec3)*mesh.getNumVertices());
	if(mismatch)
	{
		for(PxU32 i=0;i<mesh.getNumVertices();i++)
		{
#if defined(PX_WII)
			*(PxU32*)&verts[i].x = flip((PxU32*)&verts[i].x);
			*(PxU32*)&verts[i].y = flip((PxU32*)&verts[i].y);
			*(PxU32*)&verts[i].z = flip((PxU32*)&verts[i].z);
#else
			verts[i].x = flip(&verts[i].x);
			verts[i].y = flip(&verts[i].y);
			verts[i].z = flip(&verts[i].z);
#endif 
		}
	}
	//TODO: stop support for format conversion on load!!
	if(serialFlags & IMSF_8BIT_INDICES)
	{
		if (mesh.has16BitIndices())
		{
			PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
			for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
				*tris16++ = stream.readByte();
		}
		else
		{
			PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
			for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
				*tris32++ = stream.readByte();
		}
	}
	else if(serialFlags & IMSF_16BIT_INDICES)
	{
		PxU16 x;
		if (mesh.has16BitIndices())
		{
			PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
			if (mismatch)
				for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
					x = stream.readWord(), *tris16++ = flip(&x);
			else
				stream.readBuffer(tris16, 3*sizeof(PxU16)*mesh.getNumTriangles());
				//for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
				//	*tris16++ = stream.readWord();
		}
		else
		{
			PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
			if (mismatch)
				for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
					x = stream.readWord(), *tris32++ = flip(&x);
			else
				for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
					*tris32++ = stream.readWord();
		}

	}
	else
	{
		PxU32 x;
		if (mesh.has16BitIndices())
		{
			PxU16* tris16 = reinterpret_cast<PxU16*>(tris);
			if (mismatch)
				for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
					{ x = stream.readDword(); PX_ASSERT(x <= 0xffff); *tris16++ = (PxU16)flip(&x); }
			else
				for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
					{ x = stream.readDword(); PX_ASSERT(x <= 0xffff); *tris16++ = (PxU16)x; }
		}
		else
		{
			PxU32* tris32 = reinterpret_cast<PxU32*>(tris);
			if (mismatch)
				for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
					x = stream.readDword(), *tris32++ = flip(&x);
			else
				stream.readBuffer(tris32, 3*sizeof(PxU32)*mesh.getNumTriangles());
				//for(PxU32 i=0;i<3*mesh.getNumTriangles();i++)
				//	*tris32++ = stream.readDword();
		}
	}

	if(serialFlags & IMSF_MATERIALS)
	{
		PxU16* materials = mesh.allocateMaterials();
		stream.readBuffer(materials, sizeof(PxU16)*mesh.getNumTriangles());
		if(mismatch)
		{
			for(PxU32 i=0;i<mesh.getNumTriangles();i++)
				materials[i] = flip(&materials[i]);
		}
	}
	if(serialFlags & IMSF_FACE_REMAP)
	{
		PxU32* remap = mesh.allocateFaceRemap();
/*		stream.readBuffer(remap, sizeof(PxU32)*mesh.getNumTriangles());
		if(mismatch)
			{
			for(PxU32 i=0;i<mesh.getNumTriangles();i++)
				remap[i] = flip(&remap[i]);
			}*/
		readIndices(readDword(mismatch, stream), mesh.getNumTriangles(), remap, stream, mismatch);
	}

	if(version <= 6)
	{
		PxU32 dummy0 = readDword(mismatch, stream);
		PxU32 dummy1 = readDword(mismatch, stream);

		if(dummy0)
		{
			PxU16* convexParts = PX_NEW_TEMP(PxU16)[mesh.getNumTriangles()];
			stream.readBuffer(convexParts, sizeof(PxU16)*mesh.getNumTriangles());
			PX_DELETE_POD(convexParts);
		}
		if(dummy1)
		{
			if(dummy1<256)
			{
				PxU8* flatParts8 = PX_NEW_TEMP(PxU8)[mesh.getNumTriangles()];
				stream.readBuffer(flatParts8, sizeof(PxU8)*mesh.getNumTriangles());
				PX_DELETE_POD(flatParts8);
			}
			else
			{
				PxU16* flatParts16 = PX_NEW_TEMP(PxU16)[mesh.getNumTriangles()];
				stream.readBuffer(flatParts16, sizeof(PxU16)*mesh.getNumTriangles());
				PX_DELETE_POD(flatParts16);
			}
		}
	}

	// Import Opcode model
	{
		PxU32 modelSize = readDword(mismatch, stream);

		if(!mesh.loadOpcodeModel(stream, version))
			return false;
	}

	// Import local bounds
		mesh.mData.mOpcodeModel.mGeomEpsilon	= readFloat(mismatch, stream);
		if(version<4)
		{
			Gu::Sphere localSphere;	// PT: not needed anymore
			localSphere.center.x	= readFloat(mismatch, stream);
			localSphere.center.y	= readFloat(mismatch, stream);
			localSphere.center.z	= readFloat(mismatch, stream);
			localSphere.radius		= readFloat(mismatch, stream);
		}
		mesh.mData.mAABB.minimum.x		= readFloat(mismatch, stream);
		mesh.mData.mAABB.minimum.y		= readFloat(mismatch, stream);
		mesh.mData.mAABB.minimum.z		= readFloat(mismatch, stream);
		mesh.mData.mAABB.maximum.x		= readFloat(mismatch, stream);
		mesh.mData.mAABB.maximum.y		= readFloat(mismatch, stream);
		mesh.mData.mAABB.maximum.z		= readFloat(mismatch, stream);

  	// Import mass info
	if(version<3)
	{
  		PxReal mass  = readFloat(mismatch, stream);
  		if(mass!=-1.0f)
  		{
			float b[9];
  			readFloatBuffer(b, 9, mismatch, stream);
  			readFloatBuffer(b, 3, mismatch, stream);
  		}
	}

	PxU32 nb = readDword(mismatch, stream);
	if(nb)
	{
		PX_ASSERT(nb==mesh.getNumTriangles());
		mesh.mData.mExtraTrigData = PX_NEW(PxU8)[nb];
		// No need to convert those bytes
		stream.readBuffer(mesh.mData.mExtraTrigData, nb*sizeof(PxU8));
	}

	return true;
}

#if 0 // prune before shipping
bool Gu::TriangleMesh::saveToDesc(PxTriangleMeshDesc& desc) const
{
	// First, dump the "simple" mesh part.
	desc.flags						= mesh.internalFlags & ~(PxMeshFlag::e16_BIT_INDICES | PxMeshFlag::eFLIPNORMALS); // Do we want to convert the data instead?
	desc.numVertices				= mesh.getNumVertices();
	desc.numTriangles				= mesh.getNumTriangles();
	desc.points						= mesh.getVertices();
	desc.triangles					= mesh.getTriangles();
	desc.pointStrideBytes			= sizeof(PxVec3);
	desc.triangleStrideBytes		= sizeof(Ps::Triangle32);

	// Now do remaining parts from PxTriangleMeshDesc
	desc.materialIndices			= mesh.getMaterials();
	desc.materialIndexStride		= sizeof(PxU16);

	return true;
}
#endif

void Gu::TriangleMesh::release()
{
	decRefCount();
}

PxU32 Gu::TriangleMesh::getReferenceCount() const
{
	return getRefCount();
}


#if PX_ENABLE_DEBUG_VISUALIZATION

// PT: don't use a template when you don't need one. Don't pollute the header with template definition. Don't duplicate the code when one version is enough. Etc.
static void getTriangle(const Gu::TriangleMesh& mesh, PxU32 i, PxVec3* wp, const PxVec3* vertices, const void* indices, bool has16BitIndices)
{
	PxU32 ref0, ref1, ref2;

	if(!has16BitIndices)
	{
		const PxU32* dtriangles = reinterpret_cast<const PxU32*>(indices);
		ref0 = dtriangles[i*3+0];
		ref1 = dtriangles[i*3+1];
		ref2 = dtriangles[i*3+2];
	}
	else
	{
		const PxU16* wtriangles = reinterpret_cast<const PxU16*>(indices);
		ref0 = wtriangles[i*3+0];
		ref1 = wtriangles[i*3+1];
		ref2 = wtriangles[i*3+2];
	}

	wp[0] = vertices[ref0];
	wp[1] = vertices[ref1];
	wp[2] = vertices[ref2];
}

static void getTriangle(const Gu::TriangleMesh& mesh, PxU32 i, PxVec3* wp, const PxVec3* vertices, const void* indices, const Cm::Matrix34& absPose, bool has16BitIndices)
{
	PxVec3 localVerts[3];
	getTriangle(mesh, i, localVerts, vertices, indices, has16BitIndices);

	wp[0] = absPose.transform(localVerts[0]);
	wp[1] = absPose.transform(localVerts[1]);
	wp[2] = absPose.transform(localVerts[2]);
}

void Gu::TriangleMesh::debugVisualize(
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)	const
{
	bool cscale = !!(mask & ((PxU64)1 << PxVisualizationParameter::eCULL_BOX));

	const PxMat44 midt = PxMat44::createIdentity();

	const PxU32 nbTriangles = mesh.getNumTriangles();
	const PxU32 nbVertices = mesh.getNumVertices();
	const PxVec3* vertices = mesh.getVertices();
	const void* indices = getTrianglesFast();

	const bool has16BitIndices = mesh.has16BitIndices();

	if (fscale)
	{
		const PxU32 fcolor = PxDebugColor::eARGB_DARKRED;

		for (PxU32 i=0; i<nbTriangles; i++)
		{
			PxVec3 wp[3];
			getTriangle(*this, i, wp, vertices, indices, absPose, has16BitIndices);

			const PxVec3 center = (wp[0] + wp[1] + wp[2]) / 3.0f;
			PxVec3 normal = (wp[0] - wp[1]).cross(wp[0] - wp[2]);
			PX_ASSERT(!normal.isZero());
			normal = normal.getNormalized();

			if (!cscale || cullbox.contains(center))
				out << midt << fcolor <<
					Cm::DebugArrow(center, normal * fscale);
		}
	}

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxDebugColor::eARGB_MAGENTA;

		out << midt << scolor;	// PT: no need to output this for each segment!

		// PT: transform vertices only once
		PxVec3* transformed = (PxVec3*)PX_ALLOC(sizeof(PxVec3)*nbVertices);
		for(PxU32 i=0;i<nbVertices;i++)
			transformed[i] = absPose.transform(vertices[i]);

		for (PxU32 i=0; i<nbTriangles; i++)
		{
			PxVec3 wp[3];
			getTriangle(*this, i, wp, transformed, indices, has16BitIndices);

			if (!cscale || (cullbox.contains(wp[0]) && cullbox.contains(wp[1]) && cullbox.contains(wp[2])))
			{
				out.outputSegment(wp[0], wp[1]);
				out.outputSegment(wp[1], wp[2]);
				out.outputSegment(wp[2], wp[0]);
			}
		}

		PX_FREE(transformed);
	}

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_EDGES))
	{
		const PxU32 ecolor = PxDebugColor::eARGB_YELLOW;

		for (PxU32 i=0; i<nbTriangles; i++)
		{
			PxVec3 wp[3];
			getTriangle(*this, i, wp, vertices, indices, absPose, has16BitIndices);

			const PxU32 flags = mesh.getTrigSharedEdgeFlags(i);

			if(flags & Gu::ETD_CONVEX_EDGE_01)
			{
				if (!cscale || (cullbox.contains(wp[0]) && cullbox.contains(wp[1])))
					out << midt << ecolor << Cm::RenderOutput::LINES << wp[0] << wp[1];
			}
			if(flags & Gu::ETD_CONVEX_EDGE_12)
			{
				if (!cscale || (cullbox.contains(wp[1]) && cullbox.contains(wp[2])))
					out << midt << ecolor << Cm::RenderOutput::LINES << wp[1] << wp[2];
			}
			if(flags & Gu::ETD_CONVEX_EDGE_20)
			{
				if (!cscale || (cullbox.contains(wp[0]) && cullbox.contains(wp[2])))
					out << midt << ecolor << Cm::RenderOutput::LINES << wp[0] << wp[2];
			}
		}
	}
}

#endif

}
