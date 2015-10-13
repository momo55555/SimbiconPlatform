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


#include "GuDebug.h"
#include "GuHeightField.h"
#include "GuHeightFieldUtil.h"
#include "GuTriangleMesh.h"
#include "GuConvexMesh.h"
#include "PsMathUtils.h"
#include "PsIntrinsics.h"
#include "PxVisualizationParameter.h"
#include "PxBoxGeometry.h"
#include "PxSphereGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxTriangleMeshGeometry.h"
#include "PxHeightFieldGeometry.h"

using namespace physx;

#if PX_ENABLE_DEBUG_VISUALIZATION

// PT: TODO: figure out why we have both PxMat44 and Cm::Matrix34, and keep only one...
// We can't support Cm::Matrix34 directly in the RenderOutput object, since "Cm" is internal stuff only
PxMat44 Gu::Debug::convertToPxMat44(const Cm::Matrix34& absPose)
{
	PxMat44 m44;
	m44.column0 = PxVec4(absPose.base0.x, absPose.base0.y, absPose.base0.z, 0.0f);
	m44.column1 = PxVec4(absPose.base1.x, absPose.base1.y, absPose.base1.z, 0.0f);
	m44.column2 = PxVec4(absPose.base2.x, absPose.base2.y, absPose.base2.z, 0.0f);
	m44.column3 = PxVec4(absPose.base3.x, absPose.base3.y, absPose.base3.z, 0.0f);
	return m44;
}

PxMat44 Gu::Debug::convertToPxMat44(const PxTransform& tr)
{
	return convertToPxMat44(Cm::Matrix34(tr));
}

void Gu::Debug::visualize(const PxSphereGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	if ((mask & ((PxU64)1 << PxVisualizationParameter::eCULL_BOX)) &&
		!cullbox.intersects(PxBounds3(absPose.base3 - PxVec3(geometry.radius),
		absPose.base3 + PxVec3(geometry.radius)))) return;

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxU32(PxDebugColor::eARGB_MAGENTA);

		const PxMat44 m44 = Gu::Debug::convertToPxMat44(absPose);

		out << scolor << m44 << Cm::DebugCircle(100, geometry.radius);

		PxMat44 rotPose = m44;
		Ps::swap(rotPose.column1, rotPose.column2);
		rotPose.column1 = -rotPose.column1;
		out << scolor << rotPose << Cm::DebugCircle(100, geometry.radius);

		Ps::swap(rotPose.column0, rotPose.column2);
		rotPose.column0 = -rotPose.column0;
		out << scolor << rotPose << Cm::DebugCircle(100, geometry.radius);
	}
}

void Gu::Debug::visualize(const PxPlaneGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxU32(PxDebugColor::eARGB_MAGENTA);

		const PxMat44 m44 = Gu::Debug::convertToPxMat44(absPose);

//		const PxReal radius = 10.0f;

//		out << scolor << m44 << Ps::DebugCircle(100, radius);

		PxMat44 rotPose = m44;
		Ps::swap(rotPose.column1, rotPose.column2);
		rotPose.column1 = -rotPose.column1;
//		out << scolor << rotPose << Ps::DebugCircle(100, radius);

		Ps::swap(rotPose.column0, rotPose.column2);
		rotPose.column0 = -rotPose.column0;
		for(PxReal radius = 2.0f; radius < 20.0f ; radius += 2.0f)
			out << scolor << rotPose << Cm::DebugCircle(100, radius*radius);
	}
}

void Gu::Debug::visualize(const PxCapsuleGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	if (mask & ((PxU64)1 << PxVisualizationParameter::eCULL_BOX))
	{
		PxVec3 left = absPose.base3;
		const PxVec3 vleft(-geometry.halfHeight - geometry.radius, 0.0f, 0.0f);
		left += vleft;
		PxVec3 right = absPose.base3;
		const PxVec3 vright( geometry.halfHeight + geometry.radius, 0.0f, 0.0f);
		right += vright;
		if (!cullbox.intersects(PxBounds3(left, right)))
			return;
	}

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxU32(PxDebugColor::eARGB_MAGENTA);

		const PxMat44 m44 = Gu::Debug::convertToPxMat44(absPose);

		const PxVec3 vleft2(-geometry.halfHeight, 0.0f, 0.0f);
		PxMat44 left2 = m44;
		left2.column3 += PxVec4(left2.rotate(vleft2), 0.0f);
		out << scolor << left2 << Cm::DebugArc(100, geometry.radius, PxPi, PxTwoPi);

		PxMat44 rotPose = left2;
		Ps::swap(rotPose.column1, rotPose.column2);
		rotPose.column1 = -rotPose.column1;
		out << scolor << rotPose << Cm::DebugArc(100, geometry.radius, PxPi, PxTwoPi);

		Ps::swap(rotPose.column0, rotPose.column2);
		rotPose.column0 = -rotPose.column0;
		out << scolor << rotPose << Cm::DebugCircle(100, geometry.radius);

		const PxVec3 vright2(geometry.halfHeight, 0.0f, 0.0f);
		PxMat44 right2 = m44;
		right2.column3 += PxVec4(right2.rotate(vright2), 0.0f);
		out << scolor << right2 << Cm::DebugArc(100, geometry.radius, 0.0f, PxPi);

		rotPose = right2;
		Ps::swap(rotPose.column1, rotPose.column2);
		rotPose.column1 = -rotPose.column1;
		out << scolor << rotPose << Cm::DebugArc(100, geometry.radius, 0.0f, PxPi);

		Ps::swap(rotPose.column0, rotPose.column2);
		rotPose.column0 = -rotPose.column0;
		out << scolor << rotPose << Cm::DebugCircle(100, geometry.radius);

		out << m44 << scolor;	// PT: no need to output this for each segment!
		out.outputSegment(	m44.transform(PxVec3(-geometry.halfHeight,  geometry.radius, 0)),
							m44.transform(PxVec3( geometry.halfHeight,  geometry.radius, 0)));
		out.outputSegment(	m44.transform(PxVec3(-geometry.halfHeight, -geometry.radius, 0)),
							m44.transform(PxVec3( geometry.halfHeight, -geometry.radius, 0)));
		out.outputSegment(	m44.transform(PxVec3(-geometry.halfHeight,  0, geometry.radius)),
							m44.transform(PxVec3( geometry.halfHeight,  0, geometry.radius)));
		out.outputSegment(	m44.transform(PxVec3(-geometry.halfHeight, 0, -geometry.radius)),
							m44.transform(PxVec3( geometry.halfHeight, 0, -geometry.radius)));
	}
}

void Gu::Debug::visualize(const PxBoxGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	if ((mask & ((PxU64)1 << PxVisualizationParameter::eCULL_BOX)) &&
		!cullbox.intersects(PxBounds3(absPose.base3, geometry.halfExtents)))
		return;

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxU32(PxDebugColor::eARGB_MAGENTA);
		const PxMat44 m44 = Gu::Debug::convertToPxMat44(absPose);
		out << scolor << m44 << Cm::DebugBox(geometry.halfExtents);
	}
}

void Gu::Debug::visualize(const PxConvexMeshGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	(static_cast<const Gu::ConvexMesh*>(geometry.convexMesh))->debugVisualize(out, absPose, cullbox, mask, fscale);
}

void Gu::Debug::visualize(const PxTriangleMeshGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	(static_cast<const Gu::TriangleMesh*>(geometry.triangleMesh))->debugVisualize(out, absPose, cullbox, mask, fscale);
}

void Gu::Debug::visualize(const PxHeightFieldGeometry& geometry,
	Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
	const PxU64 mask, const PxReal fscale)
{
	const Gu::HeightField* heightfield = static_cast<const Gu::HeightField*>(geometry.heightField);
	const bool cscale = !!(mask & ((PxU64)1 << PxVisualizationParameter::eCULL_BOX));

	// PT: TODO: the debug viz for HFs is minimal at the moment...

	if (mask & ((PxU64)1 << PxVisualizationParameter::eCOLLISION_SHAPES))
	{
		const PxU32 scolor = PxDebugColor::eARGB_YELLOW;
		const PxMat44 midt = PxMat44::createIdentity();

		HeightFieldUtil hfUtil(geometry);

		const PxU32 nbRows = heightfield->getNbRowsFast();
		const PxU32 nbColumns = heightfield->getNbColumnsFast();
		const PxU32 nbVerts = nbRows * nbColumns;
		const PxU32 nbTriangles = 2 * nbVerts;

		out << midt << scolor;	// PT: no need to output the same matrix/color for each triangle

		// PT: transform vertices only once
		PxVec3* tmpVerts = (PxVec3*)PX_ALLOC(sizeof(PxVec3)*nbVerts);
		// PT: TODO: optimize the following line
		for(PxU32 i=0;i<nbVerts;i++)
			tmpVerts[i] = absPose.transform(hfUtil.hf2shapep(heightfield->getVertex(i)));

		for(PxU32 i=0; i<nbTriangles; i++)
		{
			// PT: TODO: optimize away the useless divisions/modulos in the lines below
			if(heightfield->isValidTriangle(i) && heightfield->getTriangleMaterial(i) != PxHeightFieldMaterial::eHOLE)
			{
				PxU32 vi0, vi1, vi2;
				heightfield->getTriangleVertexIndices(i, vi0, vi1, vi2);
				const PxVec3& vw0 = tmpVerts[vi0];
				const PxVec3& vw1 = tmpVerts[vi1];
				const PxVec3& vw2 = tmpVerts[vi2];

				if (!cscale || (cullbox.contains(vw0) && cullbox.contains(vw1) && cullbox.contains(vw2)))
				{
					// PT: the operator will multiply everything by the identity matrix
					// and do useless work for each tri. Better call "outputSegment" directly.
//					out << Cm::RenderOutput::LINESTRIP << vw0 << vw1 << vw2 << vw0;
					out.outputSegment(vw0, vw1);
					out.outputSegment(vw1, vw2);
					out.outputSegment(vw2, vw0);
				}
			}
		}
		PX_FREE(tmpVerts);
	}
}

#endif
