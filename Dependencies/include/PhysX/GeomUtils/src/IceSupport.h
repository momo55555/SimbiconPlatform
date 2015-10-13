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


// Temp file used to compile various ICE files - don't touch!
#ifndef PX_ICESUPPORT
#define PX_ICESUPPORT

#include "PxVec3.h"
#include "../GeomUtils/Opcode/Ice/IceContainer.h"

namespace physx
{

using namespace Ice;
namespace Gu
{
	class Box;
}

	class Vertices : public Container
	{
		public:
									Vertices()								{}
									Vertices(PxU32 nb, const PxVec3* verts)	{ Add(&verts->x, nb*(sizeof(PxVec3)/sizeof(float)));	}
									~Vertices()								{}

		PX_INLINE	PxU32			GetNbVertices()					const	{ return GetNbEntries()/3;							}
		PX_INLINE	const PxVec3*	GetVertices()					const	{ return (const PxVec3*)GetEntries();				}

					Vertices&		AddVertex(const PxVec3& p)				{ Add(p.x).Add(p.y).Add(p.z);	return *this;		}
					Vertices&		AddVertex(float x, float y, float z)	{ Add(x).Add(y).Add(z);			return *this;		}
	};

	class FIFOStack : public Container
	{
		public:
		//! Constructor
								FIFOStack() : mCurIndex(0)	{}
		//! Destructor
								~FIFOStack()				{}
		// Management
		PX_INLINE	FIFOStack&		Push(PxU32 entry)			{	Add(entry);	return *this;	}
				bool			Pop(PxU32 &entry);
		private:
				PxU32			mCurIndex;			//!< Current index within the container
	};

	//! This minimal interface is used to link one data structure to another in a unified way.
	struct SurfaceInterface
	{
		PX_INLINE SurfaceInterface()
			: mNbVerts	(0),
			mVerts		(NULL),
			mNbFaces	(0),
			mDFaces		(NULL),
			mWFaces		(NULL)
		{}

		PX_INLINE SurfaceInterface(
			PxU32			nb_verts,
			const PxVec3*	verts,
			PxU32			nb_faces,
			const PxU32*	dfaces,
			const PxU16*	wfaces
			)
			: mNbVerts	(nb_verts),
			mVerts		(verts),
			mNbFaces	(nb_faces),
			mDFaces		(dfaces),
			mWFaces		(wfaces)
		{}

		PxU32			mNbVerts;	//!< Number of vertices
		const PxVec3*	mVerts;		//!< List of vertices
		PxU32			mNbFaces;	//!< Number of faces
		const PxU32*	mDFaces;	//!< List of faces (dword indices)
		const PxU16*	mWFaces;	//!< List of faces (word indices)
	};

	void CreateOBB(Gu::Box& dest, const Gu::Box& box, const PxVec3& dir, float d);

}

#endif
	
