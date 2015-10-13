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


#ifndef PX_COLLISION_CONVEXMESH
#define PX_COLLISION_CONVEXMESH

#include "PxConvexMesh.h"
#include "CmPhysXCommon.h"
#include "PsUserAllocated.h"
#include "CmRefCountable.h"
#include "ConvexHull.h"
#include "PxBitAndData.h"
#include "GuConvexMeshData.h"

// PX_SERIALIZATION
#include "CmReflection.h"
//~PX_SERIALIZATION

#include "CmMetaData.h"
#include "CmRenderOutput.h"

namespace physx
{

class BigConvexData;
class GuMeshFactory;

namespace Cm
{
	class Matrix34;
}

namespace Gu
{
	struct HullPolygonData;

	PX_INLINE PxU32 computeBufferSize(const Gu::ConvexHullData& data, PxU32 nb)
	{
		PxU32 bytesNeeded = sizeof(Gu::HullPolygonData) * data.mNbPolygons;
		bytesNeeded += sizeof(PxVec3) * data.mNbHullVertices;
		bytesNeeded += sizeof(PxU8) * data.mNbEdges * 2;		// mFacesByEdges8
		bytesNeeded += sizeof(PxU8) * nb;						// mVertexData8

		//4 align the whole thing!
		const PxU32 mod = bytesNeeded % sizeof(PxReal);
		if (mod)
			bytesNeeded += sizeof(PxReal) - mod;
		return bytesNeeded;
	}

	// 0: includes raycast map
	// 1: discarded raycast map
	// 2: support map not always there
	// 3: support stackless trees for non-recursive collision queries
	// 4: no more opcode model
	// 5: valencies table and gauss map combined, only exported over a vertex count treshold that depends on the platform cooked for.
	// 6: removed support for edgeData16.
	// 7: removed support for edge8Data.
	// 8: removed support for triangles.

	// 9: removed local sphere.
	//10: removed geometric center.
	//11: removed mFlags, and mERef16 from Poly; nbVerts is just a byte.
	//12: removed explicit minimum, maximum from Poly
	//13: internal objects
#define  PX_CONVEX_VERSION 13

	class ConvexMesh : public PxConvexMesh, public Ps::UserAllocated, public Cm::RefCountable
	{
	public:
	// PX_SERIALIZATION
													ConvexMesh(PxRefResolver& v)	: PxConvexMesh(v), Cm::RefCountable(v)
													{
														mNb.setBit();
													}
													DECLARE_SERIAL_CLASS(Gu::ConvexMesh, PxConvexMesh)
		virtual			void						exportExtraData(PxSerialStream& stream);
		virtual			char*						importExtraData(char* address, PxU32& totalPadding);
		virtual			void						onRefCountZero();
		virtual			PxU32						getOrder()											const	{ return PxSerialOrder::eCONVEX;	}
		PX_FORCE_INLINE	void						setMeshFactory(GuMeshFactory* f)							{ mMeshFactory = f;					}
		static			void						getMetaData(PxSerialStream& stream);
	//~PX_SERIALIZATION
													ConvexMesh();

						bool						load(const PxStream& stream);

		// PxConvexMesh										
		virtual			void						release();
		virtual			PxU32						getNbVertices()										const	{ return chull.mNbHullVertices;		}
		virtual			const PxVec3*				getVertices()										const	{ return chull.getHullVertices();	}
		virtual			const PxU8*					getIndexBuffer()									const	{ return chull.getVertexData8();	}
		virtual			PxU32						getNbPolygons()										const	{ return chull.mNbPolygons;			}
		virtual			bool						getPolygonData(PxU32 i, PxHullPolygon& data)		const;
		virtual			PxU32						getReferenceCount()									const;
		virtual			void						getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass)	const;
		//~PxConvexMesh

		PX_FORCE_INLINE	PxU32						getNbVerts()										const	{ return chull.mNbHullVertices;		}
		PX_FORCE_INLINE	const PxVec3*				getVerts()											const	{ return chull.getHullVertices();	}
		PX_FORCE_INLINE	PxU32						getNbPolygonsFast()									const	{ return chull.mNbPolygons;			}
		PX_FORCE_INLINE	const HullPolygonData&		getPolygon(PxU32 i)									const	{ return chull.mPolygons[i];		}
		PX_FORCE_INLINE	const HullPolygonData*		getPolygons()										const	{ return chull.mPolygons;			}
		PX_FORCE_INLINE	PxU32						getNbEdges()										const	{ return chull.mNbEdges;			}

		PX_FORCE_INLINE	const ConvexHullData&		getHull()											const	{ return chull;						}
		PX_FORCE_INLINE	ConvexHullData&				getHull()													{ return chull;						}
		PX_FORCE_INLINE	const PxBounds3&			getLocalBounds()									const	{ return chull.mAABB;				}
		PX_FORCE_INLINE	PxReal						getMass()											const	{ return mass;						}
		PX_FORCE_INLINE	const PxMat33&				getInertia()										const	{ return inertia;					}

		PX_FORCE_INLINE BigConvexData*				getBigConvexData()									const	{ return bigConvexData;				}
		PX_FORCE_INLINE void						setBigConvexData(BigConvexData* bcd)						{ bigConvexData = bcd;				}

		PX_FORCE_INLINE	PxU32						getBufferSize()										const	{ return computeBufferSize(chull, getNb());	}

	protected:
						ConvexHullData				chull;
						PxBitAndDword				mNb;	// ### PT: added for serialization. Try to remove later?

						BigConvexData*				bigConvexData;		//!< optional, only for large meshes! PT: redundant with ptr in chull data? Could also be end of other buffer
						PxReal						mass;				//this is mass assuming a unit density that can be scaled by instances!
						PxMat33						inertia;			//in local space of mesh!
	protected:
		// only accessible via ref count
		virtual										~ConvexMesh();

private:
						GuMeshFactory*				mMeshFactory;	// PT: changed to pointer for serialization

		PX_FORCE_INLINE	PxU32						getNb()												const	{ return mNb;						}
		PX_FORCE_INLINE	PxU32						ownsMemory()										const	{ return !mNb.isBitSet();			}


#if PX_ENABLE_DEBUG_VISUALIZATION
public:
	/**
	\brief Perform convex mesh geometry debug visualization

	\param[out] Debug renderer.
	\param[out] World position.
	*/
	void					debugVisualize(Cm::RenderOutput& out, const Cm::Matrix34& absPose, const PxBounds3& cullbox,
											const PxU64 mask, const PxReal fscale)	const;

#endif
	};

} // namespace Gu

}

#endif
