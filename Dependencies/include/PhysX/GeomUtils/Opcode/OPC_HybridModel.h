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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef OPC_HYBRIDMODEL_H
#define OPC_HYBRIDMODEL_H

#include "Opcode.h"

#include "PsUserAllocated.h"
#include "OPC_OptimizedTree.h"
#include "OPC_TreeBuilders.h"	// PT: just for build settings! Split header!
#include "OPC_ModelData.h"
#include "RTree.h"
#include "CmMetaData.h"

namespace physx
{
namespace Ice
{
	//! Model creation structure
	struct OPCODECREATE
	{
		//! Constructor
								OPCODECREATE();

		MeshInterface*			mIMesh;			//!< Mesh interface (access to triangles & vertices) (*)
//		const CustomArray*		mModelData;		//!< Previously serialized model data. If not provided,
//												//!< Opcode builds the model using the settings below.
		//
		BuildSettings			mSettings;		//!< Builder's settings

		bool					mKeepOriginal;	//!< true => keep a copy of the original tree (debug purpose)
		bool					mCanRemap;		//!< true => allows OPCODE to reorganize client arrays

		// (*) This pointer is saved internally and used by OPCODE until collision structures are released,
		// so beware of the object's lifetime.
	};

	class HybridModel : public Ps::UserAllocated
	{
		public:
// PX_SERIALIZATION
												HybridModel(PxRefResolver& v) : mRTree(v)
												{
													mNbLeaves |= PX_SIGN_BITMASK;
												}
		static			void					getMetaData(PxSerialStream& stream);
//~PX_SERIALIZATION
		// Constructor/Destructor
												HybridModel();
												~HybridModel();

		PX_FORCE_INLINE	void					getHybridModelData(HybridModelData& data)	const
												{
													data.mIMesh			= mIMesh;
													data.mModelCode		= mModelCode;
													data.mNbLeaves		= GetNbLeaves();
													data.mTriangles		= mTriangles;
													data.mNbPrimitives	= mNbPrimitives;
													data.mIndices		= mIndices;
													data.mRTree			= &mRTree;
												}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Builds a collision model.
		 *	\param		create		[in] model creation structure
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool					Build(const OPCODECREATE& create, AABBStacklessQuantizedNoLeafTree& resultTree);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Loads a collision model.
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool					Build(const MeshInterface& mesh, const PxStream& data);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Serializes the collision model.
		 *	\param		stream		[out] model data
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool					Save(bool mismatch, PxStream& stream)	const;

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Loads a precomputed collision model.
		 *	\param		array		[in] model data
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						bool					Load(const PxStream& stream);

		PX_FORCE_INLINE	const MeshInterface*	GetMeshInterface()						const	{ return mIMesh;		}
		PX_FORCE_INLINE	void					SetMeshInterface(const MeshInterface* imesh)	{ mIMesh = imesh;		}
		PX_FORCE_INLINE	const LeafTriangles*	GetLeafTriangles()						const	{ return mTriangles;	}
		PX_FORCE_INLINE	const PxU32*			GetIndices()							const	{ return mIndices;		}

// PX_SERIALIZATION
						void					exportExtraData(PxSerialStream&);
						char*					importExtraData(char* address, PxU32& totalPadding);
		PX_FORCE_INLINE	void					SetNbLeaves(PxU32 n)							{ mNbLeaves = n;						}
		PX_FORCE_INLINE	PxU32					GetNbLeaves()							const	{ return mNbLeaves & ~PX_SIGN_BITMASK;	}
		PX_FORCE_INLINE	PxU32					isInUserMemory()						const	{ return mNbLeaves & PX_SIGN_BITMASK;	}
//~PX_SERIALIZATION

		private:
						const MeshInterface*	mIMesh;			//!< User-defined mesh interface
						PxU32					mModelCode;		//!< Model code = combination of ModelFlag(s)
						AABBTree*				mSource;		//!< Original source tree
		public:
						PxReal					mGeomEpsilon;	//!< PT: we store this one in the padding bytes. Makes sense to have the geom epsilon here anyway. 
						Gu::RTree				mRTree;
		private:
						PxU32					mNbLeaves;		//!< Number of leaf nodes in the model
						LeafTriangles*			mTriangles;		//!< Array of mNbLeaves leaf descriptors
						PxU32					mNbPrimitives;	//!< Number of primitives in the model
						PxU32*					mIndices;		//!< Array of primitive indices
		public:
						void					Release();
	};

} // namespace Ice

}

#endif // OPC_HYBRIDMODEL_H
