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
#ifndef REPX_REPXCOREEXTENSIONS_H
#define REPX_REPXCOREEXTENSIONS_H

namespace physx { 
	namespace shdfnd2 {}
	namespace repx {
	using namespace physx::shdfnd2;
}}
#include "RepXVisitorWriter.h"
#include "RepX.h"
#include "PxRigidStatic.h"
#include "PxShape.h"
#include "PxBoxGeometry.h"
#include "PxScene.h"
#include "PxMetaDataObjects.h"
#include "PxStream.h"
#include "PxStreamOperators.h"
#include "PxConvexMeshGeometry.h"
#include "PxSphereGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxHeightFieldGeometry.h"
#include "PxTriangleMesh.h"
#include "PxCooking.h"
#include "PxHeightField.h"
#include "RepXMemoryAllocator.h"
#include "PxConvexMesh.h"
#include "PxArticulation.h"
#include "PxArticulationLink.h"
#include "PxArticulationJoint.h"
#include "RepXExtensionImpl.h"
#include "PxHeightFieldDesc.h"
#include "PxArticulationLink.h"
#include "../src/PxProfileFoundationWrapper.h"


namespace physx { namespace repx {
	using namespace physx::profile;

typedef ProfileHashMap< const void*, const PxArticulationLink* > TArticulationLinkLinkMap;
typedef PxReadOnlyPropertyInfo<PxPropertyInfoName::PxArticulationLink_InboundJoint, PxArticulationLink, PxArticulationJoint *> TIncomingJointPropType;

}}

#include "RepXCoreExtensions.h"					//The core extension definition file.
#include "RepXImpl.h"							//Utility functions used by everything
#include "RepXCoreExtensionSerializer.h"		//specializations for writing
#include "RepXCoreExtensionDeserializer.h"		//specializations for reading
//The implementation must be included last as it needs to see all of the serialization
//specializations going on ahead of it when it calls write/readAllProperties.
#include "RepXExtensionImpl.h"					//The implementation of the extensions.



namespace physx { namespace repx {

	//*************************************************************
	//	Actual extension implementations
	//*************************************************************
	struct PxMaterialRepXExtension : public RepXExtensionImpl<PxMaterial>
	{
		PxMaterialRepXExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxMaterial>( inCallback ) {}
		virtual PxMaterial* allocateObject( RepXInstantiationArgs& inArgs )
		{
			return inArgs.mPhysics->createMaterial(0, 0, 0);
		}
	};

	struct PxRigidStaticRepXExtension : public RepXExtensionImpl<PxRigidStatic>
	{
		PxRigidStaticRepXExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxRigidStatic>( inCallback ) {}
		virtual PxRigidStatic* allocateObject( RepXInstantiationArgs& inArgs )
		{
			return inArgs.mPhysics->createRigidStatic( PxTransform::createIdentity() );
		}
	};
	
	struct PxRigidDynamicRepXExtension : public RepXExtensionImpl<PxRigidDynamic>
	{
		PxRigidDynamicRepXExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxRigidDynamic>( inCallback ) {}
		virtual PxRigidDynamic* allocateObject( RepXInstantiationArgs& inArgs )
		{
			return inArgs.mPhysics->createRigidDynamic( PxTransform::createIdentity() );
		}
	};

	template<typename TTriIndexElem>
	inline void writeTriangle( MemoryBuffer& inTempBuffer, const Triangle<TTriIndexElem>& inTriangle )
	{
		inTempBuffer << inTriangle.mIdx0 
			<< " " << inTriangle.mIdx1
			<< " " << inTriangle.mIdx2;
	}


	PxU32 materialAccess( const PxTriangleMesh* inMesh, PxU32 inIndex ) { return inMesh->getTriangleMaterialIndex( inIndex ); }
	template<typename TDataType>
	void writeDatatype( MemoryBuffer& inTempBuffer, const TDataType& inType ) { inTempBuffer << inType; }

	struct PxTriangleMeshExtension  : public RepXExtensionImpl<PxTriangleMesh>
	{
		PxTriangleMeshExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxTriangleMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxTriangleMesh* mesh, RepXIdToRepXObjectMap* inIdMap, RepXWriter& inWriter, MemoryBuffer& inTempBuffer )
		{
			bool hasMatIndex = mesh->getTriangleMaterialIndex(0) != 0xffff;
			PxU32 numVerts = mesh->getNbVertices();
			writeBuffer( inWriter, inTempBuffer, 2, mesh->getVertices(), mesh->getNbVertices(), "Points", writePxVec3 );
			bool isU16 = mesh->has16BitTriangleIndices();
			PxU32 triCount = mesh->getNbTriangles();
			if ( isU16 )
				writeBuffer( inWriter, inTempBuffer, 2, reinterpret_cast<const Triangle<PxU16>* >( mesh->getTriangles() ), triCount, "Triangles", writeTriangle<PxU16> );
			else
				writeBuffer( inWriter, inTempBuffer, 2, reinterpret_cast<const Triangle<PxU32>* >( mesh->getTriangles() ), triCount, "Triangles", writeTriangle<PxU32> );
			if ( hasMatIndex )
				writeBuffer( inWriter, inTempBuffer, 6, mesh, materialAccess, triCount, "materialIndices", writeDatatype<PxU32> );
		}
		virtual RepXObject fileToObject( RepXReader& inReader, RepXMemoryAllocator& inAllocator, RepXInstantiationArgs& inArgs, RepXIdToRepXObjectMap* inIdMap )
		{
			//We can't do a simple inverse; we *have* to cook data to get a mesh.
			PxTriangleMeshDesc theDesc;
			readStridedBufferProperty<PxVec3>( inReader, "points", theDesc.points, inAllocator);
			readStridedBufferProperty<Triangle<PxU32> >( inReader, "triangles", theDesc.triangles, inAllocator);
			PxU32 triCount;
			readStridedBufferProperty<PxMaterialTableIndex>( inReader, "materialIndices", theDesc.materialIndices, triCount, inAllocator);
			//Now cook the bastard.
			TMemoryPoolManager theManager(inAllocator.getAllocator());
			MemoryBuffer theTempBuf( &theManager );
			inArgs.mCooker->cookTriangleMesh( theDesc, theTempBuf );
			PxTriangleMesh* theMesh = inArgs.mPhysics->createTriangleMesh( theTempBuf );
			return createRepXObject( theMesh );
		}
		//We never allow this to be called.
		virtual PxTriangleMesh* allocateObject( RepXInstantiationArgs& inArgs ) { return NULL; }
	};

	struct PxHeightFieldExtension : public RepXExtensionImpl<PxHeightField>
	{
		PxHeightFieldExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxHeightField>( inCallback ) {}
		//Conversion from scene object to descriptor.
		virtual void objectToFileImpl( const PxHeightField* inHeightField, RepXIdToRepXObjectMap* inIdMap, RepXWriter& inWriter, MemoryBuffer& inTempBuffer )
		{
			PxHeightFieldDesc theDesc;

			theDesc.nbRows					= inHeightField->getNbRows();
			theDesc.nbColumns				= inHeightField->getNbColumns();
			theDesc.format					= inHeightField->getFormat();
			theDesc.samples.stride			= inHeightField->getSampleStride();
			theDesc.samples.data			= NULL;
			theDesc.thickness				= inHeightField->getThickness();
			theDesc.convexEdgeThreshold		= inHeightField->getConvexEdgeThreshold();
			theDesc.flags					= inHeightField->getFlags();

			PxU32 theCellCount = inHeightField->getNbRows() * inHeightField->getNbColumns();
			PxU32 theSampleStride = sizeof( PxHeightFieldSample );
			PxU32 theSampleBufSize = theCellCount * theSampleStride;
			PxHeightFieldSample* theSamples = reinterpret_cast< PxHeightFieldSample*> ( inTempBuffer.mManager->allocate( theSampleBufSize ) );
			inHeightField->saveCells( theSamples, theSampleBufSize );
			theDesc.samples.data = theSamples;
			writeAllProperties( &theDesc, inWriter, inTempBuffer, *inIdMap );
			writeStridedBufferProperty<PxHeightFieldSample>( inWriter, inTempBuffer, "samples", theDesc.samples, theDesc.nbRows * theDesc.nbColumns, 6, writeHeightFieldSample);
			inTempBuffer.mManager->deallocate( reinterpret_cast<PxU8*>(theSamples) );
		}

		virtual RepXObject fileToObject( RepXReader& inReader, RepXMemoryAllocator& inAllocator, RepXInstantiationArgs& inArgs, RepXIdToRepXObjectMap* inIdMap )
		{
			PxHeightFieldDesc theDesc;
			readAllProperties( inArgs, inReader, &theDesc, inAllocator, *inIdMap );
			//Now read the data...
			PxU32 count = 0; //ignored becaues numRows and numColumns tells the story
			readStridedBufferProperty<PxHeightFieldSample>( inReader, "samples", theDesc.samples, count, inAllocator);
			PxHeightField* retval = inArgs.mPhysics->createHeightField( theDesc );
			return createRepXObject( retval );
		}

		virtual PxHeightField* allocateObject( RepXInstantiationArgs& inArgs ) { return NULL; }
	};

	struct PxConvexMeshExtension  : public RepXExtensionImpl<PxConvexMesh>
	{
		PxConvexMeshExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxConvexMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxConvexMesh* mesh, RepXIdToRepXObjectMap* inIdMap, RepXWriter& inWriter, MemoryBuffer& inTempBuffer )
		{
			writeBuffer( inWriter, inTempBuffer, 2, mesh->getVertices(), mesh->getNbVertices(), "points", writePxVec3 );
		}

		//Conversion from scene object to descriptor.
		virtual RepXObject fileToObject( RepXReader& inReader, RepXMemoryAllocator& inAllocator, RepXInstantiationArgs& inArgs, RepXIdToRepXObjectMap* inIdMap )
		{
			PxConvexMeshDesc theDesc;
			readStridedBufferProperty<PxVec3>( inReader, "points", theDesc.points, inAllocator);
			theDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
			TMemoryPoolManager theManager(inAllocator.getAllocator());
			MemoryBuffer theTempBuf( &theManager );
			inArgs.mCooker->cookConvexMesh( theDesc, theTempBuf );
			PxConvexMesh* theMesh = inArgs.mPhysics->createConvexMesh( theTempBuf );
			return createRepXObject( theMesh );
		}

		virtual PxConvexMesh* allocateObject( RepXInstantiationArgs& inArgs ) { return NULL; }
	};

	void writeFabricPhaseType( PxStream& stream, const PxU32& phaseType )
	{
		const PxU32ToName* conversion = PxEnumTraits<PxClothFabricPhaseType::Enum>().NameConversion;
		for ( const PxU32ToName* conv = conversion; conv->mName != NULL; ++conv )
			if ( conv->mValue == phaseType ) stream << conv->mName;
	}

	template<> struct StrToImpl<PxClothFabricPhaseType::Enum> {
	void strto( PxClothFabricPhaseType::Enum& datatype, char*& ioData )
	{
		const PxU32ToName* conversion = PxEnumTraits<PxClothFabricPhaseType::Enum>().NameConversion;
		eatwhite( ioData );
		char* target = ioData;
		nullTerminateWhite( ioData );
		for ( const PxU32ToName* conv = conversion; conv->mName != NULL; ++conv )
			if ( physx::string::stricmp( target, conv->mName ) )
			{
				datatype = static_cast<PxClothFabricPhaseType::Enum>( conv->mValue );
				return;
			}
	}
	};

	struct PxClothFabricExtension : public RepXExtensionImpl<PxClothFabric>
	{
		PxClothFabricExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxClothFabric>( inCallback ) {}
		virtual void objectToFileImpl( const PxClothFabric* data, RepXIdToRepXObjectMap* inIdMap, RepXWriter& inWriter, MemoryBuffer& inTempBuffer )
		{
			
			FoundationWrapper&				wrapper( inTempBuffer.mManager->getWrapper() );
			PxU32 numPhases = data->getNbPhases();
			PxU32 numFibers = data->getNbFibers();
			PxU32 numIndices = data->getNbParticleIndices();
			PxU32 numRestLengths = data->getNbParticleIndices() - (data->getNbFibers() - 1);

			ProfileArray<PxU8> dataBuffer( wrapper );
			dataBuffer.resize( PxMax( PxMax( PxMax( numPhases, numFibers ), numRestLengths), numIndices ) * sizeof( PxU32 ) );
			PxU32* indexPtr( reinterpret_cast<PxU32*>( dataBuffer.begin() ) );
			PxU32 numParticles = 0;
			data->getParticleIndices( indexPtr, numIndices );
			for ( PxU32 idx = 0; idx < numIndices; ++idx ) numParticles = PxMax( numParticles, indexPtr[idx] + 1 );


			writeProperty( inWriter, *inIdMap, inTempBuffer, "NbParticles", numParticles );
			writeBuffer( inWriter, inTempBuffer, 18, indexPtr, PtrAccess<PxU32>, numIndices, "ParticleIndices", BasicDatatypeWrite<PxU32> );

			data->getPhases( indexPtr, numPhases );
			writeBuffer( inWriter, inTempBuffer, 18, indexPtr, PtrAccess<PxU32>, numPhases, "Phases", BasicDatatypeWrite<PxU32> );

			PX_COMPILE_TIME_ASSERT( sizeof( PxClothFabricPhaseType::Enum ) == sizeof( PxU32 ) );
			for ( PxU32 idx = 0; idx < numPhases; ++idx )
				indexPtr[idx] = static_cast<PxU32>( data->getPhaseType( idx ) );
			writeBuffer( inWriter, inTempBuffer, 18, indexPtr, PtrAccess<PxU32>, numPhases, "PhaseTypes", writeFabricPhaseType );

			data->getFibers( indexPtr, numFibers );
			writeBuffer( inWriter, inTempBuffer, 18, indexPtr, PtrAccess<PxU32>, numFibers, "Fibers", BasicDatatypeWrite<PxU32> );

			PX_COMPILE_TIME_ASSERT( sizeof( PxReal ) == sizeof( PxU32 ) );

			PxReal* realPtr = reinterpret_cast< PxReal* >( indexPtr );
			data->getRestlengths( realPtr, numRestLengths );
			writeBuffer( inWriter, inTempBuffer, 18, realPtr, PtrAccess<PxReal>, numFibers, "Restlengths", BasicDatatypeWrite<PxReal> );
		}

		//Conversion from scene object to descriptor.
		virtual RepXObject fileToObject( RepXReader& inReader, RepXMemoryAllocator& inAllocator, RepXInstantiationArgs& inArgs, RepXIdToRepXObjectMap* inIdMap )
		{
			PxU32 numParticles;
			readProperty( inReader, "NbParticles", numParticles );
			PxU32 strideIgnored;
			PxU32 numParticleIndices = 0;
			PxU32 numPhaseTypes = 0;
			PxU32 numFibers = 0;
			PxU32 numPhases = 0;
			PxU32 numRestLengths = 0;
			void* particleIndices = NULL;
			void* phaseTypes = NULL;
			void* fibers = NULL;
			void* phases = NULL;
			void* restLengths = NULL;
			readStridedBufferProperty<PxU32>( inReader, "ParticleIndices", particleIndices, strideIgnored, numParticleIndices, inAllocator );
			readStridedBufferProperty<PxU32>( inReader, "Phases", phases, strideIgnored, numPhases, inAllocator );
			readStridedBufferProperty<PxClothFabricPhaseType::Enum>( inReader, "PhaseTypes", phaseTypes, strideIgnored, numPhaseTypes, inAllocator );
			readStridedBufferProperty<PxU32>( inReader, "Fibers", fibers, strideIgnored, numFibers, inAllocator );
			readStridedBufferProperty<PxF32>( inReader, "Restlengths", restLengths, strideIgnored, numRestLengths, inAllocator );
			PxClothFabric* newFabric = inArgs.mPhysics->createClothFabric( numParticles
															, reinterpret_cast<PxU32*>( phases )
															, reinterpret_cast< PxClothFabricPhaseType::Enum* >( phaseTypes )
															, numPhases
															, reinterpret_cast<PxU32*>( fibers )
															, numFibers
															, reinterpret_cast<PxU32*>( particleIndices )
															, numParticleIndices
															, reinterpret_cast<PxReal*>( restLengths ) );
			PX_ASSERT( newFabric );
			return createRepXObject( newFabric );
		}

		virtual PxClothFabric* allocateObject( RepXInstantiationArgs& inArgs ) { return NULL; }	
	};
	
	void clothParticleWriter( PxStream& stream, const PxClothParticle& particle )
	{
		stream << particle.pos;
		stream << " ";
		stream << particle.invWeight;
	}

	template<> struct StrToImpl<PxClothParticle> {
	void strto( PxClothParticle& datatype, char*& ioData )
	{
		StrToImpl<PxF32>().strto( datatype.pos[0], ioData );
		StrToImpl<PxF32>().strto( datatype.pos[1], ioData );
		StrToImpl<PxF32>().strto( datatype.pos[2], ioData );
		StrToImpl<PxF32>().strto( datatype.invWeight, ioData );
	}
	};

	void clothSphereWriter( PxStream& stream, const PxClothCollisionSphere& particle )
	{
		stream << particle.pos;
		stream << " ";
		stream << particle.radius;
	}

	
	template<> struct StrToImpl<PxClothCollisionSphere> {
	void strto( PxClothCollisionSphere& datatype, char*& ioData )
	{
		StrToImpl<PxF32>().strto( datatype.pos[0], ioData );
		StrToImpl<PxF32>().strto( datatype.pos[1], ioData );
		StrToImpl<PxF32>().strto( datatype.pos[2], ioData );
		StrToImpl<PxF32>().strto( datatype.radius, ioData );
	}
	};

	struct PxClothExtension : public RepXExtensionImpl<PxCloth>
	{
		PxClothExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxCloth>( inCallback ) {}
		//virtual PxCloth*			createCloth(const PxTransform& globalPose, PxClothFabric& fabric, const PxClothParticle* particles, const PxClothCollisionData& collData, PxClothFlags flags) = 0;
		virtual void objectToFileImpl( const PxCloth* data, RepXIdToRepXObjectMap* inIdMap, RepXWriter& inWriter, MemoryBuffer& inTempBuffer )
		{
			PxClothReadData* readData( const_cast<PxCloth*>( data )->lockClothReadData() );
			writeBuffer( inWriter, inTempBuffer, 4, readData->particles, PtrAccess<PxClothParticle>, data->getNbParticles(), "Particles", clothParticleWriter );
			readData->unlock();

			writeReference( inWriter, *inIdMap, "Fabric", data->getFabric() );

			PxClothFlags clothFlags( data->getClothFlags() );
			FoundationWrapper& wrapper( inTempBuffer.mManager->getWrapper() );
			ProfileArray<PxU8> dataBuffer( wrapper );	
			PxU32 numSpheres = data->getNbCollisionSpheres();
			PxU32 numSpherePairs = data->getNbCollisionSpherePairs();
			dataBuffer.resize( numSpheres * sizeof( PxClothCollisionSphere ) + numSpherePairs * sizeof( PxU32 ) );
			PxClothCollisionSphere* spherePtr = reinterpret_cast<PxClothCollisionSphere*>( dataBuffer.begin() );
			PxU32* pairIndexPtr = reinterpret_cast<PxU32*>( spherePtr + numSpheres );
			data->getCollisionData( spherePtr, pairIndexPtr );
			writeBuffer( inWriter, inTempBuffer, 4, spherePtr, PtrAccess<PxClothCollisionSphere>, numSpheres, "CollisionSpheres", clothSphereWriter );
			writeBuffer( inWriter, inTempBuffer, 18, pairIndexPtr, PtrAccess<PxU32>, numSpheres, "CollisionSphereIndexes", BasicDatatypeWrite<PxU32> );
			writeFlagsProperty( inWriter, inTempBuffer, "ClothFlags", clothFlags, PxEnumTraits<PxClothFlag::Enum>().NameConversion );	
			PxU32 numVirtualParticles = data->getNbVirtualParticles();
			PxU32 numWeightTableEntries = data->getNbVirtualParticleWeights();
			PxU32 totalNeeded = static_cast<PxU32>( PxMax( numWeightTableEntries * sizeof( PxVec3 ), numVirtualParticles * sizeof( PxU32 ) ) );
			if ( dataBuffer.size() < totalNeeded )
				dataBuffer.resize( totalNeeded );
			PxVec3* weightTableEntries = reinterpret_cast<PxVec3*>( dataBuffer.begin() );
			data->getVirtualParticleWeights( weightTableEntries );
			writeBuffer( inWriter, inTempBuffer, 6, weightTableEntries, PtrAccess<PxVec3>, numWeightTableEntries, "VirtualParticleWeightTableEntries", BasicDatatypeWrite<PxVec3> );
			PxU32* virtualParticles = reinterpret_cast<PxU32*>( dataBuffer.begin() );
			data->getVirtualParticles( virtualParticles );
			writeBuffer( inWriter, inTempBuffer, 18, virtualParticles, PtrAccess<PxU32>, numVirtualParticles, "VirtualParticles", BasicDatatypeWrite<PxU32> );
			//ug.  Now write the rest of the object data that the meta data generator got.
			writeAllProperties( data, inWriter, inTempBuffer, *inIdMap );
		}
		
		virtual RepXObject fileToObject( RepXReader& inReader, RepXMemoryAllocator& inAllocator, RepXInstantiationArgs& inArgs, RepXIdToRepXObjectMap* inIdMap )
		{
			PxU32 strideIgnored;
			PxU32 numParticles;
			void* particles = NULL;
			PxU32 numCollisionSpheres;
			void* collisionSpheres = NULL;
			PxU32 numCollisionSphereIndexes;
			void* collisionSphereIndexes = NULL;
			PxU32 numVirtualParticleWeightTableEntries;
			void* virtualParticleWeightTableEntries = NULL;
			PxU32 numVirtualParticles;
			void* virtualParticles = NULL;
			PxClothFlags flags;
			PxClothFabric* fabric = readReference<PxClothFabric>( inReader, *inIdMap, "Fabric" );
			readStridedBufferProperty<PxClothParticle>( inReader, "Particles", particles, strideIgnored, numParticles, inAllocator );
			readStridedBufferProperty<PxClothCollisionSphere>( inReader, "CollisionSpheres", collisionSpheres, strideIgnored, numCollisionSpheres, inAllocator );
			readStridedBufferProperty<PxU32>( inReader, "CollisionSphereIndexes", collisionSphereIndexes, strideIgnored, numCollisionSphereIndexes, inAllocator );
			readStridedBufferProperty<PxVec3>( inReader, "VirtualParticleWeightTableEntries", virtualParticleWeightTableEntries, strideIgnored, numVirtualParticleWeightTableEntries, inAllocator );
			readStridedBufferProperty<PxU32>( inReader, "VirtualParticles", virtualParticles, strideIgnored, numVirtualParticles, inAllocator );
			readFlagsProperty( inReader, inAllocator, "ClothFlags", PxEnumTraits<PxClothFlag::Enum>().NameConversion, flags );
			PxTransform initialPose( PxTransform::createIdentity() );
			if ( fabric != NULL )
			{
				PxClothCollisionData theData;
				theData.numPairs = numCollisionSphereIndexes;
				theData.numSpheres = numCollisionSpheres;
				theData.spheres = reinterpret_cast<PxClothCollisionSphere*>( collisionSpheres );
				theData.pairIndexBuffer = reinterpret_cast<PxU32*>( collisionSphereIndexes );

				PxCloth* cloth = inArgs.mPhysics->createCloth( initialPose, *fabric, reinterpret_cast<PxClothParticle*>( particles ), theData, flags );
				readAllProperties( inArgs, inReader, cloth, inAllocator, *inIdMap );

				if ( numVirtualParticles && numVirtualParticleWeightTableEntries )
				{
					cloth->setVirtualParticles( numVirtualParticles, reinterpret_cast<PxU32*>( virtualParticles )
												, numVirtualParticleWeightTableEntries, reinterpret_cast<PxVec3*>( virtualParticleWeightTableEntries ) );
				}
				return createRepXObject( cloth );
			}
			return RepXObject();
		}

		virtual PxCloth* allocateObject( RepXInstantiationArgs& inArgs ) { return NULL; }	
	};
	
	struct PxArticulationExtension  : public RepXExtensionImpl<PxArticulation>
	{
		PxArticulationExtension( PxAllocatorCallback& inCallback ) : RepXExtensionImpl<PxArticulation>( inCallback ) {}
		virtual PxArticulation* allocateObject( RepXInstantiationArgs& inArgs ) { return inArgs.mPhysics->createArticulation(); }
		virtual void objectToFileImpl( const PxArticulation* inObj, RepXIdToRepXObjectMap* inIdMap, RepXWriter& inWriter, MemoryBuffer& inTempBuffer )
		{
			TNameStack nameStack( inTempBuffer.mManager->mWrapper );
			TArticulationLinkLinkMap linkMap( inTempBuffer.mManager->mWrapper );
			RepXVisitorWriter<PxArticulation> writer( nameStack, inWriter, inObj, inTempBuffer, *inIdMap, &linkMap );
			RepXPropertyFilter<RepXVisitorWriter<PxArticulation> > theOp( writer );
			visitAllProperties<PxArticulation>( theOp );
		}
	};

	template<typename TObjType>
	struct SpecificExtensionAllocator : ExtensionAllocator
	{
		static RepXExtension* specific_allocator(PxAllocatorCallback& inCallback) 
		{ 
			return PX_PROFILE_NEW( inCallback, TObjType )(inCallback); 
		}
		SpecificExtensionAllocator() : ExtensionAllocator( specific_allocator ) {}
	};

	static ExtensionAllocator gAllocators[] = 
	{
		SpecificExtensionAllocator<PxMaterialRepXExtension>(),
		SpecificExtensionAllocator<PxRigidStaticRepXExtension>(),
		SpecificExtensionAllocator<PxRigidDynamicRepXExtension>(),
		SpecificExtensionAllocator<PxTriangleMeshExtension>(),
		SpecificExtensionAllocator<PxHeightFieldExtension>(),
		SpecificExtensionAllocator<PxConvexMeshExtension>(),
		SpecificExtensionAllocator<PxArticulationExtension>(),
		SpecificExtensionAllocator<PxClothFabricExtension>(),
		SpecificExtensionAllocator<PxClothExtension>(),
	};
	
	static PxU32 gAllocatorCount = sizeof( gAllocators ) / sizeof( *gAllocators );
	
	PxU32 getNumCoreExtensions() { return gAllocatorCount; }
	PxU32 createCoreExtensions( RepXExtension** outExtensions, PxU32 outBufferSize, PxAllocatorCallback& inCallback )
	{
		PxU32 extCount = PxMin( outBufferSize, gAllocatorCount );
		for ( PxU32 idx =0; idx < extCount; ++idx )
			outExtensions[idx] = gAllocators[idx].allocateExtension(inCallback);
		return extCount;
	}
} }
#endif
