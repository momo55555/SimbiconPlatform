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

#ifndef PVD_PVDRENDERTYPES_H
#define PVD_PVDRENDERTYPES_H

#include "PVDCommLayerTypes.h"

namespace PVD { 

	struct RenderTransformType
	{
		enum Enum
		{
			Unknown = 0, //no further transform
			Identity, //no further information
			Position, //3 floats
			Transform, //7 floats
			Frame, //12 floats
			Last,
		};
		PxU8	mType;
		RenderTransformType( PxU8 inType=Unknown ) : mType( inType ) {}
		PX_INLINE bool operator==( const RenderTransformType& inOther ) const { return mType == inOther.mType; }
		PX_INLINE bool operator!=( const RenderTransformType& inOther ) const { return !(*this == inOther ); }
		PX_INLINE const char* toString() const
		{
			switch( mType )
			{
			case Unknown: return "Unknown"; 
			case Identity: return "Identity";
			case Position: return "Position"; 
			case Transform: return "Transform"; 
			case Frame: return "Frame"; 
			}
			return "";
		}
	};

	struct IdentityTransform
	{
		PxU32 unused;
	};

	PX_INLINE IdentityTransform createIdentityTransform() { IdentityTransform retval = { 0 }; return retval; }
	
	
	template<typename TDataType> PX_INLINE RenderTransformType getRenderTransformType() { return RenderTransformType::Unknown; }
	template<> PX_INLINE RenderTransformType getRenderTransformType<IdentityTransform>() { return RenderTransformType::Identity; }
	template<> PX_INLINE RenderTransformType getRenderTransformType<Position>() { return RenderTransformType::Position; }
	template<> PX_INLINE RenderTransformType getRenderTransformType<PxVec3>() { return RenderTransformType::Position; }
	template<> PX_INLINE RenderTransformType getRenderTransformType<Transform>() { return RenderTransformType::Transform; }
	template<> PX_INLINE RenderTransformType getRenderTransformType<PxTransform>() { return RenderTransformType::Transform; }
	template<> PX_INLINE RenderTransformType getRenderTransformType<Frame>() { return RenderTransformType::Frame; }

	union RenderTransformData
	{
		IdentityTransform mIdentity;
		Position mPosition;
		Transform mTransform;
		Frame mFrame;
	};

	template<typename TDataType> PX_INLINE RenderTransformData toTransformData( const TDataType& inItem ) { PX_ASSERT( false ); return RenderTransformData(); }
	PX_INLINE RenderTransformData toTransformData( const IdentityTransform& inItem ) { RenderTransformData retval; retval.mIdentity = inItem; return retval; }
	PX_INLINE RenderTransformData toTransformData( const Position& inItem ) { RenderTransformData retval; retval.mPosition = inItem; return retval; }
	PX_INLINE RenderTransformData toTransformData( const PxVec3& inItem ) { RenderTransformData retval; retval.mPosition = createPosition( inItem ); return retval; }
	PX_INLINE RenderTransformData toTransformData( const Transform& inItem ) { RenderTransformData retval; retval.mTransform = inItem; return retval; }
	PX_INLINE RenderTransformData toTransformData( const PxTransform& inItem ) { RenderTransformData retval; retval.mTransform = createTransform( inItem ); return retval; }
	PX_INLINE RenderTransformData toTransformData( const Frame& inItem ) { RenderTransformData retval; retval.mFrame = inItem; return retval; }
	PX_INLINE RenderTransformData toTransformData() { RenderTransformData retval; memset( &retval, 0, sizeof( retval ) ); return retval; }
	
	template<typename TDataType> PX_INLINE TDataType fromTransformData( const RenderTransformData& inData ) { PX_ASSERT( false ); return TDataType(); }
	template<> PX_INLINE IdentityTransform fromTransformData( const RenderTransformData& inData ) { return inData.mIdentity; }
	template<> PX_INLINE Position fromTransformData( const RenderTransformData& inData ) { return inData.mPosition; }
	template<> PX_INLINE PxVec3 fromTransformData( const RenderTransformData& inData ) { return inData.mPosition; }
	template<> PX_INLINE Transform fromTransformData( const RenderTransformData& inData ) { return inData.mTransform; }
	template<> PX_INLINE PxTransform fromTransformData( const RenderTransformData& inData ) { return inData.mTransform; }
	template<> PX_INLINE Frame fromTransformData( const RenderTransformData& inData ) { return inData.mFrame; }


	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType visitRenderTransform( RenderTransformType inType, const RenderTransformData& inData, TOperator inOperator )
	{
		switch( inType.mType )
		{
		case RenderTransformType::Identity: return inOperator( fromTransformData<IdentityTransform>( inData ) );
		case RenderTransformType::Position: return inOperator( fromTransformData<Position>( inData ) );
		case RenderTransformType::Transform: return inOperator( fromTransformData<Transform>( inData ) );
		case RenderTransformType::Frame: return inOperator( fromTransformData<Frame>( inData ) );
		default:
			PX_ASSERT( false );
			return inOperator();
		}
	}

	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType visitRenderTransform( RenderTransformType inType, TOperator inOperator ) { return visitRenderTransform<TReturnType, TOperator>( inType, RenderTransformData(), inOperator ); }

	class RenderTransform
	{
		RenderTransformData mData;
		RenderTransformType mType;
	public:
		PX_INLINE RenderTransform(RenderTransformType inType = RenderTransformType() )
			: mType( inType )
		{
		}
		PX_INLINE RenderTransform(RenderTransformType inType, const RenderTransformData& inData ) : mData( inData ), mType( inType ) {}
		template<typename TDataType>
		PX_INLINE RenderTransform( const TDataType& inType )
			: mData( toTransformData( inType ) )
			, mType( getRenderTransformType<TDataType>() )
		{
		}
		PX_INLINE RenderTransform( const RenderTransform& inOther )
			: mData( inOther.mData )
			, mType( inOther.mType )
		{
		}
		PX_INLINE RenderTransform& operator=( const RenderTransform& inOther )
		{
			mData = inOther.mData;
			mType = inOther.mType;
			return *this;
		}
		PX_INLINE RenderTransformData getData() const { return mData; }
		PX_INLINE RenderTransformType getType() const { return mType; }
		template<typename TDataType>
		PX_INLINE TDataType getValue() const { PX_ASSERT( mType == getRenderTransformType<TDataType>() ); return fromTransformData<TDataType>( mData ); }
		template<typename TReturnType, typename TOperator>
		PX_INLINE  TReturnType visit( TOperator inOperator ) const { return visitRenderTransform<TReturnType, TOperator>( mType, mData, inOperator ); }
	};

	
	
	PX_INLINE PxU8 colorClamp( PxF32 inItem )
	{
		inItem = inItem > 1.0f ? 1.0f : inItem;
		inItem = inItem < 0.0f ? 0.0f : inItem;
		return static_cast<PxU8>( inItem * 255.0f + .5f );
	}

	PX_INLINE PxF32 colorClamp( PxU8 inItem )
	{
		return static_cast< PxF32 >( inItem ) / 255.0f;
	}

	struct Color
	{
		PxU32 mValue;
		PX_INLINE void unpack( PxU8& outR, PxU8& outG, PxU8& outB, PxU8& outA ) const
		{
			outR = static_cast< PxU8 >( (mValue >> 16) & 0xFF );
			outG = static_cast< PxU8 >( (mValue >> 8) & 0xFF );
			outB = static_cast< PxU8 >( mValue & 0xFF );
			outA = static_cast< PxU8 >( (mValue >> 24) & 0xFF );
		}

		PX_INLINE void unpack( PxF32& outR, PxF32& outG, PxF32& outB, PxF32& outA ) const
		{
			PxU8 tempR, tempG, tempB, tempA;
			unpack( tempR, tempG, tempB, tempA );
			outR = colorClamp( tempR );
			outG = colorClamp( tempG );
			outB = colorClamp( tempB );
			outA = colorClamp( tempA );
		}
	};
	PX_INLINE Color createColor( PxU32 inColor = 0 ) { Color retval = { inColor }; return retval; }
	PX_INLINE Color createColor( PxU8 inR, PxU8 inG, PxU8 inB, PxU8 inA = 255 ) { return createColor( (((PxU32)inA) << 24) | (((PxU32)inR) << 16) | (((PxU32)inG) << 8) | inB ); }
	PX_INLINE Color createColor( PxF32 inR, PxF32 inG, PxF32 inB, PxF32 inA = 1.0f ) { return createColor( colorClamp( inR ), colorClamp( inG ), colorClamp( inB ), colorClamp( inA ) ); }

	PX_INLINE PxU32 maskInValue( PxU32 inItem, PxU32 inValue )
	{
		return inItem | inValue;
	}

	PX_INLINE PxU32 maskOutValue( PxU32 inItem, PxU32 inValue )
	{
		return inItem & (~inValue);
	}

	PX_INLINE PxU32 maskValue( PxU32 inItem, PxU32 inValue, bool inIncludeValue )
	{
		return inIncludeValue ? maskInValue( inItem, inValue ) : maskOutValue( inItem, inValue );
	}

	PX_INLINE bool containsMaskValue( PxU32 inItem, PxU32 inValue ) { return (inItem & inValue) != 0; }

	
	struct RenderFlags
	{
		enum Enum
		{
			ScreenSpace      = (1<<0),  //!< true if rendering in screenspace
			//NoZbuffer        = (1<<1),  //!< true if zbuffering is disabled.
			//SolidShaded      = (1<<2),  //!< true if rendering solid shaded.
			//SolidWireShaded  = (1<<3),  //!< Render both as a solid shaded triangle and as a wireframe overlay.
			//CounterClockwise = (1<<4),  //!< true if winding order is counter clockwise.
			//CameraFacing     = (1<<5),  //!< True if text should be displayed camera facing
			//InfiniteLifeSpan = (1<<6),  //!< True if the lifespan is infinite (overrides current display time value)
			CenterText       = (1<<7),  //!< True if the text should be centered.
		};
		PxU32 mFlags;
		RenderFlags( PxU32 inFlags = 0 ) : mFlags( inFlags ) {}
		bool operator==( const RenderFlags& inOther ) const { return mFlags == inOther.mFlags; }
		bool operator!=( const RenderFlags& inOther ) const { return !(*this == inOther ); }
		PX_INLINE void setScreenSpace( bool inScreenSpace ) { mFlags = maskValue( mFlags, ScreenSpace, inScreenSpace );	}
		PX_INLINE bool isScreenSpace() const { return containsMaskValue( mFlags, ScreenSpace ); }
		PX_INLINE void setCenterText( bool inCenterText ) { mFlags = maskValue( mFlags, CenterText, inCenterText );	}
		PX_INLINE bool isCenterText() const { return containsMaskValue( mFlags, CenterText ); }
	};

	struct RenderState
	{
		PxF32		mTextScale;
		Color		mColor;
		PxU64		mInstance;
		PxU32		mRenderFlags;

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream )
		{
			inStream.streamify( mTextScale );
			inStream.streamify( mColor.mValue );
			inStream.streamify( mRenderFlags );
			inStream.streamify( mInstance );
		}

		template<typename TComparator>
		PX_INLINE bool compare( const RenderState& inOther, TComparator inComparator ) const
		{
			return inComparator( mTextScale, inOther.mTextScale )
				 && inComparator( mColor, inOther.mColor )
				 && inComparator( mRenderFlags, inOther.mRenderFlags )
				 && inComparator( mInstance, inOther.mInstance );
		}
	};

	PX_INLINE RenderState createRenderState( PxF32 inTextScale, Color inColor, PxU64 inInstance, RenderFlags inRenderFlags )
	{
		RenderState retval = { inTextScale, inColor, inInstance, inRenderFlags.mFlags };
		return retval;
	}

	struct RenderPrimitiveType
	{
		enum Enum
		{
			Unknown = 0,
			Sphere,
			Ray,
			Line,
			Cylinder,
			Bounds,
			Text,
			Plane,
			Circle,
			IndexedTriangleMesh,
			GradientLine,
			Triangle,
			TriangleNormals,
			GradientTriangle,
			Arc,
			Point,
			GradientLines,
			Triangles,
			Points,
			Last,
		};
		PxU8	mType;
		RenderPrimitiveType( PxU8 inType=Unknown ) : mType( inType ) {}
		PX_INLINE bool operator==( const RenderPrimitiveType& inOther ) const { return mType == inOther.mType; }
		PX_INLINE bool operator!=( const RenderPrimitiveType& inOther ) const { return !(*this == inOther ); }
		PX_INLINE const char* toString() const
		{
			switch( mType )
			{
			case Unknown: return "Unknown";
			case Sphere: return "Sphere";
			case Ray: return "Ray";
			case Line: return "Line";
			case Cylinder: return "Capsule";
			case Bounds: return "Bounds";
			case Text: return "Text";
			case Plane: return "Plane";
			case Circle: return "Circle";
			case IndexedTriangleMesh: return "IndexedTriangleMesh";
			case GradientLine: return "GradientLine";
			case Triangle: return "Triangle";
			case TriangleNormals: return "TriangleNormals";
			case GradientTriangle: return "GradientTriangle";
			case Arc: return "Arc";
			case Point: return "Point";
			case GradientLines: return "GradientLines";
			}
			return "";
		}
	};

	struct Sphere
	{
		PxF32 mRadius;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream) { ioStream.streamify( mRadius ); }
		template<typename TOperator>
		PX_INLINE bool compare( const Sphere& inOther, TOperator inOperator ) const
		{ return inOperator( mRadius, inOther.mRadius ); }
	};
	PX_INLINE Sphere createSphere( PxF32 inRadius = 0.0f ) { Sphere retval = { inRadius }; return retval; }

	struct Ray
	{
		Float3 mStart;
		Float3 mEnd;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream) { ioStream.streamify( mStart ); ioStream.streamify( mEnd ); }
		template<typename TOperator>
		PX_INLINE bool compare( const Ray& inOther, TOperator inOperator ) const
		{ 
			return inOperator( mStart, inOther.mStart ) 
					&& inOperator( mEnd, inOther.mEnd );
		}
	};
	PX_INLINE Ray createRay( const Float3& inStart, const Float3& inEnd ) { Ray retval = { inStart, inEnd }; return retval; }

	
	struct Line
	{
		Float3 mStart;
		Float3 mEnd;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream) { ioStream.streamify( mStart ); ioStream.streamify( mEnd ); }
		template<typename TOperator>
		PX_INLINE bool compare( const Line& inOther, TOperator inOperator ) const
		{ 
			return inOperator( mStart, inOther.mStart ) 
					&& inOperator( mEnd, inOther.mEnd );
		}
	};
	PX_INLINE Line createLine( const Float3& inStart, const Float3& inEnd ) { Line retval = { inStart, inEnd }; return retval; }

	struct Cylinder
	{
		Float3 mStart;
		Float3 mEnd;
		PxF32 mRadius;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream) { ioStream.streamify( mStart ); ioStream.streamify( mEnd ); ioStream.streamify( mRadius ); }
		template<typename TOperator>
		PX_INLINE bool compare( const Cylinder& inOther, TOperator inOperator ) const 
		{ 
			return inOperator( mStart, inOther.mStart ) 
					&& inOperator( mEnd, inOther.mEnd ) 
					&& inOperator( mRadius, inOther.mRadius );
		}
	};

	PX_INLINE Cylinder createCylinder( const Float3& inStart, const Float3& inStop, PxF32 inRadius ) 
	{
		Cylinder retval = { inStart, inStop, inRadius };
		return retval;
	}

	struct Circle
	{
		Float3 mNormal;
		PxF32 mRadius;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream) { ioStream.streamify( mNormal ), ioStream.streamify( mRadius ); }
		template<typename TOperator>
		PX_INLINE bool compare( const Circle& inOther, TOperator inOperator )  const
		{ 
			return inOperator( mRadius, inOther.mRadius )
				&& inOperator( mNormal, inOther.mNormal );
		}
	};
	PX_INLINE Circle createCircle( const Float3& inNormal, PxF32 inRadius ) { Circle retval = { inNormal, inRadius }; return retval; }

	struct IndexedTriangleMesh
	{
		const PxF32*	mPositions;
		PxU32			mPositionCount; // number of floating point numbers * 3
		const PxU32*	m32BitIndices;
		const PxU16*	m16BitIndices;
		PxU32			mTriangleCount; //numIndices / 3
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream ) { ioStream.streamify( mPositions, mPositionCount, m32BitIndices, m16BitIndices, mTriangleCount ); }
		template<typename TOperator>
		PX_INLINE bool compare( const IndexedTriangleMesh& inOther, TOperator inOperator )  const
		{ 
			return inOperator( *this, inOther );
		}
	};

	PX_INLINE IndexedTriangleMesh createIndexedTriangleMesh( const PxF32* inPositions, PxU32 inNumPositions, const PxU32* in32BitIndices, PxU32 inTriangleCount )
	{
		IndexedTriangleMesh retval = { inPositions, inNumPositions, in32BitIndices, NULL, inTriangleCount };
		return retval;
	}

	PX_INLINE IndexedTriangleMesh createIndexedTriangleMesh( const PxF32* inPositions, PxU32 inNumPositions, const PxU16* in16BitIndices, PxU32 inTriangleCount )
	{
		IndexedTriangleMesh retval = { inPositions, inNumPositions, NULL, in16BitIndices, inTriangleCount };
		return retval;
	}

	
	struct GradientLine
	{
		Float3	mStart;
		Float3	mEnd;
		Color	mStartColor;
		Color	mEndColor;
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream ) 
		{ 
			inStream.streamify( mStart );
			inStream.streamify( mEnd );
			inStream.streamify( mStartColor.mValue );
			inStream.streamify( mEndColor.mValue );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const GradientLine& inOther, TOperator inOperator )  const
		{ 
			return inOperator( mStart, inOther.mStart )
				&& inOperator( mEnd, inOther.mEnd )
				&& inOperator( mStartColor, inOther.mStartColor )
				&& inOperator( mEndColor, inOther.mEndColor );
		}
	};
	PX_INLINE GradientLine createGradientLine( const Float3& inStart, const Float3& inEnd, const Color& inStartColor, const Color& inEndColor )
	{
		GradientLine retval = { inStart, inEnd, inStartColor, inEndColor };
		return retval;
	}

	struct Triangle
	{
		Float3 mP1;
		Float3 mP2;
		Float3 mP3;

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream ) 
		{ 
			ioStream.streamify( mP1 );
			ioStream.streamify( mP2 );
			ioStream.streamify( mP3 );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const Triangle& inOther, TOperator inOperator )  const
		{ 
			return inOperator( mP1, inOther.mP1 )
				&& inOperator( mP2, inOther.mP2 )
				&& inOperator( mP3, inOther.mP3 );
		}
	};

	PX_INLINE Triangle createTriangle( const Float3& inP1, const Float3& inP2, const Float3& inP3 )
	{
		Triangle retval = { inP1, inP2, inP3 };
		return retval;
	}

	struct TriangleNormals
	{
		Triangle mTriangle;
		Float3 mN1;
		Float3 mN2;
		Float3 mN3;

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream ) 
		{ 
			mTriangle.streamify( ioStream );
			ioStream.streamify( mN1 );
			ioStream.streamify( mN2 );
			ioStream.streamify( mN3 );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const TriangleNormals& inOther, TOperator inOperator )  const
		{ 
			
			return mTriangle.compare( inOther.mTriangle, inOperator )
				&& inOperator( mN1, inOther.mN1 )
				&& inOperator( mN2, inOther.mN2 )
				&& inOperator( mN3, inOther.mN3 );
		}
	};

	PX_INLINE TriangleNormals createTriangleNormals( const Float3& inP1, const Float3& inP2, const Float3& inP3, const Float3& inN1, const Float3& inN2, const Float3& inN3 )
	{
		TriangleNormals retval = { { inP1, inP2, inP3 }, inN1, inN2, inN3 };
		return retval;
	}

	struct GradientTriangle
	{
		Triangle mTriangle;
		Color mC1;
		Color mC2;
		Color mC3;

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream ) 
		{ 
			mTriangle.streamify( ioStream );
			ioStream.streamify( mC1.mValue );
			ioStream.streamify( mC2.mValue );
			ioStream.streamify( mC3.mValue );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const GradientTriangle& inOther, TOperator inOperator )  const
		{ 
			
			return mTriangle.compare( inOther.mTriangle, inOperator )
				&& inOperator( mC1, inOther.mC1 )
				&& inOperator( mC2, inOther.mC2 )
				&& inOperator( mC3, inOther.mC3 );
		}
	};

	PX_INLINE GradientTriangle createGradientTriangle( const Float3& inP1, const Float3& inP2, const Float3& inP3, const Color& inC1, const Color& inC2, const Color& inC3 )
	{
		GradientTriangle retval = { { inP1, inP2, inP3 }, inC1, inC2, inC3 };
		return retval;
	}

	

	struct Arc
	{
		Float3	mCenter;
		Float3	mP1;
		Float3	mP2;
		PxF32	mArrowSize;
		bool	mShowRoot;

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream ) 
		{ 
			ioStream.streamify( mCenter );
			ioStream.streamify( mP1 );
			ioStream.streamify( mP2 );
			ioStream.streamify( mArrowSize );
			ioStream.streamify( mShowRoot );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const Arc& inOther, TOperator inOperator ) const
		{ 
			return inOperator( mCenter, inOther.mCenter )
				&& inOperator( mP1, inOther.mP1 )
				&& inOperator( mP2, inOther.mP2 )
				&& inOperator( mArrowSize, inOther.mArrowSize )
				&& inOperator( mShowRoot, inOther.mShowRoot );
		}
	};

	PX_INLINE Arc createArc( const Float3& inCenter, const Float3& inP1, const Float3& inP2, PxF32 inArrowSize, bool inShowRoot )
	{
		Arc retval = { inCenter, inP1, inP2, inArrowSize, inShowRoot };
		return retval;
	}

	struct Point //Filled circle.
	{
		Float3	mCenter;
		PxF32	mRadius;
		
		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& ioStream ) 
		{ 
			ioStream.streamify( mCenter );
			ioStream.streamify( mRadius );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const Point& inOther, TOperator inOperator ) const
		{ 
			return inOperator( mCenter, inOther.mCenter )
				&& inOperator( mRadius, inOther.mRadius );
		}
	};

	PX_INLINE Point createPoint( const Float3& inCenter, PxF32 inRadius )
	{
		Point retval = { inCenter, inRadius };
		return retval;
	}

	struct GradientLines
	{
		Float3*	mPoints; //two points a line
		PxU32*	mColors; //two colors a line
		PxU32	mLineCount;

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream ) 
		{ 
			inStream.streamify( mLineCount );
			inStream.bulkStreamify( mPoints, mLineCount*2 );
			inStream.bulkStreamify( mColors, mLineCount*2 );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const GradientLines& inOther, TOperator inOperator )  const
		{ 
			return inOperator( mLineCount, inOther.mLineCount )
				&& inOperator( mPoints, inOther.mPoints, mLineCount * 2 )
				&& inOperator( mColors, inOther.mColors, mLineCount * 2 );
		}
	};

	PX_INLINE GradientLines createGradientLines( Float3* inPoints, Color* inColors, PxU32 inLineCount )
	{
		GradientLines retval = { inPoints, reinterpret_cast< PxU32* >( inColors ), inLineCount };
		return retval;
	}

	struct Triangles
	{
		Float3* mPoints;
		PxU32*	mColors; //One color per triangle, not three.
		PxU32 mTriangleCount;	

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream ) 
		{ 
			inStream.streamify( mTriangleCount );
			inStream.bulkStreamify( mPoints, mTriangleCount * 3 );
			inStream.bulkStreamify( mColors, mTriangleCount );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const Triangles& inOther, TOperator inOperator )  const
		{ 
			return inOperator( mTriangleCount, inOther.mTriangleCount )
				&& inOperator( mPoints, inOther.mPoints, mTriangleCount * 3 )
				&& inOperator( mColors, inOther.mColors, mTriangleCount );
		}
	};

	PX_INLINE Triangles createTriangles( Float3* inPoints, Color* inColors, PxU32 inTriangleCount )
	{
		Triangles retval = { inPoints, reinterpret_cast< PxU32* >( inColors ), inTriangleCount };
		return retval;
	}
	
	struct Points
	{
		Float3* mPoints;
		PxF32*	mRadii;
		PxU32*	mColors;
		PxU32 mPointCount;	

		template<typename TStreamType>
		PX_INLINE void streamify( TStreamType& inStream ) 
		{ 
			inStream.streamify( mPointCount );
			inStream.bulkStreamify( mPoints, mPointCount );
			inStream.bulkStreamify( mRadii, mPointCount );
			inStream.bulkStreamify( mColors, mPointCount );
		}

		template<typename TOperator>
		PX_INLINE bool compare( const Points& inOther, TOperator inOperator )  const
		{ 
			return inOperator( mPointCount, inOther.mPointCount )
				&& inOperator( mPoints, inOther.mPoints, mPointCount )
				&& inOperator( mRadii, inOther.mRadii, mPointCount )
				&& inOperator( mColors, inOther.mColors, mPointCount );
		}
	};

	PX_INLINE Points createPoints( Float3* inPoints, PxF32* inRadii, Color* inColors, PxU32 inPointCount )
	{
		Points retval = { inPoints, inRadii, reinterpret_cast< PxU32* >( inColors ), inPointCount};
		return retval;
	}

	template<typename TStreamType, typename TDataType>
	PX_INLINE void renderStreamify( TStreamType& inStream, TDataType& inType ) { inType.streamify( inStream ); }
	template<typename TStreamType> PX_INLINE void renderStreamify( TStreamType& inStream, Bounds3& inType ) { inStream.streamify( inType ); }
	template<typename TStreamType> PX_INLINE void renderStreamify( TStreamType& inStream, Plane& inType ) { inStream.streamify( inType ); }
	template<typename TStreamType> PX_INLINE void renderStreamify( TStreamType& inStream, String& inType ) { inStream.streamify( inType ); }
	
	template<typename TDataType, typename TComparator>
	PX_INLINE bool renderCompare( const TDataType& inLhs, const TDataType& inRhs, TComparator inComparator ) { return inLhs.compare( inRhs, inComparator ); }
	template<typename TComparator> PX_INLINE bool renderCompare( const Bounds3& inLhs, const Bounds3& inRhs, TComparator inComparator ) { return inComparator( inLhs, inRhs ); }
	template<typename TComparator> PX_INLINE bool renderCompare( const Plane& inLhs, const Plane& inRhs, TComparator inComparator ) { return inComparator( inLhs, inRhs ); }
	template<typename TComparator> PX_INLINE bool renderCompare( const String& inLhs, const String& inRhs, TComparator inComparator ) { return inComparator( inLhs, inRhs ); }

	//Now come the boring parts.
	//Mapping datatypes to types.
	template<typename TDataType> PX_INLINE RenderPrimitiveType getRenderPrimitiveType() { PX_ASSERT(false); return RenderPrimitiveType::Unknown; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Sphere>() { return RenderPrimitiveType::Sphere; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Ray>() { return RenderPrimitiveType::Ray; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Line>() { return RenderPrimitiveType::Line; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Cylinder>() { return RenderPrimitiveType::Cylinder; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Bounds3>() { return RenderPrimitiveType::Bounds; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<String>() { return RenderPrimitiveType::Text; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Plane>() { return RenderPrimitiveType::Plane; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Circle>() { return RenderPrimitiveType::Circle; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<IndexedTriangleMesh>() { return RenderPrimitiveType::IndexedTriangleMesh; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<GradientLine>() { return RenderPrimitiveType::GradientLine; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Triangle>() { return RenderPrimitiveType::Triangle; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<TriangleNormals>() { return RenderPrimitiveType::TriangleNormals; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<GradientTriangle>() { return RenderPrimitiveType::GradientTriangle; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Arc>() { return RenderPrimitiveType::Arc; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Point>() { return RenderPrimitiveType::Point; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<GradientLines>() { return RenderPrimitiveType::GradientLines; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Triangles>() { return RenderPrimitiveType::Triangles; }
	template<> PX_INLINE RenderPrimitiveType getRenderPrimitiveType<Points>() { return RenderPrimitiveType::Points; }

	union RenderPrimitiveData
	{
		Sphere				mSphere;
		Ray					mRay;
		Line				mLine;
		Cylinder			mCylinder;
		Bounds3				mBounds;
		String				mText;
		Plane				mPlane;
		Circle				mCircle;
		IndexedTriangleMesh mIndexedTriangleMesh;
		GradientLine		mGradientLine;
		Triangle			mTriangle;
		TriangleNormals		mTriangleNormals;
		GradientTriangle	mGradientTriangle;
		Arc					mArc;
		Point				mPoint;
		GradientLines		mGradientLines;
		Triangles			mTriangles;
		Points				mPoints;
	};

	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Sphere& inType ) { RenderPrimitiveData retval; retval.mSphere = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Ray& inType ) { RenderPrimitiveData retval; retval.mRay = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Line& inType ) { RenderPrimitiveData retval; retval.mLine = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Cylinder& inType ) { RenderPrimitiveData retval; retval.mCylinder = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Bounds3& inType ) { RenderPrimitiveData retval; retval.mBounds = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const String& inType ) { RenderPrimitiveData retval; retval.mText = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Plane& inType ) { RenderPrimitiveData retval; retval.mPlane = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Circle& inType ) { RenderPrimitiveData retval; retval.mCircle = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const IndexedTriangleMesh& inType ) { RenderPrimitiveData retval; retval.mIndexedTriangleMesh = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const GradientLine& inType ) { RenderPrimitiveData retval; retval.mGradientLine = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Triangle& inType ) { RenderPrimitiveData retval; retval.mTriangle= inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const TriangleNormals& inType ) { RenderPrimitiveData retval; retval.mTriangleNormals = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const GradientTriangle& inType ) { RenderPrimitiveData retval; retval.mGradientTriangle = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Arc& inType ) { RenderPrimitiveData retval; retval.mArc = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Point& inType ) { RenderPrimitiveData retval; retval.mPoint = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const GradientLines& inType ) { RenderPrimitiveData retval; retval.mGradientLines = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Triangles& inType ) { RenderPrimitiveData retval; retval.mTriangles = inType; return retval; }
	PX_INLINE RenderPrimitiveData toRenderPrimitiveData( const Points& inType ) { RenderPrimitiveData retval; retval.mPoints = inType; return retval; }

	template<typename TDataType> PX_INLINE TDataType fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { PX_ASSERT(false); return TDataType(); }
	template<> PX_INLINE Sphere fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mSphere; }
	template<> PX_INLINE Ray fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mRay; }
	template<> PX_INLINE Line fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mLine; }
	template<> PX_INLINE Cylinder fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mCylinder; }
	template<> PX_INLINE Bounds3 fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mBounds; }
	template<> PX_INLINE String fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mText; }
	template<> PX_INLINE Plane fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mPlane; }
	template<> PX_INLINE Circle fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mCircle; }
	template<> PX_INLINE IndexedTriangleMesh fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mIndexedTriangleMesh; }
	template<> PX_INLINE GradientLine fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mGradientLine; }
	template<> PX_INLINE Triangle fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mTriangle; }
	template<> PX_INLINE TriangleNormals fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mTriangleNormals; }
	template<> PX_INLINE GradientTriangle fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mGradientTriangle; }
	template<> PX_INLINE Arc fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mArc; }
	template<> PX_INLINE Point fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mPoint; }
	template<> PX_INLINE GradientLines fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mGradientLines; }
	template<> PX_INLINE Triangles fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mTriangles; }
	template<> PX_INLINE Points fromRenderPrimitiveData( const RenderPrimitiveData& inData ) { return inData.mPoints; }

	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType visitRenderPrimitive( RenderPrimitiveType inType, const RenderPrimitiveData& inData, TOperator inOperator )
	{
		switch( inType.mType )
		{
			case RenderPrimitiveType::Sphere: return inOperator( fromRenderPrimitiveData<Sphere>( inData ) );
			case RenderPrimitiveType::Ray: return inOperator( fromRenderPrimitiveData<Ray>( inData ) );
			case RenderPrimitiveType::Line: return inOperator( fromRenderPrimitiveData<Line>( inData ) );
			case RenderPrimitiveType::Cylinder: return inOperator( fromRenderPrimitiveData<Cylinder>( inData ) );
			case RenderPrimitiveType::Bounds: return inOperator( fromRenderPrimitiveData<Bounds3>( inData ) );
			case RenderPrimitiveType::Text: return inOperator( fromRenderPrimitiveData<String>( inData ) );
			case RenderPrimitiveType::Plane: return inOperator( fromRenderPrimitiveData<Plane>( inData ) );
			case RenderPrimitiveType::Circle: return inOperator( fromRenderPrimitiveData<Circle>( inData ) );
			case RenderPrimitiveType::IndexedTriangleMesh: return inOperator( fromRenderPrimitiveData<IndexedTriangleMesh>( inData ) );
			case RenderPrimitiveType::GradientLine: return inOperator( fromRenderPrimitiveData<GradientLine>( inData ) );
			case RenderPrimitiveType::Triangle: return inOperator( fromRenderPrimitiveData<Triangle>( inData ) );
			case RenderPrimitiveType::TriangleNormals: return inOperator( fromRenderPrimitiveData<TriangleNormals>( inData ) );
			case RenderPrimitiveType::GradientTriangle: return inOperator( fromRenderPrimitiveData<GradientTriangle>( inData ) );
			case RenderPrimitiveType::Arc: return inOperator( fromRenderPrimitiveData<Arc>( inData ) );
			case RenderPrimitiveType::Point: return inOperator( fromRenderPrimitiveData<Point>( inData ) );
			case RenderPrimitiveType::GradientLines: return inOperator( fromRenderPrimitiveData<GradientLines>( inData ) );
			case RenderPrimitiveType::Triangles: return inOperator( fromRenderPrimitiveData<Triangles>( inData ) );
			case RenderPrimitiveType::Points: return inOperator( fromRenderPrimitiveData<Points>( inData ) );
			default:
				PX_ASSERT(false);
				return inOperator();
		}
	}
	
	template<typename TReturnType, typename TOperator>
	PX_INLINE TReturnType visitRenderPrimitive( RenderPrimitiveType inType, TOperator inOperator ) { return visitRenderPrimitive<TReturnType, TOperator>( inType, RenderPrimitiveData(), inOperator ); }


	class RenderPrimitive
	{
		RenderPrimitiveData mData;
		RenderPrimitiveType mType;
	public:
		RenderPrimitive( RenderPrimitiveType inType = RenderPrimitiveType::Unknown ) : mType(inType) {}
		RenderPrimitive( RenderPrimitiveType inType, RenderPrimitiveData inData ) : mData( inData ), mType( inType ) {}
		RenderPrimitive( const RenderPrimitive& inOther ) : mData( inOther.mData ), mType( inOther.mType ) {}
		RenderPrimitive& operator=( const RenderPrimitive& inOther ) 
		{
			mData = inOther.mData;
			mType = inOther.mType;
			return *this;
		}
		template<typename TDataType>
		RenderPrimitive( const TDataType& inType ) 
			: mType( getRenderPrimitiveType<TDataType>() )
			, mData( toRenderPrimitiveData( inType ) ) {}

		RenderPrimitiveData getData() const { return mData; }
		RenderPrimitiveType getType() const { return mType; }
		template<typename TDataType>
		TDataType getValue() const { PX_ASSERT( mType == getRenderPrimitiveType<TDataType>() ); return fromRenderPrimitiveData<TDataType>( mData ); }
		template<typename TReturnType, typename TOperator>
		TReturnType visit(TOperator inOperator) const { return visitRenderPrimitive<TReturnType, TOperator>( mType, mData, inOperator ); }
	};
} 

#endif