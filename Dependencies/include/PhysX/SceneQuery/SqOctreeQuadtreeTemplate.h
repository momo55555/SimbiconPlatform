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


#include "SqPrunable.h"

namespace physx
{

namespace Gu
{
	class Capsule;
	class Segment;
}

#ifdef USE_QUADTREE
	#ifdef USE_OCTREE
		#error Define only one of them!
	#endif
#endif

#ifdef USE_QUADTREE
	#pragma message("Using quadtree")
#endif
#ifdef USE_OCTREE
	#pragma message("Using octree")
#endif

#ifdef USE_QUADTREE
	class Point2D
	{
		public:

		//! Empty constructor
		PX_FORCE_INLINE					Point2D()										{}
		//! Constructor from floats
		PX_FORCE_INLINE					Point2D(float _x, float _y) : x(_x), y(_y)		{}
		//! Copy constructor
		PX_FORCE_INLINE					Point2D(const Point2D& p) : x(p.x), y(p.y)		{}
		PX_FORCE_INLINE					Point2D(PxReal r): x(r), y(r)					{}
		//! Destructor
		PX_FORCE_INLINE					~Point2D()										{}

		PX_FORCE_INLINE	static Point2D	zero()											{ return Point2D(0,0); }
		PX_FORCE_INLINE	float			Max()								const		{ return PxMax(x, y);						}
		PX_FORCE_INLINE	Point2D&		Neg()											{ x = -x;		y = -y;		return *this;	}
		PX_FORCE_INLINE	Point2D&		setZero()										{ x =			y =	0.0f;	return *this;	}

		PX_FORCE_INLINE	Point2D			operator-()							const		{ return Point2D(-x, -y);					}

		//! Operator for Point2D Plus = Point2D + Point2D.
		PX_FORCE_INLINE	Point2D			operator+(const Point2D& p)			const		{ return Point2D(x + p.x, y + p.y);		}
		//! Operator for Point2D Minus = Point2D - Point2D.
		PX_FORCE_INLINE	Point2D			operator-(const Point2D& p)			const		{ return Point2D(x - p.x, y - p.y);		}

		//! Operator for Point2D Scale = Point2D * float.
		PX_FORCE_INLINE	Point2D			operator*(float s)					const		{ return Point2D(x * s,   y * s);		}
		//! Operator for Point2D Scale = float * Point2D.
		PX_FORCE_INLINE friend	Point2D	operator*(float s, const Point2D& p)			{ return Point2D(s * p.x, s * p.y);		}

		PX_FORCE_INLINE					operator	const	float*() const	{ return &x; }
		PX_FORCE_INLINE					operator			float*()		{ return &x; }

		float x,y;
	};

	class AABB_2D
	{
		public:

		PX_FORCE_INLINE	void		getCenter(Point2D& center)				const		{ center = (maximum + minimum)*0.5f;			}
		PX_FORCE_INLINE	void		getExtents(Point2D& extents)			const		{ extents = (maximum - minimum)*0.5f;			}
		PX_FORCE_INLINE	float		getCenter(PxU32 axis)					const		{ return (maximum[axis] + minimum[axis])*0.5f;	}
		PX_FORCE_INLINE	float		getExtents(PxU32 axis)					const		{ return (maximum[axis] - minimum[axis])*0.5f;	}
		PX_FORCE_INLINE	Point2D		getCenter()								const		{ return (maximum + minimum)*0.5f;	}

		PX_FORCE_INLINE	void		setCenterExtents(const Point2D& c, const Point2D& e)
									{
										minimum = c - e;
										maximum = c + e;
									}

		PX_FORCE_INLINE	Ps::IntBool	contains(const Point2D& p)		const
									{
										if(p.x > maximum.x || p.x < minimum.x) return Ps::IntFalse;
										if(p.y > maximum.y || p.y < minimum.y) return Ps::IntFalse;
										return Ps::IntTrue;
									}

/*
		//! Get minimum point of the box
		PX_FORCE_INLINE	void		GetMin(Point2D& minimum)				const		{ minimum = mMin;		}
		//! Get maximum point of the box
		PX_FORCE_INLINE	void		GetMax(Point2D& maximum)				const		{ maximum = mMax;		}

		//! Get component of the box's minimum point along a given axis
		PX_FORCE_INLINE	float		GetMin(PxU32 axis)						const		{ return mMin[axis];	}
		//! Get component of the box's maximum point along a given axis
		PX_FORCE_INLINE	float		GetMax(PxU32 axis)						const		{ return mMax[axis];	}

						void		SetMinMax(const Point2D& minimum, const Point2D& maximum)	{ mMin = minimum;		mMax = maximum;				}

		//! Operator for AABB *= float. Scales the extents, keeps same center.
		PX_FORCE_INLINE	AABB_2D&	operator*=(float s)
									{
										Point2D Center;		GetCenter(Center);
										Point2D Extents;	GetExtents(Extents);
										SetCenterExtents(Center, Extents * s);
										return *this;
									}

						float		MakeCube(AABB_2D& cube) const
									{
										Point2D Ext;	GetExtents(Ext);
										float Max = Ext.Max();

										Point2D Cnt;	GetCenter(Cnt);
										cube.SetCenterExtents(Cnt, Point2D(Max, Max));
										return Max;
									}


		PX_FORCE_INLINE	Ps::IntBool	Intersect(const AABB_2D& a)				const
									{
										if(mMax.x < a.mMin.x
										|| a.mMax.x < mMin.x
										|| mMax.y < a.mMin.y
										|| a.mMax.y < mMin.y)	return Ps::IntFalse;

										return Ps::IntTrue;
									}
*/
		Point2D	minimum;
		Point2D	maximum;
	};

	class Sphere_2D
	{
		public:

		Point2D	center;
		float	radius;
	};
#endif

#ifdef USE_QUADTREE
	typedef	AABB_2D		OctreeBox;
	typedef	Sphere_2D	OctreeSphere;
	typedef	Point2D		OctreePoint;
#else
	typedef	PxBounds3	OctreeBox;
	typedef	Gu::Sphere	OctreeSphere;
	typedef	PxVec3		OctreePoint;
#endif

#ifdef USE_QUADTREE
	#define NB_OCTREE_CHILDREN	4
	#define NB_OCTREE_SHIFT		2
	#define CELL_CLASS_NAME		QuadtreeCell
	#define TREE_CLASS_NAME		LinearLooseQuadtree
	#define CALLBACK_NAME		QuadtreeCallback
	#define PRUNER_NAME			QuadtreePruner
#else
	#define NB_OCTREE_CHILDREN	8
	#define NB_OCTREE_SHIFT		3
	#define CELL_CLASS_NAME		OctreeCell
	#define TREE_CLASS_NAME		LinearLooseOctree
	#define CALLBACK_NAME		OctreeCallback
	#define PRUNER_NAME			OctreePruner
#endif

	#define MAKE_CHILD_INDEX(parent_index, child_number)	((parent_index<<NB_OCTREE_SHIFT) + child_number)
	#define MAKE_PARENT_INDEX(current_index)				current_index--;	current_index>>=NB_OCTREE_SHIFT;

	class CELL_CLASS_NAME : public Ps::UserAllocated
	{
		public:
		PX_FORCE_INLINE						CELL_CLASS_NAME() : mCount(0), mList(NULL)	{}
		PX_FORCE_INLINE						~CELL_CLASS_NAME()							{}

#ifndef USE_IMPLICIT_BOX
		PX_FORCE_INLINE	const OctreeBox&	GetBox()				const	{ return mBox;		}
						void				ScaleBox(float s)				{ mBox.scale(s);	}
#endif
		PX_FORCE_INLINE	PxU32				GetCount()				const	{ return mCount;	}
		PX_FORCE_INLINE	Sq::Prunable*		GetObjects()			const	{ return mList;		}

		PX_FORCE_INLINE	void				Dump(Ice::ContainerSizeT& list)	const
											{
												Sq::Prunable* Obj = mList;
												while(Obj)
												{
													list.Add(size_t(Obj));
													Obj = Obj->GetNextObject();
												}
											}
		private:
#ifndef USE_IMPLICIT_BOX
						OctreeBox			mBox;		//!< Cell's AABB
#endif
						PxU32				mCount;		//!< Total number of objects in that cell AND children
						Sq::Prunable*		mList;		//!< Linked list of objects contained in that cell

		friend			class				TREE_CLASS_NAME;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	User-callback, called for each octree cell by the walking code.
	 *	\param		parent		[in] parent cell
	 *	\param		current		[in] current cell
	 *	\param		user_data	[in] user-defined data
	 *	\return		true to recurse through children, else false to bypass them
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	typedef bool (*CALLBACK_NAME)	(CELL_CLASS_NAME* parent, CELL_CLASS_NAME* current, void* user_data);

	class TREE_CLASS_NAME : public Ps::UserAllocated
	{
		public:
		// Constructor/Destructor
									TREE_CLASS_NAME();
									~TREE_CLASS_NAME();

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Initializes the loose octree.
		 *	\param		max_depth	[in] maximum depth for the linear octree
		 *	\param		world_box	[in] world's maximum bounding box
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_QUADTREE
				bool				Init(PxU32 max_depth, const OctreeBox& world_box, PxU32 quadtree_up);
#else
				bool				Init(PxU32 max_depth, const OctreeBox& world_box);
#endif

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Releases everything.
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					bool				Release();

		PX_FORCE_INLINE	bool				InsertObject(Sq::Prunable& obj)		{ return InsertObject(obj, NULL);	}
					bool				RemoveObject(Sq::Prunable& obj);
					bool				UpdateObject(Sq::Prunable& obj);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Traverses the whole octree, calling the user back for each cell.
		 *	\param		cb			[in] user-defined callback
		 *	\param		user_data	[in] user-defined data
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool				Walk(CALLBACK_NAME cb, void* user_data)	const;

		// Data access
		PX_FORCE_INLINE	PxU32				GetMaxDepth()				const	{ return mMaxDepth;						}
		PX_FORCE_INLINE	PxU32				GetNbCells()				const	{ return mNbCells;						}
		PX_FORCE_INLINE	CELL_CLASS_NAME*	GetCells()					const	{ return mCells;						}
		PX_FORCE_INLINE	const OctreeBox&	GetWorldBox()				const	{ return mWorldBox;						}
		PX_FORCE_INLINE	PxU32				GetNbContainedObjects()		const	{ return mCells ? mCells[0].mCount : 0;	}
#ifdef USE_QUADTREE
		PX_FORCE_INLINE	PxU32				GetQuadtreeUp()				const	{ return mQuadtreeUp;					}
#endif
				void				TestAgainstPlanes(const Gu::Plane* planes, PxU32 nb_planes, Ice::ContainerSizeT& box_indices_clip, Ice::ContainerSizeT& box_indices_noclip);
				void				TestAgainstSegment(const Gu::Segment& segment, Ice::ContainerSizeT& candidates);
				void				TestAgainstRay(const PxVec3& orig, const PxVec3& dir, Ice::ContainerSizeT& candidates);
				void				TestAgainstSphere(const Gu::Sphere& sphere, Ice::ContainerSizeT& candidates, Ice::ContainerSizeT& touched);
				void				TestAgainstCapsule(const Gu::Capsule& capsule, Ice::ContainerSizeT& candidates, Ice::ContainerSizeT& touched);
				void				TestAgainstAABB(const PxBounds3& aabb, Ice::ContainerSizeT& candidates, Ice::ContainerSizeT& touched);

#ifdef USE_IMPLICIT_BOX
				void				ComputeBox(PxU32 index, OctreeBox& box)	const;
				void				ComputeBox(PxU32 index, OctreePoint& center, float& extent) const;
				PxU32				ComputeChildrenBoxes(PxU32 parent_index, OctreePoint* centers, float& extent, PxU32* indices) const;
				void				ComputeOrder(const PxVec3& dir, PxU32* sorted)	const;
#endif
		private:
		// World data
				OctreeBox			mWorldBox;		//!< Original world box used to build the octree
				OctreePoint			mWorldOffset;
				float				mWorldSize;		//!< Original cube size (not loose)
				float				mWorldSize2;	//!< Precomputed mWorldSize/2
/*				float				mWorldSize4;	//!< Precomputed mWorldSize/4
				float				mWorldSize8;	//!< Precomputed mWorldSize/8
				float				mWorldSize16;	//!< Precomputed mWorldSize/16
				float				mWorldSize32;	//!< Precomputed mWorldSize/32
				float				mWorldSize64;	//!< Precomputed mWorldSize/64
				float				mWorldSize128;	//!< Precomputed mWorldSize/128*/
#ifdef USE_QUADTREE
				PxU32				mQuadtreeUp;
#endif
		// Tree data
				PxU32				mMaxDepth;		//!< Maximum octree depth
				PxU32				mNbCells;		//!< Total number of cells in the linear octree = f(mMaxDepth)
				CELL_CLASS_NAME*	mCells;			//!< Linear list of mNbCells cells
		// Internal methods
				void				_Walk(PxU32 index, CELL_CLASS_NAME* parent, CALLBACK_NAME cb, void* user_data)	const;
				bool				InsertObject(Sq::Prunable& obj, const OctreeBox* bounding_cube);
				void				_InitOctree(PxU32 index, const OctreeBox& box);
	};

}
