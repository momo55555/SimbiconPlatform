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


#ifndef PX_EPA_H
#define PX_EPA_H

#include "CmPhysXCommon.h"
#include "EPAFacet.h"
#include "PsAllocator.h"
#include "EPAFacet.h"
#include "ConvexSupportTable.h"
//#include "GuDistancePointSegment.h"
//#include "GuBarycentricCoordinates.h"

namespace physx
{

#define MaxFacets 64
#define MaxSupportPoints 64

namespace Gu
{
	template<class Element, PxU32 Size>
	class BinaryHeap 
	{
	public:
		BinaryHeap() 
		{
			heapSize = 0;
		}
		
		~BinaryHeap() 
		{
		}
		
		inline Element* getTop() 
		{
			//return heapTop;//data[0];
			return data[0];
		}
		
		inline bool isEmpty()
		{
			return (heapSize == 0);
		}
		
		inline void makeEmpty()
		{
			heapSize = 0;
		}

		PX_FORCE_INLINE void insert(Element* value)
		{
			PX_ASSERT(heapSize < Size);
			PxI32 newIndex;
			PxI32 parentIndex = parent(heapSize);
			for (newIndex = heapSize; newIndex > 0 && (*data[parentIndex]) > (*value); newIndex = parentIndex, parentIndex= parent(newIndex)) 
			{
				PX_ASSERT(newIndex >= 0 && newIndex < Size);
				data[ newIndex ] = data[parentIndex];
			}
			PX_ASSERT(newIndex >= 0 && newIndex < Size);
			data[newIndex] = value; 
		/*	if(newIndex == 0)
				heapTop = value;*/
			heapSize++;
			PX_ASSERT(isValid());
		}


		PX_FORCE_INLINE Element* deleteTop() PX_RESTRICT
		{
			PX_ASSERT(heapSize > 0);
			PxI32 i, child;
			Element* PX_RESTRICT min = data[0];
			Element* PX_RESTRICT last = data[--heapSize];
			PX_ASSERT(heapSize != -1);
			
			for (i = 0; (child = left(i)) < heapSize; i = child) 
			{
				/* Find smaller child */
				const PxI32 rightChild = child + 1;
				/*if((rightChild < heapSize) && (*data[rightChild]) < (*data[child]))
					child++;*/
				child += ((rightChild < heapSize) & (*data[rightChild]) < (*data[child])) ? 1 : 0;

				if((*data[child]) >= (*last))
					break;

				PX_ASSERT(i >= 0 && i < Size);
				data[i] = data[child];
			}
			PX_ASSERT(i >= 0 && i < Size);
			data[ i ] = last;
			/*heapTop = min;*/
			PX_ASSERT(isValid());
			return min;
		} 

		bool isValid()
		{
			Element* min = data[0];
			for(PxI32 i=1; i<heapSize; ++i)
			{
				if((*min) > (*data[i]))
					return false;
			}

			return true;
		}


		PxI32 heapSize;
//	private:
		Element* PX_RESTRICT data[Size];
		
		inline PxI32 left(PxI32 nodeIndex) 
		{
			return (nodeIndex << 1) + 1;
		}
		
		PxI32 parent(PxI32 nodeIndex) 
		{
			return (nodeIndex - 1) >> 1;
		}
	};

	class EPA
	{
	
	public:

		EPA(PxU32 index);
		EPA(): num_verts(0), num_facets(0), freeFacet(0)
		{
			closestFacet = NULL;
		}

#ifdef	__SPU__
		bool PenetrationDepth(const ConvexV& a, const ConvexV& b, Support aSupport, Support bSupport, const Ps::aos::Vec3V* PX_RESTRICT Q, const Ps::aos::Vec3V* PX_RESTRICT A, const Ps::aos::Vec3V* PX_RESTRICT B, const PxI32 size, Ps::aos::Vec3V& pa, Ps::aos::Vec3V& pb);
		void expandSegment(const ConvexV& a, const ConvexV& b, Support aSupport, Support bSupport);
		void expandTriangle(const ConvexV& a, const ConvexV& b, Support aSupport, Support bSupport);
#else

		template <class ConvexA, class ConvexB>
		bool PenetrationDepth(const ConvexA& a, const ConvexB& b, const Ps::aos::Vec3V* PX_RESTRICT Q, const Ps::aos::Vec3V* PX_RESTRICT A, const Ps::aos::Vec3V* PX_RESTRICT B, const PxI32 size, Ps::aos::Vec3V& pa, Ps::aos::Vec3V& pb);
	
		template <class ConvexA, class ConvexB>
		bool expandSegment(const ConvexA& a, const ConvexB& b);
		
		template <class ConvexA, class ConvexB>
		bool expandTriangle(const ConvexA& a, const ConvexB& b);
#endif

		Facet* addFacet(const PxU32 i0, const PxU32 i1, const PxU32 i2, const Ps::aos::FloatVArg lower2, const Ps::aos::FloatVArg upper2);
		Facet* addInitialFacet(const PxU32 i0, const PxU32 i1, const PxU32 i2, const Ps::aos::FloatVArg lower2, const Ps::aos::FloatVArg upper2);
	
		bool originInTetrahedron(const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2, const Ps::aos::Vec3VArg p3, const Ps::aos::Vec3VArg p4);
		bool isOriginInSide();
		//void DebugRender(DbUtil::I_DebugRender* pRender, const Maths::HVec3 PCREF a, const Maths::HVec3 PCREF b, const Maths::HVec3 PCREF c, const Maths::HVec3 PCREF d);
		
		/*PX_ALIGN(16, PxU8 aBufC[sizeof(Ps::aos::Vec3V) * MaxSupportPoints]);
		PX_ALIGN(16, PxU8 bBufC[sizeof(Ps::aos::Vec3V) * MaxSupportPoints]);

		PxU8 facetBufC[sizeof(Facet) * MaxFacets];*/

		//TODO - I'm seeing vector constructor iterator here. Need to allocate same size and alignment
		//buffers to avoid!
		Ps::aos::Vec3V aBuf[MaxSupportPoints];
		Ps::aos::Vec3V bBuf[MaxSupportPoints];
		Facet facetBuf[MaxFacets];
		BinaryHeap<Facet, MaxFacets> heap;
		Edge stack[MaxFacets * 2];
		EdgeBuffer edgeBuffer;
		Facet* closestFacet;
		PxI32 num_verts;
		PxI32 num_facets;
		PxI32 freeFacet;
	};

	PX_FORCE_INLINE PX_NOALIAS Ps::aos::BoolV PointOutsideOfPlane4(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg c, const Ps::aos::Vec3VArg d)
	{
		using namespace Ps::aos;
		const Vec4V zero = V4Zero();

		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		const Vec3V ad = V3Sub(d, a);
		const Vec3V bd = V3Sub(d, b);
		const Vec3V bc = V3Sub(c, b);

		const Vec3V v0 = V3Cross(ab, ac);
		const Vec3V v1 = V3Cross(ac, ad);
		const Vec3V v2 = V3Cross(ad, ab);
		const Vec3V v3 = V3Cross(bd, bc);

		const FloatV signa0 = V3Dot(v0, a);
		const FloatV signa1 = V3Dot(v1, a);
		const FloatV signa2 = V3Dot(v2, a);
		const FloatV signa3 = V3Dot(v3, b);
		const FloatV signd0 = V3Dot(v0, d);
		const FloatV signd1 = V3Dot(v1, b);
		const FloatV signd2 = V3Dot(v2, c);
		const FloatV signd3 = V3Dot(v3, a);
		const Vec4V signa = V4Merge(signa0, signa1, signa2, signa3);
		const Vec4V signd = V4Merge(signd0, signd1, signd2, signd3);
		return V4IsGrtrOrEq(V4Mul(signa, signd), zero);//same side, outside of the plane
	}


	inline bool EPA::originInTetrahedron(const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2, const Ps::aos::Vec3VArg p3, const Ps::aos::Vec3VArg p4)
	{
		using namespace Ps::aos;
		/*const FloatV zero = FZero();
		const BoolV bTrue = BTTTT();
		const Vec3V normal1 = V3Cross( V3Sub(p2, p1), V3Sub(p3, p1));
		const Vec3V normal2 = V3Cross( V3Sub(p3, p2), V3Sub(p4, p2));
		const Vec3V normal3 = V3Cross( V3Sub(p4, p3), V3Sub(p1, p3));
		const Vec3V normal4 = V3Cross( V3Sub(p1, p4), V3Sub(p2, p4));
		const FloatV n1p1 = V3Dot(normal1, p1);
		const FloatV n1p4 = V3Dot(normal1, p4);
		const FloatV n2p2 = V3Dot(normal2, p2);
		const FloatV n2p1 = V3Dot(normal2, p1);
		const FloatV n3p3 = V3Dot(normal3, p3);
		const FloatV n3p2 = V3Dot(normal3, p2);
		const FloatV n4p4 = V3Dot(normal4, p4);
		const FloatV n4p3 = V3Dot(normal4, p3);

		const FloatV f0 = FMul(n1p1, n1p4);
		const FloatV f1 = FMul(n2p2, n2p1);
		const FloatV f2 = FMul(n3p3, n3p2);
		const FloatV f3 = FMul(n4p4, n4p3);
		const BoolV con0 = FIsGrtr(zero, f0);
		const BoolV con1 = FIsGrtr(zero, f1);
		const BoolV con2 = FIsGrtr(zero, f2);
		const BoolV con3 = FIsGrtr(zero, f3);
		const BoolV con = BAnd(con0, BAnd(con1, BAnd(con2, con3)));
		return BAllEq(con, bTrue)==1;*/

		const BoolV bFalse = BFFFF();
		return BAllEq(PointOutsideOfPlane4(p1, p2, p3, p4), bFalse) == 1;
	}

	inline bool EPA::isOriginInSide()
	{

		using namespace Ps::aos;
		const FloatV zero = FEps();
		const Vec3V q0 = V3Sub(aBuf[0], bBuf[0]);
		const Vec3V q1 = V3Sub(aBuf[1], bBuf[1]);
		const Vec3V q2 = V3Sub(aBuf[2], bBuf[2]);
		const Vec3V q3 = V3Sub(aBuf[3], bBuf[3]);
		const Vec3V q4 = V3Sub(aBuf[4], bBuf[4]);
		const Vec3V q5 = V3Sub(aBuf[5], bBuf[5]);

		const Vec3V q02 = V3Sub(q2, q0);
		const Vec3V q03 = V3Sub(q3, q0);
		const Vec3V q04 = V3Sub(q4, q0);
		const Vec3V q05 = V3Sub(q5, q0);
		const Vec3V q12 = V3Sub(q2, q1);
		const Vec3V q13 = V3Sub(q3, q1);
		const Vec3V q14 = V3Sub(q4, q1);
		const Vec3V q15 = V3Sub(q5, q1);

		//q0, q3, q4, q5
		const Vec3V n0 = V3Cross( q03, q04);
		const Vec3V n1 = V3Cross( q04, q05);
		const FloatV n0q0 = V3Dot(n0, q0);
		const FloatV n0q5 = V3Dot(n0, q5);
		const FloatV n0q1 = V3Dot(n0, q1);
		const FloatV n0q2 = V3Dot(n0, q2);

		const FloatV n1q0 = V3Dot(n1, q0);
		const FloatV n1q3 = V3Dot(n1, q3);
		const FloatV n1q1 = V3Dot(n1, q1);
		const FloatV n1q2 = V3Dot(n1, q2);


		const Vec3V v0 = V3Merge(n0q5, n0q1, n0q2);
		const Vec3V tmp0 = V3Scale(v0, n0q0);
		const BoolV b0 = V3IsGrtrOrEq(zero, tmp0);
		const BoolV c0 = BAnyTrue3(b0);

		const Vec3V v1 = V3Merge(n1q3, n1q1, n1q2);
		const Vec3V tmp1 = V3Scale(v1, n1q0);
		const BoolV b1 = V3IsGrtrOrEq(zero, tmp1);
		const BoolV c1 = BAnyTrue3(b1);

		const BoolV con0 = BAnd(c0, c1);
		

		//q1, q3, q4, q5
		const Vec3V n3 = V3Cross(q13, q14);
		const Vec3V n4 = V3Cross(q14, q15);
		const FloatV n3q1 = V3Dot(n3, q1);
		const FloatV n3q5 = V3Dot(n3, q5);
		const FloatV n3q0 = V3Dot(n3, q0);
		const FloatV n3q2 = V3Dot(n3, q2);

		const FloatV n4q1 = V3Dot(n4, q1);
		const FloatV n4q3 = V3Dot(n4, q3);
		const FloatV n4q0 = V3Dot(n4, q0);
		const FloatV n4q2 = V3Dot(n4, q2);

		const Vec3V v2 = V3Merge(n3q5, n3q0, n3q2);
		const Vec3V tmp2 = V3Scale(v2, n3q1);
		const BoolV b2 = V3IsGrtrOrEq(zero, tmp2);
		const BoolV c2 = BAnyTrue3(b2);

		const Vec3V v3 = V3Merge(n4q3, n4q0, n4q2);
		const Vec3V tmp3 = V3Scale(v3, n4q1);
		const BoolV b3 = V3IsGrtrOrEq(zero, tmp3);
		const BoolV c3 = BAnyTrue3(b3);

		const BoolV con1 = BAnd(c2, c3);

		//q0, q3, q2, q5
		const Vec3V n6 = V3Cross( q03, q02);
		const Vec3V n7 = V3Cross( q02, q05);
		const FloatV n6q0 = V3Dot(n6, q0);
		const FloatV n6q5 = V3Dot(n6, q5);
		const FloatV n6q1 = V3Dot(n6, q1);
		const FloatV n6q4 = V3Dot(n6, q4);


		const FloatV n7q0 = V3Dot(n7, q0);
		const FloatV n7q3 = V3Dot(n7, q3);
		const FloatV n7q1 = V3Dot(n7, q1);
		const FloatV n7q4 = V3Dot(n7, q4);

		const Vec3V v4 = V3Merge(n6q5, n6q1, n6q4);
		const Vec3V tmp4 = V3Scale(v4, n6q0);
		const BoolV b4 = V3IsGrtrOrEq(zero, tmp4);
		const BoolV c4 = BAnyTrue3(b4);

		const Vec3V v5 = V3Merge(n7q3, n7q1, n7q4);
		const Vec3V tmp5 = V3Scale(v5, n7q0);
		const BoolV b5 = V3IsGrtrOrEq(zero, tmp5);
		const BoolV c5 = BAnyTrue3(b5);

		const BoolV con2 = BAnd(c4, c5);

		//q1, q3, q2, q5
		const Vec3V n9 =  V3Cross(q13, q12);
		const Vec3V n10 = V3Cross(q12, q15);
		const FloatV n9q1 = V3Dot(n9, q1);
		const FloatV n9q5 = V3Dot(n9, q5);
		const FloatV n9q0 = V3Dot(n9, q0);
		const FloatV n9q4 = V3Dot(n9, q4);

		const FloatV n10q1 = V3Dot(n10, q1);
		const FloatV n10q3 = V3Dot(n10, q3);
		const FloatV n10q0 = V3Dot(n10, q0);
		const FloatV n10q4 = V3Dot(n10, q4);

		const Vec3V v6 = V3Merge(n9q5, n9q0, n9q4);
		const Vec3V tmp6 = V3Scale(v6, n9q1);
		const BoolV b6 = V3IsGrtrOrEq(zero, tmp6);
		const BoolV c6 = BAnyTrue3(b6);

		const Vec3V v7 = V3Merge(n10q3, n10q0, n10q4);
		const Vec3V tmp7 = V3Scale(v7, n10q1);
		const BoolV b7 = V3IsGrtrOrEq(zero, tmp7);
		const BoolV c7 = BAnyTrue3(b7);

		const BoolV con3 = BAnd(c6, c7);

		const BoolV con = BAnd(con0, BAnd(con1, BAnd(con2, con3)));
		return BAllEq(con, BTTTT())==1;
	}

	PX_FORCE_INLINE Facet* EPA::addInitialFacet(const PxU32 i0, const PxU32 i1, const PxU32 i2, const Ps::aos::FloatVArg lower2, const Ps::aos::FloatVArg upper2)
	{
		using namespace Ps::aos;
		PX_ASSERT(i0 != i1 && i0 != i2 && i1 != i2);
		
		Ps::prefetch128(&facetBuf[freeFacet], 128);
		Facet * PX_RESTRICT facet = PX_PLACEMENT_NEW(&facetBuf[freeFacet],Facet(i0, i1, i2));
		PxI32 b1;
		facet->isValid(i0, i1, i2, aBuf, bBuf, b1);
		heap.insert(facet);
		++num_facets;
		freeFacet+=b1;
		return facet;
	}

	PX_FORCE_INLINE Facet* EPA::addFacet(const PxU32 i0, const PxU32 i1, const PxU32 i2, const Ps::aos::FloatVArg lower2, const Ps::aos::FloatVArg upper2)
	{
		using namespace Ps::aos;
		const BoolV bTrue = BTTTT();
		PX_ASSERT(i0 != i1 && i0 != i2 && i1 != i2);
		if (freeFacet < MaxFacets)
		{
			Ps::prefetch128(&facetBuf[freeFacet], 128);
			Facet * PX_RESTRICT facet = PX_PLACEMENT_NEW(&facetBuf[freeFacet],Facet(i0, i1, i2));
			PxI32 b1;
			/*const BoolV b2 = facet->isValid(i0, i1, i2, aBuf, bBuf, b1);
			const BoolV con = BAnd(b2, BAnd(FIsGrtrOrEq(facet->m_dist, lower2), FIsGrtrOrEq(upper2, facet->m_dist)));*/

			const BoolV con = facet->isValid2(i0, i1, i2, aBuf, bBuf, lower2, upper2, b1);

			if(BAllEq(con, bTrue))
			{
				heap.insert(facet);
				++num_facets;
			}
			freeFacet+=b1;
			return b1!=0 ? facet: NULL;
		}

		return NULL;

	}

	PX_FORCE_INLINE PxU32 leftChild(PxU32 nodeIndex) 
	{
		return (nodeIndex << 1) + 1;
	}

#ifndef	__SPU__
	
	template <class ConvexA, class ConvexB>
	bool EPA::expandSegment(const ConvexA& a, const ConvexB& b)
	{
		using namespace Ps::aos;
		const FloatV zero = FZero();
		const FloatV _max = FMax();

		//const FloatV sinHalfTheta0 = FloatV_From_F32(0.70710678119f);//sin(45), rotate 90 degree
		//const FloatV cosHalfTheta0 = FloatV_From_F32(0.70710678119f);  //cos(45), rotate 90 degree
		const FloatV sinHalfTheta0 = FloatV_From_F32(0.70710678119f);//sin((45), rotate 90 degree

		const Vec3V q0 = V3Sub(aBuf[0], bBuf[0]);
		const Vec3V q1 = V3Sub(aBuf[1], bBuf[1]);

	/*	FloatV t;
		const FloatV sqDist = Gu::distancePointSegmentSquared(q0, q1, V3Zero(), t);*/

	/*	const Vec3V dir = V3Normalise(V3Sub(q1, q0));
	
		const FloatV sDir = V3Dot(dir, dir);
		const Vec3V temp2 = V3Normalise(V3Splat(sDir));
		const Vec3V t1 = V3Normalise(V3Cross(temp2, dir));
		const Vec3V aux1 = V3Cross(dir, t1);*/
		const Vec3V dir = V3Normalize(V3Sub(q1, q0));
	
		const FloatV sDir = V3Dot(dir, dir);
		const Vec3V temp2 = V3Splat(sDir);
		const Vec3V t1 = V3Normalize(V3Cross(temp2, dir));
		const Vec3V aux1 = V3Cross(dir, t1);
		
		aBuf[2] = a.support(aux1);
		bBuf[2] = b.support(V3Neg(aux1));
	
		//const Vec3V q2 = V3Sub(aBuf[2], bBuf[2]);
		
		//sinHalfTheta0 == cosHalfTheta0 in 45 degree
		const Vec3V imag0 = V3Scale(dir, sinHalfTheta0);
		const QuatV qua0 = V4SetW(imag0, sinHalfTheta0);
		
		const Vec3V aux2 = QuatRotate(qua0, aux1);//aux1 * qua0;
		aBuf[3] = a.support(aux2);
		bBuf[3] = b.support(V3Neg(aux2));
		//const Vec3V q3 = V3Sub(aBuf[3], bBuf[3]);
		
		const Vec3V aux3 = QuatRotate(qua0, aux2);//(aux2 * qua0);
		aBuf[4] = a.support(aux3);
		bBuf[4] = b.support(V3Neg(aux3));
		//const Vec3V q4 = V3Sub(aBuf[4], bBuf[4]);

		const Vec3V aux4 = QuatRotate(qua0, aux3);
		aBuf[5] = a.support(aux4);
		bBuf[5] = b.support(V3Neg(aux4));
		//const Vec3V q5 = V3Sub(aBuf[5], bBuf[5]);

		Facet * PX_RESTRICT f0 = addFacet(2, 0, 5, zero, _max);
		Facet * PX_RESTRICT f1 = addFacet(3, 0, 2, zero, _max);
		Facet * PX_RESTRICT f2 = addFacet(4, 0, 3, zero, _max);
		Facet * PX_RESTRICT f3 = addFacet(5, 0, 4, zero, _max);
		Facet * PX_RESTRICT f4 = addFacet(2, 1, 3, zero, _max);
		Facet * PX_RESTRICT f5 = addFacet(3, 1, 4, zero, _max);
		Facet * PX_RESTRICT f6 = addFacet(4, 1, 5, zero, _max);
		Facet * PX_RESTRICT f7 = addFacet(5, 1, 2, zero, _max);

		if( (f0== NULL) | (f1 == NULL)| (f2 == NULL) | (f3 == NULL) | (f4 == NULL) | (f5 == NULL ) | (f6 == NULL) | (f7 == NULL) | heap.isEmpty())
		{
			return false;
		}

		f0->link(0, f1, 1);
		f0->link(1, f3, 0);
		f0->link(2, f7, 2);
		f1->link(0, f2, 1);
		f1->link(2, f4, 2);
		f2->link(0, f3, 1);
		f2->link(2, f5, 2);
		f3->link(2, f6, 2);
		f4->link(0, f7, 1);
		f4->link(1, f5, 0);
		f5->link(1, f6, 0);
		f6->link(1, f7, 0);

		num_verts = 6;

		return true;
	}


	template <class ConvexA, class ConvexB>
	bool EPA::expandTriangle(const ConvexA& a, const ConvexB& b)
	{

		using namespace Ps::aos;

		const FloatV zero = FZero();
		const FloatV _max = FMax();

		const Vec3V q0 = V3Sub(aBuf[0], bBuf[0]);
		const Vec3V q1 = V3Sub(aBuf[1], bBuf[1]);
		const Vec3V q2 = V3Sub(aBuf[2], bBuf[2]);
		
		const Vec3V v1 = V3Sub(q1, q0);
		const Vec3V v2 = V3Sub(q2, q0);
		const Vec3V vv = V3Normalize(V3Cross(v1,v2));
		const Vec3V nvv = V3Neg(vv);

	/*	const FloatV t = V3Dot(vv, q0);
		const Vec3V projP = V3Scale(vv, t);
		FloatV v, w;
		Gu::barycentricCoordinates(projP, q0, q1, q2, v, w);*/

		aBuf[3] = a.support(vv);
		bBuf[3] = b.support(nvv);
		const Vec3V q3 = V3Sub(aBuf[3], bBuf[3]);
		aBuf[4] = a.support(nvv);
		bBuf[4] = b.support(vv);
		const Vec3V q4 = V3Sub(aBuf[4], bBuf[4]);

		bool bCon0 = originInTetrahedron(q0, q1, q2, q3);
		bool bCon1 = originInTetrahedron(q0, q1, q2, q4);
		if(bCon0 | bCon1)
		{

			Facet * PX_RESTRICT f0 = addFacet(0, 3, 2, zero, _max);
			Facet * PX_RESTRICT f1 = addFacet(1, 3, 0, zero, _max);
			Facet * PX_RESTRICT f2 = addFacet(2, 3, 1, zero, _max);
			Facet * PX_RESTRICT f3 = addFacet(2, 4, 0, zero, _max);
			Facet * PX_RESTRICT f4 = addFacet(0, 4, 1, zero, _max);
			Facet * PX_RESTRICT f5 = addFacet(1, 4, 2, zero, _max);

			if((f0 == NULL)| (f1 == NULL) | (f2 == NULL) | (f3 == NULL)| (f4==NULL) | (f5==NULL) | heap.isEmpty())
			{
				return false;
			}
			
			f0->link(0, f1, 1);
			f0->link(1, f2, 0);
			f0->link(2, f3, 2);
			f1->link(0, f2, 1);
			f1->link(2, f4, 2);
			f2->link(2, f5, 2);
			f3->link(0, f5, 1);
			f3->link(1, f4, 0);
			f4->link(1, f5, 0);

			num_verts = 5;
			return true;
		}
		return false;
	}

	template <class ConvexA, class ConvexB>
	bool EPA::PenetrationDepth(const ConvexA& a, const ConvexB& b, const Ps::aos::Vec3V* PX_RESTRICT Q, const Ps::aos::Vec3V* PX_RESTRICT A, const Ps::aos::Vec3V* PX_RESTRICT B, const PxI32 size, Ps::aos::Vec3V& pa, Ps::aos::Vec3V& pb)
	{
		using namespace Ps::aos;
		const FloatV zero = FZero();
	
		const FloatV _max = FMax();
	
		aBuf[0]=A[0]; aBuf[1]=A[1]; aBuf[2]=A[2]; aBuf[3]=A[3];
		bBuf[0]=B[0]; bBuf[1]=B[1]; bBuf[2]=B[2]; bBuf[3]=B[3];

		num_verts=0;
		num_facets = 0;
		freeFacet = 0;

		heap.makeEmpty();

		//if the simplex isn't a tetrahedron, we need to construct one before we can expand it
		switch (size) 
		{
		case 1:
			{
				// Touching contact. Yes, we have a collision,
				// but no penetration, will still have a contact point, but we treat it as nonintersect because it don't have any penetration
				return false;
			}
		case 2: 
			{
				// We have a line segment inside the Minkowski sum containing the
				// origin. Blow it up by adding three additional support points.
				if(!expandSegment<ConvexA, ConvexB>(a, b))
					return false;
				break;
			}
		case 3: 
			{
				// We have a triangle inside the Minkowski sum containing
				// the origin. First blow it up.
				if(!expandTriangle<ConvexA, ConvexB>(a, b))
					return false;
				break;
				
			}
		case 4:
			{
				Facet * PX_RESTRICT f0 = addFacet(0, 1, 2, zero, _max);
				Facet * PX_RESTRICT f1 = addFacet(0, 3, 1, zero, _max);
				Facet * PX_RESTRICT f2 = addFacet(0, 2, 3, zero, _max);
				Facet * PX_RESTRICT f3 = addFacet(1, 3, 2, zero, _max);
				if((f0 == NULL)| (f1 == NULL) | (f2 == NULL) | (f3 == NULL)| heap.isEmpty())
				{
					return false;
				}
			
				f0->link(0, f1, 2);
				f0->link(1, f3, 2);
				f0->link(2, f2, 0);
				f1->link(0, f2, 2);
				f1->link(1, f3, 0);
				f2->link(1, f3, 1);
			
				num_verts = 4;

				break;
			}
		}

		FloatV upper_bound2(_max);
		//const FloatV eps = FEps();

		PxI32 numVertsLocal = num_verts;

		PX_ASSERT(!heap.isEmpty());

		Facet* PX_RESTRICT facet;
		do 
		{
			facet = heap.deleteTop(); //get the shortest distance triangle of origin from the list
			closestFacet = facet;

			if (!facet->isObsolete()) 
			{
				//PX_ASSERT(facet->m_UDist >= 0);

				/*if (numVertsLocal == MaxSupportPoints) 
					break;*/

				const FloatV fSqDist = facet->getDist();
				const FloatV fRSqDist = FRecip(fSqDist);
				const Vec3V closest = facet->getClosest();
				const Vec3V tempa = a.support(closest);
				const Vec3V tempb = b.support(V3Neg(closest));
				 
				aBuf[numVertsLocal]=tempa;
				bBuf[numVertsLocal]=tempb;
				const Vec3V q = V3Sub(tempa, tempb);


				const PxU32 index =numVertsLocal++;

				//if the support point and closest point close enough, found the contact point, break
				const FloatV dist = V3Dot(q, closest); 

				const FloatV sqDist = FMul(dist, dist);

				upper_bound2 = FMin(upper_bound2, FMul(sqDist, fRSqDist));

				if(FAllGrtrOrEq(fSqDist, upper_bound2))//(upper_bound2 <= sqDist)
					break;

				//upper_bound2=Min(upper_bound2, (dist * dist) / scdist);
				// Compute the silhouette cast by the new vertex
				// Note that the new vertex is on the positive side
				// of the current facet, so the current facet is will
				// not be in the convex hull. Start local search
				// from this facet.

			
				edgeBuffer.MakeEmpty();

				facet->silhouette(q, edgeBuffer, stack);

				if(edgeBuffer.IsEmpty()) 
					return false;

				Edge* PX_RESTRICT edge=edgeBuffer.Get(0);

				Facet *firstFacet = addFacet(edge->getTarget(), edge->getSource(),index, fSqDist, upper_bound2);

				if (!firstFacet) 
					break;

				firstFacet->link(0, edge->getFacet(), edge->getIndex());
				Facet * PX_RESTRICT lastFacet = firstFacet;

				PxU32 bufferSize=edgeBuffer.Size();

				for(PxU32 i=1; i<bufferSize; ++i)
				{
					edge=edgeBuffer.Get(i);
					Facet* PX_RESTRICT newFacet = addFacet(edge->getTarget(), edge->getSource(),index, fSqDist, upper_bound2);
					if (!newFacet)
					{
						//facet->getClosestPoint(aBuf, bBuf, pa, pb);
						pa = facet->m_closestA;
						pb = facet->m_closestB;
						return true;
					}

					const bool b0 = newFacet->link(0, edge->getFacet(), edge->getIndex());
					const bool b1 = newFacet->link(2, lastFacet, 1);
					if((!b0)|(!b1))
					{
						//facet->getClosestPoint(aBuf, bBuf, pa, pb);
						pa = facet->m_closestA;
						pb = facet->m_closestB;
						return true;
					}

					lastFacet = newFacet; 
				}
				firstFacet->link(2, lastFacet, 1);
			}
		}
		while ((--num_facets) && FAllGrtrOrEq(upper_bound2, heap.getTop()->getDist()));//heap.getTop()->getDist() <= upper_bound2;
		//while((heap.heapSize > 0) & FAllGrtrOrEq(upper_bound2, heap.getTop()->getDist()) & (numVertsLocal != MaxSupportPoints));

		//facet->getClosestPoint(aBuf, bBuf, pa, pb);
		pa = facet->m_closestA;
		pb = facet->m_closestB;

		return true;
	}

#endif
}

}

#endif
