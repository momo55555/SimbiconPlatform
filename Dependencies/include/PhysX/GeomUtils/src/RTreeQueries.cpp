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


#include "RTree.h"
#include "PsIntrinsics.h"
#include "PxBounds3.h"
#include "GuBox.h"
#include "PsVecMath.h"
#include "../../LowLevel/common/include/utils/PxcMemFetch.h"

//#define VERIFY_RTREE
#ifdef VERIFY_RTREE
#include "GuIntersectionRayBox.h"
#include "GuIntersectionBoxBox.h"
#include "GuIntersectionCapsuleBox.h"
#include "stdio.h"
#endif

using namespace physx;
using namespace physx::shdfnd3;
using namespace Ps::aos;
//using _Vec4V::readX; using _Vec4V::readY; using _Vec4V::readZ; using _Vec4V::readW;
//struct U16 { PxU16 u16[8]; } tmpu16; // for 360 debugging only since it doesn't support viewing v_ushort8 in the debugger

namespace physx
{
namespace Gu {

#define v_float4 Ps::aos::Vec4V
#define v_float4a const Ps::aos::Vec4V&
#define v_float4_set4(v, a, b, c, d) v = Ps::aos::Vec4V_From_XYZW(a, b, c, d);
#define v_uint4_set4(v, a, b, c, d) v = Ps::aos::VecU32V_From_XYZW(a, b, c, d);
#define v_uint4 Ps::aos::VecU32V
#define v_int4 Ps::aos::VecI32V
#define v_ushort8 Ps::aos::VecU16V
#define v_madd(a, b, c) V4MulAdd(a, b, c)
#define v_vsubfp(a, b) V4Sub(a, b)
#define v_vaddfp(a, b) V4Add(a, b)
#define v_vrfip(a) V4Ceil(a)
#define v_vrfim(a) V4Floor(a)
#define v_vctuxs(a, p) V4ConvertToU32VSaturate(a, p)
#define v_vpkuwus(a, b) V4U32PK(a, b)
#define v_vor16(a, b) V4U16Or(a, b)
#define v_vor32(a, b) V4U32or(a, b)
#define v_splat_s32(a) V4ISplat<a>()
#define v_stvx_(a, ptr) V4U16StoreAligned(a, (VecU16V*)ptr);
#define v_stvx_32(a, ptr) V4U32StoreAligned(a, (VecU32V*)ptr);
#define v_lvx_float(ptr) V4LoadAligned((Vec4V*)ptr)
#define v_lvx_u16(ptr) V4U16LoadAligned((VecU16V*)ptr)
#define v_vcmpgtuh(a, b) V4U16CompareGt(a, b)
#define v_vcfux0(a) Vec4V_From_VecU32V(a)
#define v_and16(a, b) V4U16And(a, b)
#define v_and32(a, b) V4U32and(a, b)
#define v_andc(a, b) V4U16Andc(a, b)
#define v_andc32(a, b) V4U32Andc(a, b)
#define v_vnmsubfp(a, b, c) V4NegMulSub(a, b, c) // = -ab+c
#define v_vspltw(a, index) V4U32SplatElement<index>(a)
#define v_vspltf(a, index) V4SplatElement<index>(a)
#define v_vsplth(a, index) V4U16SplatElement<index>(a)
#define v_vspltish(imm) VecU16V(V4I16SplatImmediate<imm>())
#define v_vsubuhm(a, b) V4U16SubtractModulo(a, b)
#define v_vadduhm(a, b) V4U16AddModulo(a, b)
#define v_lo_16(a) V4U16GetLo16(a)
#define v_hi_16(a) V4U16GetHi16(a)
#define v_vmaxfp(a, b) V4Max(a, b)
#define v_vminfp(a, b) V4Min(a, b)
#define v_vcmpgtfp(a, b) V4IsGrtrV32u(a, b)
#define v_absm(a) V4Andc(a, signMask)
#define v_vrefp(a) V4RecipFast(a)

// quantize min and max into [xMin,yMin,zMin,0,xMax,yMax,zMax,0] x16 bits each
static v_ushort8 quantizeAABBQueryFast(
	v_float4a nqMin, v_float4a nqMax, v_float4a boundsMin, v_float4a invDiagonal)
{
	v_float4 hiQ;
	v_float4_set4(hiQ, 65535.0f, 65535.0f, 65535.0f, 0.0f);
	v_float4 loQ;
	v_float4_set4(loQ, 0.0f, 0.0f, 0.0f, 0.0f);
	v_float4 scaledMin = v_madd(v_madd(v_vsubfp(nqMin, boundsMin), invDiagonal, loQ), hiQ, loQ);
	v_float4 scaledMax = v_madd(v_madd(v_vsubfp(nqMax, boundsMin), invDiagonal, loQ), hiQ, loQ);
	// clamp to [1,65534] range so we can have the [65535,0] inverted sentinel range used for empty nodes
	// so we can always return no intersection without additional computation
	// the bounds are inflated during offline build so that all the contents are inside the [1,65534] range
	scaledMin = v_vminfp(v_vmaxfp(scaledMin, Vec4V_From_F32(1.0f)), Vec4V_From_F32(65534.0f));
	scaledMax = v_vminfp(v_vmaxfp(scaledMax, Vec4V_From_F32(1.0f)), Vec4V_From_F32(65534.0f));
	scaledMin = v_vrfim(scaledMin);
	scaledMax = v_vrfip(scaledMax);
	v_uint4 qMin = v_vctuxs(scaledMin, 0);
	v_uint4 qMax = v_vctuxs(scaledMax, 0);
	v_ushort8 q = (v_ushort8)v_vpkuwus(qMin, qMax);
	return q;
}

// dequantize mins for component C (0 for X etc) into res1 and res2
// ok to use modulo subtract (saturate not supported on spu) because mns is guaranteed not to wrap around
#define DEQ_MIN(mns, C, res1, res2) { \
	v_ushort8 tmp = v_vsubuhm(mns, short8_one); \
	res1 = v_vcfux0(v_and32(v_lo_16(tmp), ffff)); \
	res1 = v_madd(res1, scalerSplat##C, treeMinSplat##C); \
	res2 = v_vcfux0(v_and32(v_hi_16(tmp), ffff)); \
	res2 = v_madd(res2, scalerSplat##C, treeMinSplat##C); \
	}

// dequantize maxes for component C (0 for X etc) into res1 and res2
#define DEQ_MAX(mxs, C, res1, res2) { \
	v_ushort8 tmp = v_vadduhm(short8_one, mxs); \
	res1 = v_vcfux0(v_and32(v_lo_16(tmp), ffff)); \
	res1 = v_madd(res1, scalerSplat##C, treeMinSplat##C); \
	res2 = v_vcfux0(v_and32(v_hi_16(tmp), ffff)); \
	res2 = v_madd(res2, scalerSplat##C, treeMinSplat##C); \
	}

/* DEQ_MIN(minx8, 0, minx4a, minx4b expansion) for debugging
		v_ushort8 tmp = v_vsubuhm(minx8, short8_one);
		minx4a = v_vcfux0(v_and32(v_lo_16(tmp), ffff));
		minx4a = v_madd(minx4a, scalerSplat0, treeMinSplat0);
		minx4b = v_vcfux0(v_and32(v_hi_16(tmp), ffff));
		minx4b = v_madd(minx4b, scalerSplat0, treeMinSplat0);
*/



PX_INLINE bool isPowerOfTwo(PxU32 n) { return ((n&(n-1))==0); }

/////////////////////////////////////////////////////////////////////////
void RTree::traverseAABB(
	const PxVec3& boxMin, const PxVec3& boxMax,
	const PxU32 maxResults, PxU32* resultsPtr,
	Callback* callback
) const
{
	PX_ASSERT(callback);
	PX_ASSERT(maxResults >= mPageSize);

	// try top node stack cache, try fixed traversal path somehow (stackless with computation)
	const PxU32 maxStack = 128;
	PxU32 stack[maxStack];

	PX_ASSERT(mPages);
	PX_ASSERT((PxMemFetchPtr(mPages) & 127) == 0);
	PX_ASSERT((PxMemFetchPtr(this) & 15) == 0);

	PxU32* resultsBegin = resultsPtr;

	// conservatively quantize the input box
	v_float4 nqMin = V4LoadUnaligned((Vec4V*)&boxMin.x);
	v_float4 nqMax = V4LoadUnaligned((Vec4V*)&boxMax.x);

	v_ushort8 query = quantizeAABBQueryFast(nqMin, nqMax, v_lvx_float(&mBoundsMin), v_lvx_float(&mInvDiagonal));
	v_ushort8 qminx8 = v_vsplth(query, 0);
	v_ushort8 qminy8 = v_vsplth(query, 1);
	v_ushort8 qminz8 = v_vsplth(query, 2);
	v_ushort8 qmaxx8 = v_vsplth(query, 4);
	v_ushort8 qmaxy8 = v_vsplth(query, 5);
	v_ushort8 qmaxz8 = v_vsplth(query, 6);
	const v_ushort8 ones8 = v_vspltish(1);
	v_ushort8 * treeNodes8 = reinterpret_cast<v_ushort8 *>(mPages);
	PxU32 bottomLevelFirstNodeIndexM1 = mBottomLevelFirstNodeIndex-1;
	PxU32* stackPtr;

	stackPtr = stack;

	// AP potential perf optimization - fetch top level right away
	PX_ASSERT(mPageSize == 8);
	PX_ASSERT(isPowerOfTwo(mPageSize));
	PX_ASSERT(mNumRootPages > 0);

	for (PxI32 j = PxI32(mNumRootPages-1); j >= 0; j --)
		*stackPtr++ = j*8;
	PxU32 cacheTopValid = true;
	PxU32 cacheTop = 0;

	do {
		stackPtr--;
		PxU32 top;
		if (cacheTopValid) // branch is faster than lhs
		{
			PX_ASSERT(cacheTop < 0xFFFFF);
			top = cacheTop;
		} else
			top = stackPtr[0];
		PX_ASSERT(!cacheTopValid || top == cacheTop);
		v_ushort8* __restrict tn = treeNodes8 + top;
#ifdef __SPU__
		RTreePage tmpPage;
		pxMemFetchAlignedAsync(PxU64(&tmpPage), PxU64(treeNodes8+top), sizeof(RTreePage), 5);
		pxMemFetchWait(5);
		tn = reinterpret_cast<v_ushort8*>(&tmpPage);
#endif
		PxU32* ptrs = ((RTreePage *)tn)->ptrs;
		PxU32 ptr0 = ptrs[0], ptr1 = ptrs[1], ptr2 = ptrs[2], ptr3 = ptrs[3];
		PxU32 ptr4 = ptrs[4], ptr5 = ptrs[5], ptr6 = ptrs[6], ptr7 = ptrs[7];
		PxU32* rr = resultsPtr;
		PxU32* ss = stackPtr;
		PxU32 bottomLevel = ((bottomLevelFirstNodeIndexM1-top)>>31);

		v_ushort8
			minx8(v_lvx_u16(tn+0)), miny8(v_lvx_u16(tn+1)), minz8(v_lvx_u16(tn+2)),
			maxx8(v_lvx_u16(tn+3)), maxy8(v_lvx_u16(tn+4)), maxz8(v_lvx_u16(tn+5));

		if (0)
		{
			#define PF prefetch128 // __dcbt
			PF(treeNodes8+ptr0); PF(treeNodes8+ptr1);
			PF(treeNodes8+ptr2); PF(treeNodes8+ptr3);
			PF(treeNodes8+ptr4); PF(treeNodes8+ptr5);
			PF(treeNodes8+ptr6); PF(treeNodes8+ptr7);
			#undef PF
		}

		// 1 in any halfword after vcmpgtuh = no intersection
		v_ushort8 res0 = (v_ushort8)v_vcmpgtuh(qminx8, maxx8);
		v_ushort8 res1 = (v_ushort8)v_vcmpgtuh(qminy8, maxy8);
		v_ushort8 res2 = (v_ushort8)v_vcmpgtuh(qminz8, maxz8);
		v_ushort8 res3 = (v_ushort8)v_vcmpgtuh(minx8, qmaxx8);
		v_ushort8 res4 = (v_ushort8)v_vcmpgtuh(miny8, qmaxy8);
		v_ushort8 res5 = (v_ushort8)v_vcmpgtuh(minz8, qmaxz8);
		v_ushort8 resx = v_vor16(v_vor16(v_vor16(res0, res1), v_vor16(res2, res3)), v_vor16(res4, res5));
		PX_ALIGN_PREFIX(16) PxU16 res[8] PX_ALIGN_SUFFIX(16);
		v_ushort8 res8 = v_andc(ones8, resx);
		v_stvx_(res8, res);

		PxU32 sum0 = 0;
		PxU32 sum1 = 0 + res[0];
		PxU32 sum2 = sum1 + res[1];
		PxU32 sum3 = sum2 + res[2];
		PxU32 sum4 = sum3 + res[3];
		PxU32 sum5 = sum4 + res[4];
		PxU32 sum6 = sum5 + res[5];
		PxU32 sum7 = sum6 + res[6];
		PxU32 sum8 = sum7 + res[7];

		if (bottomLevel) {
			cacheTopValid = true;
			cacheTop = stackPtr[-1];
			rr[sum0] = ptr0;
			rr[sum1] = ptr1;
			rr[sum2] = ptr2;
			rr[sum3] = ptr3;
			rr[sum4] = ptr4;
			rr[sum5] = ptr5;
			rr[sum6] = ptr6;
			rr[sum7] = ptr7;
			resultsPtr = rr + sum8;
			if (rr+sum8+mPageSize > resultsBegin+maxResults)
			{
				// flush partial results via callback.
				// We flush mPageSize entries too early to make it so the api works with immediate early out for maxResults = 8
				if (!callback->processResults(PxU32((rr+sum8)-resultsBegin), resultsBegin))
					return;
				resultsPtr = resultsBegin;
			}
		} else {
			PxU32 bitmask = (res[0]<<24)|(res[1]<<25)|(res[2]<<26)|(res[3]<<27)|(res[4]<<28)|(res[5]<<29)|(res[6]<<30)|(res[7]<<31);
			PxI32 lastIndex = 7 - PxI32(countLeadingZeros(bitmask));
			cacheTopValid = (lastIndex >= 0);
			// worst case scenario is 7-32=-25
			// it just happens that we can reference up to ptrs[-25] safely since sizeof(RTreePage) is 128
			// so we don't need to clamp lastIndex here
			cacheTop = ptrs[lastIndex];
			ss[sum0] = ptr0;
			ss[sum1] = ptr1;
			ss[sum2] = ptr2;
			ss[sum3] = ptr3;
			ss[sum4] = ptr4;
			ss[sum5] = ptr5;
			ss[sum6] = ptr6;
			ss[sum7] = ptr7;
			stackPtr = ss + sum8;
		}

	} while (stackPtr > stack);

	if (resultsPtr-resultsBegin > 0)
		callback->processResults(PxU32(resultsPtr-resultsBegin), resultsBegin);
}

/////////////////////////////////////////////////////////////////////////
template <int useRadius, int raySegment>
void RTree::traverseRay(
	const PxVec3& rayOrigin, const PxVec3& rayDir,
	const PxU32 maxResults, PxU32* resultsPtr, Gu::RTree::Callback* callback, const PxVec3& fattenAABBs) const
{
	const PxU32 maxStack = 128;
	PxU32 stack[maxStack];

	PX_ASSERT(mPages);
	PX_ASSERT((PxMemFetchPtr(mPages) & 127) == 0);
	PX_ASSERT((PxMemFetchPtr(this) & 15) == 0);

	PxU32* resultsBegin = resultsPtr;

	v_ushort8 * treeNodes8 = reinterpret_cast<v_ushort8 *>(mPages);
	PxU32 bottomLevelFirstNodeIndexM1 = mBottomLevelFirstNodeIndex-1;
	PxU32* stackPtr;

	stackPtr = stack;

	v_float4 fattenAABBsX, fattenAABBsY, fattenAABBsZ;
	PX_FORCE_PARAMETER_REFERENCE(fattenAABBsX);
	PX_FORCE_PARAMETER_REFERENCE(fattenAABBsY);
	PX_FORCE_PARAMETER_REFERENCE(fattenAABBsZ);
	if (useRadius)
	{
		v_float4 fattenAABBs4 = V4LoadUnaligned((Vec4V*)&fattenAABBs.x);
		fattenAABBsX = v_vspltf(fattenAABBs4, 0);
		fattenAABBsY = v_vspltf(fattenAABBs4, 1);
		fattenAABBsZ = v_vspltf(fattenAABBs4, 2);
	}

	v_uint4 signMask, ffff;
	v_uint4_set4(signMask, (PxU32(1)<<31), (PxU32(1)<<31), (PxU32(1)<<31), (PxU32(1)<<31));
	v_float4 epsFloat4;
	v_float4_set4(epsFloat4, 1e-9f, 1e-9f, 1e-9f, 1e-9f);
	v_float4 ones;
	if (raySegment)
		v_float4_set4(ones, 0.9999999f, 0.9999999f, 0.9999999f, 0.9999999f);
	v_uint4_set4(ffff, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF);
	v_float4 zeroes = v_float4(v_splat_s32(0));
	v_float4 rayP = V4LoadUnaligned((Vec4V*)&rayOrigin.x);
	v_float4 rayD = V4LoadUnaligned((Vec4V*)&rayDir.x);
	v_uint4 raySign = v_and32((v_uint4)rayD, (v_uint4)signMask);
	v_float4 rayDAbs = v_float4(V4Abs(rayD)); // abs value of rayD
	v_float4 rayInvD = v_float4(v_vor32((v_uint4)raySign, (v_uint4)v_vmaxfp(rayDAbs, epsFloat4))); // clamp near-zero components up to epsilon
	rayInvD = v_vrefp(rayInvD);
	// P+tD=a; t=(a-P)/D
	// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
	v_float4 rayPinvD = v_vnmsubfp(rayInvD, rayP, zeroes);
	v_float4 rayInvDsplatX = v_vspltf(rayInvD, 0);
	v_float4 rayInvDsplatY = v_vspltf(rayInvD, 1);
	v_float4 rayInvDsplatZ = v_vspltf(rayInvD, 2);
	v_float4 rayPinvDsplatX = v_vspltf(rayPinvD, 0);
	v_float4 rayPinvDsplatY = v_vspltf(rayPinvD, 1);
	v_float4 rayPinvDsplatZ = v_vspltf(rayPinvD, 2);
	v_float4 scaler = v_lvx_float(&mDiagonalScaler);
	v_float4 treeMin = v_lvx_float(&mBoundsMin);
	v_float4 scalerSplat0 = v_vspltf(scaler, 0);
	v_float4 scalerSplat1 = v_vspltf(scaler, 1);
	v_float4 scalerSplat2 = v_vspltf(scaler, 2);
	v_float4 treeMinSplat0 = v_vspltf(treeMin, 0);
	v_float4 treeMinSplat1 = v_vspltf(treeMin, 1);
	v_float4 treeMinSplat2 = v_vspltf(treeMin, 2);
	v_ushort8 short8_one = v_vspltish(1);

	// top level will be included with the initial DMA fetch in SPU implementation
	PX_ASSERT(mPageSize == 8); // scaffold
	for (PxI32 j = PxI32(mNumRootPages-1); j >= 0; j --)
		*stackPtr++ = j*8;
	PxU32 cacheTopValid = true;
	PxU32 cacheTop = 0;

	do {
		stackPtr--;
		PxU32 top;
		if (cacheTopValid) // branch is faster than lhs
		{
			PX_ASSERT(cacheTop < 0xFFFFF);
			top = cacheTop;
		} else
			top = stackPtr[0];
		PX_ASSERT(!cacheTopValid || top == cacheTop);
		v_ushort8* __restrict tn = treeNodes8 + top;
#ifdef __SPU__
		RTreePage tmpPage;
		pxMemFetchAlignedAsync(PxU64(&tmpPage), PxU64(treeNodes8+top), sizeof(RTreePage), 5);
		pxMemFetchWait(5);
		tn = reinterpret_cast<v_ushort8*>(&tmpPage);
#endif
		PxU32* ptrs = ((RTreePage *)tn)->ptrs;
		PxU32 ptr0 = ptrs[0], ptr1 = ptrs[1], ptr2 = ptrs[2], ptr3 = ptrs[3];
		PxU32 ptr4 = ptrs[4], ptr5 = ptrs[5], ptr6 = ptrs[6], ptr7 = ptrs[7];
		PxU32* rr = resultsPtr;
		PxU32* ss = stackPtr;
		PxU32 bottomLevel = ((bottomLevelFirstNodeIndexM1-top)>>31);

		if (0)
		{
			#define PF prefetch128 // __dcbt
			PF(treeNodes8+ptr0); PF(treeNodes8+ptr1);
			PF(treeNodes8+ptr2); PF(treeNodes8+ptr3);
			PF(treeNodes8+ptr4); PF(treeNodes8+ptr5);
			PF(treeNodes8+ptr6); PF(treeNodes8+ptr7);
			#undef PF
		}

		v_ushort8
			minx8(v_lvx_u16(tn+0)), miny8(v_lvx_u16(tn+1)), minz8(v_lvx_u16(tn+2)),
			maxx8(v_lvx_u16(tn+3)), maxy8(v_lvx_u16(tn+4)), maxz8(v_lvx_u16(tn+5));

		v_float4 minx4a, minx4b, miny4a, miny4b, minz4a, minz4b;
		v_float4 maxx4a, maxx4b, maxy4a, maxy4b, maxz4a, maxz4b;

		DEQ_MIN(minx8, 0, minx4a, minx4b);
		DEQ_MIN(miny8, 1, miny4a, miny4b);
		DEQ_MIN(minz8, 2, minz4a, minz4b);
		DEQ_MAX(maxx8, 0, maxx4a, maxx4b);
		DEQ_MAX(maxy8, 1, maxy4a, maxy4b);
		DEQ_MAX(maxz8, 2, maxz4a, maxz4b);

		v_uint4 ignore4a = (v_uint4)v_vcmpgtfp(minx4a, maxx4a); // 1 if degenerate box (empty slot)
		v_uint4 ignore4b = (v_uint4)v_vcmpgtfp(minx4b, maxx4b);

		if (useRadius)
		{
			maxx4a = v_vaddfp(maxx4a, fattenAABBsX); maxy4a = v_vaddfp(maxy4a, fattenAABBsY); maxz4a = v_vaddfp(maxz4a, fattenAABBsZ);
			maxx4b = v_vaddfp(maxx4b, fattenAABBsX); maxy4b = v_vaddfp(maxy4b, fattenAABBsY); maxz4b = v_vaddfp(maxz4b, fattenAABBsZ);
			minx4a = v_vsubfp(minx4a, fattenAABBsX); miny4a = v_vsubfp(miny4a, fattenAABBsY); minz4a = v_vsubfp(minz4a, fattenAABBsZ);
			minx4b = v_vsubfp(minx4b, fattenAABBsX); miny4b = v_vsubfp(miny4b, fattenAABBsY); minz4b = v_vsubfp(minz4b, fattenAABBsZ);
		}

		// P+tD=a; t=(a-P)/D
		// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
		v_float4 tminxa0 = v_madd(minx4a, rayInvDsplatX, rayPinvDsplatX);
		v_float4 tminxb0 = v_madd(minx4b, rayInvDsplatX, rayPinvDsplatX);
		v_float4 tminya0 = v_madd(miny4a, rayInvDsplatY, rayPinvDsplatY);
		v_float4 tminyb0 = v_madd(miny4b, rayInvDsplatY, rayPinvDsplatY);
		v_float4 tminza0 = v_madd(minz4a, rayInvDsplatZ, rayPinvDsplatZ);
		v_float4 tminzb0 = v_madd(minz4b, rayInvDsplatZ, rayPinvDsplatZ);
		v_float4 tmaxxa0 = v_madd(maxx4a, rayInvDsplatX, rayPinvDsplatX);
		v_float4 tmaxxb0 = v_madd(maxx4b, rayInvDsplatX, rayPinvDsplatX);
		v_float4 tmaxya0 = v_madd(maxy4a, rayInvDsplatY, rayPinvDsplatY);
		v_float4 tmaxyb0 = v_madd(maxy4b, rayInvDsplatY, rayPinvDsplatY);
		v_float4 tmaxza0 = v_madd(maxz4a, rayInvDsplatZ, rayPinvDsplatZ);
		v_float4 tmaxzb0 = v_madd(maxz4b, rayInvDsplatZ, rayPinvDsplatZ);

		// now compute tnear and tfar for each pair of planes for each box
		v_float4 tminxa = v_vminfp(tminxa0, tmaxxa0); v_float4 tmaxxa = v_vmaxfp(tminxa0, tmaxxa0);
		v_float4 tminya = v_vminfp(tminya0, tmaxya0); v_float4 tmaxya = v_vmaxfp(tminya0, tmaxya0);
		v_float4 tminza = v_vminfp(tminza0, tmaxza0); v_float4 tmaxza = v_vmaxfp(tminza0, tmaxza0);
		v_float4 tminxb = v_vminfp(tminxb0, tmaxxb0); v_float4 tmaxxb = v_vmaxfp(tminxb0, tmaxxb0);
		v_float4 tminyb = v_vminfp(tminyb0, tmaxyb0); v_float4 tmaxyb = v_vmaxfp(tminyb0, tmaxyb0);
		v_float4 tminzb = v_vminfp(tminzb0, tmaxzb0); v_float4 tmaxzb = v_vmaxfp(tminzb0, tmaxzb0);

		v_float4 maxOfNeasa = v_vmaxfp(v_vmaxfp(tminxa, tminya), tminza);
		v_float4 maxOfNeasb = v_vmaxfp(v_vmaxfp(tminxb, tminyb), tminzb);
		v_float4 minOfFarsa = v_vminfp(v_vminfp(tmaxxa, tmaxya), tmaxza);
		v_float4 minOfFarsb = v_vminfp(v_vminfp(tmaxxb, tmaxyb), tmaxzb);
		ignore4a = v_vor32(ignore4a, v_vcmpgtfp(epsFloat4, minOfFarsa));  // if tfar is negative, ignore since its a ray, not a line
		ignore4b = v_vor32(ignore4b, v_vcmpgtfp(epsFloat4, minOfFarsb));
		if (raySegment)
		{
			ignore4a = v_vor32(ignore4a, v_vcmpgtfp(maxOfNeasa, ones));  // if tnear is over 1, ignore for ray segments
			ignore4b = v_vor32(ignore4b, v_vcmpgtfp(maxOfNeasb, ones));
		}

		v_uint4 resa4 = (v_uint4)v_vcmpgtfp(maxOfNeasa, minOfFarsa); // if 1 => fail
		v_uint4 resb4 = (v_uint4)v_vcmpgtfp(maxOfNeasb, minOfFarsb); // if 1 => fail
		resa4 = v_vor32(resa4, ignore4a);
		resb4 = v_vor32(resb4, ignore4b);

		PX_ALIGN_PREFIX(16) PxU32 resa[4] PX_ALIGN_SUFFIX(16);
		PX_ALIGN_PREFIX(16) PxU32 resb[4] PX_ALIGN_SUFFIX(16);
		v_uint4 resa4u = v_andc32(v_uint4(v_splat_s32(1)), resa4);
		v_uint4 resb4u = v_andc32(v_uint4(v_splat_s32(1)), resb4);
		v_stvx_32(resa4u, resa);
		v_stvx_32(resb4u, resb);
		// resa -> 1, 3, 5, 7; resb -> 0, 2, 4, 6

		#ifdef VERIFY_RTREE
		// verification
		{
			PxVec3 mn0, mx0, mn1, mx1, mn2, mx2, mn3, mx3, mn4, mx4, mn5, mx5, mn6, mx6, mn7, mx7;
			// radius is already added/subtracted from dimensions
			mn0 = PxVec3(readX(minx4a), readX(miny4a), readX(minz4a)); mx0 = PxVec3(readX(maxx4a), readX(maxy4a), readX(maxz4a));
			mn1 = PxVec3(readY(minx4a), readY(miny4a), readY(minz4a)); mx1 = PxVec3(readY(maxx4a), readY(maxy4a), readY(maxz4a));
			mn2 = PxVec3(readZ(minx4a), readZ(miny4a), readZ(minz4a)); mx2 = PxVec3(readZ(maxx4a), readZ(maxy4a), readZ(maxz4a));
			mn3 = PxVec3(readW(minx4a), readW(miny4a), readW(minz4a)); mx3 = PxVec3(readW(maxx4a), readW(maxy4a), readW(maxz4a));
			mn4 = PxVec3(readX(minx4b), readX(miny4b), readX(minz4b)); mx4 = PxVec3(readX(maxx4b), readX(maxy4b), readX(maxz4b));
			mn5 = PxVec3(readY(minx4b), readY(miny4b), readY(minz4b)); mx5 = PxVec3(readY(maxx4b), readY(maxy4b), readY(maxz4b));
			mn6 = PxVec3(readZ(minx4b), readZ(miny4b), readZ(minz4b)); mx6 = PxVec3(readZ(maxx4b), readZ(maxy4b), readZ(maxz4b));
			mn7 = PxVec3(readW(minx4b), readW(miny4b), readW(minz4b)); mx7 = PxVec3(readW(maxx4b), readW(maxy4b), readW(maxz4b));
			PxF32 tn, tf;
			PxU32* ignore4a_ = (PxU32*)&ignore4a;
			PxU32* ignore4b_ = (PxU32*)&ignore4b;
			if (raySegment)
			{
				if (!ignore4a_[0] && resa[0] != (intersectRayAABB(mn0, mx0, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4a_[1] && resa[1] != (intersectRayAABB(mn1, mx1, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4a_[2] && resa[2] != (intersectRayAABB(mn2, mx2, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4a_[3] && resa[3] != (intersectRayAABB(mn3, mx3, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[0] && resb[0] != (intersectRayAABB(mn4, mx4, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[1] && resb[1] != (intersectRayAABB(mn5, mx5, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[2] && resb[2] != (intersectRayAABB(mn6, mx6, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[3] && resb[3] != (intersectRayAABB(mn7, mx7, rayOrigin, rayDir, tn, tf)>=0) && tn <= 1.0f && fabsf(tf-tn)>0.01f)
					bool break_here = true;
			} else
			{
				if (!ignore4a_[0] && resa[0] != (intersectRayAABB(mn0, mx0, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4a_[1] && resa[1] != (intersectRayAABB(mn1, mx1, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4a_[2] && resa[2] != (intersectRayAABB(mn2, mx2, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4a_[3] && resa[3] != (intersectRayAABB(mn3, mx3, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[0] && resb[0] != (intersectRayAABB(mn4, mx4, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[1] && resb[1] != (intersectRayAABB(mn5, mx5, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[2] && resb[2] != (intersectRayAABB(mn6, mx6, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
				if (!ignore4b_[3] && resb[3] != (intersectRayAABB(mn7, mx7, rayOrigin, rayDir, tn, tf)>=0) && fabsf(tf-tn)>0.01f)
					bool break_here = true;
			}
		}
		#endif // VERIFY_RTREE

		// results got swizzled during decoding to floats
		PxU32 sum0 = 0;
		PxU32 sum1 = 0 + resa[0];
		PxU32 sum2 = sum1 + resb[0];
		PxU32 sum3 = sum2 + resa[1];
		PxU32 sum4 = sum3 + resb[1];
		PxU32 sum5 = sum4 + resa[2];
		PxU32 sum6 = sum5 + resb[2];
		PxU32 sum7 = sum6 + resa[3];
		PxU32 sum8 = sum7 + resb[3];

		if (bottomLevel) {
			cacheTopValid = true;
			cacheTop = stackPtr[-1];
			rr[sum0] = ptr0;
			rr[sum1] = ptr1;
			rr[sum2] = ptr2;
			rr[sum3] = ptr3;
			rr[sum4] = ptr4;
			rr[sum5] = ptr5;
			rr[sum6] = ptr6;
			rr[sum7] = ptr7;
			resultsPtr = rr + sum8;
			if (rr+sum8+mPageSize > resultsBegin+maxResults)
			{
				// flush partial results via callback.
				// We flush mPageSize entries too early to make it so the api works with immediate early out for maxResults = 8
				if (!callback->processResults(PxU32((rr+sum8)-resultsBegin), resultsBegin))
					return;
				resultsPtr = resultsBegin;
			}
		} else {
			PxU32 bitmask = (resa[0]<<24)|(resb[0]<<25)|(resa[1]<<26)|(resb[1]<<27)|(resa[2]<<28)|(resb[2]<<29)|(resa[3]<<30)|(resb[3]<<31);
			PxI32 lastIndex = 7 - PxI32(countLeadingZeros(bitmask));
			cacheTopValid = (lastIndex >= 0);
			// worst case scenario is 7-32=-25
			// it just happens that we can reference up to ptrs[-25] safely since sizeof(RTreePage) is 128
			// so we don't need to clamp lastIndex here
			cacheTop = ptrs[lastIndex];
			ss[sum0] = ptr0;
			ss[sum1] = ptr1;
			ss[sum2] = ptr2;
			ss[sum3] = ptr3;
			ss[sum4] = ptr4;
			ss[sum5] = ptr5;
			ss[sum6] = ptr6;
			ss[sum7] = ptr7;
			stackPtr = ss + sum8;
		}
	} while (stackPtr > stack);

	if (resultsPtr-resultsBegin > 0)
		callback->processResults(PxU32(resultsPtr-resultsBegin), resultsBegin);
}

template void RTree::traverseRay<0,0>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
template void RTree::traverseRay<0,1>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
template void RTree::traverseRay<1,0>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;
template void RTree::traverseRay<1,1>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::Callback*, const PxVec3&) const;

/////////////////////////////////////////////////////////////////////////
void RTree::traverseOBB(
	const Gu::Box& obb, const PxU32 maxResults, PxU32* resultsPtr, Gu::RTree::Callback* callback) const
{
	const PxU32 maxStack = 128;
	PxU32 stack[maxStack];

	PX_ASSERT(mPages);
	PX_ASSERT((PxMemFetchPtr(mPages) & 127) == 0);
	PX_ASSERT((PxMemFetchPtr(this) & 15) == 0);

	PxU32* resultsBegin = resultsPtr;

	v_ushort8 * treeNodes8 = reinterpret_cast<v_ushort8 *>(mPages);
	PxU32 bottomLevelFirstNodeIndexM1 = mBottomLevelFirstNodeIndex-1;
	PxU32* stackPtr;

	stackPtr = stack;

	v_uint4 signMask, ffff;
	v_float4 ones, halves, eps;
	v_uint4_set4(signMask, PxU32(1)<<31, PxU32(1)<<31, PxU32(1)<<31, PxU32(1)<<31);
	v_float4_set4(ones, 1.0f, 1.0f, 1.0f, 1.0f);
	v_float4_set4(halves, 0.5f, 0.5f, 0.5f, 0.5f);
	v_float4_set4(eps, 1e-6f, 1e-6f, 1e-6f, 1e-6f);
	v_uint4_set4(ffff, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF);
	
	PX_UNUSED(ones);

	v_float4 zeroes; v_float4_set4(zeroes, 0.0f, 0.0f, 0.0f, 0.0f);
	v_ushort8 short8_one = v_vspltish(1);
	// project 
	v_float4 obbO = V4LoadUnaligned((Vec4V*)&obb.center.x);
	v_float4 obbE = V4LoadUnaligned((Vec4V*)&obb.extents.x);
	// Gu::Box::rot matrix columns are the OBB axes
	v_float4 obbX = V4LoadUnaligned((Vec4V*)&obb.rot.column0.x);
	v_float4 obbY = V4LoadUnaligned((Vec4V*)&obb.rot.column1.x);
	v_float4 obbZ = V4LoadUnaligned((Vec4V*)&obb.rot.column2.x);

	v_float4 scaler = V4LoadAligned((Vec4V*)&mDiagonalScaler);
	v_float4 treeMin = V4LoadAligned((Vec4V*)&mBoundsMin);
	v_float4 scalerSplat0 = v_vspltf(scaler, 0);
	v_float4 scalerSplat1 = v_vspltf(scaler, 1);
	v_float4 scalerSplat2 = v_vspltf(scaler, 2);
	v_float4 treeMinSplat0 = v_vspltf(treeMin, 0);
	v_float4 treeMinSplat1 = v_vspltf(treeMin, 1);
	v_float4 treeMinSplat2 = v_vspltf(treeMin, 2);
#ifdef PX_WINDOWS
	// Visual Studio compiler hangs with #defines
	// On VMX platforms we use #defines in the other branch of this #ifdef to avoid register spills (LHS)
	v_float4 obbESplatX = v_vspltf(obbE, 0);
	v_float4 obbESplatY = v_vspltf(obbE, 1);
	v_float4 obbESplatZ = v_vspltf(obbE, 2);
	v_float4 obbESplatNegX = v_vsubfp(zeroes, obbESplatX);
	v_float4 obbESplatNegY = v_vsubfp(zeroes, obbESplatY);
	v_float4 obbESplatNegZ = v_vsubfp(zeroes, obbESplatZ);
	v_float4 obbXE = v_madd(obbX, obbESplatX, zeroes); // scale axii by E
	v_float4 obbYE = v_madd(obbY, obbESplatY, zeroes); // scale axii by E
	v_float4 obbZE = v_madd(obbZ, obbESplatZ, zeroes); // scale axii by E
	v_float4 obbOSplatX = v_vspltf(obbO, 0);
	v_float4 obbOSplatY = v_vspltf(obbO, 1);
	v_float4 obbOSplatZ = v_vspltf(obbO, 2);
	v_float4 obbXSplatX = v_vspltf(obbX, 0);
	v_float4 obbXSplatY = v_vspltf(obbX, 1);
	v_float4 obbXSplatZ = v_vspltf(obbX, 2);
	v_float4 obbYSplatX = v_vspltf(obbY, 0);
	v_float4 obbYSplatY = v_vspltf(obbY, 1);
	v_float4 obbYSplatZ = v_vspltf(obbY, 2);
	v_float4 obbZSplatX = v_vspltf(obbZ, 0);
	v_float4 obbZSplatY = v_vspltf(obbZ, 1);
	v_float4 obbZSplatZ = v_vspltf(obbZ, 2);
	v_float4 obbXESplatX = v_vspltf(obbXE, 0);
	v_float4 obbXESplatY = v_vspltf(obbXE, 1);
	v_float4 obbXESplatZ = v_vspltf(obbXE, 2);
	v_float4 obbYESplatX = v_vspltf(obbYE, 0);
	v_float4 obbYESplatY = v_vspltf(obbYE, 1);
	v_float4 obbYESplatZ = v_vspltf(obbYE, 2);
	v_float4 obbZESplatX = v_vspltf(obbZE, 0);
	v_float4 obbZESplatY = v_vspltf(obbZE, 1);
	v_float4 obbZESplatZ = v_vspltf(obbZE, 2);
#else
	#define obbESplatX v_vspltf(obbE, 0)
	#define obbESplatY v_vspltf(obbE, 1)
	#define obbESplatZ v_vspltf(obbE, 2)
	#define obbESplatNegX v_vsubfp(zeroes, obbESplatX)
	#define obbESplatNegY v_vsubfp(zeroes, obbESplatY)
	#define obbESplatNegZ v_vsubfp(zeroes, obbESplatZ)
	#define obbXE v_madd(obbX, obbESplatX, zeroes)
	#define obbYE v_madd(obbY, obbESplatY, zeroes)
	#define obbZE v_madd(obbZ, obbESplatZ, zeroes)
	#define obbOSplatX v_vspltf(obbO, 0)
	#define obbOSplatY v_vspltf(obbO, 1)
	#define obbOSplatZ v_vspltf(obbO, 2)
	#define obbXSplatX v_vspltf(obbX, 0)
	#define obbXSplatY v_vspltf(obbX, 1)
	#define obbXSplatZ v_vspltf(obbX, 2)
	#define obbYSplatX v_vspltf(obbY, 0)
	#define obbYSplatY v_vspltf(obbY, 1)
	#define obbYSplatZ v_vspltf(obbY, 2)
	#define obbZSplatX v_vspltf(obbZ, 0)
	#define obbZSplatY v_vspltf(obbZ, 1)
	#define obbZSplatZ v_vspltf(obbZ, 2)
	#define obbXESplatX v_vspltf(obbXE, 0)
	#define obbXESplatY v_vspltf(obbXE, 1)
	#define obbXESplatZ v_vspltf(obbXE, 2)
	#define obbYESplatX v_vspltf(obbYE, 0)
	#define obbYESplatY v_vspltf(obbYE, 1)
	#define obbYESplatZ v_vspltf(obbYE, 2)
	#define obbZESplatX v_vspltf(obbZE, 0)
	#define obbZESplatY v_vspltf(obbZE, 1)
	#define obbZESplatZ v_vspltf(obbZE, 2)
#endif

	// top level will be included with the initial DMA fetch in SPU implementation
	PX_ASSERT(mPageSize == 8);
	for (PxI32 j = PxI32(mNumRootPages-1); j >= 0; j --)
		*stackPtr++ = j*8;
	PxU32 cacheTopValid = true;
	PxU32 cacheTop = 0;

	PX_ALIGN_PREFIX(16) PxU32 resa_[4] PX_ALIGN_SUFFIX(16);
	PX_ALIGN_PREFIX(16) PxU32 resb_[4] PX_ALIGN_SUFFIX(16);

	do {
		stackPtr--;
		PxU32 top;
		if (cacheTopValid) // branch is faster than lhs
		{
			PX_ASSERT(cacheTop < 0xFFFFF);
			top = cacheTop;
		} else
			top = stackPtr[0];
		PX_ASSERT(!cacheTopValid || top == cacheTop);
		v_ushort8* __restrict tn = treeNodes8 + top;
#ifdef __SPU__
		RTreePage tmpPage;
		pxMemFetchAlignedAsync(PxU64(&tmpPage), PxU64(treeNodes8+top), sizeof(RTreePage), 5);
		pxMemFetchWait(5);
		tn = reinterpret_cast<v_ushort8*>(&tmpPage);
#endif
		PxU32* ptrs = ((RTreePage *)tn)->ptrs;
		PxU32 ptr0 = ptrs[0], ptr1 = ptrs[1], ptr2 = ptrs[2], ptr3 = ptrs[3];
		PxU32 ptr4 = ptrs[4], ptr5 = ptrs[5], ptr6 = ptrs[6], ptr7 = ptrs[7];
		PxU32* rr = resultsPtr;
		PxU32* ss = stackPtr;
		PxU32 bottomLevel = ((bottomLevelFirstNodeIndexM1-top)>>31);

		if (0)
		{
			#define PF prefetch128 // __dcbt
			PF(treeNodes8+ptr0); PF(treeNodes8+ptr1); PF(treeNodes8+ptr2); PF(treeNodes8+ptr3);
			PF(treeNodes8+ptr4); PF(treeNodes8+ptr5); PF(treeNodes8+ptr6); PF(treeNodes8+ptr7);
			#undef PF
		}

		v_ushort8
			minx8(v_lvx_u16(tn+0)), miny8(v_lvx_u16(tn+1)), minz8(v_lvx_u16(tn+2)),
			maxx8(v_lvx_u16(tn+3)), maxy8(v_lvx_u16(tn+4)), maxz8(v_lvx_u16(tn+5));

		v_float4 minx4a, minx4b, miny4a, miny4b, minz4a, minz4b;
		v_float4 maxx4a, maxx4b, maxy4a, maxy4b, maxz4a, maxz4b;

		// 8instr each/48 instructions
		DEQ_MIN(minx8, 0, minx4a, minx4b);
		DEQ_MIN(miny8, 1, miny4a, miny4b);
		DEQ_MIN(minz8, 2, minz4a, minz4b);
		DEQ_MAX(maxx8, 0, maxx4a, maxx4b);
		DEQ_MAX(maxy8, 1, maxy4a, maxy4b);
		DEQ_MAX(maxz8, 2, maxz4a, maxz4b);

		v_uint4 noOverlapa, noOverlapb;
		v_uint4 resa4u, resb4u;
		// PRECOMPUTE FOR A BLOCK
		// 109 instr per 4 OBB/AABB
		// ABB iteration 1, start with OBB origin as other point -- 6
		v_float4 p1ABBxa = v_vmaxfp(minx4a, v_vminfp(maxx4a, obbOSplatX));
		v_float4 p1ABBya = v_vmaxfp(miny4a, v_vminfp(maxy4a, obbOSplatY));
		v_float4 p1ABBza = v_vmaxfp(minz4a, v_vminfp(maxz4a, obbOSplatZ));

		// OBB iteration 1, move to OBB space first -- 12
		v_float4 p1ABBOxa = v_vsubfp(p1ABBxa, obbOSplatX);
		v_float4 p1ABBOya = v_vsubfp(p1ABBya, obbOSplatY);
		v_float4 p1ABBOza = v_vsubfp(p1ABBza, obbOSplatZ);
		v_float4 obbPrjXa = v_madd(p1ABBOxa, obbXSplatX, v_madd(p1ABBOya, obbXSplatY, v_madd(p1ABBOza, obbXSplatZ, zeroes)));
		v_float4 obbPrjYa = v_madd(p1ABBOxa, obbYSplatX, v_madd(p1ABBOya, obbYSplatY, v_madd(p1ABBOza, obbYSplatZ, zeroes)));
		v_float4 obbPrjZa = v_madd(p1ABBOxa, obbZSplatX, v_madd(p1ABBOya, obbZSplatY, v_madd(p1ABBOza, obbZSplatZ, zeroes)));
		// clamp AABB point in OBB space to OBB extents. Since we scaled the axii, the extents are [-1,1] -- 6
		v_float4 pOBBxa = v_vmaxfp(obbESplatNegX, v_vminfp(obbPrjXa, obbESplatX));
		v_float4 pOBBya = v_vmaxfp(obbESplatNegY, v_vminfp(obbPrjYa, obbESplatY));
		v_float4 pOBBza = v_vmaxfp(obbESplatNegZ, v_vminfp(obbPrjZa, obbESplatZ));
		// go back to AABB space. we have x,y,z in obb space, need to multiply by axii -- 9
		v_float4 p1OBBxa = v_madd(pOBBxa, obbXSplatX, v_madd(pOBBya, obbYSplatX, v_madd(pOBBza, obbZSplatX, obbOSplatX)));
		v_float4 p1OBBya = v_madd(pOBBxa, obbXSplatY, v_madd(pOBBya, obbYSplatY, v_madd(pOBBza, obbZSplatY, obbOSplatY)));
		v_float4 p1OBBza = v_madd(pOBBxa, obbXSplatZ, v_madd(pOBBya, obbYSplatZ, v_madd(pOBBza, obbZSplatZ, obbOSplatZ)));

		// ABB iteration 2 -- 6 instructions
		v_float4 p2ABBxa = v_vmaxfp(minx4a, v_vminfp(maxx4a, p1OBBxa));
		v_float4 p2ABBya = v_vmaxfp(miny4a, v_vminfp(maxy4a, p1OBBya));
		v_float4 p2ABBza = v_vmaxfp(minz4a, v_vminfp(maxz4a, p1OBBza));
		// above blocks add up to 12+12+15=39 instr
		// END PRECOMPUTE FOR A BLOCK

		// PRECOMPUTE FOR B BLOCK
		// ABB iteration 1, start with OBB origin as other point -- 6 instructions
		v_float4 p1ABBxb = v_vmaxfp(minx4b, v_vminfp(maxx4b, obbOSplatX));
		v_float4 p1ABByb = v_vmaxfp(miny4b, v_vminfp(maxy4b, obbOSplatY));
		v_float4 p1ABBzb = v_vmaxfp(minz4b, v_vminfp(maxz4b, obbOSplatZ));

		// OBB iteration 1, move to OBB space first -- 12 instructions
		v_float4 p1ABBOxb = v_vsubfp(p1ABBxb, obbOSplatX);
		v_float4 p1ABBOyb = v_vsubfp(p1ABByb, obbOSplatY);
		v_float4 p1ABBOzb = v_vsubfp(p1ABBzb, obbOSplatZ);
		v_float4 obbPrjXb = v_madd(p1ABBOxb, obbXSplatX, v_madd(p1ABBOyb, obbXSplatY, v_madd(p1ABBOzb, obbXSplatZ, zeroes)));
		v_float4 obbPrjYb = v_madd(p1ABBOxb, obbYSplatX, v_madd(p1ABBOyb, obbYSplatY, v_madd(p1ABBOzb, obbYSplatZ, zeroes)));
		v_float4 obbPrjZb = v_madd(p1ABBOxb, obbZSplatX, v_madd(p1ABBOyb, obbZSplatY, v_madd(p1ABBOzb, obbZSplatZ, zeroes)));
		// clamp AABB point in OBB space to OBB extents. Since we scaled the axii, the extents are [-1,1] -- 6 instructions
		v_float4 pOBBxb = v_vmaxfp(obbESplatNegX, v_vminfp(obbPrjXb, obbESplatX));
		v_float4 pOBByb = v_vmaxfp(obbESplatNegY, v_vminfp(obbPrjYb, obbESplatY));
		v_float4 pOBBzb = v_vmaxfp(obbESplatNegZ, v_vminfp(obbPrjZb, obbESplatZ));
		// go back to AABB space. we have x,y,z in obb space, need to multiply by axii -- 9 instructions
		v_float4 p1OBBxb = v_madd(pOBBxb, obbXSplatX, v_madd(pOBByb, obbYSplatX, v_madd(pOBBzb, obbZSplatX, obbOSplatX)));
		v_float4 p1OBByb = v_madd(pOBBxb, obbXSplatY, v_madd(pOBByb, obbYSplatY, v_madd(pOBBzb, obbZSplatY, obbOSplatY)));
		v_float4 p1OBBzb = v_madd(pOBBxb, obbXSplatZ, v_madd(pOBByb, obbYSplatZ, v_madd(pOBBzb, obbZSplatZ, obbOSplatZ)));

		// ABB iteration 2 -- 6 instructions
		v_float4 p2ABBxb = v_vmaxfp(minx4b, v_vminfp(maxx4b, p1OBBxb));
		v_float4 p2ABByb = v_vmaxfp(miny4b, v_vminfp(maxy4b, p1OBByb));
		v_float4 p2ABBzb = v_vmaxfp(minz4b, v_vminfp(maxz4b, p1OBBzb));
		// 12+12+15=39 instr
		// END PRECOMPUTE FOR B BLOCK

		// for AABBs precompute extents and center -- 18
		v_float4 abbCxa = v_madd(v_vaddfp(maxx4a, minx4a), halves, zeroes);
		v_float4 abbCya = v_madd(v_vaddfp(maxy4a, miny4a), halves, zeroes);
		v_float4 abbCza = v_madd(v_vaddfp(maxz4a, minz4a), halves, zeroes);
		v_float4 abbExa = v_vsubfp(maxx4a, abbCxa);
		v_float4 abbEya = v_vsubfp(maxy4a, abbCya);
		v_float4 abbEza = v_vsubfp(maxz4a, abbCza);
		v_float4 abbCxb = v_madd(v_vaddfp(maxx4b, minx4b), halves, zeroes);
		v_float4 abbCyb = v_madd(v_vaddfp(maxy4b, miny4b), halves, zeroes);
		v_float4 abbCzb = v_madd(v_vaddfp(maxz4b, minz4b), halves, zeroes);
		v_float4 abbExb = v_vsubfp(maxx4b, abbCxb);
		v_float4 abbEyb = v_vsubfp(maxy4b, abbCyb);
		v_float4 abbEzb = v_vsubfp(maxz4b, abbCzb);

		// now test separating axes D1 = p1OBB-p1ABB and D2 = p1OBB-p2ABB -- 37 instructions per axis
		// D1 first -- 3 instructions
		v_float4 d1xa = v_vsubfp(p1OBBxa, p1ABBxa), d1ya = v_vsubfp(p1OBBya, p1ABBya), d1za = v_vsubfp(p1OBBza, p1ABBza);
		// for AABB compute projections of extents and center -- 6
		v_float4 abbExd1Prja = v_madd(d1xa, abbExa, zeroes);
		v_float4 abbEyd1Prja = v_madd(d1ya, abbEya, zeroes);
		v_float4 abbEzd1Prja = v_madd(d1za, abbEza, zeroes);
		v_float4 abbCd1Prja = v_madd(d1xa, abbCxa, v_madd(d1ya, abbCya, v_madd(d1za, abbCza, zeroes)));
		// for obb project each halfaxis and origin and add abs values of half-axis projections -- 12 instructions
		v_float4 obbXEd1Prja = v_madd(d1xa, obbXESplatX, v_madd(d1ya, obbXESplatY, v_madd(d1za, obbXESplatZ, zeroes)));
		v_float4 obbYEd1Prja = v_madd(d1xa, obbYESplatX, v_madd(d1ya, obbYESplatY, v_madd(d1za, obbYESplatZ, zeroes)));
		v_float4 obbZEd1Prja = v_madd(d1xa, obbZESplatX, v_madd(d1ya, obbZESplatY, v_madd(d1za, obbZESplatZ, zeroes)));
		v_float4 obbOd1Prja = v_madd(d1xa, obbOSplatX, v_madd(d1ya, obbOSplatY, v_madd(d1za, obbOSplatZ, zeroes)));
		// compare lengths between projected centers with sum of projected radii
		v_float4 originDiffd1a = v_absm(v_vsubfp(abbCd1Prja, obbOd1Prja));
		v_float4 absABBRd1a = v_vaddfp(v_vaddfp(v_absm(abbExd1Prja), v_absm(abbEyd1Prja)), v_absm(abbEzd1Prja));
		v_float4 absOBBRd1a = v_vaddfp(v_vaddfp(v_absm(obbXEd1Prja), v_absm(obbYEd1Prja)), v_absm(obbZEd1Prja));
		v_uint4 noOverlapd1a = (v_uint4)v_vcmpgtfp(v_vsubfp(originDiffd1a, eps), v_vaddfp(absABBRd1a, absOBBRd1a));
		v_uint4 epsNoOverlapd1a = (v_uint4)v_vcmpgtfp(originDiffd1a, eps);

		// D2 next (35 instr)
		v_float4 d2xa = v_vsubfp(p1OBBxa, p2ABBxa), d2ya = v_vsubfp(p1OBBya, p2ABBya), d2za = v_vsubfp(p1OBBza, p2ABBza);
		// for AABB compute projections of extents and center -- 6
		v_float4 abbExd2Prja = v_madd(d2xa, abbExa, zeroes);
		v_float4 abbEyd2Prja = v_madd(d2ya, abbEya, zeroes);
		v_float4 abbEzd2Prja = v_madd(d2za, abbEza, zeroes);
		v_float4 abbCd2Prja = v_madd(d2xa, abbCxa, v_madd(d2ya, abbCya, v_madd(d2za, abbCza, zeroes)));
		// for obb project each halfaxis and origin and add abs values of half-axis projections
		v_float4 obbXEd2Prja = v_madd(d2xa, obbXESplatX, v_madd(d2ya, obbXESplatY, v_madd(d2za, obbXESplatZ, zeroes)));
		v_float4 obbYEd2Prja = v_madd(d2xa, obbYESplatX, v_madd(d2ya, obbYESplatY, v_madd(d2za, obbYESplatZ, zeroes)));
		v_float4 obbZEd2Prja = v_madd(d2xa, obbZESplatX, v_madd(d2ya, obbZESplatY, v_madd(d2za, obbZESplatZ, zeroes)));
		v_float4 obbOd2Prja = v_madd(d2xa, obbOSplatX, v_madd(d2ya, obbOSplatY, v_madd(d2za, obbOSplatZ, zeroes)));
		// compare lengths between projected centers with sum of projected radii
		v_float4 originDiffd2a = v_absm(v_vsubfp(abbCd2Prja, obbOd2Prja));
		v_float4 absABBRd2a = v_vaddfp(v_vaddfp(v_absm(abbExd2Prja), v_absm(abbEyd2Prja)), v_absm(abbEzd2Prja));
		v_float4 absOBBRd2a = v_vaddfp(v_vaddfp(v_absm(obbXEd2Prja), v_absm(obbYEd2Prja)), v_absm(obbZEd2Prja));
		v_uint4 noOverlapd2a = (v_uint4)v_vcmpgtfp(v_vsubfp(originDiffd2a, eps), v_vaddfp(absABBRd2a, absOBBRd2a));
		v_uint4 epsNoOverlapd2a = (v_uint4)v_vcmpgtfp(originDiffd2a, eps);

		// Also might test the cross product of D1 and D2 (extra 3 instr for cross product) = 38 instr for D3
		// D3 = (y1z2 - z1y2, z1x2-x1z2, x1y2-y1x2)
		#if 0
		v_float4 d3xa = v_vnmsubfp(d1za, d2ya, v_madd(d1ya, d2za, zeroes));
		v_float4 d3ya = v_vnmsubfp(d1xa, d2za, v_madd(d1za, d2xa, zeroes));
		v_float4 d3za = v_vnmsubfp(d1ya, d2xa, v_madd(d1xa, d2ya, zeroes));
		#endif
		noOverlapa = v_vor32(v_and32(noOverlapd1a, epsNoOverlapd1a), v_and32(noOverlapd2a, epsNoOverlapd2a));
		v_uint4 ignore4a = (v_uint4)v_vcmpgtfp(minx4a, maxx4a); // 1 if degenerate box (empty slot)
		noOverlapa = v_vor32(noOverlapa, ignore4a);
		resa4u = v_andc32(v_uint4(v_splat_s32(1)), noOverlapa); // 1 & ~noOverlap
		v_stvx_32(resa4u, resa_);

		// resa -> 1, 3, 5, 7; resb -> 0, 2, 4, 6
		// ---------------------------------- end of A block

		// now test separating axes D1 = p1OBB-p1ABB and D2 = p1OBB-p2ABB -- 37 instructions per axis
		// D1 first -- 3
		v_float4 d1xb = v_vsubfp(p1OBBxb, p1ABBxb), d1yb = v_vsubfp(p1OBByb, p1ABByb), d1zb = v_vsubfp(p1OBBzb, p1ABBzb);
		// for AABB compute projections of extents and center -- 6
		v_float4 abbExd1Prjb = v_madd(d1xb, abbExb, zeroes);
		v_float4 abbEyd1Prjb = v_madd(d1yb, abbEyb, zeroes);
		v_float4 abbEzd1Prjb = v_madd(d1zb, abbEzb, zeroes);
		v_float4 abbCd1Prjb = v_madd(d1xb, abbCxb, v_madd(d1yb, abbCyb, v_madd(d1zb, abbCzb, zeroes)));
		// for obb project each halfaxis and origin and add abs values of half-axis projections -- 12
		v_float4 obbXEd1Prjb = v_madd(d1xb, obbXESplatX, v_madd(d1yb, obbXESplatY, v_madd(d1zb, obbXESplatZ, zeroes)));
		v_float4 obbYEd1Prjb = v_madd(d1xb, obbYESplatX, v_madd(d1yb, obbYESplatY, v_madd(d1zb, obbYESplatZ, zeroes)));
		v_float4 obbZEd1Prjb = v_madd(d1xb, obbZESplatX, v_madd(d1yb, obbZESplatY, v_madd(d1zb, obbZESplatZ, zeroes)));
		v_float4 obbOd1Prjb = v_madd(d1xb, obbOSplatX, v_madd(d1yb, obbOSplatY, v_madd(d1zb, obbOSplatZ, zeroes)));
		// compare lengths between projected centers with sum of projected radii -- 12
		v_float4 originDiffd1b = v_absm(v_vsubfp(abbCd1Prjb, obbOd1Prjb));
		v_float4 absABBRd1b = v_vaddfp(v_vaddfp(v_absm(abbExd1Prjb), v_absm(abbEyd1Prjb)), v_absm(abbEzd1Prjb));
		v_float4 absOBBRd1b = v_vaddfp(v_vaddfp(v_absm(obbXEd1Prjb), v_absm(obbYEd1Prjb)), v_absm(obbZEd1Prjb));
		v_uint4 noOverlapd1b = (v_uint4)v_vcmpgtfp(v_vsubfp(originDiffd1b, eps), v_vaddfp(absABBRd1b, absOBBRd1b));
		v_uint4 epsNoOverlapd1b = (v_uint4)v_vcmpgtfp(originDiffd1b, eps);

		// D2 next (35 instr)
		v_float4 d2xb = v_vsubfp(p1OBBxb, p2ABBxb), d2yb = v_vsubfp(p1OBByb, p2ABByb), d2zb = v_vsubfp(p1OBBzb, p2ABBzb);
		// for AABB compute projections of extents and center -- 6
		v_float4 abbExd2Prjb = v_madd(d2xb, abbExb, zeroes);
		v_float4 abbEyd2Prjb = v_madd(d2yb, abbEyb, zeroes);
		v_float4 abbEzd2Prjb = v_madd(d2zb, abbEzb, zeroes);
		v_float4 abbCd2Prjb = v_madd(d2xb, abbCxb, v_madd(d2yb, abbCyb, v_madd(d2zb, abbCzb, zeroes)));
		// for obb project each halfaxis and origin and add abs values of half-axis projections
		v_float4 obbXEd2Prjb = v_madd(d2xb, obbXESplatX, v_madd(d2yb, obbXESplatY, v_madd(d2zb, obbXESplatZ, zeroes)));
		v_float4 obbYEd2Prjb = v_madd(d2xb, obbYESplatX, v_madd(d2yb, obbYESplatY, v_madd(d2zb, obbYESplatZ, zeroes)));
		v_float4 obbZEd2Prjb = v_madd(d2xb, obbZESplatX, v_madd(d2yb, obbZESplatY, v_madd(d2zb, obbZESplatZ, zeroes)));
		v_float4 obbOd2Prjb = v_madd(d2xb, obbOSplatX, v_madd(d2yb, obbOSplatY, v_madd(d2zb, obbOSplatZ, zeroes)));
		// compare lengths between projected centers with sum of projected radii
		v_float4 originDiffd2b = v_absm(v_vsubfp(abbCd2Prjb, obbOd2Prjb));
		v_float4 absABBRd2b = v_vaddfp(v_vaddfp(v_absm(abbExd2Prjb), v_absm(abbEyd2Prjb)), v_absm(abbEzd2Prjb));
		v_float4 absOBBRd2b = v_vaddfp(v_vaddfp(v_absm(obbXEd2Prjb), v_absm(obbYEd2Prjb)), v_absm(obbZEd2Prjb));
		v_uint4 noOverlapd2b = (v_uint4)v_vcmpgtfp(v_vsubfp(originDiffd2b, eps), v_vaddfp(absABBRd2b, absOBBRd2b));
		v_uint4 epsNoOverlapd2b = (v_uint4)v_vcmpgtfp(originDiffd2b, eps);

		// Also might test the cross product of D1 and D2 (extrb 3 instr for cross product) = 38 instr for D3
		// D3 = (y1z2 - z1y2, z1x2-x1z2, x1y2-y1x2)
		#if 0
		v_float4 d3xb = v_vnmsubfp(d1zb, d2yb, v_madd(d1yb, d2zb, zeroes));
		v_float4 d3yb = v_vnmsubfp(d1xb, d2zb, v_madd(d1zb, d2xb, zeroes));
		v_float4 d3zb = v_vnmsubfp(d1yb, d2xb, v_madd(d1xb, d2yb, zeroes));
		#endif
		noOverlapb = v_vor32(v_and32(noOverlapd1b, epsNoOverlapd1b), v_and32(noOverlapd2b, epsNoOverlapd2b));
		v_uint4 ignore4b = (v_uint4)v_vcmpgtfp(minx4b, maxx4b); // 1 if degenerate box (empty slot)
		noOverlapb = v_vor32(noOverlapb, ignore4b);
		resb4u = v_andc32(v_uint4(v_splat_s32(1)), noOverlapb);
		v_stvx_32(resb4u, resb_);
		// --------------------------------------- end of B block

		#ifdef VERIFY_RTREE
		{
			PxVec3 mn0, mx0, mn1, mx1, mn2, mx2, mn3, mx3, mn4, mx4, mn5, mx5, mn6, mx6, mn7, mx7;
			mn0 = PxVec3(readX(minx4a), readX(miny4a), readX(minz4a)); mx0 = PxVec3(readX(maxx4a), readX(maxy4a), readX(maxz4a));
			mn1 = PxVec3(readY(minx4a), readY(miny4a), readY(minz4a)); mx1 = PxVec3(readY(maxx4a), readY(maxy4a), readY(maxz4a));
			mn2 = PxVec3(readZ(minx4a), readZ(miny4a), readZ(minz4a)); mx2 = PxVec3(readZ(maxx4a), readZ(maxy4a), readZ(maxz4a));
			mn3 = PxVec3(readW(minx4a), readW(miny4a), readW(minz4a)); mx3 = PxVec3(readW(maxx4a), readW(maxy4a), readW(maxz4a));
			mn4 = PxVec3(readX(minx4b), readX(miny4b), readX(minz4b)); mx4 = PxVec3(readX(maxx4b), readX(maxy4b), readX(maxz4b));
			mn5 = PxVec3(readY(minx4b), readY(miny4b), readY(minz4b)); mx5 = PxVec3(readY(maxx4b), readY(maxy4b), readY(maxz4b));
			mn6 = PxVec3(readZ(minx4b), readZ(miny4b), readZ(minz4b)); mx6 = PxVec3(readZ(maxx4b), readZ(maxy4b), readZ(maxz4b));
			mn7 = PxVec3(readW(minx4b), readW(miny4b), readW(minz4b)); mx7 = PxVec3(readW(maxx4b), readW(maxy4b), readW(maxz4b));
			struct MaxExport {
				static void execute(
					const Gu::Box& box, const PxVec3& mn, const PxVec3& mx,
					const PxVec3& p1, const PxVec3& p2, const PxVec3& p3)
				{
					PxVec3 pts[8];
					box.computeBoxPoints(pts);
					printf("box name: \"obb\" pos:[0, 0, 0]\n select $obb\n convertToMesh $\n");
					for (int j = 0; j < 8; j++)
						printf("meshOp.setVert $.baseObject %d [%.3f, %.3f, %.3f]\n", j+1, pts[j].x, pts[j].y, pts[j].z);

					pts[0] = PxVec3(mn.x, mn.y, mn.z);
					pts[1] = PxVec3(mx.x, mn.y, mn.z);
					pts[2] = PxVec3(mn.x, mx.y, mn.z);
					pts[3] = PxVec3(mn.x, mn.y, mx.z);
					pts[4] = PxVec3(mx.x, mx.y, mn.z);
					pts[5] = PxVec3(mx.x, mn.y, mx.z);
					pts[6] = PxVec3(mn.x, mx.y, mx.z);
					pts[7] = PxVec3(mx.x, mx.y, mx.z);
					printf("box name: \"aabb\" pos:[0, 0, 0]\n select $aabb\n convertToMesh $\n");
					for (int j = 0; j < 8; j++)
						printf("meshOp.setVert $.baseObject %d [%.3f, %.3f, %.3f]\n", j+1, pts[j].x, pts[j].y, pts[j].z);
					printf("sphere name: \"p1_abb\" pos:[%.3f, %.3f, %.3f] radius: %.3f\n", p1.x, p1.y, p1.z, 0.01f);
					printf("sphere name: \"p2_abb\" pos:[%.3f, %.3f, %.3f] radius: %.3f\n", p2.x, p2.y, p2.z, 0.01f);
					printf("sphere name: \"p1_obb\" pos:[%.3f, %.3f, %.3f] radius: %.3f\n", p3.x, p3.y, p3.z, 0.01f);
				}
			};

			PxVec3 abbp1a1 = PxVec3(readX(p1ABBxa), readX(p1ABBya), readX(p1ABBza));
			PxVec3 abbp1a2 = PxVec3(readY(p1ABBxa), readY(p1ABBya), readY(p1ABBza));
			PxVec3 abbp1a3 = PxVec3(readZ(p1ABBxa), readZ(p1ABBya), readZ(p1ABBza));
			PxVec3 abbp1a4 = PxVec3(readW(p1ABBxa), readW(p1ABBya), readW(p1ABBza));
			PxVec3 abbp2a1 = PxVec3(readX(p2ABBxa), readX(p2ABBya), readX(p2ABBza));
			PxVec3 abbp2a2 = PxVec3(readY(p2ABBxa), readY(p2ABBya), readY(p2ABBza));
			PxVec3 abbp2a3 = PxVec3(readZ(p2ABBxa), readZ(p2ABBya), readZ(p2ABBza));
			PxVec3 abbp2a4 = PxVec3(readW(p2ABBxa), readW(p2ABBya), readW(p2ABBza));
			PxVec3 obbp1a1 = PxVec3(readX(p1OBBxa), readX(p1OBBya), readX(p1OBBza));
			PxVec3 obbp1a2 = PxVec3(readY(p1OBBxa), readY(p1OBBya), readY(p1OBBza));
			PxVec3 obbp1a3 = PxVec3(readZ(p1OBBxa), readZ(p1OBBya), readZ(p1OBBza));
			PxVec3 obbp1a4 = PxVec3(readW(p1OBBxa), readW(p1OBBya), readW(p1OBBza));

			PxVec3 abbp1b1 = PxVec3(readX(p1ABBxb), readX(p1ABByb), readX(p1ABBzb));
			PxVec3 abbp1b2 = PxVec3(readY(p1ABBxb), readY(p1ABByb), readY(p1ABBzb));
			PxVec3 abbp1b3 = PxVec3(readZ(p1ABBxb), readZ(p1ABByb), readZ(p1ABBzb));
			PxVec3 abbp1b4 = PxVec3(readW(p1ABBxb), readW(p1ABByb), readW(p1ABBzb));
			PxVec3 abbp2b1 = PxVec3(readX(p2ABBxb), readX(p2ABByb), readX(p2ABBzb));
			PxVec3 abbp2b2 = PxVec3(readY(p2ABBxb), readY(p2ABByb), readY(p2ABBzb));
			PxVec3 abbp2b3 = PxVec3(readZ(p2ABBxb), readZ(p2ABByb), readZ(p2ABBzb));
			PxVec3 abbp2b4 = PxVec3(readW(p2ABBxb), readW(p2ABByb), readW(p2ABBzb));
			PxVec3 obbp1b1 = PxVec3(readX(p1OBBxb), readX(p1OBByb), readX(p1OBBzb));
			PxVec3 obbp1b2 = PxVec3(readY(p1OBBxb), readY(p1OBByb), readY(p1OBBzb));
			PxVec3 obbp1b3 = PxVec3(readZ(p1OBBxb), readZ(p1OBByb), readZ(p1OBBzb));
			PxVec3 obbp1b4 = PxVec3(readW(p1OBBxb), readW(p1OBByb), readW(p1OBBzb));
			union A { PxU32 res[8]; struct { v_uint4 res4ua; v_uint4 res4ub; }; } r;
			r.res4ua = resa4u;
			r.res4ub = resb4u;

			bool ignore0 = readX(minx4a) > readX(maxx4a); bool ignore1 = readY(minx4a) > readY(maxx4a);
			bool ignore2 = readZ(minx4a) > readZ(maxx4a); bool ignore3 = readW(minx4a) > readW(maxx4a);
			bool ignore4 = readX(minx4b) > readX(maxx4b); bool ignore5 = readY(minx4b) > readY(maxx4b);
			bool ignore6 = readZ(minx4b) > readZ(maxx4b); bool ignore7 = readW(minx4b) > readW(maxx4b);

			// false positives - ok
			/*
			if (r.res[0] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn0, mx0)))
				MaxExport::execute(obb, mn0, mx0, abbp1a1, abbp2a1, obbp1a1), failedTests ++;
			if (r.res[1] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn1, mx1)))
				MaxExport::execute(obb, mn1, mx1, abbp1a2, abbp2a2, obbp1a2), failedTests++;
			if (r.res[2] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn2, mx2)))
				MaxExport::execute(obb, mn2, mx2, abbp1a3, abbp2a3, obbp1a3), failedTests++;
			if (r.res[3] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn3, mx3)))
				MaxExport::execute(obb, mn3, mx3, abbp1a4, abbp2a4, obbp1a4), failedTests++;
			if (r.res[4] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn4, mx4)))
				MaxExport::execute(obb, mn4, mx4, abbp1b1, abbp2b1, obbp1b1), failedTests ++;
			if (r.res[5] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn5, mx5)))
				MaxExport::execute(obb, mn5, mx5, abbp1b2, abbp2b2, obbp1b2), failedTests++;
			if (r.res[6] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn6, mx6)))
				MaxExport::execute(obb, mn6, mx6, abbp1b3, abbp2b3, obbp1b3), failedTests++;
			if (r.res[7] && !(PxU32)intersectOBBAABB(obb, PxBounds3(mn7, mx7)))
				MaxExport::execute(obb, mn7, mx7, abbp1b4, abbp2b4, obbp1b4), failedTests++;
				*/

			// false negatives - bad
			if (!ignore0 && !r.res[0] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn0, mx0)))
				MaxExport::execute(obb, mn0, mx0, abbp1a1, abbp2a1, obbp1a1);
			if (!ignore1 && !r.res[1] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn1, mx1)))
				MaxExport::execute(obb, mn1, mx1, abbp1a2, abbp2a2, obbp1a2);
			if (!ignore2 && !r.res[2] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn2, mx2)))
				MaxExport::execute(obb, mn2, mx2, abbp1a3, abbp2a3, obbp1a3);
			if (!ignore3 && !r.res[3] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn3, mx3)))
				MaxExport::execute(obb, mn3, mx3, abbp1a4, abbp2a4, obbp1a4);
			if (!ignore4 && !r.res[4] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn4, mx4)))
				MaxExport::execute(obb, mn4, mx4, abbp1b1, abbp2b1, obbp1b1);
			if (!ignore5 && !r.res[5] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn5, mx5)))
				MaxExport::execute(obb, mn5, mx5, abbp1b2, abbp2b2, obbp1b2);
			if (!ignore6 && !r.res[6] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn6, mx6)))
				MaxExport::execute(obb, mn6, mx6, abbp1b3, abbp2b3, obbp1b3);
			if (!ignore7 && !r.res[7] && (PxU32)intersectOBBAABB(obb, PxBounds3(mn7, mx7)))
				MaxExport::execute(obb, mn7, mx7, abbp1b4, abbp2b4, obbp1b4);
		}
		#endif // VERIFY_RTREE

		// results got swizzled during decoding to floats
		PxU32* resa = resa_;
		PxU32* resb = resb_;
		PxU32 sum0 = 0;
		PxU32 sum1 = 0 + resa[0];
		PxU32 sum2 = sum1 + resb[0];
		PxU32 sum3 = sum2 + resa[1];
		PxU32 sum4 = sum3 + resb[1];
		PxU32 sum5 = sum4 + resa[2];
		PxU32 sum6 = sum5 + resb[2];
		PxU32 sum7 = sum6 + resa[3];
		PxU32 sum8 = sum7 + resb[3];

		if (bottomLevel) {
			cacheTopValid = true;
			cacheTop = stackPtr[-1];
			rr[sum0] = ptr0;
			rr[sum1] = ptr1;
			rr[sum2] = ptr2;
			rr[sum3] = ptr3;
			rr[sum4] = ptr4;
			rr[sum5] = ptr5;
			rr[sum6] = ptr6;
			rr[sum7] = ptr7;
			resultsPtr = rr + sum8;
			if (rr+sum8+mPageSize > resultsBegin+maxResults)
			{
				// flush partial results via callback.
				// We flush mPageSize entries too early to make it so the api works with immediate early out for maxResults = 8
				if (!callback->processResults(PxU32((rr+sum8)-resultsBegin), resultsBegin))
					return;
				resultsPtr = resultsBegin;
			}
		} else {
			PxU32 bitmask = (resa[0]<<24)|(resb[0]<<25)|(resa[1]<<26)|(resb[1]<<27)|(resa[2]<<28)|(resb[2]<<29)|(resa[3]<<30)|(resb[3]<<31);
			PxI32 lastIndex = 7 - PxI32(countLeadingZeros(bitmask));
			cacheTopValid = (lastIndex >= 0);
			// worst case scenario is 7-32=-25
			// it just happens that we can reference up to ptrs[-25] safely since sizeof(RTreePage) is 128
			// so we don't need to clamp lastIndex here
			cacheTop = ptrs[lastIndex];
			ss[sum0] = ptr0;
			ss[sum1] = ptr1;
			ss[sum2] = ptr2;
			ss[sum3] = ptr3;
			ss[sum4] = ptr4;
			ss[sum5] = ptr5;
			ss[sum6] = ptr6;
			ss[sum7] = ptr7;
			stackPtr = ss + sum8;
		}
	} while (stackPtr > stack);

	if (resultsPtr-resultsBegin > 0)
		callback->processResults(PxU32(resultsPtr-resultsBegin), resultsBegin);
}

} // namespace Gu

}
