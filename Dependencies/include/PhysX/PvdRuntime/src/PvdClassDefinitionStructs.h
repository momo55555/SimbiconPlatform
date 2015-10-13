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



#ifndef CLASSDEFINITIONSTRUCTS_H
#define CLASSDEFINITIONSTRUCTS_H

#if PX_SUPPORT_VISUAL_DEBUGGER

namespace physx
{
namespace Pvd
{

// ------------------------------------------------------------------------------------------------------------------------ //

#define TABLE_SIZE(table) sizeof(table)/sizeof(table[0])
#define PROPERTY_ROW(_name, _type) PropertyRow(#_name, PVD::PvdCommLayerDatatype::_type)
#define BITFLAG_PROPERTY_ROW(_name, _table) PropertyRow(#_name, _table, TABLE_SIZE(_table), PVD::PvdCommLayerDatatype::Bitflag)
#define ENUM_PROPERTY_ROW(_name, _table) PropertyRow(#_name, _table, TABLE_SIZE(_table), PVD::PvdCommLayerDatatype::EnumerationValue)
#define ARRAY_PROPERTY_ROW( _name, _arrayClass ) PropertyRow( #_name, 0, PvdClassKeys::_arrayClass + 1 ) //Pvd doesn't accept 0 based keys, 0 is an invalid key.

struct PropertyRow
{
	PropertyRow(const char* n, PxU32 dt)
		: name(n)
		, dataType(dt)
		, table(NULL)
		, size(0)
		, arrayClass(0)
	{}

	PropertyRow(const char* n, const PVD::NamedValueDefinition* t, PxU32 s, PxU32 dt)
		: name(n)
		, table(t)
		, size(s)
		, dataType(dt)
		, arrayClass(0)
	{}

	PropertyRow( const char* n, PxU32 /*unused*/, PxU16 ac )
		: name(n)
		, table(NULL)
		, size(0)
		, dataType(-1)
		, arrayClass(ac)
	{
	}

	const char*							name;
	const PVD::NamedValueDefinition*	table;
	PxU32								dataType;
	PxU16								arrayClass;
	PxU8								size;
};

// ------------------------------------------------------------------------------------------------------------------------ //

#define PROP_OFFSET(_parentClass) _parentClass##Prop::NUM_ELEMENTS
#define CLASS_ROW(_className) ClassRow(PvdClassKeys::_className, "Px"#_className, -1, g##_className##Prop, TABLE_SIZE(g##_className##Prop), 0, true)
#define CLASS_ROW_EMPTY(_className) ClassRow(PvdClassKeys::_className, "Px"#_className, -1, NULL, 0, 0, true)
#define CLASS_ROW_NOPREFIX(_className) ClassRow(PvdClassKeys::_className, #_className, -1, g##_className##Prop, TABLE_SIZE(g##_className##Prop), 0, false)
#define DERIVED_CLASS_ROW(_className, _parentClass) ClassRow(PvdClassKeys::_className, "Px"#_className, PvdClassKeys::_parentClass, g##_className##Prop, TABLE_SIZE(g##_className##Prop), PROP_OFFSET(_parentClass), true)
#define DERIVED_CLASS_ROW_EMPTY(_className, _parentClass) ClassRow(PvdClassKeys::_className, "Px"#_className, PvdClassKeys::_parentClass, NULL, 0, PROP_OFFSET(_parentClass), true)


struct ClassRow
{
	ClassRow(PxU16 k, const char* n, PxU16 pk, const PropertyRow* propTable, PxU32 propTableSize, PxU32 propTableOffset, bool phsyxNs)
	: classKey( k + 1 )
	, name(n)
	, parentId(pk)
	, propertyTable(propTable)
	, propertyTableSize(propTableSize)
	, propertyOffset(propTableOffset)
	, physxNamespace( phsyxNs )
	{}

	const char*			name;
	const PropertyRow*	propertyTable;
	PxU16				parentId;
	PxU16				propertyTableSize;
	PxU16				propertyOffset;
	PxU16				classKey;
	bool				physxNamespace;
};

// ------------------------------------------------------------------------------------------------------------------------ //

#define FLAG_ROW(_en, _name) {#_name, _en::_name }

} // namespace Pvd

}

#endif // PX_SUPPORT_VISUAL_DEBUGGER
#endif // CLASSDEFINITIONSTRUCTS_H

