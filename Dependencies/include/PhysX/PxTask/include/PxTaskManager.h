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

#ifndef PX_TASK_MANAGER_H
#define PX_TASK_MANAGER_H

#include "Px.h"

namespace physx
{

class PxProfileZoneManager;

namespace pxtask
{

PX_PUSH_PACK_DEFAULT

class BaseTask;
class Task;
class LightCpuTask;
typedef unsigned long TaskID;

struct TaskType
{
	enum Enum
	{
		TT_CPU,
		TT_GPU,
		TT_NOT_PRESENT,
		TT_COMPLETED
	};
};

class CpuDispatcher;
class SpuDispatcher;
class GpuDispatcher;

class TaskManager
{
public:
    virtual void     setCpuDispatcher(CpuDispatcher& ref) = 0;
    virtual void     setGpuDispatcher(GpuDispatcher& ref) = 0;
    virtual void     setSpuDispatcher(SpuDispatcher& ref) = 0;
	virtual void     initializeProfiling(physx::PxProfileZoneManager& ref) = 0;

	virtual CpuDispatcher*			getCpuDispatcher() const = 0;
	virtual GpuDispatcher*			getGpuDispatcher() const = 0;
	virtual SpuDispatcher*			getSpuDispatcher() const = 0;

	virtual void	resetDependencies() = 0;
	virtual void	startSimulation() = 0;
	virtual void	stopSimulation() = 0;
	virtual void	taskCompleted(Task& task) = 0;

	virtual TaskID  getNamedTask(const char* name) = 0;
	virtual TaskID  submitNamedTask(Task* task, const char* name, TaskType::Enum type = TaskType::TT_CPU) = 0;
	virtual TaskID  submitUnnamedTask(Task& task, TaskType::Enum type = TaskType::TT_CPU) = 0;
	virtual Task*   getTaskFromID(TaskID) = 0;

	virtual void        release() = 0;
	static TaskManager* createTaskManager(CpuDispatcher* = 0, GpuDispatcher* = 0, SpuDispatcher* = 0);

protected:
	virtual ~TaskManager() {}

	virtual void finishBefore(Task& task, TaskID taskID) = 0;
	virtual void startAfter(Task& task, TaskID taskID) = 0;

	virtual void addReference(TaskID taskID) = 0;
	virtual void decrReference(TaskID taskID) = 0;

	virtual void decrReference(LightCpuTask&) = 0;
	virtual void addReference(LightCpuTask&) = 0;

	virtual void emitStartEvent(BaseTask&) = 0;
	virtual void emitStopEvent(BaseTask&) = 0;

	friend class BaseTask;
	friend class Task;
	friend class LightCpuTask;
	friend class GpuWorkerThread;
};

PX_POP_PACK

} // end pxtask namespace
} // end physx namespace

#endif
