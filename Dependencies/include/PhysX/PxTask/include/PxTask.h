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

#ifndef PX_TASK_H
#define PX_TASK_H

#include "Px.h"
#include "PxTaskManager.h"
#include "PxAssert.h"

namespace physx
{
// pull public foundation into physx namespace
using namespace pubfnd;

namespace pxtask
{

/**
 * \brief Base class of all task types
 *
 * BaseTask defines a runnable reference counted task with built-in profiling.
 */
class BaseTask
{
public:
	BaseTask() : mEventID(0xFFFF), mProfileStat(0), mTm(0) {}
	virtual ~BaseTask() {}

    /**
     * \brief user implemented abstract method
     *
     * run() methods must be thread safe, stack friendly (no alloca, etc), and
     * must never block.
     */
    virtual void        run() = 0;

    /**
     * \brief user implemented abstract method
     *
     * The returned name is used for profiling purposes.  It does not have to be
     * unique, but unique names are helpful.
     */
    virtual const char *getName() const = 0;

    //! \brief implemented by derived implimementation classes
    virtual void		addReference() = 0;
    //! \brief implemented by derived implimementation classes
    virtual void		removeReference() = 0;
    //! \brief implemented by derived implimementation classes
    virtual void		release() = 0;

	/**
     * \brief Execute user run method with wrapping profiling events.
     *
     * Optional entry point for use by CpuDispatchers.
	 */
	PX_INLINE void runProfiled()
	{
		mTm->emitStartEvent(*this);
		run();
		mTm->emitStopEvent(*this);
	}

	/**
     * \brief Specify stop event statistic
     *
     * If called before or while the task is executing, the given value
	 * will appear in the task's event bar in the profile viewer
	 */
	PX_INLINE void setProfileStat( PxU16 stat )
	{
		mProfileStat = stat;
	}

    /**
     * \brief Return TaskManager to which this task was submitted
     *
     * Note, can return NULL if task was not submitted, or has been
     * completed.
     */
	PX_INLINE TaskManager* getTaskManager() const
	{
		return mTm;
	}

protected:
	PxU16				mEventID;       //!< Registered profile event ID
	PxU16               mProfileStat;   //!< Profiling statistic
	TaskManager *       mTm;            //!< Owning TaskManager instance

	friend class TaskMgr;
};


/**
 * \brief a BaseTask implementation with deferred execution and full dependencies
 *
 * A Task must be submitted to a TaskManager to to be executed.  Tasks may
 * optionally be named when they are submitted.
 */
class Task : public BaseTask
{
public:
	Task() : mTaskID(0) {}
	virtual ~Task() {}

    //! \brief Task release method implementation
    virtual void release()
	{
		PX_ASSERT(mTm);

        // clear mTm before calling taskCompleted() for safety
		TaskManager *save = mTm;
		mTm = NULL;
		save->taskCompleted( *this );
	}

    //! \brief Inform the TaskManager this task must finish before the given
    //         task is allowed to start.
    PX_INLINE void finishBefore( TaskID taskID )
	{
		PX_ASSERT(mTm);
		mTm->finishBefore( *this, taskID);
	}

    //! \brief Inform the TaskManager this task cannot start until the given
    //         task has completed.
    PX_INLINE void startAfter( TaskID taskID )
	{
		PX_ASSERT(mTm);
		mTm->startAfter( *this, taskID );
	}

    /**
     * Manually increment this task's reference count.  The task will
     * not be allowed to run until removeReference() is called.
     */
    PX_INLINE void addReference()
	{
		PX_ASSERT(mTm);
		mTm->addReference( mTaskID );
	}

    /**
     * Manually decrement this task's reference count.  If the reference
     * count reaches zero, the task will be dispatched.
     */
    PX_INLINE void removeReference()
	{
		PX_ASSERT(mTm);
		mTm->decrReference( mTaskID );
	}

	PX_INLINE TaskID	    getTaskID() const
	{
		return mTaskID;
	}

	/**
	 * \brief Called by TaskManager at submission time for initialization
	 *
	 * Perform simulation step initialization here.
	 */
	virtual void submitted()
	{
		mStreamIndex = 0;
		mPreSyncRequired = false;
		mProfileStat = 0;
	}

	PX_INLINE void		requestSyncPoint()
	{
		mPreSyncRequired = true;
	}


protected:
    TaskID				mTaskID;            //!< ID assigned at submission
    PxU32               mStreamIndex;       //!< GpuTask CUDA stream index
    bool				mPreSyncRequired;   //!< GpuTask sync flag

	friend class TaskMgr;
    friend class GpuWorkerThread;
};


/**
 * \brief a BaseTask implementation with immediate execution and simple dependencies
 *
 * A LightCpuTask bypasses the TaskManager launch dependencies and will be
 * submitted directly to your scene's CpuDispatcher.  When the run() function
 * completes, it will decrement the reference count of the specified
 * continuation task.
 *
 * You must use a full-blown pxtask::Task if you want your task to be resolved
 * by another pxtask::Task, or you need more than a single dependency to be
 * resolved when your task completes, or your task will not run on the
 * CpuDispatcher.
 */
class LightCpuTask : public BaseTask
{
public:
	LightCpuTask()
		: mCont( NULL )
		, mRefCount( 0 )
	{
	}
	virtual ~LightCpuTask()
	{
	}

    /**
     * \brief initialize this task, prepare to run
     *
     * Provide a TaskManager instance and continuation task. Submission is
     * deferred until the task's mRefCount is decremented to zero.  Note that we
     * only use the TaskManager to query the CpuDispatcher.
	 */
	PX_INLINE void setContinuation(TaskManager& tm, BaseTask* c)
	{
		PX_ASSERT( mRefCount == 0 );
		mRefCount = 1;
		mCont = c;
		mTm = &tm;
		if( mCont )
		{
			mCont->addReference();
	    }
	}

    /**
     * \brief initialize this task, prepare to run
     *
     * This overload of init() query's the TaskManager from the continuation
     * task, which cannot be NULL.
	 */
	PX_INLINE void setContinuation( BaseTask *c )
	{
		PX_ASSERT( c );
		PX_ASSERT( mRefCount == 0 );
		mRefCount = 1;
		mCont = c;
		if( mCont )
		{
			mCont->addReference();
			mTm = mCont->getTaskManager();
			PX_ASSERT( mTm );
		}
	}

    /**
     * Manually decrement this task's reference count.  If the reference
     * count reaches zero, the task will be dispatched.
     */
	PX_INLINE void removeReference()
	{
		mTm->decrReference(*this);
	}

    /**
     * Manually increment this task's reference count.  The task will
     * not be allowed to run until removeReference() is called.
     */
	PX_INLINE void addReference()
	{
		mTm->addReference(*this);
	}

    /**
     * \brief called by CpuDispatcher after run method has completed
     *
     * Decrements the continuation task's reference count, if specified.
     */
	PX_INLINE void release()
	{
		if( mCont )
		{
			mCont->removeReference();
	    }
	}

protected:

	BaseTask *          mCont;          //!< Continuation task, can be NULL
	volatile PxI32      mRefCount;      //!< Task is dispatched when reaches 0

	friend class TaskMgr;
};


} // end pxtask namespace
} // end physx namespace

#endif
