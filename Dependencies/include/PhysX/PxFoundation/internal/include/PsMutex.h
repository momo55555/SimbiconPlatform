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


#ifndef PX_FOUNDATION_PSMUTEX_H
#define PX_FOUNDATION_PSMUTEX_H

#include "PsAllocator.h"
#include "PsNoCopy.h"

/*
 * This <new> inclusion is a best known fix for gcc 4.4.1 error:
 * Creating object file for apex/src/PsAllocator.cpp ...
 * In file included from apex/include/PsFoundation.h:30,
 *                from apex/src/PsAllocator.cpp:26:
 * apex/include/PsMutex.h: In constructor  'physx::shdfnd3::MutexT<Alloc>::MutexT(const Alloc&)':
 * apex/include/PsMutex.h:92: error: no matching function for call to 'operator new(unsigned int,
 * physx::shdfnd3::MutexImpl*&)'
 * <built-in>:0: note: candidates are: void* operator new(unsigned int)
 */
#include <new>

namespace physx
{
namespace shdfnd3
{
    class MutexImpl
    {
    public:

        /**
        The constructor for Mutex creates a mutex. It is initially unlocked.
        */
        MutexImpl();

        /**
        The destructor for Mutex deletes the mutex.
        */
        ~MutexImpl();

        /**
        Acquire (lock) the mutex. If the mutex is already locked
        by another thread, this method blocks until the mutex is
        unlocked.
        */
        bool lock();

        /**
        Acquire (lock) the mutex. If the mutex is already locked
        by another thread, this method returns false without blocking.
        */
        bool trylock();

        /**
        Release (unlock) the mutex.
        */
        bool unlock();

        /**
        Size of this class.
        */
        static const PxU32 size;
    };

	template <typename Alloc = ReflectionAllocator<MutexImpl> >
	class MutexT : protected Alloc
	{
	public:

		class ScopedLock : private NoCopy
		{
			MutexT<Alloc>& mMutex;
		public:
			PX_INLINE	ScopedLock(MutexT<Alloc> &mutex): mMutex(mutex) { mMutex.lock(); }
			PX_INLINE	~ScopedLock() { mMutex.unlock(); }
		};

		/**
		The constructor for Mutex creates a mutex. It is initially unlocked.
		*/
		MutexT(const Alloc& alloc = Alloc())
			: Alloc(alloc)
		{
			mImpl = (MutexImpl*)Alloc::allocate(MutexImpl::size, __FILE__, __LINE__);
			PX_PLACEMENT_NEW(mImpl, MutexImpl)();
		}

		/**
		The destructor for Mutex deletes the mutex.
		*/
		~MutexT()
		{
			mImpl->~MutexImpl();
			Alloc::deallocate(mImpl);
		}

		/**
		Acquire (lock) the mutex. If the mutex is already locked
		by another thread, this method blocks until the mutex is
		unlocked.
		*/
		bool lock()		const	{ return mImpl->lock(); }

		/**
		Acquire (lock) the mutex. If the mutex is already locked
		by another thread, this method returns false without blocking.
		*/
		bool trylock()	const	{ return mImpl->trylock(); }

		/**
		Release (unlock) the mutex.
		*/
		bool unlock()	const	{ return mImpl->unlock(); }

	private:
		MutexImpl* mImpl;
	};

    class ReadWriteLock : private NoCopy
    {
    public:
        ReadWriteLock();
        ~ReadWriteLock();

        void lockReader();
        void lockWriter();

        void unlockReader();
        void unlockWriter();

    private:
        class ReadWriteLockImpl*    mImpl;
    };

	class ScopedReadLock : private NoCopy
	{
	public:
		PX_INLINE	ScopedReadLock(ReadWriteLock& lock): mLock(lock)	{			mLock.lockReader(); 	}
		PX_INLINE	~ScopedReadLock()									{			mLock.unlockReader();	}

	private:
		ReadWriteLock& mLock;
	};

	class ScopedWriteLock : private NoCopy
	{
	public:
		PX_INLINE	ScopedWriteLock(ReadWriteLock& lock): mLock(lock)	{		mLock.lockWriter(); 	}
		PX_INLINE	~ScopedWriteLock()									{		mLock.unlockWriter();	}

	private:
		ReadWriteLock& mLock;
	};

	typedef MutexT<> Mutex;

	/*
	 * Use this type of lock for mutex behaviour that must operate on SPU and PPU
	 * On non-PS3 platforms, it is implemented using Mutex
	 */
#ifndef PX_PS3
	class AtomicLock : private NoCopy
	{
		Mutex mMutex;

	public:
		AtomicLock()
		{
		}

		bool lock()
		{
			return mMutex.lock();
		}

		bool trylock()
		{
			return mMutex.trylock();
		}

		bool unlock()
		{
			return mMutex.unlock();
		}
	};

	class AtomicLockCopy
	{
		AtomicLock* pLock;

	public:
		AtomicLockCopy() : pLock(NULL)
		{
		}
		
		AtomicLockCopy& operator = (AtomicLock& lock)
		{
			pLock = &lock;
			return *this;
		}

		bool lock()
		{
			return pLock->lock();
		}

		bool trylock()
		{
			return pLock->trylock();
		}

		bool unlock()
		{
			return pLock->unlock();
		}

	};
#else
	struct AtomicLockImpl
	{
		PX_ALIGN(128, PxU32 m_Lock);
		PxI32 m_LockId;
		PxU32 m_LockCount;

		AtomicLockImpl();
	};
	class AtomicLock //: private NoCopy
	{
		friend class AtomicLockCopy;
		AtomicLockImpl* m_pImpl;
public:

		AtomicLock();

		~AtomicLock();

		bool lock();

		bool trylock();

		bool unlock();
	};


	// if an AtomicLock is copied and then the copy goes out of scope, it'll delete the atomic primitive
	// (just a 128-byte aligned int) and cause a crash when it tries to delete it again
	// This class just uses the atomic primitive without releasing it in the end.

	class AtomicLockCopy
	{
		AtomicLockImpl* m_pImpl;
public:

		AtomicLockCopy() : m_pImpl(NULL)
		{
		}

		AtomicLockCopy(const AtomicLock& lock) : m_pImpl(lock.m_pImpl)
		{
		}

		~AtomicLockCopy()
		{
		}

		AtomicLockCopy& operator = (const AtomicLock& lock)
		{
			m_pImpl = lock.m_pImpl;
			return *this;
		}

		bool lock();

		bool trylock();

		bool unlock();
	};
#endif

#ifndef PX_PS3

	class AtomicRwLock : private NoCopy
	{
		ReadWriteLock m_Lock;

	public:

		AtomicRwLock()
		{
		}

		void lockReader()
		{
			m_Lock.lockReader();
		}
        void lockWriter()
		{
			m_Lock.lockWriter();
		}

		bool tryLockReader()
		{
			//Todo - implement this
			m_Lock.lockReader();
			return true;
		}

        void unlockReader()
		{
			m_Lock.unlockReader();
		}
        void unlockWriter()
		{
			m_Lock.unlockWriter();
		}
	};
#else

	struct AtomicRwLockImpl
	{
		PX_ALIGN(128, volatile PxU32 m_Lock);
		PX_ALIGN(128, volatile PxU32 m_ReadCounter);
		PxI32 m_LockId;
		PxU32 m_LockCount;

		AtomicRwLockImpl();
	};

	class AtomicRwLock : private NoCopy
	{
		AtomicRwLockImpl* m_pImpl;
	public:

		AtomicRwLock();

		~AtomicRwLock();

		void lockReader();

		bool tryLockReader();

        void lockWriter();

        void unlockReader();

        void unlockWriter();
	};

#endif

	class ScopedAtomicLock : private NoCopy
	{
		PX_INLINE	ScopedAtomicLock(AtomicLock& lock): mLock(lock)	{			mLock.lock(); 	}
		PX_INLINE	~ScopedAtomicLock()								{			mLock.unlock();	}

	private:
		AtomicLock& mLock;
	};

} // namespace shdfnd3
} // namespace physx

#endif
