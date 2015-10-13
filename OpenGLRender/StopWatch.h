/*

Copyright 2014 Rudy Snow

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef STOPWATCH_HEADER
#define STOPWATCH_HEADER

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif


///////////////////////////////////////////////////////////////////////////////
// Simple Stopwatch class. Use this for high resolution timing 
// purposes (or, even low resolution timings)
// Pretty self-explanatory.... 
// Reset(), or GetElapsedSeconds().
class CStopWatch
{
public:
    CStopWatch(void)	// Constructor
    {
#ifdef WIN32
        QueryPerformanceFrequency(&m_CounterFrequency);
        QueryPerformanceCounter(&m_LastCount);
#else
        gettimeofday(&m_LastCount, 0);
#endif
    }

    // Resets timer (difference) to zero
    inline void Reset(void) 
    {
#ifdef WIN32
        QueryPerformanceCounter(&m_LastCount);
#else
        gettimeofday(&m_LastCount, 0);
#endif
    }					

    // Get elapsed time in seconds
    float GetElapsedSeconds(void)
    {
        // Get the current count
#ifdef WIN32
        LARGE_INTEGER lCurrent;
        QueryPerformanceCounter(&lCurrent);

        return float((lCurrent.QuadPart - m_LastCount.QuadPart) /
            double(m_CounterFrequency.QuadPart));
#else
        timeval lcurrent;
        gettimeofday(&lcurrent, 0);
        float fSeconds = (float)(lcurrent.tv_sec - m_LastCount.tv_sec);
        float fFraction = (float)(lcurrent.tv_usec - m_LastCount.tv_usec) * 0.000001f;
        return fSeconds + fFraction;
#endif
    }	

protected:
#ifdef WIN32
    LARGE_INTEGER m_CounterFrequency;
    LARGE_INTEGER m_LastCount;
#else
    timeval m_LastCount;
#endif
};


#endif
