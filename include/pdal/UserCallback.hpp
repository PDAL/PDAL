/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#pragma once

#include <pdal/pdal_internal.hpp>

namespace pdal
{

// This is the base class used for out-of-band interactions
// between the executing pipeline and the main app:
//   - recording and reporting a "hearbeat"
//   - recording and reporting percent-complete
//   - requesting the pipeline to be interrupted
//
// (The heartbeat feature can be used when the pipeline can't report
// a meaningful percentage complete value, e.g. when the number of
// points being processed is not a prioi known.)
//
// Apps should override the callback() function as desired.
//
class PDAL_DLL UserCallback
{
public:
    class interrupted : public pdal_error
    {
    public:
        interrupted() : pdal_error("UserCallback: interrupted by user")
            {}
    };

    UserCallback() : m_percentComplete(0.0), m_interruptFlag(false),
        m_heartbeats(0), m_total(1)
    {}

    void setTotal(point_count_t total)
        { m_total = std::max((point_count_t)1, total); }

    // The pipeline calls this to report the percentage progress, check for
    // interrupts, and call the user's callback function.
    //
    // returns true if everything is okay, or false if an interrupt has
    // been requested
    void invoke(point_count_t count)
    {
        m_percentComplete = 100.0 * count / m_total;
        invoke();
    }

    // The pipeline calls this to report the progress in an indeterminate way,
    // check for interrupts, and call the user's callback function.
    //
    // This is same as the check(double) function, except it doesn't record
    // a percent-complete value.
    //
    // returns true if everything is okay, or false if an interrupt has
    // been requested
    //
    void invoke()
    {
        ++m_heartbeats;
        callback();
        if (m_interruptFlag)
            throw interrupted();
    }

    // This will be called by the pipeline (via check()) at various times
    // during the execution. Applications should override this to perform
    // any desired actions.
    //
    // Examples of things you might do here include printing the percent done,
    // printing a '.' representing a heartbeat, checking some external condition
    // to raise the interrupt flag, etc.
    virtual void callback()
    {}

    double getPercentComplete() const
        { return m_percentComplete; }

    inline uint64_t getHeartbeats() const
        { return m_heartbeats; }

protected:
    void setInterruptFlag(bool value)
        { m_interruptFlag = value; }

private:
    double m_percentComplete;  // in range [0..100]
    bool m_interruptFlag; // true iff user would like the pipeline to abort
    uint64_t m_heartbeats; // number of times the check routine has been called
    point_count_t m_total;
};

} // namespace pdal

