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

#include <pdal/pdal_test_main.hpp>

#include <pdal/UserCallback.hpp>

using namespace pdal;

// our implementation will be that we will request an interrupt
// when we are more than half done
class MyUserCallback : public UserCallback
{
public:
    virtual void callback()
    {
        if (getPercentComplete() > 50.0)
            setInterruptFlag(true);
    }
};


class Worker
{
public:
    Worker(UserCallback& cb) : m_cb(cb), m_ticks(0)
    {
        m_cb.setTotal(300);
    }

    // each invocation of doWork will represent 1% more done
    // returns true if work is going along okay, false otherwise
    bool doWork()
    {
        m_cb.invoke(++m_ticks);
        m_cb.invoke(++m_ticks);
        m_cb.invoke(++m_ticks);
        return true;
    }

private:
    UserCallback& m_cb;
    int m_ticks;

    Worker& operator=(const Worker&); // not implemented
};


TEST(UserCallbackTest, test1)
{
    MyUserCallback cb;

    Worker worker(cb);
    bool ok;

    // first 50%
    for (int i = 0; i < 50; i++)
    {
        ok = worker.doWork();
        EXPECT_TRUE(ok);
        uint32_t hb = 3 * (i + 1);
        EXPECT_EQ(cb.getHeartbeats(), hb);
        EXPECT_DOUBLE_EQ(cb.getPercentComplete(), (double)(i + 1));
    }

    // to 51%...
    try
    {
        ok = true;
        ok = worker.doWork();
    }
    catch (UserCallback::interrupted)
    {
        ok = false;
    }
    EXPECT_TRUE(!ok);
    EXPECT_DOUBLE_EQ(100.0*(151.0/300.0), cb.getPercentComplete());
}
