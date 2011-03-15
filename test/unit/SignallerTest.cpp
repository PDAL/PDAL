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

#include <boost/test/unit_test.hpp>

#include <libpc/Signaller.hpp>

using namespace libpc;

BOOST_AUTO_TEST_SUITE(SignallerTest)


class MySignaller : public Signaller
{
public:
    MySignaller()
        : m_perc(0)
        , m_stop(false)
    {
    }

    // override
    void setPercentComplete(double value)
    {
        m_perc = (int)(value * 100);
    }

    // override
    bool isInterruptRequested() const
    {
        return m_stop;
    }

    int m_perc;
    bool m_stop;
};


class Worker
{
public:
    Worker(Signaller& sig)
      : m_sig(sig)
      , m_ticks(0)
    {
    }

    bool tick()
    {
        if (m_sig.isInterruptRequested())
            return false;

        ++m_ticks;
        m_sig.setPercentComplete((double)m_ticks / 100.0);
        return true;
    }

private:
    Signaller& m_sig;
    int m_ticks;

    Worker& operator=(const Worker&); // not implemented
};


BOOST_AUTO_TEST_CASE(test_ctor)
{
    MySignaller sig;

    Worker worker(sig);
    bool ok;

    ok = worker.tick();
    BOOST_CHECK(ok);
    BOOST_CHECK(sig.m_perc == 1);
    ok = worker.tick();
    BOOST_CHECK(ok);
    BOOST_CHECK(sig.m_perc == 2);
    ok = worker.tick();
    BOOST_CHECK(ok);
    BOOST_CHECK(sig.m_perc == 3);

    sig.m_stop = true;

    ok = worker.tick();
    BOOST_CHECK(!ok);
    BOOST_CHECK(sig.m_perc == 3);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
