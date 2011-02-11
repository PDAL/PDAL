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

#include "libpc/Header.hpp"

#include <iostream>

using std::cout;
using std::endl;

using namespace libpc;


Header::Header() :
    m_numPoints(0)
{
    return;
}


Header::Header(const Header& other)
{
    this->m_numPoints = other.m_numPoints;
    this->m_pointLayout = other.m_pointLayout;
    this->m_bounds = other.m_bounds;
    return;
}


Header& Header::operator=(const Header& other)
{
    if (this != &other)
    {
        this->m_numPoints = other.m_numPoints;
        this->m_pointLayout = other.m_pointLayout;
        this->m_bounds = other.m_bounds;
    }
    return *this;
}


const Bounds<double>& Header::getBounds() const
{
    return m_bounds;
}


void Header::setBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


const PointLayout& Header::getLayout() const
{
    return m_pointLayout;
}


PointLayout& Header::getLayout()
{
    return m_pointLayout;
}


void Header::setLayout(const PointLayout& layout)
{
    m_pointLayout = layout;
}


int Header::getNumPoints() const
{
    return m_numPoints;
}


void Header::setNumPoints(int numPoints)
{
    m_numPoints = numPoints;
}


void Header::dump() const
{
    cout << "Header:" << endl;

    cout << "  Num points: " << m_numPoints << endl;

    cout << "  Bounds: ";
    cout << m_bounds;
    cout << endl;

    m_pointLayout.dump("  ");

    cout << endl;

    return;
}
