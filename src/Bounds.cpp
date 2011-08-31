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


#include <pdal/Bounds.hpp>
#include <pdal/Range.hpp>
#include <pdal/Utils.hpp>
#include <iostream>
namespace pdal
{


std::istream& operator>>(std::istream& istr, Bounds<double>& bounds)
{

    
    istr.get();
    if (istr.eof())
    {
        Bounds<double> output;
        bounds = output;
        return istr;
    } 

    if (!istr.good()) throw pdal_error("stream is unreadable, unable to parse Bounds");

    istr.unget();
        
    // A really dirty way to check for an empty bounds object right off 
    // the bat
    const char left_paren = (char)istr.get(); if (!istr.good()) throw pdal_error("stream0 is no good unable to parse Bounds");
    const char right_paren = (char)istr.get();
    
    if (left_paren == '(' && 
        right_paren == ')')
    {
        Bounds<double> output;
        bounds = output;
        return istr;
    }
    istr.unget(); istr.unget(); // ()
    
    Bounds<double>::RangeVector v;
    
    Utils::eatwhitespace(istr);
    
    if (!Utils::eatcharacter(istr,'('))
        throw pdal_error("Bounds parser failed at first parentheses");

    bool done = false;
    while (!done)
    {
        Utils::eatwhitespace(istr);
        
        Range<double> r;
        istr >> r;
        v.push_back(r);

        Utils::eatwhitespace(istr);
        if (Utils::eatcharacter(istr,','))
        {
            done = false;
        }
        else if (Utils::eatcharacter(istr,')'))
        {
            done = true;
        }
        else
        {
            throw pdal_error("Bounds parser failed");
        }
    }
    
    Bounds<double> xxx(v);
    bounds = xxx;

    Utils::eatwhitespace(istr);

    return istr;
}

} // namespace
