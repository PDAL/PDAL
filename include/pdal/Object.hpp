/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_OBJECT_HPP
#define INCLUDED_OBJECT_HPP

#include <pdal/pdal_internal.hpp>

#include <boost/property_tree/ptree.hpp>


namespace pdal
{


/// pdal::Object is a base class that provides serialization, allocation, and
/// deallocation control for all basic objects inside of PDAL.

class PDAL_DLL Object
{
public:

    /** @name Constructors
    */
    Object(std::string const& type_name) : m_typename(type_name) {};

    /** @name Destructor
    */
    virtual ~Object() {};

    virtual std::string to_xml() const;
    virtual std::string to_json() const;

    virtual boost::property_tree::ptree toPTreeImpl() const
    {
        return boost::property_tree::ptree();
    }

    boost::property_tree::ptree toPTree() const;
    inline std::string getTypeName() const
    {
        return m_typename;
    }
    /** @name private attributes
    */


private:
    Object(const Object&);
    Object& operator =(const Object&);

    std::string m_typename;

};



} // namespace pdal

#endif
