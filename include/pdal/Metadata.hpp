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

#ifndef INCLUDED_METADATARECORD_HPP
#define INCLUDED_METADATARECORD_HPP

#include <pdal/pdal_internal.hpp>
#include <pdal/Options.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/SpatialReference.hpp>



#include <boost/shared_array.hpp>
#include <boost/variant.hpp>
#include <vector>
#include <map>

namespace pdal
{


namespace metadata {
    
    typedef std::map<std::string, std::string> MetadataAttributeM;

    class PDAL_DLL ByteArray 
    {
    public:

        ByteArray(std::vector<boost::uint8_t> const& data) : m_bytes(data) {}
        
        inline void set(std::vector<boost::uint8_t> const& input) { m_bytes = input; }
        inline std::vector<boost::uint8_t> const& get() const { return m_bytes; }

    private:
        
        std::vector<boost::uint8_t> m_bytes;
    };


    enum Type
    {
        SignedByte,
        UnsignedByte,
        SignedInteger,
        UnsignedInteger,
        Pointer,
        Float,
        Undefined,
        String,
        Bytes,
        Bounds,
        SpatialReference
        
    };


    typedef boost::variant< 
                            float,
                            double,
                            boost::int8_t,
                            boost::uint8_t,
                            boost::int16_t,
                            boost::uint16_t,
                            boost::int32_t,
                            boost::uint32_t,
                            boost::int64_t,
                            boost::uint64_t,
                            std::string, 
                            pdal::metadata::ByteArray, 
                            pdal::SpatialReference, 
                            pdal::Bounds<double> > Variant;


} // metadata




class PDAL_DLL Metadata 
{
public:

    Metadata(   std::string const& name, 
                std::string const& ns,
                pdal::metadata::Type t);

    Metadata(const Metadata&);

    ~Metadata();
    
    template <class T> inline void set(T const& v) { m_variant = v; } 
    template <class T> inline T get() { return boost::get<T>(m_variant); }

    template <class T> inline T cast() { return boost::lexical_cast<T>(m_variant); }    
    Metadata& operator=(Metadata const& rhs);

    bool operator==(Metadata const& rhs) const;

    metadata::Variant const getVariant() const { return m_variant; }
    
    std::string const& getName() const { return m_name; }
    void setName(std::string const& name) { m_name = name; }
    
    std::string const& getNamespace() const { return m_namespace; }
    void setNamespace(std::string const& ns) { m_namespace = ns; }
    
private:
    metadata::Variant m_variant;
    std::string m_name;
    std::string m_namespace;
    metadata::Type m_type;
    metadata::MetadataAttributeM m_attributes;
    
};


std::ostream& operator<<(std::ostream& ostr, const Metadata& srs);


} // namespace pdal

namespace std
{
std::ostream& operator<<(std::ostream& ostr, const pdal::metadata::ByteArray& output);
}

#endif
