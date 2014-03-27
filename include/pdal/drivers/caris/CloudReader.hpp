/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/
#ifndef INCLUDED_DRIVERS_CSAR_CLOUDREADER_HPP
#define INCLUDED_DRIVERS_CSAR_CLOUDREADER_HPP

#include "config.h"

#include "caris/caris_pc_wrapper.h"

#ifdef _MSC_VER
#   pragma warning(push, 3)
#   pragma warning(disable : DISABLED_3RDPARTY_WARNINGS)
#endif 

#include <pdal/StageBase.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Reader.hpp>

#ifdef _MSC_VER
#   pragma warning(pop)
#endif

namespace csar {

//! Base Reader implementaion of CARIS Clouds
class CloudReader : public pdal::Reader
{
public:
    
    explicit CloudReader(const pdal::Options& options);
    virtual ~CloudReader();
    
    
    virtual void initialize();
    virtual pdal::StageSequentialIterator*
    createSequentialIterator(pdal::PointBuffer& in_buffer) const;
    
    //! Info for mapping between pdal and caris dimensions
    struct DimInfo
    {
        DimInfo()
            : dimIndex(), tupleIndex(), dimension()
        {}
        DimInfo(int in_dimIndex, int in_tupleIndex,
                caris_dimension const* in_dimension) :
            dimIndex(in_dimIndex), tupleIndex(in_tupleIndex),
            dimension(in_dimension)
        {}
        
        //! index of the dimension in the related caris_cloud
        int dimIndex;
        //! the tuple index of the caris_dimension to be mapped to pdal
        int tupleIndex;
        //! related dimension
        caris_dimension const* dimension;
    };
    
    std::map<pdal::dimension::id, DimInfo> const& getDimInfo() const;
    caris_cloud * getCarisCloud() const;
    
protected:
    virtual std::string getURI() const = 0;
    
private:
    caris_cloud * m_cloud;
    std::map<pdal::dimension::id, DimInfo> m_dimInfo;
};

//************************************************************************
//! info for mapping from pdal to caris dimensions
/*!
\return 
    \li info for mapping from pdal to caris dimensions
*/
//************************************************************************
inline
std::map<pdal::dimension::id, CloudReader::DimInfo> const& CloudReader::getDimInfo() const
{
    return m_dimInfo;
}

//************************************************************************
//! get underlying caris_cloud
/*!
\return 
    \li underlying caris_cloud
*/
//************************************************************************
inline
caris_cloud * CloudReader::getCarisCloud() const
{
    return m_cloud;
}

} // namespace

#endif
