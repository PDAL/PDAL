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
#include <pdal/drivers/caris/CloudReader.hpp>
#include <pdal/drivers/caris/CloudIterator.hpp>

#include "Utils.hpp"


#ifdef _MSC_VER
#   pragma warning(push, 3)
#   pragma warning(disable : DISABLED_3RDPARTY_WARNINGS)
#endif

#include <pdal/Dimension.hpp>
#include <pdal/Schema.hpp>
#include <ogr_spatialref.h>
#include <boost/foreach.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/name_generator.hpp>

#ifdef _MSC_VER
#   pragma warning(pop)

// decorated name length exceeded, name was truncated
#   pragma warning(disable : 4503)
#endif

namespace csar
{

using namespace csar::utils;

namespace
{


//************************************************************************
//! callback for logging messages
/*!
\param in_reader
    \li CloudReader* to log to
\param in_message
    \li message to log
*/
//************************************************************************
void logCallback(void* in_reader, const char* in_message)
{
    CloudReader * reader = (CloudReader *)in_reader;
    if (reader)
    {
        reader->log()->get(pdal::logINFO) << in_message << std::flush;
    }
}

}

//************************************************************************
//! constructor
/*!
\param options
    \li Reader options
*/
//************************************************************************
CloudReader::CloudReader(
    const pdal::Options& options)
    : pdal::Reader(options)
{
}

//************************************************************************
//! destructor
//************************************************************************
CloudReader::~CloudReader()
{
    if (m_cloud)
        caris_cloud_release(m_cloud);
}

//! \copydoc pdal::Reader::initialize
void CloudReader::initialize()
{
    pdal::Reader::initialize();

    int status = caris_cloud_open(getURI().c_str(), &m_cloud, &logCallback, this);
    if (status)
    {
        std::string msg = caris_status_message(status, isDebug());
        throw pdal::pdal_error(msg);
    }

    assert(m_cloud);

    // bounds
    {
        double minMaxXYZ[2][3];
        caris_cloud_extents(m_cloud,
                            &minMaxXYZ[0][0], &minMaxXYZ[0][1], &minMaxXYZ[0][2],
                            &minMaxXYZ[1][0], &minMaxXYZ[1][1], &minMaxXYZ[1][2]);

        setBounds(pdal::Bounds<double>(
                      minMaxXYZ[0][0], minMaxXYZ[0][1], minMaxXYZ[0][2],
                      minMaxXYZ[1][0], minMaxXYZ[1][1], minMaxXYZ[1][2]
                  ));
    }

    setNumPoints(caris_cloud_num_points(m_cloud));

/**
    if (const char* wktSR = caris_cloud_spatial_reference(m_cloud))
        setSpatialReference(pdal::SpatialReference(wktSR));
**/

    // Dimensions
    pdal::Schema & schema = getSchemaRef();

    // generate dimension UUIDs such that they are unique to the driver, but
    // consistent accoss loads
    boost::uuids::name_generator uuidGen(
        boost::uuids::string_generator()(
            "{5C668903-34CF-40d3-92C3-B8D9AB070902}"));

    int numDims = 0;
    caris_dimension const* dimArray = NULL;
    caris_cloud_dimensions(m_cloud, &dimArray, &numDims);

    // caris_dimesions may contain a tuple of numeric elements which need
    // to be mapped to multiple pdal dimenions
    for (int dimIndex = 0; dimIndex < numDims; ++dimIndex)
    {
        caris_dimension const& carisDim = dimArray[dimIndex];

        for (int tupleIndex = 0; tupleIndex < carisDim.tuple_length; ++tupleIndex)
        {
            std::string name;

            if (carisDim.tuple_length == 1)
            {
                name = carisDim.name;
            }
            else
            {
                if (dimIndex == 0
                        && carisDim.type == CARIS_TYPE_FLOAT64
                        && carisDim.tuple_length == 3)
                {
                    // position is always the first dim, name them X,Y,Z
                    char const* xyzStr[] = {"X", "Y", "Z"};
                    name = xyzStr[tupleIndex];
                }
                else
                {
                    name = std::string(carisDim.name)
                           + "." + boost::lexical_cast<std::string>(tupleIndex);
                }
            }

            pdal::Dimension pdalDim(
                name,
                carisTypeToInterpretation(caris_type(carisDim.type)),
                carisTypeToSize(caris_type(carisDim.type)));
            pdalDim.setUUID(uuidGen(name));
            pdalDim.setNamespace(getName());
            m_dimInfo[pdalDim.getUUID()] = DimInfo(dimIndex, tupleIndex, &carisDim);
            schema.appendDimension(pdalDim);
        }
    }

    if (caris_cloud_status(m_cloud))
    {
        std::string msg = caris_status_message(caris_cloud_status(m_cloud), isDebug());
        throw pdal::pdal_error(msg);
    }
}


//! \copydoc pdal::Reader::createSequentialIterator
pdal::StageSequentialIterator* CloudReader::createSequentialIterator(
    pdal::PointBuffer& in_buffer) const
{
    assert(m_cloud);
    return new CloudIterator(in_buffer, getCarisCloud());
}

} // namespace
