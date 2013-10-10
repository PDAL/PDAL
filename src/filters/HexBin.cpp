/******************************************************************************
* Copyright (c) 2013, Andrew Bell (andrew.bell.ia@gmail.com)
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

#include <pdal/filters/HexBin.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning

#include <pdal/PointBuffer.hpp>
#include <pdal/GlobalEnvironment.hpp>



namespace pdal
{
namespace filters
{


HexBin::HexBin(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
{

    return;
}

HexBin::~HexBin()
{

}
void HexBin::initialize()
{
    Filter::initialize();
    return;
}




Options HexBin::getDefaultOptions()
{
    Options options;
    return options;
}

pdal::StageSequentialIterator* HexBin::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::HexBin(*this, buffer);
}

// pdal::StageRandomIterator* HexBin::createRandomIterator(PointBuffer& buffer) const
// {
//     return new pdal::filters::iterators::random::HexBin(*this, buffer);
// }


namespace iterators
{

namespace hexbin
{

IteratorBase::IteratorBase(pdal::filters::HexBin const& filter,
                           PointBuffer& buffer)
    : m_filter(filter)
#ifdef PDAL_HAVE_HEXER
    , m_grid(0)
#endif
    , m_sample_size(0)
    , m_sample_number(0)
    , m_density(10)
{
    Schema const& schema = buffer.getSchema();

    const std::string x_name = filter.getOptions().getValueOrDefault<std::string>("x_dim", "X");
    const std::string y_name = filter.getOptions().getValueOrDefault<std::string>("y_dim", "Y");

    filter.log()->get(logDEBUG2) << "x_dim '" << x_name <<"' requested" << std::endl;
    filter.log()->get(logDEBUG2) << "y_dim '" << y_name <<"' requested" << std::endl;

    m_dim_x = &(schema.getDimension(x_name));
    m_dim_y = &(schema.getDimension(y_name));

    filter.log()->get(logDEBUG2) << "Fetched x_name: " << x_name << std::endl;
    filter.log()->get(logDEBUG2) << "Fetched y_name: " << y_name << std::endl;;

    m_sample_size = filter.getOptions().getValueOrDefault<boost::uint32_t>("sample_size", 5000);
    m_density = filter.getOptions().getValueOrDefault<boost::uint32_t>("threshold", 15);
    m_edge_size = filter.getOptions().getValueOrDefault<double>("edge_size", 0.0);
}

IteratorBase::~IteratorBase()
{
#ifdef PDAL_HAVE_HEXER
    //if (m_grid)
    //delete m_grid;
#endif

}

} // hexbin

namespace sequential
{


HexBin::HexBin(const pdal::filters::HexBin& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , hexbin::IteratorBase(filter, buffer)

{
    return;
}

boost::uint32_t HexBin::readBufferImpl(PointBuffer& buffer)
{

    const boost::uint32_t numPoints = getPrevIterator().read(buffer);

#ifdef PDAL_HAVE_HEXER
    for (boost::uint32_t i = 0; i < buffer.getNumPoints(); ++i)
    {
        boost::int32_t xi = buffer.getField<boost::int32_t>(*m_dim_x, i);
        boost::int32_t yi = buffer.getField<boost::int32_t>(*m_dim_y, i);
        double x(0.0);
        x = m_dim_x->applyScaling<boost::int32_t>(xi);
        double y(0.0);
        y = m_dim_y->applyScaling<boost::int32_t>(yi);

        if (!m_grid)
        {
            bool bDoSample = pdal::Utils::compare_distance(m_edge_size, 0.0);
            if (bDoSample)
            {
                if (getStage().getNumPoints() < m_sample_size)
                {
                    if (getStage().getNumPoints() >= std::numeric_limits<boost::uint32_t>::max())
                    {
                        throw pdal_error("point count >= size of 32bit integer, set a sample size");
                    }

                    m_sample_size = static_cast<boost::uint32_t>(getStage().getNumPoints());
                }

                if (m_sample_number < m_sample_size)
                {
                    m_samples.push_back(hexer::Point(x,y));
                    m_sample_number++;

                    if (m_sample_number == m_sample_size)
                    {
                        m_edge_size = hexer::computeHexSize(m_samples, m_density);
                        m_grid = new hexer::HexGrid(m_edge_size, m_density);

                        getStage().log()->get(logDEBUG2) << "Created hexgrid of edge size :'" << m_edge_size << "' and density '" << m_density << "'" << std::endl;
                        // Add back the points we used for the sample.
                        typedef std::vector<hexer::Point>::const_iterator iterator;
                        iterator i;
                        for (i=m_samples.begin(); i != m_samples.end(); ++i)
                        {
                            m_grid->addPoint(*i);
                        }
                    }
                }
            }
            else
            {
                m_grid = new hexer::HexGrid(m_edge_size, m_density);
                getStage().log()->get(logDEBUG2) << "Created hexgrid of edge size :'" << m_edge_size << "' and density '" << m_density << "'" << std::endl;

            }
        }
        else
        {
            m_grid->addPoint(hexer::Point(x,y));
        }
    }
#endif

    return numPoints;
}
    

boost::uint64_t HexBin::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool HexBin::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

HexBin::~HexBin()
{


#ifdef PDAL_HAVE_HEXER
    if (m_grid)
    {
        m_grid->findShapes();
        m_grid->findParentPaths();

        pdal::Metadata& metadata = getBuffer().getMetadataRef();


        pdal::Metadata m;
        m.setName(getStage().getName());
        m.addMetadata<double>("edge_size",
                              m_edge_size,
                              "The edge size of the hexagon to use in situations where you do not want to estimate based on a sample");
        m.addMetadata<boost::uint32_t>("threshold",
                                       m_density,
                                       "Number of points necessary inside a hexagon to be considered full");
        m.addMetadata<boost::uint32_t>("sample_size",
                                       m_sample_size,
                                       "Number of samples to use when estimating hexagon edge size. Specify 0.0 for edge_size if you want to compute one.");

        std::ostringstream polygon;
        polygon.setf(std::ios_base::fixed, std::ios_base::floatfield);
        polygon.precision(getStage().getOptions().getValueOrDefault<boost::uint32_t>("precision", 8));
        m_grid->toWKT(polygon);
        m.addMetadata<std::string>("boundary",
                                   polygon.str(),
                                   "Boundary MULTIPOLYGON of domain");

        metadata.setMetadata(m);

    }

#endif

}



} // sequential

// namespace random
// {
//
//
// HexBin::HexBin(const pdal::filters::HexBin& filter, PointBuffer& buffer)
//     : pdal::FilterRandomIterator(filter, buffer)
//     , hexbin::IteratorBase(filter, buffer)
// {
//     return;
// }
//
// boost::uint32_t HexBin::readBufferImpl(PointBuffer& buffer)
// {
//
//     pdal::StageRandomIterator& iterator = getPrevIterator();
//
//     const boost::uint32_t numPoints = iterator.read(buffer);
//
//     return numPoints;
// }
//
//
// boost::uint64_t HexBin::seekImpl(boost::uint64_t count)
// {
//
//     return getPrevIterator().seek(count);
// }
//
// } // random



} // filters
} // pdal
} // namespaces
