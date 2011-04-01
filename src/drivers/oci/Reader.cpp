/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include <libpc/drivers/oci/Reader.hpp>
#include <libpc/drivers/oci/Iterator.hpp>

#include <libpc/exceptions.hpp>

#include <iostream>

namespace libpc { namespace drivers { namespace oci {


Reader::Reader(Options& options)
    : libpc::Stage()
    , m_options(options)
{

    Debug();
    
    m_connection = Connect(m_options);

    if (getQuery().size() == 0 )
        throw libpc_error("'select_sql' statement is empty. No data can be read from libpc::drivers::oci::Reader");
    
    registerFields();
    
    setNumPoints(1000);
}    

bool Reader::isVerbose() const
{
    return m_options.GetPTree().get<bool>("verbose");
}

std::string Reader::getQuery() const
{
    return m_options.GetPTree().get<std::string>("select_sql");
}

void Reader::Debug()
{
    bool debug = m_options.IsDebug();

    CPLPopErrorHandler();

    if (debug)
    {
        const char* gdal_debug = Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            Utils::putenv("CPL_DEBUG=ON");
        }
        
        const char* gdal_debug2 = getenv("CPL_DEBUG");
        std::cout << "Setting GDAL debug handler CPL_DEBUG=" << gdal_debug2 << std::endl;
        CPLPushErrorHandler(OCIGDALDebugErrorHandler);
        
    }
    else 
    {
        CPLPushErrorHandler(OCIGDALErrorHandler);        
    }
}

void Reader::registerFields()
{
    Schema& schema = getSchemaRef();

    Dimension xDim(Dimension::Field_X, Dimension::Int32);
    Dimension yDim(Dimension::Field_Y, Dimension::Int32);
    Dimension zDim(Dimension::Field_Z, Dimension::Int32);
    
    boost::property_tree::ptree tree = m_options.GetPTree();
    double scalex = tree.get<double>("scale.x");
    double scaley = tree.get<double>("scale.y");
    double scalez = tree.get<double>("scale.z");

    double offsetx = tree.get<double>("offset.x");
    double offsety = tree.get<double>("offset.y");
    double offsetz = tree.get<double>("offset.z");
        
    xDim.setNumericScale(scalex);
    yDim.setNumericScale(scaley);
    zDim.setNumericScale(scalez);
    xDim.setNumericOffset(offsetx);
    yDim.setNumericOffset(offsety);
    zDim.setNumericOffset(offsetz);

    schema.addDimension(xDim);
    schema.addDimension(yDim);
    schema.addDimension(zDim);

    schema.addDimension(Dimension(Dimension::Field_Time, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Classification, Dimension::Uint8));

    
    return;
}

const std::string& Reader::getName() const
{
    static std::string name("OCI Reader");
    return name;
}

Reader::~Reader()
{

    // m_connection->DestroyType(&m_pc);
    return;
}

libpc::SequentialIterator* Reader::createSequentialIterator() const
{
    return new libpc::drivers::oci::SequentialIterator(*this);
}


}}} // namespace libpc::driver::oci
