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


#include <libpc/drivers/oci/Common.hpp>

#include <iostream>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning
#include <boost/make_shared.hpp>

#include <libpc/Bounds.hpp>
#include <libpc/exceptions.hpp>

namespace libpc { namespace drivers { namespace oci {


Options::Options()
{
    m_tree.put("is3d", false);
    m_tree.put("solid", false);
    m_tree.put("overwrite", false);
    m_tree.put("debug", false);
    m_tree.put("verbose", false);
    m_tree.put("srid", 4269);
    m_tree.put("capacity", 8000);
    m_tree.put("precision", 8);
    m_tree.put("cloud_id", -1);
    m_tree.put("connection", std::string(""));
    m_tree.put("block_table_name", std::string("output"));
    m_tree.put("block_table_partition_column", std::string(""));
    m_tree.put("block_table_partition_value", boost::int32_t(0));
    m_tree.put("base_table_name", std::string("hobu"));
    m_tree.put("cloud_column_name", std::string("cloud"));
    m_tree.put("header_blob_column_name", std::string("header"));
    m_tree.put("base_table_aux_columns", std::string(""));
    m_tree.put("base_table_aux_values", std::string(""));
    m_tree.put("base_table_boundary_column", std::string(""));
    m_tree.put("base_table_boundary_wkt", std::string(""));
    m_tree.put("pre_block_sql", std::string(""));
    m_tree.put("pre_sql", std::string(""));
    m_tree.put("post_block_sql", std::string(""));
    m_tree.put("select_sql", std::string(""));
    m_tree.put("base_table_bounds", libpc::Bounds<double>());
    m_tree.put("blob_read_byte_size", boost::uint32_t(2000));
    
    boost::property_tree::ptree scales;
    scales.put("x", double(0.01));
    scales.put("y", double(0.01));
    scales.put("z", double(0.01));

    boost::property_tree::ptree offsets;
    offsets.put("x", double(0.0));
    offsets.put("y", double(0.0));
    offsets.put("z", double(0.0));
    
    m_tree.add_child("scale", scales);
    m_tree.add_child("offset", offsets);
}    

bool Options::IsDebug() const
{
    bool debug = false;
    try
    {
        debug = m_tree.get<bool>("debug");
    }
    catch (boost::property_tree::ptree_bad_path const& e) {
      ::boost::ignore_unused_variable_warning(e);

    }
    return debug;
}

bool Options::Is3d() const
{
    bool is3d = false;
    try
    {
        is3d = m_tree.get<bool>("is3d");
    }
    catch (boost::property_tree::ptree_bad_path const& e) {
      ::boost::ignore_unused_variable_warning(e);

    }
    return is3d;
}

bool Options::IsSolid() const
{
    bool IsSolid = false;
    try
    {
        IsSolid = m_tree.get<bool>("solid");
    }
    catch (boost::property_tree::ptree_bad_path const& e) {
      ::boost::ignore_unused_variable_warning(e);

    }
    return IsSolid;
}


std::string to_upper(const std::string& input)
{
    std::string inp = std::string(input);
    std::string output = std::string(input);
    
    std::transform(inp.begin(), inp.end(), output.begin(), static_cast < int(*)(int) > (toupper));
    
    return output;
}


Connection Connect(Options const& options)
{
    std::string connection;
    try {
        connection = options.GetPTree().get<std::string>("connection");
        
    } catch (boost::property_tree::ptree_bad_path const& ) 
    {
        throw libpc_error("'connection' string for oracle not set in options");
    }

    bool verbose = false;
    try {
        verbose = options.GetPTree().get<bool>("verbose");
        
    } catch (boost::property_tree::ptree_bad_path const& ) 
    {
        throw libpc_error("'verbose' bool for oracle not set in options");
    }
    
    if (connection.empty())
        throw libpc_error("Oracle connection string empty! Unable to connect");

    
    std::string::size_type slash_pos = connection.find("/",0);
    std::string username = connection.substr(0,slash_pos);
    std::string::size_type at_pos = connection.find("@",slash_pos);

    std::string password = connection.substr(slash_pos+1, at_pos-slash_pos-1);
    std::string instance = connection.substr(at_pos+1);
    
    Connection con = boost::make_shared<OWConnection>(username.c_str(),password.c_str(),instance.c_str());
    
    if (con->Succeeded())
        if (verbose)
            std::cout << "Oracle connection succeeded" << std::endl;
    else
        throw libpc_error("Oracle connection failed");
        
    return con;
    
}


}}} // namespace libpc::driver::oci

void CPL_STDCALL OCIGDALErrorHandler(CPLErr eErrClass, int err_no, const char *msg)
{
    std::ostringstream oss;
    
    if (eErrClass == CE_Failure || eErrClass == CE_Fatal) {
        oss <<"GDAL Failure number=" << err_no << ": " << msg;
        throw libpc::libpc_error(oss.str());
    } else {
        return;
    }
}

void CPL_STDCALL OCIGDALDebugErrorHandler(CPLErr eErrClass, int err_no, const char *msg)
{
    std::ostringstream oss;
    
    if (eErrClass == CE_Failure || eErrClass == CE_Fatal) {
        oss <<"GDAL Failure number=" << err_no << ": " << msg;
        throw libpc::libpc_error(oss.str());
    } else if (eErrClass == CE_Debug) {
        std::cout <<"GDAL Debug: " << msg << std::endl;
    } else {
        return;
    }
}
