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

#ifndef INCLUDED_PDAL_DRIVER_OCI_READER_HPP
#define INCLUDED_PDAL_DRIVER_OCI_READER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Reader.hpp>
#include <pdal/drivers/oci/common.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>

namespace pdal { namespace drivers { namespace oci {



class PDAL_DLL Reader : public pdal::Reader, pdal::drivers::oci::OracleDriver
{
public:
    SET_STAGE_NAME("drivers.oci.reader", "OCI Reader")

    Reader(const Options&);
    ~Reader();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;
 
    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        return false;
    }

    boost::uint64_t getNumPoints() { return 0; }
    
    pdal::StageSequentialIterator* createSequentialIterator() const;
    
    Connection getConnection () const { return m_connection;}
    Statement getStatement () const { return m_statement;}
    CloudPtr getCloud() const;
    BlockPtr getBlock() const { return m_block; }
    std::string getQuery() const;
    void defineBlock(Statement statement, BlockPtr block) const;
    
        
    QueryType getQueryType() const {return m_querytype; }
    Schema fetchSchema(Statement statement, sdo_pc* pc, boost::uint32_t& capacity) const;
    pdal::SpatialReference fetchSpatialReference(Statement statement, sdo_pc* pc) const;
    // for dumping
    virtual boost::property_tree::ptree toPTree() const;

  
    static void CPL_STDCALL trampoline(::CPLErr code, int num, char const* msg)
    {
#if GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR >= 9
        static_cast<Reader*>(CPLGetErrorHandlerUserData())->m_gdal_callback(code, num, msg);
#else
        if (code == CE_Failure || code == CE_Fatal) {
            std::ostringstream oss;
            oss <<"GDAL Failure number=" << num << ": " << msg;
            throw gdal_error(oss.str());
        } else if (code == CE_Debug) {
            std::clog << " (no log control stdlog) GDAL debug: " << msg << std::endl;
        } else {
            return;
        }
#endif
    }
    
    void CPL_STDCALL GDAL_log(::CPLErr code, int num, char const* msg);
    void CPL_STDCALL GDAL_error(::CPLErr code, int num, char const* msg);
    
private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
    // 
    
    QueryType describeQueryType() ;

    Connection m_connection;
    Statement m_statement;
    QueryType m_querytype;
    
    sdo_pc* m_pc;
    BlockPtr m_block;
    sdo_pc_blk* m_pc_block;
    boost::uint32_t m_capacity;
    
    // Fields in the form of NAME:TYPE
    std::map<std::string, int> m_fields;

    boost::function<void(CPLErr, int, char const*)> m_gdal_callback;
  

};

}}} // namespace pdal::driver::oci


#endif // INCLUDED_PDAL_DRIVER_OCI_READER_HPP
