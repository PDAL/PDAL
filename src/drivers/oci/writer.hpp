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

#ifndef INCLUDED_OCIWRITER_HPP
#define INCLUDED_OCIWRITER_HPP

#include <libpc/Consumer.hpp>
#include <libpc/chipper.hpp>

#include "block.hpp"
#include "oci_wrapper.h"

#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/shared_ptr.hpp>

#include <cpl_port.h>

namespace libpc { namespace driver { namespace oci {

#ifdef _WIN32
#define compare_no_case(a,b,n)  _strnicmp( (a), (b), (n) )
#else
#define compare_no_case(a,b,n)  strncasecmp( (a), (b), (n) )
#endif


class Options;

typedef boost::shared_ptr<OWConnection> Connection ;
typedef boost::shared_ptr<OWStatement> Statement ;

void CPL_STDCALL OCIGDALErrorHandler(CPLErr eErrClass, int err_no, const char *msg);
void CPL_STDCALL OCIGDALDebugErrorHandler(CPLErr eErrClass, int err_no, const char *msg);
std::string to_upper(std::string const& input);

class LIBPC_DLL Options
{

private:
    boost::property_tree::ptree m_tree;

public:

    Options();
    bool IsDebug() const;
    boost::property_tree::ptree GetPTree() const {return m_tree; }

};



class LIBPC_DLL Writer : public Consumer
{
    typedef std::vector<libpc::driver::oci::Block> Blocks;
    
public:
    Writer(Stage& prevStage, Options& options);
    ~Writer();
    
    const std::string& getName() const;

    void run(std::ostringstream const& command);
    inline void setBounds(libpc::Bounds<double> bounds) {m_bounds = bounds; }
    inline libpc::Bounds<double>  getBounds() const { return m_bounds; }
protected:
    // this is called once before the loop with the writeBuffer calls
    virtual void writeBegin();

    // called repeatedly, until out of data
    virtual boost::uint32_t writeBuffer(const PointData&);

    // called once, after the writeBuffer calls
    virtual void writeEnd();

private:


    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented
    // 
    Connection Connect();
    void Debug();
    void WipeBlockTable();
    void CreateBlockIndex();
    void CreateBlockTable();
    long CreatePCEntry(std::vector<boost::uint8_t> const* header_data);
    long GetGType();
    std::string CreatePCElemInfo();
    bool BlockTableExists();
    void RunFileSQL(std::string const& filename);
    bool IsGeographic(boost::int32_t srid);
    std::string LoadSQLData(std::string const& filename);
    
    Stage& m_stage;
    chipper::Chipper m_chipper;
    
    Options& m_options;
    Blocks m_blocks;
    libpc::Bounds<double> m_bounds; // Bounds of the entire point cloud
    Connection m_connection;
    bool m_verbose;
};

}}} // namespace libpc::driver::oci


#endif // INCLUDED_OCIWRITER_HPP
