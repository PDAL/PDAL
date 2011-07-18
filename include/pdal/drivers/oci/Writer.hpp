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

#ifndef INCLUDED_PDAL_DRIVER_OCI_WRITER_HPP
#define INCLUDED_PDAL_DRIVER_OCI_WRITER_HPP

#include <pdal/pdal.hpp>

#include <pdal/Writer.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/filters/Chipper.hpp>

#include "common.hpp"

namespace pdal { namespace drivers { namespace oci {



class PDAL_DLL Writer : public pdal::Writer
{
    
public:
    Writer(const DataStagePtr& prevStage, const Options& options);
    Writer(const DataStagePtr& prevStage, OptionsOld& options);
    ~Writer();
    
    const std::string& getDescription() const;
    const std::string& getName() const;

    void run(std::ostringstream const& command);
    inline void setBounds(pdal::Bounds<double> bounds) {m_bounds = bounds; }
    inline pdal::Bounds<double>  getBounds() const { return m_bounds; }
    
    inline Connection getConnection() const { return m_connection;}



protected:
    // this is called once before the loop with the writeBuffer calls
    virtual void writeBegin();

    // called repeatedly, until out of data
    virtual boost::uint32_t writeBuffer(const PointBuffer&);

    // called once, after the writeBuffer calls
    virtual void writeEnd();

private:


    Writer& operator=(const Writer&); // not implemented
    Writer(const Writer&); // not implemented
    // 

    void Debug();
    void WipeBlockTable();
    void CreateBlockIndex();
    void CreateBlockTable();
    void CreateSDOEntry();
    void CreatePCEntry(std::vector<boost::uint8_t> const* header_data);
    long GetGType();
    std::string CreatePCElemInfo();
    bool BlockTableExists();
    void RunFileSQL(std::string const& filename);
    bool IsGeographic(boost::int32_t srid);
    std::string LoadSQLData(std::string const& filename);
    
    bool FillOraclePointBuffer(PointBuffer const& buffer, 
                             std::vector<boost::uint8_t>& point_data);
    bool WriteBlock(PointBuffer const& buffer);

    void SetOrdinates(Statement statement,
                      OCIArray* ordinates, 
                      pdal::Bounds<double> const& extent);
    void SetElements(Statement statement,
                     OCIArray* elem_info);

    bool isVerbose() const;
    bool isDebug() const;
    bool is3d() const;
    bool isSolid() const;
    // PointBuffer& ConstructBuffer(const PointBuffer& input) const;
    
    OptionsOld& m_optionsOld;
    pdal::Bounds<double> m_bounds; // Bounds of the entire point cloud
    Connection m_connection;
    bool m_verbose;
    bool m_doCreateIndex;
};

}}} // namespace pdal::driver::oci


#endif // INCLUDED_OCIWRITER_HPP
