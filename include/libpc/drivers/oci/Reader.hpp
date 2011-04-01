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

#ifndef INCLUDED_LIBPC_DRIVER_OCI_READER_HPP
#define INCLUDED_LIBPC_DRIVER_OCI_READER_HPP

#include <libpc/libpc.hpp>

#include <libpc/Stage.hpp>
#include <libpc/drivers/oci/Common.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>

namespace libpc { namespace drivers { namespace oci {

class Block
{
    
public:
    
    Block() {};
    
    boost::int32_t           obj_id;
    boost::int32_t           blk_id;
    sdo_geometry*   blk_extent;
    sdo_orgscl_type* blk_domain;

    double           pcblk_min_res;
    double           pcblk_max_res;
    boost::int32_t           num_points;
    boost::int32_t           num_unsorted_points;
    boost::int32_t           pt_sort_dim;


private:
    boost::uint32_t m_capacity;
};

class LIBPC_DLL Reader : public libpc::Stage
{

public:
    Reader(Options& options);
    ~Reader();
    
    const std::string& getName() const;
 
    bool supportsIterator (StageIteratorType t) 
    {   
        if (t == StageIterator_Sequential ) return true;
        return false;
    }

    enum QueryType
    {
        QUERY_SDO_PC,
        QUERY_SDO_PC_BLK,
        QUERY_BLK_TABLE,
        QUERY_UNKNOWN
    };
    
    libpc::SequentialIterator* createSequentialIterator() const;
    Connection getConnection () const { return m_connection;}
    Options& getOptions() const { return m_options; }
    
    sdo_pc* getPCObject() const { return m_pc; }
    sdo_pc_blk* getPCBlockObject() const { return m_block_table_type; }
    
    Block* getBlock() const { return m_block_table; }
    bool fetchNext() const;
private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
    // 
    
    void Debug();
    void registerFields();
    void fetchPCFields();
    QueryType describeQueryType();
    void doBlockTableDefine();

    Options& m_options;
    Connection m_connection;
    Statement m_statement;
    bool m_verbose;
    QueryType m_qtype;
    
    sdo_pc* m_pc;
    sdo_pc_blk* m_block_table_type;
    Block* m_block_table;
    OCILobLocator* m_locator;
    std::vector<boost::uint8_t> m_points;

};

}}} // namespace libpc::driver::oci


#endif // INCLUDED_LIBPC_DRIVER_OCI_READER_HPP
