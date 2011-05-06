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

#ifndef INCLUDED_LIBPC_DRIVER_QFIT_READER_HPP
#define INCLUDED_LIBPC_DRIVER_QFIT_READER_HPP

#include <libpc/libpc.hpp>

#include <libpc/Stage.hpp>

#include <libpc/SchemaLayout.hpp>

#include <libpc/Iterator.hpp>
#include <libpc/exceptions.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <vector>

#include <boost/detail/endian.hpp>

#ifdef BOOST_LITTLE_ENDIAN
# define QFIT_SWAP_BE_TO_LE(p) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + sizeof(p) - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)

# define QFIT_SWAP_BE_TO_LE_N(p, n) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + n - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)
#endif

namespace libpc { namespace drivers { namespace qfit {


enum QFIT_Format_Type
{
    QFIT_Format_10 = 10,
    QFIT_Format_12 = 12,
    QFIT_Format_14 = 14,
    QFIT_Format_Unknown = 128
};

class qfit_error : public libpc_error
{
public:

    qfit_error(std::string const& msg)
        : libpc_error(msg)
    {}
};

class LIBPC_DLL Reader : public libpc::Stage
{

public:
    Reader(Options& options);
    ~Reader();
    
    const std::string& getDescription() const;
    const std::string& getName() const;
 
    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        return false;
    }

    boost::uint64_t getNumPoints() { return 0; }
    
    libpc::SequentialIterator* createSequentialIterator() const;
    
    Options& getOptions() const { return m_options; }

protected:
    inline QFIT_Format_Type getFormat() const { return m_format; }

private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented

    Options& m_options;
    QFIT_Format_Type m_format;
    
    void registerFields();


};

}}} // namespace libpc::driver::oci


#endif // INCLUDED_LIBPC_DRIVER_OCI_READER_HPP
