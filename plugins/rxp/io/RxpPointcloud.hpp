/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski (pete.gadomski@gmail.com)
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
*
*
* This software has not been developed by RIEGL, and RIEGL will not provide any
* support for this driver. Please do not contact RIEGL with any questions or
* issues regarding this driver. RIEGL is not responsible for damages or other
* issues that arise from use of this driver. This driver has been tested
* against RiVLib version 1.39 on a Ubuntu 14.04 using gcc43.
 ****************************************************************************/

#pragma once

#include <riegl/scanlib.hpp>

#include <pdal/pdal_macros.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>


namespace pdal
{


Dimension::Id::Enum getTimeDimensionId(bool syncToPps);


class PDAL_DLL RxpPointcloud : public scanlib::pointcloud
{
public:
    RxpPointcloud(
            const std::string& uri,
            bool isSyncToPps,
            bool m_minimal,
            PointTableRef table);
    virtual ~RxpPointcloud();

    point_count_t read(PointViewPtr view, point_count_t count);

    inline bool isSyncToPps() const
    {
        return m_syncToPps;
    }

protected:
    void on_echo_transformed(echo_type echo);

private:
    virtual point_count_t writeSavedPoints(PointViewPtr view, point_count_t count);

    PointViewPtr m_view;
    point_count_t m_idx;
    bool m_syncToPps;
    bool m_minimal;
    std::shared_ptr<scanlib::basic_rconnection> m_rc;
    scanlib::decoder_rxpmarker m_dec;
    scanlib::buffer m_rxpbuf;

};


} // namespace pdal
