/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#pragma warning(push)
#pragma warning(disable: 4251)
#include <gdal.h>
#include <gdal_priv.h>
#pragma warning(pop)

#include "Raster.hpp"

namespace pdal
{
namespace gdal
{

BaseBand::BaseBand(GDALDataset *ds, int bandNum, const std::string& name)
{
    m_band = ds->GetRasterBand(bandNum);
    if (!m_band)
        throw InvalidBand();

    if (name.size())
    {
        m_band->SetDescription(name.data());
        // We don't care about offset, but this sets the flag to indicate
        // that the metadata has changed.
        m_band->SetOffset(m_band->GetOffset(NULL) + .00001);
        m_band->SetOffset(m_band->GetOffset(NULL) - .00001);
    }
}

void BaseBand::totalSize(int& x, int& y)
{
    x = m_band->GetXSize();
    y = m_band->GetYSize();
}

void BaseBand::blockSize(int& x, int& y)
{
    m_band->GetBlockSize(&x, &y);
}

void BaseBand::readBlockBuf(int x, int y, uint8_t *buf)
{
    if (m_band->ReadBlock(x, y, buf) != CPLE_None)
        throw CantReadBlock();
}

void BaseBand::writeBlockBuf(int x, int y, const uint8_t *buf)
{
    void *v = reinterpret_cast<void *>(const_cast<uint8_t *>(buf));
    if (m_band->WriteBlock(x, y, reinterpret_cast<void *>(v)) != CPLE_None)
        throw CantWriteBlock();
}

void BaseBand::statistics(double* minimum, double* maximum,
                    double* mean, double* stddev,
                    bool approx, bool force) const
{
    m_band->GetStatistics(approx, force, minimum, maximum, mean, stddev);
}

} // namespace gdal
} // namespace pdal
