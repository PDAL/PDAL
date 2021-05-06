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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <io/LasReader.hpp>
#include <pdal/compression/ZstdCompression.hpp>

#include "Connector.hpp"
#include "EptInfo.hpp"
#include "TileContents.hpp"
#include "VectorPointTable.hpp"

namespace pdal
{

void TileContents::read()
{
    try
    {
        if (m_info.dataType() == EptInfo::DataType::Laszip)
            readLaszip();
        else if (m_info.dataType() == EptInfo::DataType::Binary)
            readBinary();
#ifdef PDAL_HAVE_ZSTD
        else if (m_info.dataType() == EptInfo::DataType::Zstandard)
            readZstandard();
#endif
        else
            throw pdal_error("Unrecognized EPT dataType");
//ABELL - Should check that we read the number of points specified in the
//  overlap.
        // Read addon information after the native data, we'll possibly
        // overwrite attributes.
        for (const Addon& addon : m_addons)
            readAddon(addon);
    }
    catch (const std::exception& ex)
    {
        m_error = ex.what();
    }
    catch (...)
    {
        m_error = "Unknown exception when reading tile contents";
    }
}

void TileContents::readLaszip()
{
    // If the file is remote (HTTP, S3, Dropbox, etc.), getLocalHandle will
    // download the file and `localPath` will return the location of the
    // downloaded file in a temporary directory.  Otherwise it's a no-op.
    std::string filename = m_info.dataDir() + key().toString() + ".laz";
    auto handle = m_connector.getLocalHandle(filename);

    m_table.reset(new ColumnPointTable);

    Options options;
    options.add("filename", handle.localPath());
    options.add("use_eb_vlr", true);
    options.add("nosrs", true);

    LasReader reader;
    reader.setOptions(options);

    reader.prepare(*m_table);
    reader.execute(*m_table);
}

void TileContents::readBinary()
{
    std::string filename = m_info.dataDir() + key().toString() + ".bin";
    auto data(m_connector.getBinary(filename));

    VectorPointTable *vpt = new VectorPointTable(m_info.remoteLayout());
    vpt->buffer() = std::move(data);
    m_table.reset(vpt);

    transform();
}

#ifdef PDAL_HAVE_ZSTD
void TileContents::readZstandard()
{
    std::string filename = m_info.dataDir() + key().toString() + ".zst";
    auto compressed(m_connector.getBinary(filename));
    std::vector<char> data;
    pdal::ZstdDecompressor dec([&data](char* pos, std::size_t size)
    {
        data.insert(data.end(), pos, pos + size);
    });

    dec.decompress(compressed.data(), compressed.size());

    VectorPointTable *vpt = new VectorPointTable(m_info.remoteLayout());
    vpt->buffer() = std::move(data);
    m_table.reset(vpt);

    transform();
}
#else
void TileContents::readZstandard()
{}
#endif // PDAL_HAVE_ZSTD

void TileContents::readAddon(const Addon& addon)
{
    m_addonTables[addon.localId()] = nullptr;

    point_count_t addonPoints = addon.points(key());
    if (addonPoints == 0)
        return;

    // If the addon hierarchy exists, it must match the EPT data.
    if (addonPoints != size())
        throw pdal_error("Invalid addon hierarchy");

    std::string filename = addon.dataDir() + key().toString() + ".bin";
    const auto data(m_connector.getBinary(filename));

    if (size() * Dimension::size(addon.type()) != data.size())
        throw pdal_error("Invalid addon content length");

    VectorPointTable *vpt = new VectorPointTable(addon.layout());
    vpt->buffer() = std::move(data);
    m_addonTables[addon.localId()] = BasePointTablePtr(vpt);
}

void TileContents::transform()
{
    using D = Dimension::Id;

    // Shorten long name.
    const XForm& xf = m_info.dimType(Dimension::Id::X).m_xform;
    const XForm& yf = m_info.dimType(Dimension::Id::Y).m_xform;
    const XForm& zf = m_info.dimType(Dimension::Id::Z).m_xform;

    PointRef p(*m_table);
    for (PointId i = 0; i < size(); ++i)
    {
        p.setPointId(i);

        // Scale the XYZ values.
        p.setField(D::X, p.getFieldAs<double>(D::X) * xf.m_scale.m_val +
            xf.m_offset.m_val);
        p.setField(D::Y, p.getFieldAs<double>(D::Y) * yf.m_scale.m_val +
            yf.m_offset.m_val);
        p.setField(D::Z, p.getFieldAs<double>(D::Z) * zf.m_scale.m_val +
            zf.m_offset.m_val);
    }
}

} // namespace pdal

