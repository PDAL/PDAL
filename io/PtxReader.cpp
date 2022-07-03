/******************************************************************************
* Copyright (c) 2022, Daniel Brookes (dbrookes@micromine.com)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "PtxReader.hpp"

namespace pdal
{

static const StaticPluginInfo sc_info
{
    "readers.ptx",
    "Ptx Reader",
    "http://pdal.io/stages/readers.ptx.html",
    { "ptx" }
};

CREATE_STATIC_STAGE(PtxReader, sc_info);

PtxReader::~PtxReader()
{
    if (m_istream)
        Utils::closeFile(m_istream);
}

std::string PtxReader::getName() const
{
    return sc_info.name;
}

PtxReader::PtxHeader PtxReader::readHeader()
{
    // Read header from input stream. Ptx files can have multiple headers per 
    // file, each with their own column count, row count, scanner posisition and 
    // scanner transform. The 4x4 transform combines the scanner position
    // translation and 3x3 scanner transform afaik. For the sake of this reader
    // we actually ignore the scanner position and 3x3 transform and apply the
    // 4x4 transform matrix instead. It possibly makes sense to validate the
    // scanner position and 3x3 transform is the same as the 4x4 transformation
    // matrix. Possibly this is incorrect to assume?
    //
    //      c                   (Column count)
    //      r                   (Row count)
    //      sx sy sz            (Scanner position)
    //      s11 s21 s31         (Scanner 3x3 transformation matrix)
    //      s12 s22 s32
    //      s13 s23 s33
    //      t11 t21 t31 t41     (4x4 transformation matrix)
    //      t12 t22 t32 t42
    //      t13 t23 t33 t43
    //      t14 t24 t34 t44
    //
    
    PtxHeader header;
    std::string buf;

    if (!m_istream || !m_istream->good())
        throwError("Unable to read header in '" + m_filename + "'.");
    
    // Read column count and row count.

    std::getline(*m_istream, buf);
    if (!m_istream->good() || !Utils::fromString(buf, header.m_columns))
    {
        throwError("Invalid column size '" + buf + "' in header for file '" + 
                   m_filename + "'.");
    }
    std::getline(*m_istream, buf);
    if (!m_istream->good() || !Utils::fromString(buf, header.m_rows))
    {
        throwError("Invalid row size '" + buf + "' in header for file '" +  
                   m_filename + "'.");
    }

    // Skip scanner position and scanner 3x3 transformation matrix.

    for (size_t skip = 0; skip < 4; ++skip)
    {
        std::getline(*m_istream, buf);
        if (!m_istream->good())
        {
            throwError("Unable to skip scanner position and scanner transform "
                       "in header for file '" + m_filename + "'.");
        }
    }

    // Read 4x4 transformation matrix.

    for (size_t ty = 0; ty < 4; ++ty)
    {
        std::getline(*m_istream, buf);

        const StringList fields = Utils::split2(buf, ' ');
        if (!m_istream->good() || fields.size() != 4)
        {
            throwError("Invalid transform row '" + buf + "' in header for file'"
                       + m_filename + "'.");
        }
        
        for (size_t tx = 0; tx < 4; ++tx)
        {
            if (!Utils::fromString(fields[tx], header.m_transform[tx + ty * 4]))
            {
                throwError("Invalid transform value '" + fields[tx] + "' in "
                           "header for file '" + m_filename + "'.");
            }
        }
    }

    return header;
}

void PtxReader::PtxHeader::applyTransform(double &x, double &y, double &z) const
{
    const double x2 = x * m_transform[0] + y * m_transform[4] + 
        z * m_transform[8] + m_transform[12];
    const double y2 = x * m_transform[1] + y * m_transform[5] + 
        z * m_transform[9] + m_transform[13];
    const double z2 = x * m_transform[2] + y * m_transform[6] + 
        z * m_transform[10] + m_transform[14];
    x = x2;
    y = y2;
    z = z2;
}

void PtxReader::initialize(PointTableRef table)
{
    m_istream = Utils::openFile(m_filename);
    if (!m_istream)
        throwError("Unable to open file '" + m_filename + "'.");
    
    // We read past the header and peek the first point of the file to determine 
    // what dimensions we are going to have. We assume each cloud in the Ptx 
    // file has the same layout.

    readHeader();

    // Read point from input stream. Ptx files have a similar-ish point layout
    // to Pts files. Points will have X, Y, Z, intensity (between 0.0 and 1.0)
    // and *optional* R, G, B. The valid configurations are:
    //
    //      X Y Z Intensity
    //
    //          or
    //
    //      X Y Z Intensity R G B
    //

    std::string buf;
    std::getline(*m_istream, buf);
    const StringList fields = Utils::split2(buf, ' ');

    m_dimensions = {};
    switch (fields.size())
    {
        case 7:
            m_dimensions.push_back(Dimension::Id::Blue);
            m_dimensions.push_back(Dimension::Id::Green);
            m_dimensions.push_back(Dimension::Id::Red);
            [[fallthrough]];
        case 4:
            m_dimensions.push_back(Dimension::Id::Intensity);
            m_dimensions.push_back(Dimension::Id::Z);
            m_dimensions.push_back(Dimension::Id::Y);
            m_dimensions.push_back(Dimension::Id::X);
            break;
        default:
            throwError("Invalid number of fields for the first point in file '" 
                       + m_filename + "'.");
    }
    std::reverse(m_dimensions.begin(), m_dimensions.end());

    Utils::closeFile(m_istream);
    m_istream = nullptr;
}

void PtxReader::addArgs(ProgramArgs& args)
{
    args.add("discard_missing_points", 
             "Skip over missing input points with XYZ values of \"0 0 0\".", 
             m_discardMissingPoints, true);
}

void PtxReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims(m_dimensions);
}

void PtxReader::ready(PointTableRef table)
{
    m_istream = Utils::openFile(m_filename);
    if (!m_istream)
        throwError("Unable to open file '" + m_filename + "'.");
}

point_count_t PtxReader::read(PointViewPtr view, point_count_t numPts)
{
    PointId index = view->size();
    point_count_t count = 0;

    PtxHeader header;
    size_t line = 1;
    point_count_t countPerHeader = 0;
    std::string buf;

    while (m_istream && m_istream->good() && count < numPts)
    {
        if (countPerHeader == 0 ||
            countPerHeader == (header.m_columns * header.m_rows))
        {
            // Either we are at the start of the file OR we have finished
            // reading the expected number of points for our previous header.
            // In either case we expect now to read the next header. I wonder
            // if it makes sense to write a cloud index, or similar dimension?

            if (m_istream->peek() == EOF)
                break; // We have reached the end of the file, so we break out!

            // Read next header. This will throw on failure. We are a bit
            // stricter about this than we are with point read failures.

            try
            {
                header = readHeader();
            }
            catch (...)
            {
                log()->get(LogLevel::Error) << "Line " << line <<
                    " in '" << m_filename << "' contains an invalid header!"
                    << std::endl;
                throw;
            }

            line += 10;
            countPerHeader = 0;
        }

        ++countPerHeader;

        std::getline(*m_istream, buf);
        ++line;
        if (buf.empty())
            continue;

        const StringList fields = Utils::split2(buf, ' ');
        if (fields.size() != m_dimensions.size())
        {
            // As mentioned above. We assume each cloud in the Ptx file has
            // the same number of fields. As far as I know that's okay!

            log()->get(LogLevel::Error) << "Line " << line <<
               " in '" << m_filename << "' contains " << fields.size() <<
               " fields when " << m_dimensions.size() << " were expected.  "
               "Ignoring." << std::endl;
            continue;
        }

        // Note that similar to the Ptx reader we lazily treat RGB as doubles,
        // for simplicity. It of course gets casted when we set the dimension
        // field to the dimension's type.

        std::array<double, 7> values{ 0.0 };
        for (size_t i = 0; i < fields.size(); ++i)
        {
            double value;
            if (!Utils::fromString(fields[i], value))
            {
                log()->get(LogLevel::Error) << "Can't convert "
                    "field '" << fields[i] << "' to numeric value on line " <<
                    line << " in '" << m_filename << "'.  Setting to 0." <<
                    std::endl;
                value = 0.0;
            }

            if (m_dimensions[i] == Dimension::Id::Intensity) 
            {
                // Intensity field in Ptx is 0.0 to 1.0, we map to PDAL 0 to 
                // 4096. We don't actually check the intensity is between 0.0 
                // and 1.0?

                value *= 4096;
            }

            values[i] = value;
        }

        if (m_discardMissingPoints)
        {
            // Ptx files contain "fully populated" point clouds. This means they
            // can (and likely will) contain missing points. If the discard 
            // missing points argument was set we will skip over these. A 
            // missing point is defined as a point with XYZ values of "0 0 0". 
            // We check the XYZ values were exactly 0 to determine if the point 
            // was a missing point.

            if (values[0] == 0.0 && values[1] == 0.0 && values[2] == 0.0)
            {
                log()->get(LogLevel::Debug) << "Line " << line << " in '" <<
                    m_filename << "' is a missing point. Ignoring." << 
                    std::endl;
                continue;
            }
        }

        // Apply the 4x4 transformation matrix for our current header to the 
        // point's X, Y and Z values.

        header.applyTransform(values[0], values[1], values[2]);

        // Write our field values to their dimensions.

        for (size_t i = 0; i < fields.size(); ++i)
            view->setField(m_dimensions[i], index, values[i]);

        ++count;
        ++index;
    }

    if (countPerHeader < (header.m_columns * header.m_rows))
    {
        log()->get(LogLevel::Warning) << "Expected " << 
            (header.m_columns * header.m_rows) << " points but only " << 
            countPerHeader << " were found." << std::endl;
    }

    return count;
}

void PtxReader::done(PointTableRef table)
{
    if (m_istream)
    {
        Utils::closeFile(m_istream);
        m_istream = nullptr;
    }
}

} // namespace pdal
