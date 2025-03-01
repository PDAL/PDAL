
#include "SpzReader.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/IStream.hpp>

#include <arbiter/arbiter.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.spz",
    "SPZ Reader",
    "http://pdal.io/stages/readers.spz.html",
    { "spz" }
};

CREATE_STATIC_STAGE(SpzReader, s_info)
std::string SpzReader::getName() const { return s_info.name; }

SpzReader::SpzReader()
{}

void SpzReader::addArgs(ProgramArgs& args)
{}

void SpzReader::extractHeaderData()
{
    m_numPoints = m_data->numPoints;

    // total number of harmonics / 3
    constexpr std::array<int, 4> numHarmonics { 0, 3, 8, 15 };
    m_numSh = numHarmonics[m_data->shDegree];
}

void SpzReader::initialize()
{
    // copy the file to local if it's remote
    m_isRemote = Utils::isRemote(m_filename);
    if (m_isRemote)
    {
        std::string remoteFilename = Utils::tempFilename(m_filename);
        // swap m_filename to temp file, remoteFilename to original
        std::swap(remoteFilename, m_filename);

        arbiter::Arbiter a;
        a.put(m_filename, a.getBinary(remoteFilename));
    }

    ILeStream stream(m_filename);
    if (!stream)
        throwError("Unable to open file '" + m_filename + "'");
    stream.seek(0, std::ios::end);
    std::vector<uint8_t> data(stream.position());
    stream.seek(0, std::ios::beg);
    stream.get(reinterpret_cast<char *>(data.data()), data.size());
    stream.close();

    m_data.reset(new spz::PackedGaussians(spz::loadSpzPacked(data)));
    if (m_data->usesFloat16())
        throwError("SPZ float16 point encoding not supported!");

    extractHeaderData();
}

void SpzReader::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    layout->registerDim(Id::X);
    layout->registerDim(Id::Y);
    layout->registerDim(Id::Z);

    // RGB, alpha, scale, rotation, SH dimensions set with PLY naming conventions
    m_alphaDim = layout->assignDim("opacity", Type::Float);
    for (int i = 0; i < 3; ++i)
    {
        m_colorDims.push_back(layout->assignDim("f_dc_" + std::to_string(i),
            Type::Float));
        m_scaleDims.push_back(layout->assignDim("scale_" + std::to_string(i),
            Type::Float));
    }

    for (int i = 0; i < 4; ++i)
        m_rotDims.push_back(layout->assignDim("rot_" + std::to_string(i),
            Type::Float));

    for (int i = 0; i < (m_numSh * 3); ++i)
        m_shDims.push_back(layout->assignDim("f_rest_" + std::to_string(i),
            Type::Float));
}

void SpzReader::ready(PointTableRef table)
{
    m_index = 0;
}

point_count_t SpzReader::read(PointViewPtr view, point_count_t count)
{
    PointId idx = view->size();

    count = (std::min)(m_numPoints - m_index, count);
    point_count_t numRead = m_index;
    while (numRead < count)
    {
        spz::UnpackedGaussian unpacked = m_data->unpack(numRead);

        view->setField(Dimension::Id::X, idx, unpacked.position[0]);
        view->setField(Dimension::Id::Y, idx, unpacked.position[1]);
        view->setField(Dimension::Id::Z, idx, unpacked.position[2]);

        // set RGB
        for (int i =0; i < 3; ++i)
            view->setField(m_colorDims[i], idx, unpacked.color[i]);

        view->setField(m_alphaDim, idx, unpacked.alpha);

        // rotation W - xyzw in unpacked, needs to be stored as wxyz for PLY
        view->setField(m_rotDims[0], idx, unpacked.rotation[3]);
        // scale and rotation x/y/z
        for (int i = 0; i < 3; ++i)
        {
            view->setField(m_rotDims[i + 1], idx, unpacked.rotation[i]);
            view->setField(m_scaleDims[i], idx, unpacked.scale[i]);
        }

        // spherical harmonics
        for (int i = 0; i < m_numSh; ++i)
            view->setField(m_shDims[i], idx, unpacked.shR[i]);
        for (int i = 0; i < m_numSh; ++i)
            view->setField(m_shDims[m_numSh + i], idx, unpacked.shG[i]);
        for (int i = 0; i < m_numSh; ++i)
            view->setField(m_shDims[2 * m_numSh + i], idx, unpacked.shB[i]);

        numRead++;
        idx++;
    }
    m_index += numRead;

    return numRead;
}

void SpzReader::done(PointTableRef table)
{
    // delete tmp file
    if (m_isRemote)
        FileUtils::deleteFile(m_filename);
}

} // namespace pdal