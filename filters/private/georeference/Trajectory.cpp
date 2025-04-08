/******************************************************************************
 * Copyright (c) 2023, Guilhem Villemin (guilhem.villemin@altametris.com)
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
 *     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include "Trajectory.hpp"
#include "Utils.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <pdal/Options.hpp>
#include <pdal/pdal_types.hpp>

#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{
namespace georeference
{
using DimId = Dimension::Id;

Trajectory::Trajectory(const std::string& filename, const nlohmann::json& opts)
{
    std::string driver("");
    if (opts.contains("type"))
        driver = opts.at("type").get<std::string>();
    if (driver.empty())
        driver = StageFactory::inferReaderDriver(filename);
    if (driver.empty())
        throw pdal_error("Cannot determine reader for input file: " + filename);

    Options readerOptions;
    readerOptions.add("filename", filename);
    for (auto& arg : opts.items())
    {
        if (arg.key() == "type")
            continue;

        nlohmann::detail::value_t type = opts.at(arg.key()).type();
        switch (type)
        {
        case nlohmann::detail::value_t::string:
        {
            std::string val = arg.value().get<std::string>();
            readerOptions.add(arg.key(), arg.value().get<std::string>());
            break;
        }
        case nlohmann::detail::value_t::number_float:
        {
            readerOptions.add(arg.key(), arg.value().get<float>());
            break;
        }
        case nlohmann::detail::value_t::number_integer:
        {
            readerOptions.add(arg.key(), arg.value().get<int>());
            break;
        }
        case nlohmann::detail::value_t::boolean:
        {
            readerOptions.add(arg.key(), arg.value().get<bool>());
            break;
        }
        default:
        {
            readerOptions.add(arg.key(), arg.value());
            break;
        }
        }
    }

    std::unique_ptr<StageFactory> factory(new StageFactory);
    Stage* reader = factory->createStage(driver);
    reader->setOptions(readerOptions);
    reader->prepare(m_table);
    m_set = reader->execute(m_table);
    m_pointView = *(m_set.begin());
}

bool Trajectory::getTrajPoint(double time, TrajPoint& output) const
{
    PointViewIter upper = std::lower_bound(
        m_pointView->begin(), m_pointView->end(), time,
        [](const PointRef pt, double time)
        { return pt.getFieldAs<double>(DimId::GpsTime) < time; });
    if (upper != m_pointView->begin() && upper != m_pointView->end())
    {
        PointRef p1 = *(upper - 1);
        PointRef p2 = *upper;
        const double t1 = p1.getFieldAs<double>(DimId::GpsTime);
        const double t2 = p2.getFieldAs<double>(DimId::GpsTime);
        const double frac = (time - t1) / (t2 - t1);

        output.roll = Utils::getAngle(p1.getFieldAs<double>(DimId::Roll),
                                      p2.getFieldAs<double>(DimId::Roll), frac);
        output.pitch =
            Utils::getAngle(p1.getFieldAs<double>(DimId::Pitch),
                            p2.getFieldAs<double>(DimId::Pitch), frac);
        output.azimuth =
            Utils::getAngle(p1.getFieldAs<double>(DimId::Azimuth),
                            p2.getFieldAs<double>(DimId::Azimuth), frac);
        output.wanderAngle =
            Utils::getAngle(p1.getFieldAs<double>(DimId::WanderAngle),
                            p2.getFieldAs<double>(DimId::WanderAngle), frac);
        output.x = Utils::getAngle(p1.getFieldAs<double>(DimId::X),
                                   p2.getFieldAs<double>(DimId::X), frac);
        output.y = Utils::getAngle(p1.getFieldAs<double>(DimId::Y),
                                   p2.getFieldAs<double>(DimId::Y), frac);
        output.z = Utils::getValue(p1.getFieldAs<double>(DimId::Z),
                                   p2.getFieldAs<double>(DimId::Z), frac);
        output.time =
            Utils::getValue(p1.getFieldAs<double>(DimId::GpsTime),
                            p2.getFieldAs<double>(DimId::GpsTime), frac);
        return true;
    }
    return false;
}
} // namespace georeference
} // namespace pdal
