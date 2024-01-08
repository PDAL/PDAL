/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <pdal/util/Algorithm.hpp>
#include <pdal/util/FileUtils.hpp>

// This must come before CopcWriter.hpp as it contains a specialization for
// fromString() that must come be available before the Util template when
// ProgramArgs is seen.
#include "HeaderVal.hpp"

#include "CopcWriter.hpp"
#include <arbiter/arbiter.hpp>

#include "private/las/Header.hpp"
#include "private/las/Utils.hpp"
#include "private/copcwriter/BuPyramid.hpp"
#include "private/copcwriter/CellManager.hpp"
#include "private/copcwriter/Grid.hpp"
#include "private/copcwriter/Reprocessor.hpp"

namespace pdal
{

namespace
{

const StaticPluginInfo s_info
{
    "writers.copc",
    "COPC Writer",
    "http://pdal.io/stages/writer.copc.html",
    {}
};

}

CREATE_STATIC_STAGE(CopcWriter, s_info);

CopcWriter::CopcWriter() : b(new copcwriter::BaseInfo), isRemote(false)
{}

CopcWriter::~CopcWriter()
{}

std::string CopcWriter::getName() const { return s_info.name; }

void CopcWriter::initialize(PointTableRef table)
{
    fillForwardList();
}

void CopcWriter::addArgs(ProgramArgs& args)
{
    std::time_t now;
    std::time(&now);
    uint16_t year = 1900;
    uint16_t doy = 1;
    std::tm* ptm = std::gmtime(&now);
    if (ptm)
    {
        year += ptm->tm_year;
        doy += ptm->tm_yday;
    }

    args.add("filename", "Output filename.", b->opts.filename);
    args.add("forward", "Dimensions to forward from LAS reader", b->opts.forwardSpec);

    args.add("filesource_id", "File source ID number.", b->opts.filesourceId,
        decltype(b->opts.filesourceId)(0));
    args.add("global_encoding", "Global encoding byte", b->opts.globalEncoding,
        decltype(b->opts.globalEncoding)(0));
    args.add("project_id", "Project ID", b->opts.projectId);
    args.add("system_id", "System ID", b->opts.systemId, decltype(b->opts.systemId)("PDAL"));
    args.add("software_id", "Software ID", b->opts.softwareId,
        decltype(b->opts.softwareId)(las::generateSoftwareId()));
    args.add("creation_doy", "Creation day of year", b->opts.creationDoy,
        decltype(b->opts.creationDoy)(doy));
    args.add("creation_year", "Creation year", b->opts.creationYear,
        decltype(b->opts.creationYear)(year));
    args.add("scale_x", "X scale factor", b->opts.scaleX, decltype(b->opts.scaleX)(".01"));
    args.add("scale_y", "Y scale factor", b->opts.scaleY, decltype(b->opts.scaleY)(".01"));
    args.add("scale_z", "Z scale factor", b->opts.scaleZ, decltype(b->opts.scaleZ)(".01"));
    args.add("offset_x", "X offset", b->opts.offsetX);
    args.add("offset_y", "Y offset", b->opts.offsetY);
    args.add("offset_z", "Z offset", b->opts.offsetZ);
    args.add("vlrs", "List of VLRs to set", b->opts.userVlrs);
    args.add("pipeline", "Emit a JSON-represetation of the pipeline as a VLR",
        b->opts.emitPipeline);
    args.add("fixed_seed", "Fix the random seed", b->opts.fixedSeed).setHidden();
    args.add("a_srs", "Spatial reference to use to write output", b->opts.aSrs);
    args.add("threads", "", b->opts.threadCount).setHidden();
    args.add("enhanced_srs_vlrs", "Write WKT2 and PROJJSON as VLR?", b->opts.enhancedSrsVlrs,
        decltype(b->opts.enhancedSrsVlrs)(false));
}

void CopcWriter::fillForwardList()
{
    static const StringList header {
        "dataformat_id", "major_version", "minor_version", "filesource_id",
        "global_encoding", "project_id", "system_id", "software_id",
        "creation_doy", "creation_year"
    };
    static const StringList scale { "scale_x", "scale_y", "scale_z" };
    static const StringList offset { "offset_x", "offset_y", "offset_z" };

    // Build the forward list, replacing special keywords with the proper
    // field names.
    for (auto& name : b->opts.forwardSpec)
    {
        if (name == "all")
        {
            b->forwards.insert(header.begin(), header.end());
            b->forwards.insert(scale.begin(), scale.end());
            b->forwards.insert(offset.begin(), offset.end());
            b->forwardVlrs = true;
        }
        else if (name == "header")
            b->forwards.insert(header.begin(), header.end());
        else if (name == "scale")
            b->forwards.insert(scale.begin(), scale.end());
        else if (name == "offset")
            b->forwards.insert(offset.begin(), offset.end());
        else if (name == "format")
            b->forwards.insert("dataformat_id");
        else if (name == "vlr")
            b->forwardVlrs = true;
        else if (Utils::contains(header, name) ||
                Utils::contains(scale, name) ||
                Utils::contains(offset, name))
            b->forwards.insert(name);
        else
            throwError("Error in 'forward' option.  Unknown field for "
                "forwarding: '" + name + "'.");
    }
}

template <typename T>
void CopcWriter::handleHeaderForward(const std::string& s, T& headerVal, const MetadataNode& base)
{
    if (Utils::contains(b->forwards, s) && !headerVal.valSet())
    {
        MetadataNode invalid = base.findChild(s + "INVALID");
        MetadataNode m = base.findChild(s);
        if (!invalid.valid() && m.valid())
            headerVal.setVal(m.value<typename T::type>());
    }
}

void CopcWriter::handleHeaderForwards(MetadataNode& forward)
{
    handleHeaderForward("filesource_id", b->opts.filesourceId, forward);
    handleHeaderForward("global_encoding", b->opts.globalEncoding, forward);
    handleHeaderForward("project_id", b->opts.projectId, forward);
    handleHeaderForward("system_id", b->opts.systemId, forward);
    handleHeaderForward("software_id", b->opts.softwareId, forward);
    handleHeaderForward("creation_doy", b->opts.creationDoy, forward);
    handleHeaderForward("creation_year", b->opts.creationYear, forward);

    handleHeaderForward("scale_x", b->opts.scaleX, forward);
    handleHeaderForward("scale_y", b->opts.scaleY, forward);
    handleHeaderForward("scale_z", b->opts.scaleZ, forward);
    handleHeaderForward("offset_x", b->opts.offsetX, forward);
    handleHeaderForward("offset_y", b->opts.offsetY, forward);
    handleHeaderForward("offset_z", b->opts.offsetZ, forward);

    b->scaling.m_xXform.m_scale.set(b->opts.scaleX.val());
    b->scaling.m_yXform.m_scale.set(b->opts.scaleY.val());
    b->scaling.m_zXform.m_scale.set(b->opts.scaleZ.val());
    b->scaling.m_xXform.m_offset.set(b->opts.offsetX.val());
    b->scaling.m_yXform.m_offset.set(b->opts.offsetY.val());
    b->scaling.m_zXform.m_offset.set(b->opts.offsetZ.val());
}

void CopcWriter::handleForwardVlrs(MetadataNode& forwards)
{
    MetadataNodeList nodes = forwards.findChildren([](MetadataNode n)
        { return Utils::startsWith(n.name(), "vlr_"); });
    for (auto& n : nodes)
    {
        const MetadataNode& userIdNode = n.findChild("user_id");
        const MetadataNode& recordIdNode = n.findChild("record_id");
        if (userIdNode.valid() && recordIdNode.valid())
        {

            const MetadataNode& descriptionNode = n.findChild("description");
            const std::vector<uint8_t>& data = Utils::base64_decode(n.findChild("data").value());
            const char *d = (const char *)data.data();
            std::vector<char> buf(d, d + data.size());
            las::Evlr e(userIdNode.value(), (uint16_t)std::stoi(recordIdNode.value()),
                descriptionNode.value(), std::move(buf));
            b->vlrs.push_back(e);
        }
    }
}

void CopcWriter::prepared(PointTableRef table)
{
    // Set the pointFormatId based on whether or not colors exist in the output.
    PointLayoutPtr layout = table.layout();
    if (layout->hasDim(Dimension::Id::Infrared))
        b->pointFormatId = 8;
    else if (layout->hasDim(Dimension::Id::Red) ||
             layout->hasDim(Dimension::Id::Green) ||
             layout->hasDim(Dimension::Id::Blue))
        b->pointFormatId = 7;
    else
        b->pointFormatId = 6;

    Dimension::IdList allDims = layout->dims();
    Dimension::IdList stdDims = las::pdrfDims(b->pointFormatId);
    std::sort(allDims.begin(), allDims.end());
    std::sort(stdDims.begin(), stdDims.end());
    // Fill in the extraDim lists with the difference between the total list and the
    // standard list.
    Dimension::IdList edIds;
    std::set_difference(allDims.begin(), allDims.end(), stdDims.begin(), stdDims.end(),
        std::inserter(edIds, edIds.end()));

    b->numExtraBytes = 0;
    for (Dimension::Id id : edIds)
    {
        std::string name (layout->dimName(id));
        Dimension::Type type (layout->dimType(id));
        DimType dimType = layout->findDimType(name);
        Dimension::Detail const* detail = layout->dimDetail(id);
        size_t size = detail->size();
        b->extraDims.emplace_back(name,
                                  type,
                                  id,
                                  size,
                                  las::baseCount(b->pointFormatId) + b->numExtraBytes,
                                  dimType.m_xform.m_scale.m_val,
                                  dimType.m_xform.m_offset.m_val);
        b->numExtraBytes += size;
    }
}

void CopcWriter::ready(PointTableRef table)
{
    // Deal with remote files
    if (Utils::isRemote(b->opts.filename))
    {

        // swap our filename for a tmp file
        std::string tmpname = Utils::tempFilename(b->opts.filename);
        remoteFilename = b->opts.filename;
        b->opts.filename = tmpname;
        isRemote = true;
    }

    MetadataNode forwardMetadata = table.privateMetadata("lasforward");
    handleHeaderForwards(forwardMetadata);
    handleForwardVlrs(forwardMetadata);

    if (b->opts.emitPipeline)
        handlePipelineVlr();
    handleUserVlrs(table.metadata());
}

void CopcWriter::handlePipelineVlr()
{
    // Write a VLR with the PDAL pipeline.
    std::ostringstream ostr;
    PipelineWriter::writePipeline(this, ostr);

    const std::string& json = ostr.str();
    std::vector<char> data(json.begin(), json.end());

    las::Evlr v(las::PdalUserId, las::PdalPipelineRecordId, "PDAL pipeline", std::move(data));
    b->vlrs.push_back(v);
}

void CopcWriter::handleUserVlrs(MetadataNode m)
{
    for (las::Evlr& v : b->opts.userVlrs)
    {
        v.fillData(m);
        b->vlrs.push_back(v);
    }
}

void CopcWriter::write(const PointViewPtr v)
{
    using namespace copcwriter;

    if (v->empty())
    {
        log()->get(LogLevel::Warning) << "writers.copc skipping empty point view.\n";
        return;
    }

    if (++b->viewCount > 1)
        log()->get(LogLevel::Warning) << "writers.copc does not support multiple views "
            "and will overwrite files for earlier views. Consider adding a merge filter.\n";

    BOX3D box;
    v->calculateBounds(box);

    Grid grid(box, v->size());

    CellManager mgr(v);

    for (PointRef p : *v)
    {
        double x = p.getFieldAs<double>(Dimension::Id::X);
        double y = p.getFieldAs<double>(Dimension::Id::Y);
        double z = p.getFieldAs<double>(Dimension::Id::Z);
        double t = p.getFieldAs<double>(Dimension::Id::GpsTime);
        double r = p.getFieldAs<double>(Dimension::Id::ReturnNumber);
        b->stats[(int)stats::Index::X].insert(x);
        b->stats[(int)stats::Index::Y].insert(y);
        b->stats[(int)stats::Index::Z].insert(z);
        b->stats[(int)stats::Index::GpsTime].insert(t);
        b->stats[(int)stats::Index::ReturnNumber].insert(r);

        VoxelKey key = grid.key(x, y, z);
        PointViewPtr& cell = mgr.get(key);
        cell->appendPoint(*v, p.pointId());
    }

    // New cells from reprocessing go on the reprocessing manager. They get merged
    // at the end.
    //ABELL - This should be threaded. These reprocessors should be able to run independently
    // since their data doesn't overlap spatially. Probably need a separate CellManager
    // for each that is merged under lock at completion.
    CellManager reprocessMgr(v);
    auto it = mgr.begin();
    while (it != mgr.end())
    {
        PointViewPtr& v = it->second;
        if (v->size() >= MaxPointsPerNode)
        {
            Reprocessor r(reprocessMgr, v, grid);
            r.run();
            // Remove the reprocessed cell from the manager.
            it = mgr.erase(it);
        }
        else
            it++;
    }
    mgr.merge(reprocessMgr);

    b->bounds = grid.processingBounds();
    b->trueBounds = grid.conformingBounds();
    if (!b->opts.aSrs.empty())
       b->srs = b->opts.aSrs;
    else
       b->srs = v->spatialReference();

    if (b->opts.enhancedSrsVlrs) {
        auto addVlr = [&](const std::string& userId, uint16_t recordId, const std::string& desc, const std::string& str)
        {
            if (!str.empty()) {
                std::vector<char> strBytes(str.begin(), str.end());
                strBytes.resize(strBytes.size() + 1, 0);
                las::Evlr v(userId, recordId, desc, std::move(strBytes));
                b->vlrs.push_back(v);
            }
        };
        addVlr(las::TransformUserId, las::LASFWkt2recordId, "PDAL WKT2 Record", b->srs.getWKT2());
        addVlr(las::PdalUserId, las::PdalProjJsonRecordId, "PDAL PROJJSON Record", b->srs.getPROJJSON());
    }

    // Set the input string into scaling.
    b->scaling.m_xXform.m_scale.set(b->opts.scaleX.val());
    b->scaling.m_yXform.m_scale.set(b->opts.scaleY.val());
    b->scaling.m_zXform.m_scale.set(b->opts.scaleZ.val());
    b->scaling.m_xXform.m_offset.set(b->opts.offsetX.val());
    b->scaling.m_yXform.m_offset.set(b->opts.offsetY.val());
    b->scaling.m_zXform.m_offset.set(b->opts.offsetZ.val());

    // Update scale and offset according to our computed value if "auto" is set.
    std::array<double, 3> t;
    grid.scale(t);
    if (b->scaling.m_xXform.m_scale.m_auto)
        b->scaling.m_xXform.m_scale = XForm::XFormComponent(t[0]);
    if (b->scaling.m_yXform.m_scale.m_auto)
        b->scaling.m_yXform.m_scale = XForm::XFormComponent(t[1]);
    if (b->scaling.m_zXform.m_scale.m_auto)
        b->scaling.m_zXform.m_scale = XForm::XFormComponent(t[2]);

    grid.offset(t);
    if (b->scaling.m_xXform.m_offset.m_auto)
        b->scaling.m_xXform.m_offset = XForm::XFormComponent(t[0]);
    if (b->scaling.m_yXform.m_offset.m_auto)
        b->scaling.m_yXform.m_offset = XForm::XFormComponent(t[1]);
    if (b->scaling.m_zXform.m_offset.m_auto)
        b->scaling.m_zXform.m_offset = XForm::XFormComponent(t[2]);

    BuPyramid bu(*b);
    bu.run(mgr);
}

void CopcWriter::done(PointTableRef table)
{


    if (isRemote)
    {
        arbiter::Arbiter a;
        a.put(remoteFilename, a.getBinary(b->opts.filename));

        // Clean up temporary
        FileUtils::deleteFile(b->opts.filename);
    }

}

} // namespace pdal
