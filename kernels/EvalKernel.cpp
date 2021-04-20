/******************************************************************************
 * Copyright (c) 2020, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "EvalKernel.hpp"

#include <pdal/KDIndex.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

using namespace Dimension;

static StaticPluginInfo const s_info{
    "kernels.eval", "Eval Kernel", "http://pdal.io/kernels/kernels.eval.html"};

CREATE_STATIC_KERNEL(EvalKernel, s_info)

std::string EvalKernel::getName() const
{
    return s_info.name;
}

void EvalKernel::addSwitches(ProgramArgs& args)
{
    args.add("predicted", "Point cloud filename containing predicted labels",
             m_predictedFile)
        .setPositional();
    args.add("truth", "Point cloud filename containing truth labels",
             m_truthFile)
        .setPositional();
    args.add("labels",
             "Comma-separated list of classification labels to evaluate",
             m_labelStrList);
    args.add("prediction_dim", "Dimension containing predicted labels",
             m_predictedDimName, "Classification");
    args.add("truth_dim", "Dimension containing truth labels", m_truthDimName,
             "Classification");
}

void EvalKernel::validateSwitches(ProgramArgs& args)
{
    if (m_labelStrList.empty())
        throw pdal_error(
            "Must specify comma-separated list of labels to evaluate.");
}

PointViewPtr EvalKernel::loadSet(const std::string& filename,
                                 PointTableRef table)
{
    Stage& reader = makeReader(filename, "");
    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    assert(viewSet.size() == 1);
    return *viewSet.begin();
}

int EvalKernel::execute()
{
    ColumnPointTable predictedTable;
    PointViewPtr predictedView = loadSet(m_predictedFile, predictedTable);
    PointLayoutPtr predictedLayout(predictedTable.layout());
    m_predictedDimId = predictedLayout->findDim(m_predictedDimName);
    if (m_predictedDimId == Dimension::Id::Unknown)
        throw pdal_error("Predicted dimension '" + m_predictedDimName +
                         "' does not exist.");

    ColumnPointTable truthTable;
    PointViewPtr truthView = loadSet(m_truthFile, truthTable);
    PointLayoutPtr truthLayout(truthTable.layout());
    m_truthDimId = truthLayout->findDim(m_truthDimName);
    if (m_truthDimId == Dimension::Id::Unknown)
        throw pdal_error("Truth dimension '" + m_truthDimName +
                         "' does not exist.");

    assert(predictedView->size() == truthView->size());

    KD3Index& kdi = truthView->build3dIndex();

    int dim = m_labelStrList.size();

    std::vector<int> labelList;
    for (auto const& label : m_labelStrList)
        labelList.push_back(std::stoi(label));
    std::sort(labelList.begin(), labelList.end());

    LabelStats ls(dim);

    for (PointRef p : *predictedView)
    {
        // It would be nice if we could expect that the points are aligned in
        // both the predicted and truth views, but this often cannot be
        // guaranteed, so rather than using the same PointId, we search for the
        // nearest neighbor.
        PointId qid = kdi.neighbor(p);
        PointRef q = truthView->point(qid);

        // TODO (chambbj): We should perhaps look at the distance to the
        // nearest point and reject or otherwise report distances greater than
        // 0.0, indicating some sort of mismatch between files.

        int pc = p.getFieldAs<int>(m_predictedDimId);
        int qc = q.getFieldAs<int>(m_truthDimId);

        auto it = std::find(labelList.begin(), labelList.end(), qc);
        size_t qci;
        if (it != labelList.end())
            qci = std::distance(labelList.begin(), it);
        else
            qci = dim;

        it = std::find(labelList.begin(), labelList.end(), pc);
        size_t pci;
        if (it != labelList.end())
            pci = std::distance(labelList.begin(), it);
        else
            pci = dim;

        ls.insert(qci, pci);
    }

    MetadataNode root;
    for (int label = 0; label < dim; ++label)
    {
        MetadataNode elem = root.addList("labels");
        elem.add("label", m_labelStrList[label]);
        elem.add("support", ls.getSupport(label));
        elem.add("intersection_over_union", ls.getIntersectionOverUnion(label),
                 "", 3);
        elem.add("f1_score", ls.getF1Score(label), "", 3);
        elem.add("sensitivity", ls.getSensitivity(label), "", 3);
        elem.add("specificity", ls.getSpecificity(label), "", 3);
        elem.add("precision", ls.getPrecision(label), "", 3);
        elem.add("accuracy", ls.getAccuracy(label), "", 3);
    }
    root.add("mean_intersection_over_union", ls.getMeanIntersectionOverUnion(),
             "", 3);
    root.add("predicted_file", m_predictedFile);
    root.add("truth_file", m_truthFile);
    root.add("overall_accuracy", ls.getOverallAccuracy(), "", 3);
    root.add("f1_score", ls.getF1Score(), "", 3);
    root.add("confusion_matrix", ls.prettyPrintConfusionMatrix());
    root.add("pdal_version", Config::fullVersionString());

    Utils::toJSON(root, std::cout);

    return 0;
}

} // namespace pdal
