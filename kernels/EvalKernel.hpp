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

#pragma once

#include <pdal/Kernel.hpp>
#include <pdal/Stage.hpp>

#include <Eigen/Dense>

namespace pdal
{
using namespace Eigen;

class PDAL_DLL LabelStats
{
public:
    LabelStats(int dim) : m_dim(dim)
    {
        m_confusionMatrix = MatrixXi::Zero(m_dim + 1, m_dim + 1);
    }

    point_count_t getSupport(int label)
    {
        return m_confusionMatrix.row(label).sum();
    }

    point_count_t getTruePositives(int label)
    {
        return m_confusionMatrix(label, label);
    }

    point_count_t getFalsePositives(int label)
    {
        return m_confusionMatrix.topRows(m_dim).col(label).sum() -
               getTruePositives(label);
    }

    point_count_t getFalseNegatives(int label)
    {
        return m_confusionMatrix.row(label).sum() - getTruePositives(label);
    }

    point_count_t getTrueNegatives(int label)
    {
        return m_confusionMatrix.topRows(m_dim).sum() -
               getTruePositives(label) - getFalsePositives(label) -
               getFalseNegatives(label);
    }

    double getIntersectionOverUnion(int label)
    {
        if (getTrueNegatives(label) ==
            (point_count_t)m_confusionMatrix.topRows(m_dim).sum())
            return 0.0;
        else
            return (double)getTruePositives(label) /
                   ((double)getTruePositives(label) +
                    (double)getFalsePositives(label) +
                    (double)getFalseNegatives(label));
    }

    double getMeanIntersectionOverUnion()
    {
        double miou = 0.0;
        for (int label = 0; label < m_dim; ++label)
            miou += getIntersectionOverUnion(label);
        return miou / (double)m_dim;
    }

    double getF1Score(int label)
    {
        return 2.0 * getIntersectionOverUnion(label) /
               (1.0 + getIntersectionOverUnion(label));
    }

    double getF1Score()
    {
        return 2.0 * getMeanIntersectionOverUnion() /
               (1.0 + getMeanIntersectionOverUnion());
    }

    double getSensitivity(int label)
    {
        if ((getTruePositives(label) + getFalseNegatives(label)) == 0)
            return 0.0;
        else
            return (double)getTruePositives(label) /
                   ((double)getTruePositives(label) +
                    (double)getFalseNegatives(label));
    }

    double getSpecificity(int label)
    {
        if ((getFalsePositives(label) + getTrueNegatives(label)) == 0)
            return 0.0;
        else
            return (double)getTrueNegatives(label) /
                   ((double)getFalsePositives(label) +
                    (double)getTrueNegatives(label));
    }

    double getPrecision(int label)
    {
        if ((getTruePositives(label) + getFalsePositives(label)) == 0)
            return 0.0;
        else
            return (double)getTruePositives(label) /
                   ((double)getTruePositives(label) +
                    (double)getFalsePositives(label));
    }

    double getAccuracy(int label)
    {
        return ((double)getTruePositives(label) +
                (double)getTrueNegatives(label)) /
               (double)m_confusionMatrix.topRows(m_dim).sum();
    }

    double getOverallAccuracy()
    {
        return (double)m_confusionMatrix.topRows(m_dim).trace() /
               (double)m_confusionMatrix.topRows(m_dim).sum();
    }

    std::string prettyPrintConfusionMatrix()
    {
        std::stringstream ss;
        Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols,
                                  ",", ",", "[", "]", "[", "]");
        ss << m_confusionMatrix.format(CSVFormat);
        return ss.str();
    }

    void insert(int qci, int pci)
    {
        m_confusionMatrix(qci, pci)++;
    }

private:
    MatrixXi m_confusionMatrix;
    int m_dim;
};

class PDAL_DLL EvalKernel : public Kernel
{
public:
    std::string getName() const;
    int execute();

private:
    void addSwitches(ProgramArgs& args);
    void validateSwitches(ProgramArgs& args);
    PointViewPtr loadSet(const std::string& filename, PointTableRef table);

    std::string m_predictedFile;
    std::string m_truthFile;
    StringList m_labelStrList;

    std::string m_predictedDimName;
    std::string m_truthDimName;
    Dimension::Id m_predictedDimId;
    Dimension::Id m_truthDimId;
};

} // namespace pdal
