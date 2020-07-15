/******************************************************************************
* Copyright (c) 2019, Helix Re Inc. nicolas@helix.re
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
*     * Neither the name of Helix Re Inc. nor the
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

#include <thread>

#include <pdal/Filter.hpp>

namespace pdal {

class FeatureType
{
public:
    static const int Linearity = 1;
    static const int Planarity = 2;
    static const int Scattering = 4;
    static const int Verticality = 8;
    static const int Omnivariance = 16;
    static const int Anisotropy = 32;
    static const int Eigenentropy = 64;
    static const int Sum = 128;
    static const int SurfaceVariation = 256;
    static const int DemantkeVerticality = 512;
    static const int Density = 1024;
};

class PDAL_DLL CovarianceFeaturesFilter: public Filter
{
public:
    CovarianceFeaturesFilter() : m_featureTypes(0) {}
    CovarianceFeaturesFilter &operator=(const CovarianceFeaturesFilter &) = delete;
    CovarianceFeaturesFilter(const CovarianceFeaturesFilter &) = delete;

    std::string getName() const;

private:
    enum class Mode
    {
        Raw,
        SQRT,
        Normalized
    };

    enum class FeatureSet
    {
        Dimensionality,
    };

    int m_knn;
    int m_threads;
    FeatureSet m_featureSet;
    std::map<std::string,Dimension::Id> m_extraDims;
    size_t m_stride;
    double m_radius;
    int m_minK;
    Arg* m_featureSetArg;
    StringList m_features;
    int m_featureTypes;
    Mode m_mode;
    Dimension::Id m_kopt, m_ropt;
    Arg* m_radiusArg;
    bool m_optimal;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void addArgs(ProgramArgs &args);
    virtual void filter(PointView &view);
    virtual void initialize();
    virtual void prepared(PointTableRef table);

    void setDimensionality(PointView &view, const PointId &id, const KD3Index &kid);

    friend std::istream& operator>>(std::istream& in,
        CovarianceFeaturesFilter::Mode& mode);
    friend std::ostream& operator<<(std::ostream& in,
        const CovarianceFeaturesFilter::Mode& mode);
    friend std::istream& operator>>(std::istream& in,
        CovarianceFeaturesFilter::FeatureSet& featureset);
    friend std::ostream& operator<<(std::ostream& in,
        const CovarianceFeaturesFilter::FeatureSet& featureset);
};
}

