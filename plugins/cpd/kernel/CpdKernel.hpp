/******************************************************************************
 * Copyright (c) 2014, Pete Gadomski (pete.gadomski@gmail.com)
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

#include <cpd/matrix.hpp>
#include <pdal/Kernel.hpp>
#include <pdal/pdal_export.hpp>

namespace pdal
{

class PDAL_DLL CpdKernel : public Kernel
{
public:
    static void *create();
    static int32_t destroy(void *);
    std::string getName() const;
    int execute();

private:
    CpdKernel() : Kernel() {};
    virtual void addSwitches(ProgramArgs& args);
    cpd::Matrix readFile(const std::string& filename);

    std::string m_method;
    std::string m_fixed;
    std::string m_moving;
    std::string m_output;
    BOX3D m_bounds;

    // cpd::Transform
    size_t m_max_iterations;
    bool m_normalize;
    double m_outliers;
    double m_sigma2;
    double m_tolerance;

    // cpd::Rigid
    bool m_reflections;
    bool m_scale;

    // cpd::Nonrigid
    double m_beta;
    double m_lambda;
};

} // namespace pdal
