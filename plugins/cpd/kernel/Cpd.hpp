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

#include <cpd/nonrigid_lowrank.hpp>

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
    CpdKernel() {};
    void addSwitches();
    void validateSwitches();
    PointViewPtr readFile(const std::string& filename, PointTableRef table,
        arma::mat& mat);
    cpd::Registration::ResultPtr chipThenRegister(
        const cpd::NonrigidLowrank& reg, const arma::mat& X, const arma::mat& Y,
        const PointViewPtr& bufX, PointTableRef table);

    std::string m_filex;
    std::string m_filey;
    std::string m_output;
    float m_tolerance;
    int m_max_it;
    float m_outliers;
    bool m_fgt;
    float m_epsilon;
    float m_beta;
    float m_lambda;
    arma::uword m_numeig;
    BOX3D m_bounds;
    bool m_auto_z_exaggeration;
    float m_auto_z_exaggeration_ratio;
    bool m_chipped;
    int m_chip_capacity;
    float m_chip_buffer;
    float m_sigma2;
};

} // namespace pdal
