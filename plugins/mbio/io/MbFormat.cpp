/******************************************************************************
* Copyright (c) 2017, Howard Butler (hobu@hob.co)
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

#include <pdal/util/Utils.hpp>

#include "MbFormat.hpp"

typedef std::pair<std::string, int> MbFormatInfo;

namespace pdal
{

namespace
{

std::vector<MbFormatInfo> formats =
{
    { "MBF_SBSIOMRG", 11 },
    { "MBF_SBSIOCEN", 12 },
    { "MBF_SBSIOLSI", 13 },
    { "MBF_SBURICEN", 14 },
    { "MBF_SBURIVAX", 15 },
    { "MBF_SBSIOSWB", 16 },
    { "MBF_SBIFREMR", 17 },
    { "MBF_HSATLRAW", 21 },
    { "MBF_HSLDEDMB", 22 },
    { "MBF_HSURICEN", 23 },
    { "MBF_HSLDEOIH", 24 },
    { "MBF_HSURIVAX", 25 },
    { "MBF_HSUNKNWN", 26 },
    { "MBF_SB2000RW", 31 },
    { "MBF_SB2000SB", 32 },
    { "MBF_SB2000SS", 33 },
    { "MBF_SB2100RW", 41 },
    { "MBF_SB2100B1", 42 },
    { "MBF_SB2100B2", 43 },
    { "MBF_EMOLDRAW", 51 },
    { "MBF_EM12IFRM", 53 },
    { "MBF_EM12DARW", 54 },
    { "MBF_EM300RAW", 56 },
    { "MBF_EM300MBA", 57 },
    { "MBF_EM710RAW", 58 },
    { "MBF_EM710MBA", 59 },
    { "MBF_MR1PRHIG", 61 },
    { "MBF_MR1ALDEO", 62 },
    { "MBF_MR1BLDEO", 63 },
    { "MBF_MR1PRVR2", 64 },
    { "MBF_MBLDEOIH", 71 },
    { "MBF_MBNETCDF", 75 },
    { "MBF_MBNCDFXT", 76 },
    { "MBF_CBAT9001", 81 },
    { "MBF_CBAT8101", 82 },
    { "MBF_HYPC8101", 83 },
    { "MBF_XTFR8101", 84 },
    { "MBF_RESONS8K", 85 },
    { "MBF_SBATPROC", 86 },
    { "MBF_RESON7KR", 88 },
    { "MBF_RESON7KP", 89 },
    { "MBF_BCHRTUNB", 91 },
    { "MBF_ELMK2UNB", 92 },
    { "MBF_BCHRXUNB", 93 },
    { "MBF_L3XSERAW", 94 },
    { "MBF_HSMDARAW", 101 },
    { "MBF_HSMDLDIH", 102 },
    { "MBF_DSL120PF", 111 },
    { "MBF_DSL120SF", 112 },
    { "MBF_GSFGENMB", 121 },
    { "MBF_MSTIFFSS", 131 },
    { "MBF_EDGJSTAR", 132 },
    { "MBF_EDGJSTR2", 133 },
    { "MBF_OICGEODA", 141 },
    { "MBF_OICMBARI", 142 },
    { "MBF_OMGHDCSJ", 151 },
    { "MBF_SEGYSEGY", 160 },
    { "MBF_MGD77DAT", 161 },
    { "MBF_ASCIIXYZ", 162 },
    { "MBF_ASCIIYXZ", 163 },
    { "MBF_HYDROB93", 164 },
    { "MBF_MBARIROV", 165 },
    { "MBF_MBPRONAV", 166 },
    { "MBF_NVNETCDF", 167 },
    { "MBF_ASCIIXYT", 168 },
    { "MBF_ASCIIYXT", 169 },
    { "MBF_MBARROV2", 170 },
    { "MBF_HS10JAMS", 171 },
    { "MBF_HIR2RNAV", 172 },
    { "MBF_MGD77TXT", 173 },
    { "MBF_MGD77TAB", 174 },
    { "MBF_SAMESURF", 181 },
    { "MBF_HSDS2RAW", 182 },
    { "MBF_HSDS2LAM", 183 },
    { "MBF_IMAGE83P", 191 },
    { "MBF_IMAGEMBA", 192 },
    { "MBF_HYSWEEP1", 201 },
    { "MBF_XTFB1624", 211 },
    { "MBF_SWPLSSXI", 221 },
    { "MBF_SWPLSSXP", 222 },
    { "MBF_3DDEPTHP", 231 },
    { "MBF_WASSPENL", 241 },
    { "MBF_PHOTGRAM", 251 }
};

} // unnamed namespace

MbFormat::MbFormat() : m_value(0)
{}


MbFormat::operator int () const
{
    return m_value;
}


std::istream& operator>>(std::istream& in, MbFormat& f)
{
    std::string s;

    f.m_value = 0;
    in >> s;
    try
    {
        int val = stoi(s);
        for (MbFormatInfo& fi : formats)
            if (val == fi.second)
            {
                f.m_value = val;
                break;
            }
    }
    catch (std::exception)
    {
        s = Utils::toupper(s);
        for (MbFormatInfo& fi : formats)
            if (s == fi.first)
            {
                f.m_value = fi.second;
                break;
            }
    }
    if (f.m_value == 0)
        in.setstate(std::ios_base::failbit);
    return in;
}


std::ostream& operator<<(std::ostream& out, const MbFormat& f)
{
    std::string sval("MY_SYS_NONE");

    for (MbFormatInfo& fi : formats)
    {
        if (f.m_value == fi.second)
        {
            sval = fi.first;
            break;
        }
    }
    out << sval;
    return out;
}

} // namespace pdal
