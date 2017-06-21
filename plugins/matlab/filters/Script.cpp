/******************************************************************************
* Copyright (c) 2017, Howard Butler (howard@hobu.co)
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

#include "Script.hpp"


namespace pdal
{
namespace mlang
{


std::ostream& operator << (std::ostream& os, Script const& script)
{
    os << "source=[" << script.m_source.size() << " bytes], ";
    os << std::endl;

    return os;
}


mxArray* Script::getMatlabStruct(PointViewPtr view, const Dimension::IdList& indims)
{

    std::vector<mxArray*> arrays;
    std::vector<std::string> dimNames;

    Dimension::IdList  dims;
    if (!indims.size())
        dims = view->dims();
    else
        dims = indims;

    mwSize mdims[2] = {0, 0};
    mdims[0] = view->size();
    mdims[1] = 1;

    std::stringstream dimensionsString;
    bool first(true);
    for (auto d: dims)
    {
        std::string dimName = Dimension::name(d);
        mxArray* array = mxCreateNumericArray(  2,
                                                mdims,
                                                (mxClassID)mlang::Script::getMatlabDataType(view->dimType(d)),
                                                (mxComplexity)0);
        arrays.push_back(array);
        dimNames.push_back(dimName);
    }

    // Matlab is column-major
    for (size_t j = 0; j < dims.size(); ++j)
    {
        Dimension::Id id = dims[j];
        Dimension::Type t = view->dimType(id);
        mxArray* array = arrays[j];
        char* pointsPtr = (char*)mxGetData(array);

        for (point_count_t i = 0; i < view->size(); ++i)
        {
            view->getField(pointsPtr, id, t, i);
            pointsPtr += view->dimSize(id);
        }
    }

    // Push the dimension names into a char**
    auto convert = [](std::string& s)
    {
        return s.c_str();
    };

    std::vector<const char*> fieldNames;
    std::transform(dimNames.begin(), dimNames.end(), std::back_inserter(fieldNames), convert);


    // Going into a 1x1 struct
    mdims[0] = 1;
    mdims[1] = 1;

    mxArray* s = mxCreateStructArray(2, mdims, dimNames.size(), fieldNames.data());
    for (size_t j = 0; j < dims.size(); ++j)
    {
        std::string dimName = dimNames[j];
        mxArray* array = arrays[j];
        mxSetField(s, 0, dimName.c_str(), array);
    }


    return s;

}

int Script::getMatlabDataType(Dimension::Type t)
{
    using namespace Dimension;

    switch (t)
    {
    case Type::Float:
        return mxSINGLE_CLASS;
    case Type::Double:
        return mxDOUBLE_CLASS;
    case Type::Signed8:
        return mxDOUBLE_CLASS;
    case Type::Signed16:
        return mxINT16_CLASS;
    case Type::Signed32:
        return mxINT64_CLASS;
    case Type::Signed64:
        return mxINT64_CLASS;
    case Type::Unsigned8:
        return mxUINT8_CLASS;
    case Type::Unsigned16:
        return mxUINT16_CLASS;
    case Type::Unsigned32:
        return mxUINT32_CLASS;
    case Type::Unsigned64:
        return mxUINT64_CLASS;
    default:
        return -1;
    }
    assert(0);

    return -1;
}

} //namespace plang
} //namespace pdal

