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

std::string Script::getLogicalMask(mxArray* array, LogPtr log)
{

    std::string output("");
    mxClassID ml_id = mxGetClassID(array);
    if (ml_id != mxSTRUCT_CLASS)
        throw pdal::pdal_error("Selected array must be a Matlab struct array!");

    for (int n = 0; n < mxGetNumberOfFields(array); ++n)
    {

        mxArray* f = mxGetFieldByNumber(array, 0, n);

        if (mxIsLogical(f))
        {
            const char* name = mxGetFieldNameByNumber(array, n);
            output = std::string(name);
        }
    }
    return output;
}

std::string Script::getSRSWKT(mxArray* array, LogPtr log)
{
    std::string output("");

    mxArray* metadata = mxGetField(array, 0, "metadata");
    if (metadata)
    {
        mxArray* s = mxGetField(metadata, 0, "wkt");
        mxClassID mt = mxGetClassID(s);
        if (mt == mxCHAR_CLASS)
        {
            size_t len = mxGetN(s);
            std::string data;
            data.resize(len+1);
            int ret = mxGetString(s, const_cast<char*>(data.data()), len+1);
            output = std::string(data);
        }
    }

    return output;
}


PointLayoutPtr Script::getStructLayout(mxArray* array, LogPtr log)
{

    PointLayoutPtr layout = PointLayoutPtr(new PointLayout());
    std::vector<mxArray*> arrays;

    mxClassID ml_id = mxGetClassID(array);
    if (ml_id != mxSTRUCT_CLASS)
        throw pdal::pdal_error("Selected array must be a Matlab struct array!");


    int numFields = mxGetNumberOfFields(array);

    if (!numFields)
        throw pdal::pdal_error("Selected struct array must have fields!");

    // This needs to skip mxClassID types that
    // don't map to PDAL
    for (int i=0; i < numFields; ++i)
    {
        const char* fieldName = mxGetFieldNameByNumber(array, i);
        mxArray* f = mxGetFieldByNumber(array, 0, i);
        mxClassID mt = mxGetClassID(f);
        Dimension::Type pt = Script::getPDALDataType(mt);
        if (mt == mxDOUBLE_CLASS ||
            mt == mxSINGLE_CLASS ||
            mt == mxINT8_CLASS   ||
            mt == mxUINT8_CLASS  ||
            mt == mxINT16_CLASS  ||
            mt == mxUINT16_CLASS ||
            mt == mxINT32_CLASS  ||
            mt == mxUINT32_CLASS ||
            mt == mxINT64_CLASS  ||
            mt == mxUINT64_CLASS )
        {
            layout->registerOrAssignDim(fieldName, pt);
        }
    }

    layout->finalize();
    return layout;
}


void Script::getMatlabStruct(mxArray* array, PointViewPtr view, const Dimension::IdList& indims, std::string& pdalargs, MetadataNode pdalmetadata, LogPtr log)
{
    std::vector<mxArray*> arrays;
    std::vector<std::string> dimNames;

    mxClassID ml_id = mxGetClassID(array);
    if (ml_id != mxSTRUCT_CLASS)
        throw pdal::pdal_error("Selected array must be a Matlab struct array!");

    mxArray* metadata = mxGetField(array, 0, "metadata");
    if (metadata)
    {

        mxArray* args = mxGetField(metadata, 0, "pdalargs");
        if (args)
        {
            mxClassID mt = mxGetClassID(args);
            if (mt == mxCHAR_CLASS)
            {
                size_t len = mxGetN(args);
                std::string data;
                data.resize(len+1);
                int ret = mxGetString(args, const_cast<char*>(data.data()),
                    len+1);
                pdalargs = std::string(data);
            }
        }
    }
    Dimension::IdList  dims;
    if (!indims.size())
        dims = view->dims();
    else
        dims = indims;

    for (auto d: dims)
    {
        std::string dimName = view->dimName(d);

        mxArray* f = mxGetField(array, 0, dimName.c_str());
        if (!f)
        {
            std::ostringstream oss;
            oss << "No dimension named '" << dimName << "' exists on struct array.";
            throw pdal::pdal_error(oss.str());
        }

        mwSize numElements = mxGetNumberOfElements(f);
        if (numElements != view->size())
        {
            std::ostringstream oss;
            oss << "Array shape is not the same as the PDAL PointView. ";
            oss << "Matlab array is has '" << numElements << "' elements. ";
            oss << "PointView has '" << view->size() << "' elements.";
            throw pdal::pdal_error(oss.str());
        }

        mxClassID mt = mxGetClassID(f);
        Dimension::Type pt = Script::getPDALDataType(mt);

        char* p = (char*)mxGetData(f);
        if (!p)
        {
            std::ostringstream oss;
            oss << "Unable to fetch Matlab pointer to array for dimension '" << dimName << "'";
            throw pdal::pdal_error(oss.str());
        }

        for (PointId i = 0; i < view->size(); ++i)
        {
            view->setField(d, pt, i, p);
            p += view->dimSize(d);
        }
    }

    // TODO: add back metadata
}

mxArray* Script::setMatlabStruct(PointViewPtr view, const Dimension::IdList& indims, const std::string& pdalargs, MetadataNode mdataNode,  LogPtr log)
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
    //
    // Going into a 1x1 struct
    mdims[0] = 1;
    mdims[1] = 1;


    std::vector<const char*> fieldNames;

    // Metadata
    //
    std::vector<std::string> metadataDimNames;
    dimNames.push_back("metadata");

    metadataDimNames.push_back("json");
    metadataDimNames.push_back("wkt");
    metadataDimNames.push_back("horizontal");
    metadataDimNames.push_back("vertical");
    metadataDimNames.push_back("pdalargs");


    std::transform(metadataDimNames.begin(), metadataDimNames.end(), std::back_inserter(fieldNames), convert);
    mxArray* metadata = mxCreateStructArray(2, mdims, metadataDimNames.size(), fieldNames.data());

    std::stringstream strm;
    MetadataNode root = mdataNode.clone("metadata");
    pdal::Utils::toJSON(root, strm);

    mxArray* json = mxCreateString(strm.str().c_str());
    mxSetField(metadata, 0, "json", json);

    SpatialReference srs = view->spatialReference();

    std::string wkt_s = srs.getWKT();
    mxArray* wkt = mxCreateString(wkt_s.c_str());
    mxSetField(metadata, 0, "wkt", wkt);

    std::string horizontal_s = srs.getHorizontal();
    mxArray* horizontal = mxCreateString(horizontal_s.c_str());
    mxSetField(metadata, 0, "horizontal", horizontal);

    std::string vertical_s = srs.getVertical();
    mxArray* vertical = mxCreateString(vertical_s.c_str());
    mxSetField(metadata, 0, "vertical", vertical);

    mxArray* pargs = mxCreateString(pdalargs.c_str());
    mxSetField(metadata, 0, "pdalargs", pargs);

    fieldNames.clear();
    std::transform(dimNames.begin(), dimNames.end(), std::back_inserter(fieldNames), convert);
    mxArray* s = mxCreateStructArray(2, mdims, dimNames.size(), fieldNames.data());
    for (size_t j = 0; j < dims.size(); ++j)
    {
        std::string dimName = dimNames[j];
        mxArray* array = arrays[j];
        mxSetField(s, 0, dimName.c_str(), array);
    }

    mxSetField(s, 0, "metadata", metadata);



    return s;

}

Dimension::Type Script::getPDALDataType(mxClassID t)
{
    using namespace Dimension;

    switch (t)
    {
    case mxSINGLE_CLASS:
        return Type::Float;
    case mxDOUBLE_CLASS:
        return Type::Double;
    case mxINT8_CLASS:
        return Type::Signed8;
    case mxINT16_CLASS:
        return Type::Signed16;
    case mxINT32_CLASS:
        return Type::Signed32;
    case mxINT64_CLASS:
        return Type::Signed64;
    case mxUINT8_CLASS:
        return Type::Unsigned8;
    case mxUINT16_CLASS:
        return Type::Unsigned16;
    case mxUINT32_CLASS:
        return Type::Unsigned32;
    case mxUINT64_CLASS:
        return Type::Unsigned64;
    default:
        return Type::None;
    }
    assert(0);

    return Type::None;
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
        return mxINT8_CLASS;
    case Type::Signed16:
        return mxINT16_CLASS;
    case Type::Signed32:
        return mxINT32_CLASS;
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

