/******************************************************************************
* Copyright (c) 2014, Hobu Inc.
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

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <pdal/DimDetail.hpp>
#include <pdal/DimType.hpp>
#include <pdal/Metadata.hpp>

namespace pdal
{

class  PointLayout
{
public:
    /**
      Default constructor.
    */
    PDAL_DLL PointLayout();
    PDAL_DLL virtual ~PointLayout() {}

    /**
      Mark a layout as finalized.  Dimensions can't be added to a finalized
      PointLayout.
    */
    PDAL_DLL void finalize();

    /**
      Determine if the PointLayout is finalized.

      \return  Whether the PointLayout is finalized.
    */
    PDAL_DLL bool finalized() const
        { return m_finalized; }

    /**
      Register a vector of dimensions.

      \param ids  Vector of IDs to register.
    */
    PDAL_DLL void registerDims(std::vector<Dimension::Id> ids);

    /**
      Register a list of dimensions.

      \param id  Pointer to list of IDs to register.  The last ID in the list
        must have the value Unknown.
    */
    PDAL_DLL void registerDims(Dimension::Id *id);

    /**
      Register use of a standard dimension (declare that a point will contain
      data for this dimension).  Use the default type for the dimension.

      \param id  ID of dimension to be registered.
    */
    PDAL_DLL void registerDim(Dimension::Id id);

    /**
      Register use of a standard dimension (declare that a point will contain
      data for this dimension) if it hasn't already been registered with a
      "larger" type.  It the dimension already exists with a larger type, this
      does nothing.

      \param id  ID of dimension to be registered.
      \param type  Minimum type to assign to the dimension.
    */
    PDAL_DLL void registerDim(Dimension::Id id, Dimension::Type type);

    /**
      Assign a non-existing (proprietary) dimension with the given name and
      type.  No check is made to see if the dimension exists as a standard
      (non-propietary) dimension.  If the dimension has already been
      assigned as a proprietary dimension, update the type but use the
      existing Id.  If the dimension has already been assigned with a
      larger type, this does nothing.

      \param name  Name of the proprietary dimension to add.
      \param type  Minimum type to assign to the dimension.
      \return  ID of the new or existing dimension, or Unknown on failure.
    */
    PDAL_DLL Dimension::Id assignDim(const std::string& name,
        Dimension::Type type);

    /**
      Register a dimension if one already exists with the given name using the
      provided type.  If the dimension doesn't already exist, create it.

      \param name  Name of the dimension to register or assign.
      \param type  Requested type of the dimension.  Dimension will at least
        accomodate values of this type.
      \return  ID of dimension registered or assigned.
    */
    PDAL_DLL Dimension::Id registerOrAssignDim(const std::string name,
        Dimension::Type type);

    /**
      Get a list of DimType objects that define the layout.

      \return  A list of DimType objects.
    */
    PDAL_DLL DimTypeList dimTypes() const;

    /**
      Get a DimType structure for a named dimension.

      \param name  Name of the dimension
      \return  A DimType associated with the named dimension.  Returns a
        DimType with an Unknown ID if the dimension isn't part of the layout.
    */
    PDAL_DLL DimType findDimType(const std::string& name) const;

    /**
      Get the ID of a dimension (standard or proprietary) given its name.

      \param name  Name of the dimension.
      \return  ID of the dimension or Unknown.
    */
    PDAL_DLL Dimension::Id findDim(const std::string& name) const;

    /**
      Get the ID of a proprietary dimension given its name.

      \param name  Name of the dimension.
      \return  ID of the dimension or Unknown.
    */
    PDAL_DLL Dimension::Id findProprietaryDim(const std::string& name) const;

    /**
      Get the name of a dimension give its ID.  A dimension may have more
      than one name.  The first one associated with the ID is returned.

      \param id  ID of the dimension.
      \return  A name associated with the dimension, or a NULL string.
    */
    PDAL_DLL std::string dimName(Dimension::Id id) const;

    /**
      Determine if the PointLayout uses the dimension with the given ID.

      \param id  ID of the dimension to check.
      \return \c true if the layout uses the dimension, \c false otherwise.
    */
    PDAL_DLL bool hasDim(Dimension::Id id) const;

    /**
      Get a reference to vector of the IDs of currently used dimensions.

      \return  Vector of IDs of dimensions that are part of the layout.
    */
    PDAL_DLL const Dimension::IdList& dims() const;

    /**
      Get the type of a dimension.

      \param id  ID of the dimension.
      \return  Type of the dimension.
    */
    PDAL_DLL Dimension::Type dimType(Dimension::Id id) const;

    /**
      Get the current size in bytes of the dimension.

      \param id  ID of the dimension.
      \return  Size of the dimension in bytes.
    */
    PDAL_DLL size_t dimSize(Dimension::Id id) const;

    /**
      Get the offset of the dimension in the layout.

      \param id  ID of the dimension.
      \return  Offset of the dimension in bytes.
    */
    PDAL_DLL size_t dimOffset(Dimension::Id id) const;

    /**
      Get number of bytes that make up a point.  Returns the sum of the dimSize
      for all dimensions in the layout.

      \return  Size of a point in bytes.
    */
    PDAL_DLL size_t pointSize() const;

    /**
      Get a pointer to a dimension's detail information.

      \param id  ID of the dimension.
      \return  A pointer a dimension's detail.
    */
    PDAL_DLL const Dimension::Detail* dimDetail(Dimension::Id id) const
    {
        return &(m_detail[Utils::toNative(id)]);
    }

    /**
      Change from offset to order.  Only call during finalize.
    */
    void orderDimensions()
    {
        int order = 0;
        for (Dimension::Id id : m_used)
            m_detail[Utils::toNative(id)].setOrder(order++);
    }

    /**
        Convert the point layout to a metadata format.

        \return  A metadata node that contains the layout information.
    */
    PDAL_DLL MetadataNode toMetadata() const;

    /**
        Set the names of dimensions to which the layout is restricted.
    */
    PDAL_DLL void setAllowedDims(StringList dimNames);

private:
    PDAL_DLL virtual bool update(Dimension::Detail dd, const std::string& name);

    Dimension::Type resolveType( Dimension::Type t1,
        Dimension::Type t2);

protected:
    std::vector<Dimension::Detail> m_detail;
    Dimension::IdList m_used;
    std::map<std::string, Dimension::Id> m_propIds;
    int m_nextFree;
    std::size_t m_pointSize;
    bool m_finalized;
    StringList m_allowedDimNames;
};

typedef PointLayout* PointLayoutPtr;

} // namespace pdal

