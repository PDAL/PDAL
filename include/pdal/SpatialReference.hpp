/******************************************************************************
 * Copyright (c) 2009, Howard Butler
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/pdal_internal.hpp>

namespace pdal
{

class PDAL_DLL BOX3D;

/// A SpatialReference defines a model of the earth that is used to describe
/// the location of points.
/// A SpatialReference is part of input data and is automatically loaded
/// into PDAL by readers, or it's provided explicitly by a user through an
/// option.  All points in a point view share a common spatial reference.  When
/// a stage finishes processing point view, the point view takes on the
/// spatial reference of that stage, if it had one.
/// A point table tracks the spatial references of the views currently being
/// processed by a stage.  If a point table being processed by a stage has
/// more than one spatial reference, PointTable::spatialReference() will
/// return an empty spatial reference and PointTable::spatialReferenceUnique()
/// will return false.
class PDAL_DLL SpatialReference
{
public:
    enum WKTModeFlag
    {
        eHorizontalOnly = 1,
        eCompoundOK = 2
    };

    /// Default constructor.
    SpatialReference()
    {}

    // calls setFromUserInput() with the given string
    SpatialReference(const std::string& userInput);

    bool equals(const SpatialReference& other) const;
    bool operator==(const SpatialReference& other) const;
    bool operator!=(const SpatialReference& other) const;
    bool operator<(const SpatialReference& other) const
        { return m_wkt < other.m_wkt; }

    // Returns true iff the object doesn't contain a valid srs.
    // (this is a cleaner way of saying "getWKT() == "")
    bool empty() const;


    // Returns true of OSR can validate the SRS
    bool valid() const;

    /// Returns the OGC WKT describing Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine
    /// the WKT.  If GDAL is not linked, no WKT is returned.
    /// \param mode_flag May be eHorizontalOnly indicating the WKT will not
    /// include vertical coordinate system info (the default), or
    /// eCompoundOK indicating the the returned WKT may be a compound
    /// coordinate system if there is vertical coordinate system info
    /// available.
    std::string getWKT(WKTModeFlag mode_flag = eHorizontalOnly) const;
    std::string getWKT(WKTModeFlag mode_flag, bool pretty) const;
    std::string getRawWKT() const
        { return m_wkt; }

    /// Sets the SRS using GDAL's OGC WKT. If GDAL is not linked, this
    /// operation has no effect.
    /// \param v - a string containing the WKT string.
    void setWKT(std::string const& v)
        { m_wkt = v; }

    /// Sets the SRS using GDAL's SetFromUserInput function. If GDAL is
    /// not linked, this operation has no effect.
    /// \param v - a string containing the definition (filename, proj4,
    ///    wkt, etc).
    void setFromUserInput(std::string const& v);

    /// Returns the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses
    /// that.  Note that GDAL's operations are much more mature and
    /// support more coordinate systems and descriptions.
    std::string getProj4() const;

    std::string getHorizontal() const;
    std::string getVertical() const;

    /// Sets the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses
    /// that.  Note that GDAL's operations are much more mature and
    /// support more coordinate systems and descriptions.
    /// \param v - a string containing the Proj.4 string.
    void setProj4(std::string const& v);

    void dump() const;

    bool isGeographic() const;
    int computeUTMZone(const BOX3D& box) const;
    const std::string& getName() const;
    static int calculateZone(double lon, double lat);

private:
    std::string m_wkt;
    friend PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
        const SpatialReference& srs);
    friend PDAL_DLL std::istream& operator>>(std::istream& istr,
        SpatialReference& srs);
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
    const SpatialReference& srs);
PDAL_DLL std::istream& operator>>(std::istream& istr, SpatialReference& srs);

} // namespace pdal

