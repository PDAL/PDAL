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
class PDAL_DLL MetadataNode;

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
    /**
      Constructor.  Create an empty SRS.
    */
    SpatialReference()
    {}

    /**
      Construct a spatial reference from well-known text.

      \param wkt  Well-known text from which to construct SRS.
    */
    SpatialReference(const std::string& wkt);

    /**
      Construct a spatial reference from well-known text.

      \param wkt  Well-known text from which to construct SRS.
    */
    SpatialReference(const char *wkt);


    /**
      Determine if this spatial reference is the same as another.

      \param other  SRS to compare with this one.
      \return  \c true if the SRSs match
    */
    bool equals(const SpatialReference& other) const;

    /**
      See \ref equals.
    */
    bool operator==(const SpatialReference& other) const;

    /**
      Determine if this spatial reference is different from another.

      \param other  SRS to compare with this one.
      \return \c true if the SRSs don't match.
    */
    bool operator!=(const SpatialReference& other) const;

    /**
       Determine if the well-known text representation of this SRS is
       lexographically less than that of another.

       \param other  SRS to compare with this one.
       \return  \c true if this SRS is lexographically less.
    */
    bool operator<(const SpatialReference& other) const
        { return m_wkt < other.m_wkt; }

    /**
      Returns true iff the object doesn't contain a valid srs.

      \return  Whether the SRS is empty.
    */
    bool empty() const;


    // Returns true of OSR can validate the SRS
    bool valid() const;

    std::string getWKT() const;
    std::string getWKT1() const;

    /// Parse the string starting at position `pos` as a spatial reference.
    /// \param s    String to parse.
    /// \param pos  Position to start parsing string.
    void parse(const std::string& s, std::string::size_type& pos);

    /// Sets the SRS from a string representation.  WKT is saved as
    /// provided.
    /// \param v - a string containing the definition (filename, proj4,
    ///    wkt, etc).
    void set(std::string v);

    /// Returns the Proj.4 string describing the Spatial Reference System.
    /// If GDAL is linked, it uses GDAL's operations and methods to determine
    /// the Proj.4 string -- otherwise, if libgeotiff is linked, it uses
    /// that.  Note that GDAL's operations are much more mature and
    /// support more coordinate systems and descriptions.
    std::string getProj4() const;

    std::string getHorizontal() const;
    std::string getHorizontalUnits() const;
    std::string getVertical() const;
    std::string getVerticalUnits() const;

    /// Attempt to identify an EPSG code from the spatial reference.  Returns
    /// an empty string if a code could not be identified.
    std::string identifyHorizontalEPSG() const;
    std::string identifyVerticalEPSG() const;

    /// Returns UTM zone **if** the coordinate system is actually UTM.
    /// The method simply forwards down to OSRGetUTMZone
    int getUTMZone() const;

    void dump() const;
    MetadataNode toMetadata() const;

    bool isGeographic() const;
    bool isGeocentric() const;
    bool isProjected() const;

    std::vector<int> getAxisOrdering() const;

    int computeUTMZone(const BOX3D& box) const;

    const std::string& getName() const;
    static int calculateZone(double lon, double lat);
    static SpatialReference wgs84FromZone(int zone);
    static bool isWKT(const std::string& wkt);
    static std::string prettyWkt(const std::string& wkt);

private:
    std::string m_wkt;
    mutable std::string m_horizontalWkt;
    friend PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
        const SpatialReference& srs);
    friend PDAL_DLL std::istream& operator>>(std::istream& istr,
        SpatialReference& srs);
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr,
    const SpatialReference& srs);
PDAL_DLL std::istream& operator>>(std::istream& istr, SpatialReference& srs);

} // namespace pdal

