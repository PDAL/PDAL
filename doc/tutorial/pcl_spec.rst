.. _pcl_json_specification:

============================
Draft PCL JSON Specification
============================

:Author: Bradley J. Chambers (RadiantBlue Technologies, Inc.)
:Revision: 0.2
:Date: 13 December 2016
:Copyright: Copyright (c) 2014-2016, RadiantBlue Technologies, Inc. This work is licensed under a Creative Commons Attribution 3.0 United States License.

The PCL JSON specification is a point cloud processing pipeline interchange
format based on JavaScript Object Notation (JSON), drawing inspiration from
both GeoJSON and TopoJSON.

.. sectnum::
.. contents::
   :depth: 4
   :backlinks: none



============
Introduction
============

A PCL JSON array represents a processing pipeline.

A complete PCL JSON data structure is always an array of objects (in JSON
terms). In PCL JSON, an object consists of a collection of name/value pairs --
also called members. For each member, the name is always a string. Member values
are either a string, number, object, array or one of the literals: "true",
"false", and "null". An array consists of elements where each element is a value
as described above.



Examples
--------

A very simple PCL JSON pipeline:

.. code-block:: json

    [
        {
            "name": "VoxelGrid",
            "setLeafSize":
            {
                "x": 1.0,
                "y": 1.0,
                "z": 1.0
            }
        }
    ]

A more complex pipeline, containing two filters:

.. code-block:: json

    [
        {
            "name": "PassThrough",
            "setFilterFieldName": "z",
            "setFilterLimits":
            {
                "min": 410.0,
                "max": 440.0
            }
        },
        {
            "name": "StatisticalOutlierRemoval",
            "setMeanK": 8,
            "setStddevMulThresh": 0.2
        }
    ]

A PCL pipeline is embedded within a PDAL pipeline as the "methods" option to :ref:`filters.pclblock` as shown below:

.. code-block:: json

    {
        "pipeline": [
            "input.las",
            {
                "type": "filters.pclblock",
                "methods": [
                    {
                        "name": "VoxelGrid",
                        "setLeafSize":
                        {
                            "x": 1.0,
                            "y": 1.0,
                            "z": 1.0
                        }
                    }
                ]
            },
            "output.las"
        ]
    }



Definitions
-----------

* JavaScript Object Notation (JSON), and the terms object, name, value, array,
  and number, are defined in IETF RTC 4627, at
  http://www.ietf.org/rfc/rfc4627.txt.

* The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD",
  "SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this documention are to
  be interpreted as described in IETF RFC 2119, at
  http://www.ietf.org/rfc/rfc2119.txt.



================
PCL JSON Objects
================

PCL JSON always consists of a single array of PCL JSON objects. This array
(referred to as the PCL JSON array below) represents a processing pipeline.

* The PCL JSON array may have any number of PCL JSON objects.

* A PCL JSON object shall have a "name" member that identifies a supported PCL
  filter (as documented below).

* A PCL JSON object may have any number of members (name/value pairs).



Filters
--------------------------------------------------------------------------------

A filter is any of the PCL filters that has been exposed through the PCL
pipeline class.

In the following descriptions, all parameters are optional unless otherwise
noted.

Any JSON keys not recognized by the spec are blissfully ignored.



ApproximateProgressiveMorphologicalFilter (APMF)
................................................................................

.. seealso:

    :ref:`filters.ground` utilizes ApproximateProgressiveMorphologicalFilter in
    the context of a PDAL filter

This filter removes nonground points to produce a bare-earth point cloud. It is
similar to the ProgressiveMorphologicalFilter, but is potentially faster (and
correspondingly less accurate).

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_approximate_progressive_morphological_filter.html

Example:

.. code-block:: json

    [
        {
            "name": "ApproximateProgressiveMorphologicalFilter",
            "setMaxWindowSize": 65,
            "setSlope": 0.7,
            "setMaxDistance": 10,
            "setInitialDistance": 0.3,
            "setCellSize": 1,
            "setBase": 2,
            "setExponential": false,
            "setNegative": false
        }
    ]

**Parameters**

setMaxWindowSize: int
  Set the maximum window size to be used for filtering ground returns.
  [float, default: 33]

setSlope: float
  Set the slope value to be used in computing the height threshold. [default:
  1.0]

setMaxDistance: float
  Set the maximum height above the parameterized ground surface to be
  considered a ground return. [default: 2.5]

setInitialDistance: float
  Set the initial height above the parameterized ground surface to be
  considered a ground return. [default: 0.15]

setCellSize: float
  Set the cell size. [default: 1.0]

setBase: float
  Set the base to be used in computing progressive window sizes. [default: 2.0]

setExponential: bool
  Set flag indicating whether or not to exponentially grow window sizes.
  [default: true]

setNegative: bool
  If set to false, include all points indicated by the indices (treat as
  "inliers"). If true, include the "outlier" points. [default: false]



GridMinimum
................................................................................

This filter assembles a local 2D grid over a given PointCloud, then downsamples
the data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_grid_minimum.html

Example:

.. code-block:: json

    [
        {
            "name": "GridMinimum",
            "setResolution": 2.0
        }
    ]

**Parameters**

setResolution: float
  Set the grid resolution. [default: 1.0]



PassThrough
................................................................................

**Description**

This filter allows the user to set min/max bounds on one dimension of the data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_pass_through_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    [
        {
            "name": "PassThrough",
            "setFilterFieldName": "z",
            "setFilterLimits":
            {
                "min": 3850100,
                "max": 3850200
            }
        }
    ]

**Parameters**

setFilterFieldName: string (required)
  Provide the name of the field to be used for filtering data.

.. note::

    Only the `X`, `Y`, `Z`, `R`, `G`, `B`, and `Intensity` dimensions are
    supported.

.. note::

    Although PDAL capitalizes the dimension names ("Z", "Intensity"), PCL
    requires the names be given in lower case ("z", "intensity").

setFilterLimits: object `{"min": float, "max": float}`
  Set the numerical limits for the field for filtering data.
  [default: `{"min": -FLT_MAX, "max": +FLT_MAX}`]



ProgressiveMorphologicalFilter (PMF)
................................................................................


.. seealso::

    :ref:`filters.pmf` implements support for this operation as a
    PDAL filter

**Description**

This filter removes nonground points to produce a bare-earth point cloud.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_progressive_morphological_filter.html

Example:

.. code-block:: json

    [
        {
            "name": "ProgressiveMorphologicalFilter",
            "setMaxWindowSize": 65,
            "setSlope": 0.7,
            "setMaxDistance": 10,
            "setInitialDistance": 0.3,
            "setCellSize": 1,
            "setBase": 2,
            "setExponential": false,
            "setNegative": true
        }
    ]

**Parameters**

setMaxWindowSize: int
  Set the maximum window size to be used for filtering ground returns.
  [default: 33]

setSlope: float
  Set the slope value to be used in computing the height threshold. [default:
  1]

setMaxDistance: float
  Set the maximum height above the parameterized ground surface to be
  considered a ground return. [default: 2.5]

setInitialdistance: float
  Set the initial height above the parameterized ground surface to be
  considered a ground return. [default: 0.15]

setCellSize: float
  Set the cell size. [default: 1]

setBase: float
  Set the base to be used in computing progressive window sizes. [default: 2]

setExponential: bool
  Set flag indicating whether or not to exponentially grow window sizes.
  [default: true]

setNegative: bool
  If set to false, include all points indicated by the indices (treat as
  "inliers"). If true, include the "outlier" points. [default: false]



RadiusOutlierRemoval
................................................................................

.. seealso::

    :ref:`filters.outlier` implements support for this operation
    as a PDAL filter


**Description**

This filter removes outliers if the number of neighbors in a certain search
radius is smaller than a given K.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_radius_outlier_removal_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    [
        {
            "name": "RadiusOutlierRemoval",
            "setMinNeighborsInRadius": 8,
            "setRadiusSearch": 1.0
        }
    ]

**Parameters**

setMinNeighborsInRadius: int
  Set the number of neighbors that need to be present in order to be
  classified as an inlier. [default: 2]

setRadiusSearch: float
  Set te radius of the sphere that will determine which points are neighbors.
  [default: 1.0]



StatisticalOutlierRemoval
................................................................................

.. seealso::

    :ref:`filters.outlier` implements support for this
    operation as a PDAL filter

**Description**

This filter uses point neighborhood statistics to filter outlier data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_statistical_outlier_removal_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    [
        {
            "name": "StatisticalOutlierRemoval",
            "setMeanK": 8,
            "setStddevMulThresh": 1.17
        }
    ]

**Parameters**

setMeanK: int
  Set the number of nearest neighbors to use for mean distance estimation.
  [default: 2]

setStddevMulThresh: float
  Set the standard deviation multiplier for the distance threshold
  calculation. [default: 0.0]



VoxelGrid
................................................................................

.. seealso::

    :ref:`filters.voxelgrid` implements support for this operation as a
    PDAL filter


This filter assembles a local 3D grid over a given PointCloud, then downsamples
and filters the data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_voxel_grid_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    [
        {
            "name": "VoxelGrid",
            "setLeafSize":
            {
                "x": 1.0,
                "y": 1.0,
                "z": 1.0
            }
        }
    ]

**Parameters**

setLeafSize: object `{"x": float, "y": float, "z": float}`
  Set the voxel grid leaf size. [default: `{"x": 1.0, "y": 1.0, "z": 1.0}`]
