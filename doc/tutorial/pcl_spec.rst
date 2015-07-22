.. _pcl_json_specification:

============================
Draft PCL JSON Specification
============================

:Author: Bradley J. Chambers (RadiantBlue Technologies, Inc.)
:Revision: 0.1
:Date: 28 February 2014
:Copyright: Copyright (c) 2014, RadiantBlue Technologies, Inc. This work is licensed under a Creative Commons Attribution 3.0 United States License.

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

A PCL JSON object represents a processing pipeline.

A complete PCL JSON data structure is always an object (in JSON terms). In PCL
JSON, an object consists of a collection of name/value pairs -- also called
members. For each member, the name is always a string. Member values are either
a string, number, object, array or one of the literals: "true", "false", and
"null". An array consists of elements where each element is a value as
described above.



Examples
--------

A very simple PCL JSON pipeline:

.. code-block:: json

    {
        "pipeline":
        {
            "name": "My cool pipeline",
            "filters":
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
        }
    }

A more complex pipeline, containing two filters:

.. code-block:: json

    {
        "pipeline":
        {
            "name": "CombinedExample",
            "help": "Apply passthrough filter followed by statistical outlier removal",
            "version": 1.0,
            "author": "Bradley J Chambers",
            "filters":
            [
                {
                    "name": "PassThrough",
                    "help": "filter z values to the range [410,440]",
                    "setFilterFieldName": "z",
                    "setFilterLimits":
                    {
                        "min": 410.0,
                        "max": 440.0
                    }
                },
                {
                    "name": "StatisticalOutlierRemoval",
                    "help": "apply outlier removal",
                    "setMeanK": 8,
                    "setStddevMulThresh": 0.2
                }
            ]
        }
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

PCL JSON always consists of a single object. This object (referred to as the
PCL JSON object below) represents a processing pipeline.

* The PCL JSON object may have any number of members (name/value pairs).

* The PCL JSON object must have a "pipeline" object.



Pipeline Objects
----------------

* A pipeline may have a member with the name "name" whose value is a string.

* A pipeline may have a member with the name "help" whose value is a string.

* A pipeline may have a member with the name "version" whose value is a number.

* A pipeline must have a member with the name "filters" whose value is an array
  of filters.



Filters
.......

A pipeline must have a "filters" member whose value is an array of zero or more
filters.

A filter is any of the PCL filters that has been exposed through the PCL
pipeline class.

In the following descriptions, all parameters are optional unless otherwise
noted.

Any JSON keys not recognized by the spec are blissfully ignored.



ApproximateProgressiveMorphologicalFilter (APMF)
````````````````````````````````````````````````

:pcl:`ApproximateProgressiveMorphologicalFilter <pcl::ApproximateProgressiveMorphologicalFilter>`

This filter removes nonground points to produce a bare-earth point cloud. It is
similar to the ProgressiveMorphologicalFilter, but is potentially faster (and
correspondingly less accurate).

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_approximate_progressive_morphological_filter.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
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
        }
    }

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



ConditionalRemoval
``````````````````

:pcl:`ConditionalRemoval <pcl::ConditionalRemoval>`

This filter removes normals outside of a given Z range.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_conditional_removal.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
            [
                {
                    "name": "ConditionalRemoval",
                    "normalZ":
                    {
                        "min": 0,
                        "max": 0.95
                    }
                }
            ]
        }
    }

**Parameters**

normalZ: object `{"min": float, "max": float}`
  Set the numerical limits for filtering points based on the z component of
  their normal. [default: `{"min": 0.0, "max": FLT_MAX}`]



GridMinimum
```````````

:pcl:`GridMinimum <pcl::GridMinimum>`

This filter assembles a local 2D grid over a given PointCloud, then downsamples
the data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_grid_minimum.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
            [
                {
                    "name": "GridMinimum",
                    "setResolution": 2.0
                }
            ]
        }
    }

**Parameters**

setResolution: float
  Set the grid resolution. [default: 1.0]



NormalEstimation
````````````````

:pcl:`NormalEstimation <pcl::NormalEstimation>`

**Description**

This filter computes the surfaces normals of the points in the input.

PCL details: http://docs.pointclouds.org/1.7.1/classpcl_1_1_normal_estimation.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
            [
                {
                    "name": "NormalEstimation",
                    "setRadiusSearch": 2
                }
            ]
        }
    }

**Parameters**

setKSearch: float
    Set the number of k nearest neighbors to use for the feature estimation.
    [default: 0.0]

setRadiusSearch: float
    Set the sphere radius that is to be used for determining the nearest
    neighbors used for the feature estimation. [default: 1.0]



PassThrough
```````````

:pcl:`PassThrough <pcl::PassThrough>`

**Description**

This filter allows the user to set min/max bounds on one dimension of the data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_pass_through_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
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
        }
    }

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
````````````````````````````````````

:pcl:`ProgressiveMorphologicalFilter <pcl::ProgressiveMorphologialFilter>`

**Description**

This filter removes nonground points to produce a bare-earth point cloud.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_progressive_morphological_filter.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
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
        }
    }
   
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
````````````````````

:pcl:`RadiusOutlierRemoval <pcl::RadiusOutlierRemoval>`

**Description**

This filter removes outliers if the number of neighbors in a certain search
radius is smaller than a given K.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_radius_outlier_removal_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
            [
                {
                    "name": "RadiusOutlierRemoval",
                    "setMinNeighborsInRadius": 8,
                    "setRadiusSearch": 1.0
                }
            ]
        }
    }

**Parameters**

setMinNeighborsInRadius: int
  Set the number of neighbors that need to be present in order to be
  classified as an inlier. [default: 2]

setRadiusSearch: float
  Set te radius of the sphere that will determine which points are neighbors.
  [default: 1.0]



StatisticalOutlierRemoval
`````````````````````````

:pcl:`StatisticalOutlierRemoval <pcl::StatisticalOutlierRemoval>`

**Description**

This filter uses point neighborhood statistics to filter outlier data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_statistical_outlier_removal_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
            [
                {
                    "name": "StatisticalOutlierRemoval",
                    "setMeanK": 8,
                    "setStddevMulThresh": 1.17
                }
            ]
        }
    }

**Parameters**

setMeanK: int
  Set the number of nearest neighbors to use for mean distance estimation.
  [default: 2]

setStddevMulThresh: float
  Set the standard deviation multiplier for the distance threshold
  calculation. [default: 0.0]



VoxelGrid
`````````

:pcl:`VoxelGrid <pcl::VoxelGrid>`

This filter assembles a local 3D grid over a given PointCloud, then downsamples
and filters the data.

PCL details: http://docs.pointclouds.org/trunk/classpcl_1_1_voxel_grid_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html

Example:

.. code-block:: json

    {
        "pipeline":
        {
            "filters":
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
        }
    }

**Parameters**

setLeafSize: object `{"x": float, "y": float, "z": float}`
  Set the voxel grid leaf size. [default: `{"x": 1.0, "y": 1.0, "z": 1.0}`]
