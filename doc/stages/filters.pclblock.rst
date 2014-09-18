.. _filters.pclblock:

===============================================================================
filters.pclblock
===============================================================================

The PCL Block filter allows users to specify a block of Point Cloud Library
(PCL) operations on a PDAL PointBuffer, applying the necessary conversions
between PDAL and PCL point cloud representations.

This filter is under active development. The current implementation serves as a
proof of concept for linking PCL into PDAL and converting data. The PCL Block
creates a PCL Pipeline object (only available in development branch) and passes
a single argument, the JSON file containing the PCL block definition. After
filtering, the resulting indices can be retrieved and used to create a new PDAL
PointBuffer containing only those points that passed the filtering stages.

At this stage in its development, the PCL Pipeline does not allow complex
operations that may change the point type (e.g., PointXYZ to PointNormal) or
alter points.  We will continue to look into use cases that are of value and
feasible, but for now are limited primarily to PCL functions that filter or
segment the point cloud, returning a list of indices of the filtered points
(e.g., ground or object, noise or signal). The main reason for this design
decision is that we want to avoid converting all PointBuffer dimensions to the
PCL PointCloud. In the case of an LAS reader, we may very well not want to
operate on fields such as return number, but we do not want to lose this
information post PCL filtering. The easy solution is to simply retain the index
between the PointBuffer and PointCloud objects and update as necessary.

.. seealso::

    See :ref:`pcl_block_tutorial` for more on using the PCL Block including
    examples.

Options
-------------------------------------------------------------------------------

filename
  JSON file to read [Required]

PCL Block Schema
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline": {
        "name": "PCL-Block-Name",
        "filters": [{
            "name": "FilterName",
            "setParameter": "value"
          }, {
            "name": "AnotherFilterName",
            "setParameter": "value"
        }]
      }
    }

Implemented Filters
-------------------------------------------------------------------------------

The list of PCL filters that are accessible through the PCL Block depends on
PCL itself. PDAL is rather dumb in this respect, merely converting the PDAL
PointBuffer to a PCL PointCloud object and passing the JSON filename. The
parsing of the JSON file and implementation of the PCL filters is entirely
embedded within the PCL Pipeline. The list of filters and their paremeters as
of this writing includes:

PassThrough
...............................................................................

setFilterFieldName
  TBD

setFilterLimits (min, max)
  TBD - default = +/- FLT_MAX

StatisticalOutlierRemoval
...............................................................................

setMeanK
  default = 2

setStddevMulThresh
  default = 0.0

RadiusOutlierRemoval
...............................................................................

setMinNeighborsInRadius
  default = 2

setRadiusSearch
  default = 1.0

GridMinimum
...............................................................................

setLeafSize (x, y, z)
  default = 1.0

.. _ProgressiveMorphologicalFilter:

ProgressiveMorphologicalFilter
...............................................................................

setMaxWindowSize
  The maximum window size for applying morphological operations. Objects larger
  than the specified window size will be retained, while objects that are
  smaller than the window size will be replaced by the minimum z value for the
  window. [Default: **33.0**]

setSlope
  The terrain slope is used to compute the elevation difference threshold for a
  given iteration (i.e., window size). Slope is given as the ratio of rise over
  run, not degrees. Therefore, the default of 1 corresponds to a slope of 45
  degrees. [Default: **1.0**]

setMaxDistance
  The maximum elevation difference threshold. For scenes with buildings, this
  should be the height of the lowest building. For more rural settings, this is
  commonly the largest elevation difference in the study area. [Default:
  **2.5**]

setInitialDistance
  The initial elevation difference threshold. Points within this threshold of
  the estimated surface are classified as ground returns. [Default: **0.15**]

setCellSize
  The cell size. [Default: **1.0**]

setBase
  The base of an exponential function/initial cell size in computing
  progressive window size at each iteration. [Default: **2.0**]

setExponential
  Whether or not to increase the window size exponentially, thus reducing the
  number of iterations. [Default: **true**]

setNegative
  Whether to return non-ground (true) or ground (false) measurements. [Default:
  **false**]
