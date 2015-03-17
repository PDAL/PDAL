.. _filters.pclblock:

===============================================================================
filters.pclblock
===============================================================================

The PCL Block filter allows users to specify a block of Point Cloud Library
(`PCL`_) operations on a PDAL ``PointView``, applying the necessary conversions
between PDAL and PCL point cloud representations.

This filter is under active development. The current implementation serves as a
proof of concept for linking PCL into PDAL and converting data. The PCL Block
filter creates a PCL ``Pipeline`` object and passes it a single argument, the JSON
file containing the PCL block definition. After filtering, the resulting indices
can be retrieved and used to create a new PDAL ``PointView`` containing only
those points that passed the filtering stages.

At this stage in its development, the PCL ``Pipeline`` does not allow complex
operations that may change the point type (e.g., ``PointXYZ`` to ``PointNormal``) or
alter points.  We will continue to look into use cases that are of value and
feasible, but for now are limited primarily to PCL functions that filter or
segment the point cloud, returning a list of indices of the filtered points
(e.g., ground or object, noise or signal). The main reason for this design
decision is that we want to avoid converting all ``PointView`` dimensions to the
PCL ``PointCloud``. In the case of an LAS reader, we may very well not want to
operate on fields such as return number, but we do not want to lose this
information post PCL filtering. The easy solution is to simply retain the index
between the ``PointView`` and ``PointCloud`` objects and update as necessary.

.. seealso::

    See :ref:`pcl_block_tutorial` for more on using the PCL Block including
    examples.

    See :ref:`pcl_json_specification` for complete details on the PCL Block JSON syntax
    and the filters available.

.. _`PCL`: http://www.pointclouds.org



Options
-------------------------------------------------------------------------------

filename
  JSON file to read [Required]



PCL Block Schema
-------------------------------------------------------------------------------

The PCL Block json object describes the filter chain to be constructed within
PCL. Here is an example:

.. code-block:: json

    {
        "pipeline":
        {
            "name": "PCL-Block-Name",
            "help": "This is an example pipeline with two filters.",
            "author": "mpg",
            "filters":
            [
                {
                    "name": "FilterOne",
                    "setFooParameter": "value"
                },
                {
                    "name": "FilterTwo",
                    "setBarParameter": false,
                    "setBounds":
                    {
                        "upper": 42,
                        "lower": 17
                    }
                }
            ]
        }
    }



Implemented Filters
-------------------------------------------------------------------------------

The list of PCL filters that are accessible through the PCL Block depends on PCL
itself. PDAL is rather dumb in this respect, merely converting the PDAL
``PointView`` to a PCL ``PointCloud`` object and passing the JSON filename. The
parsing of the JSON file and implementation of the PCL filters is entirely
embedded within the PCL ``Pipeline``.

A summary of the currently available filters is listed below. For full details
of the filters and their parameters, see the :ref:`pcl_json_specification`.

ApproximateProgressiveMorphologicalFilter
    faster (and potentially less accurate) version of the
    **ProgressiveMorphologicalFilter**

ConditionalRemoval
    filters the data to remove normals outside of a given Z range

GridMinimum
    assembles a local 2D grid over a given PointCloud, then downsamples the data

NormalEstimation
    computes the surfaces normals of the points in the input

PassThrough
    allows the user to set min/max bounds on one dimension of the data

ProgressiveMorphologicalFilter
    removes nonground points to produce a bare-earth point cloud

RadiusOutlierRemoval
    removes outliers if the number of neighbors in a certain search radius is
    smaller than a given K

StatisticalOutlierRemoval
    uses point neighborhood statistics to filter outlier data

VoxelGrid
    assembles a local 3D grid over a given PointCloud, then downsamples and
    filters the data



Adding a New Filter
-------------------------------------------------------------------------------

Adding a new PCL filter to the PCLBlock ecosystem is mostly a process of
judicious copying and pasting.

1. Add the filter function declaration of the form ``applyMyFilter`` to
   ``PCLPipeline.h``.

2. Add the implementation of ``applyMyFilter`` to ``PCLPipeline.hpp``.

3. Add a one-line description of the shiny new filter to this file,
   ``filters.pclblock.rst``.

4. Add a full description of the new filter to :ref:`pcl_spec.rst
   <pcl_json_specification>`, including example JSON, all parameters, and
   default settings.

5. Add a test to ``PCLBlockFilterTest.cpp``. Make sure each parameter is
   independently verified.
