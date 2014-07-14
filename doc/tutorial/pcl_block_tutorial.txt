.. _pcl_block_tutorial:

===============================================================================
Filtering data with PCL
===============================================================================

Introduction
------------------------------------------------------------------------------

PDAL is both a C++ library and a collection of command-line utilities for data
processing operations. While the PDAL library addresses point cloud
exploitation and filtering, this takes a back seat to its primary objective of
being a data translation library, helping developers to navigate the a wide
variety of point cloud formats. `PCL`_ is another C++ library that is focused
on developing a rich set of point cloud processing routines, with less of a
focus on formats and data translation. Acknowledging this, the PCL Block filter
was developed to serve as a bridge between the two libraries, enabling rapid
development of point cloud processing pipelines.

.. seealso::
    
    See :ref:`filters.pclblock` for details on PDAL's PCL Block filter.

.. _`PCL`: http://www.pointclouds.org

.. contents:: Contents
   :depth: 3
   :backlinks: none

Quick Start
------------------------------------------------------------------------------

.. note::

    Instructions for getting started with PDAL using Vagrant and VirtualBox
    can be found in the :ref:`vagrant` document.

PDAL's Vagrant configuration now includes the PCL Block and all necessary
dependencies. Assuming you've got Vagrant all set up, once you checkout the
PDAL source code, just do the following:

::

    $ cd pdal
    $ vagrant up
    $ vagrant ssh

The ``vagrant up`` command will take a considerable amount of time, but once
its completed you have a fully functional VM with both PCL and PDAL installed.
Try entering

::

    $ pdal pipeline --version

at the command line. You should see output similar to the following:

::
    
    ------------------------------------------------------------------------------------------
    pdal pipeline (PDAL 0.9.9 (2c6aa8) with GeoTIFF 1.4.0 GDAL 1.9.2 LASzip 2.2.0 System )
    ------------------------------------------------------------------------------------------

PDAL Pipeline kernel
------------------------------------------------------------------------------

.. note::

    A full description of the PDAL pipeline concept is beyond the scope of this
    tutorial but the :ref:`pipeline`, :ref:`pipeline_command`, and
    :ref:`reading` documents contain detailed examples and background
    information.

The :ref:`filters.pclblock` is implemented as a PDAL filter stage and as such
is easily accessed via the PDAL pipeline. It accepts a single, required option
- the name of the `JSON`_ file describing the PCL Block.

A sample pipeline XML which reads/writes LAS and has a single PCL Block filter
is shown below.

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
      <Writer type="drivers.las.writer">
          <Option name="filename">
              filtered_cloud.las
          </Option>
          <Filter type="filters.pclblock">
              <Option name="filename">
                  block.json
              </Option>
              <Reader type="drivers.las.reader">
                  <Option name="filename">
                      input_cloud.las
                  </Option>
              </Reader>
          </Filter>
      </Writer>
  </Pipeline>

And is run from the command line thusly.

::

    $ pdal pipeline -i /path/to/xml

This simple pipeline reads the input LAS (``input_cloud.las``), passes it
through the PCL Block (``block.json``), and writes the output LAS
(``filtered_cloud.las``).

You can also try running the PassThrough PCL Block pipeline similar to our
example above by typing

::

    $ pdal pipeline -i pdal/test/data/filters/pcl_passthrough.xml -v4

which should produce

::

    Requested to read 106 points
    Requested to write 106 points
    0
    Processing /home/vagrant/pdal/test/data/filters/pcl/passthrough.json

    --------------------------------------------------------------------------------
    NAME:   PassThroughExample ()
    HELP:
    AUTHOR:
    --------------------------------------------------------------------------------
    106 points copied

       Step 1) PassThrough

          Field name: z
          Limits: 410.000000, 440.000000

    76(drivers.las.writer DEBUG: 3): Wrote 81 points to the LAS file
    .100

PDAL PCL kernel
------------------------------------------------------------------------------

For users that would like to bypass the creation (and subsequent modification)
of the pipeline XML for every file they wish to process, there is another
option--the ``pdal pcl`` command.

::

    $ pdal pcl -i /path/to/input/las -p /path/to/pcl/block/json -o /path/to/output/las

This is functionally equivalent to the original pipeline, but does not afford
the flexibility of constructing the pipeline (i.e., none the other PDAL filters
are accessible).

The same can be accomplished with the ``pdal pcl`` command. The basic syntax
for the command is

::

    $ pdal pcl -i <input cloud> -p <PCL Block JSON> -o <output cloud>

where the JSON file specified with ``-p`` is the same file that would be
embedded in the pipeline XML file. This can be useful when the pipeline
does not change frequently, but the input/output filenames do.

::

    $ pdal pcl -i test/data/autzen-point-format-3.las -p test/data/filters/pcl/passthrough.json -o test/temp/foo.las -v4

which should produce

::

    Requested to read 106 points
    Requested to write 106 points
    0
    Processing /home/vagrant/pdal/test/data/filters/pcl/passthrough.json

    --------------------------------------------------------------------------------
    NAME:   PassThroughExample ()
    HELP:
    AUTHOR:
    --------------------------------------------------------------------------------
    106 points copied

       Step 1) PassThrough

          Field name: z
          Limits: 410.000000, 440.000000

    76(drivers.las.writer DEBUG: 3): Wrote 81 points to the LAS file
    .100

Examples
------------------------------------------------------------------------------

Simple point cloud cropping
..............................................................................

The power of the PCL Block is really exposed through the JSON description. In
this example, we apply a single PCL filter to the PointBuffer. The
`PassThrough`_ filter removes points that lie outside a given range for the
specified dimension. Here, we are asking PCL to crop the input point cloud,
returning only those points with z values in the range 100 to 200.

.. code-block:: json

  {
    "pipeline": {
      "name": "PassThroughExample",
      "filters": [{
          "name": "PassThrough",
          "setFilterFieldName": "z",
          "setFilterLimits": {
            "min": 100,
            "max": 200
          }
      }]
    }
  }

Point cloud cropping with outlier removal
..............................................................................

Building on the previous example, we can string together multiple PCL filtering
stages, such as the `StatisticalOutlierRemoval`_ filter. Note that the name
field identifies the PCL filter by its class name, and furthermore that as of
now only a handful of the PCL filtering options are accessible through the PCL
Block. Similarly, select parameters of these classes can be set by specifying
their public member functions by name.

.. code-block:: json

  {
    "pipeline": {
      "name": "PassThroughAndOutlierRemovalExample",
      "filters": [{
          "name": "PassThrough",
          "setFilterFieldName": "z",
          "setFilterLimits": {
            "min": 100,
            "max": 200
          }
        }, {
          "name": "StatisticalOutlierRemoval",
          "setMeanK": 8,
          "setStddevMulThresh": 2.0
      }]
    }
  }

Ground return filtering
..............................................................................

The Progressive Morphological Filter (PMF) is an openly published approach to
identifying ground vs. non-ground returns in point cloud data. An
implementation of PMF is included with PCL and accessible through the PDAL's
PCL Block filter.

A complete description of the algorithm can be found in the article `"A
Progressive Morphological Filter for Removing Nonground Measurements from
Airborne LIDAR Data" <http://users.cis.fiu.edu/~chens/PDF/TGRS.pdf>`_ by K.
Zhang, S.  Chen, D. Whitman, M. Shyu, J. Yan, and C. Zhang.

To run the PMF with default settings, the PCL Block JSON is simply:

.. code-block:: json

  {
    "pipeline": {
      "name": "ProgressiveMorphologicalFilterDefaultExample",
      "filters": [{
          "name": "ProgressiveMorphologicalFilter"
      }]
    }
  }

Additional parameters can be set by advanced users:

.. code-block:: json

  {
    "pipeline": {
      "name": "ProgressiveMorphologicalFilterAdvancedExample",
      "filters": [{
          "name": "ProgressiveMorphologicalFilter",
          "setCellSize": 1.0,
          "setMaxWindowSize": 20,
          "setSlope": 1.0,
          "setInitialDistance": 0.5,
          "setMaxDistance": 3.0,
          "setExponential": true
      }]
    }
  }

See :ref:`here <ProgressiveMorphologicalFilter>` for a more detailed
explanation of the PMF parameters.

.. _`JSON`: http://www.json.org/
.. _`PassThrough`: http://pointclouds.org/documentation/tutorials/passthrough.php
.. _`StatisticalOutlierRemoval`: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
