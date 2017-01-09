.. _pcl_block_tutorial:

===============================================================================
Filtering data with PCL
===============================================================================

Introduction
------------------------------------------------------------------------------

PDAL is both a C++ library and a collection of command-line utilities for data
processing operations. While the PDAL library addresses point cloud exploitation
and filtering, this takes a back seat to its primary objective of being a data
translation library, helping developers to navigate the a wide variety of point
cloud formats. `PCL`_ is another C++ library that is focused on developing a
rich set of point cloud processing routines, with less of a focus on formats and
data translation. Acknowledging this, the PCL Block filter was developed to
serve as a bridge between the two libraries, enabling rapid development of point
cloud processing pipelines.

.. seealso::

    See :ref:`filters.pclblock` for details on PDAL's PCL Block filter.

.. _`PCL`: http://www.pointclouds.org

.. contents:: Contents
   :depth: 3
   :backlinks: none



Quick Start
------------------------------------------------------------------------------

The :ref:`quickstart` document describes how to use PDAL with Docker, which
includes built-in PCL support. After you have worked through that document, you
should be able to run any PDAL PCL operations.


PDAL Pipeline kernel
------------------------------------------------------------------------------

.. note::

    A full description of the PDAL pipeline concept is beyond the scope of this
    tutorial but the :ref:`pipeline`, :ref:`pipeline_command`, and
    :ref:`reading` documents contain detailed examples and background
    information.

The :ref:`filters.pclblock` is implemented as a PDAL filter stage and as such is
easily accessed via the PDAL pipeline. It accepts a single, required option -
the name of the `JSON`_ file describing the PCL Block.

A sample pipeline JSON which reads/writes LAS and has a single PCL Block filter
is shown below.

.. code-block:: json

  {
    "pipeline":[
      "autzen-point-format-3.las",
      {
        "type":"filters.pclblock",
        "filename":"passthrough.json"
      },
      "foo.las"
    ]
  }

And is run from the command line thusly.

::

    $ pdal pipeline passthrough.json

This simple pipeline reads the input LAS (``autzen-point-format-3.las``), passes
it through the PCL Block (``passthrough.json``), and writes the output LAS
(``foo.las``).

When run, it should produce output similar to this:

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

    76(writers.las DEBUG: 3): Wrote 81 points to the LAS file
    .100



PDAL PCL kernel
------------------------------------------------------------------------------

For users that would like to bypass the creation (and subsequent modification)
of the pipeline JSON for every file they wish to process, there is another
option: the ``pdal pcl`` command.

::

    $ pdal pcl -i /path/to/input/las -p /path/to/pcl/block/json -o /path/to/output/las

This is functionally equivalent to the original `pdal pipeline` command, but
does not afford the flexibility of constructing the pipeline (i.e., none the
other PDAL filters are accessible).

The same can be accomplished with the ``pdal pcl`` command. The basic syntax for
the command is

::

    $ pdal pcl -i <input cloud> -p <PCL Block JSON> -o <output cloud>

where the JSON file specified with ``-p`` is the same file that would be
embedded in the pipeline JSON file. This can be useful when the pipeline does not
change frequently, but the input/output filenames do.

For example, the above `pdal pipeline` example can be written with `pdal pcl`
like this:

::

    $ cd pdal  # your PDAL source tree
    $ cd test/data
    $ ../../bin/pdal pcl -i autzen/autzen-point-format-3.las -p filters/pcl/example_PassThrough_1.json -o ../temp/foo.las -v4

This should produce the output

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

    76(writers.las DEBUG: 3): Wrote 81 points to the LAS file
    .100



Examples
------------------------------------------------------------------------------



Simple point cloud cropping
..............................................................................

The power of the PCL Block is really exposed through the JSON description. In
this example, we apply a single PCL filter to the PointView. The
`PassThrough`_ filter removes points that lie outside a given range for the
specified dimension. Here, we are asking PCL to crop the input point cloud,
returning only those points with z values in the range 100 to 200.

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
        }
    ]

(This example is taken from the unit test
`PCLBlockFilterTest_example_PassThrough_1`.)



Point cloud cropping with outlier removal
..............................................................................

Building on the previous example, we can string together multiple PCL filtering
stages, such as the `StatisticalOutlierRemoval`_ filter. Note that the name
field identifies the PCL filter by its class name, and furthermore that as of
now only a handful of the PCL filtering options are accessible through the PCL
Block. Similarly, select parameters of these classes can be set by specifying
their public member functions by name.

.. code-block:: json

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

(This example is taken from the unit test
`PCLBlockFilterTest_example_PassThrough_2`.)


Ground return filtering
..............................................................................

The Progressive Morphological Filter (PMF) is an openly published approach to
identifying ground vs. non-ground returns in point cloud data. An implementation
of PMF is included with PCL and accessible through the PDAL's PCL Block filter.

A complete description of the algorithm can be found in the article `"A
Progressive Morphological Filter for Removing Nonground Measurements from
Airborne LIDAR Data" <http://users.cis.fiu.edu/~chens/PDF/TGRS.pdf>`_ by K.
Zhang, S.  Chen, D. Whitman, M. Shyu, J. Yan, and C. Zhang.

To run the PMF with default settings, the PCL Block JSON is simply:

.. code-block:: json

    [
        {
            "name": "ProgressiveMorphologicalFilter"
            "setMaxWindowSize": 200,
        }
    ]

Additional parameters can be set by advanced users:

.. code-block:: json

    [
        {
            "name": "ProgressiveMorphologicalFilter",
            "setCellSize": 1.0,
            "setMaxWindowSize": 200,
            "setSlope": 1.0,
            "setInitialDistance": 0.5,
            "setMaxDistance": 3.0,
            "setExponential": true
        }
    ]

(These examples are taken from the unit tests
`PCLBlockFilterTest_example_PMF_1` and `PCLBlockFilterTest_example_PMF_2`.)

See :ref:`here <pcl_ground>` for a more detailed
explanation of the PMF parameters.

.. _`JSON`: http://www.json.org/
.. _`PassThrough`: http://pointclouds.org/documentation/tutorials/passthrough.php
.. _`StatisticalOutlierRemoval`: http://pointclouds.org/documentation/tutorials/statistical_outlier.php
