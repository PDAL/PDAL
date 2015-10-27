.. _reading:

===============================================================================
Reading with PDAL
===============================================================================

:Author: Bradley Chambers
:Contact: brad.chambers@gmail.com
:Date: 01/21/2015



.. contents:: Contents
   :depth: 3
   :backlinks: none

This tutorial will be presented in two parts -- the first being an introduction
to the command-line utilities that can be used to perform processing operations
with PDAL, and the second being an introductory C++ tutorial of how to use the
:ref:`PDAL API <cppapi>` to accomplish similar tasks.

Introduction
------------------------------------------------------------------------------

PDAL is both a C++ library and a collection of command-line utilities for
data processing operations.  While it is similar to `LAStools`_ in a few
aspects, and borrows some of its lineage in others, the PDAL library
is an attempt to construct a library that is primarily intended as a
data translation library first, and a exploitation and filtering library
second.  PDAL exists to provide an abstract API for software developers
wishing to navigate the multitude of point cloud formats that are out there.
Its value and niche is explicitly modeled after the hugely successful `GDAL`_
library, which provides an abstract API for data formats in the GIS raster
data space.

.. _`GDAL`: http://www.gdal.org
.. _`LAStools`: http://lastools.org

A basic inquiry example
------------------------------------------------------------------------------

Our first example to demonstrate PDAL's utility will be to simply query an
`ASPRS LAS`_ file to determine the data that are in it in the very first point.

.. note::

    The `interesting.las`_ file in these examples can be found on github.

`pdal info` outputs JavaScript `JSON`_.

.. _`JSON`: http://www.json.org/

::

    $ pdal info interesting.las -p 0

.. code-block:: javascript

    {
      "filename": "interesting.las",
      "pdal_version": "1.0.1 (git-version: 80644d)",
      "points":
      {
        "point":
        {
          "Blue": 88,
          "Classification": 1,
          "EdgeOfFlightLine": 0,
          "GpsTime": 245381,
          "Green": 77,
          "Intensity": 143,
          "NumberOfReturns": 1,
          "PointId": 0,
          "PointSourceId": 7326,
          "Red": 68,
          "ReturnNumber": 1,
          "ScanAngleRank": -9,
          "ScanDirectionFlag": 1,
          "UserData": 132,
          "X": 637012,
          "Y": 849028,
          "Z": 431.66
        }
      }
    }

A conversion example
------------------------------------------------------------------------------

Conversion of one file format to another can be a hairy topic. You should
expect *leakage* of details of data in the source format as it is converted to
the destination format. :ref:`metadata`, file organization, and data themselves
may not be able to be represented as you move from one format to another.
Conversion is by definition lossy, if not in terms of the actual data
themselves, but possibly in terms of the auxiliary data the format also
carries.

It is also important to recognize that both fixed and flexible point cloud
formats exist, and conversion of flexible formats to fixed formats will often
leak. The dimensions might even match in terms of type or name, but not in
terms of width or interpretation.

.. seealso::

    See :cpp:class:`pdal::Dimension` for details on PDAL dimensions.

::

    $ pdal translate interesting.las output.txt

::

    "X","Y","Z","Intensity","ReturnNumber","NumberOfReturns","ScanDirectionFlag","EdgeOfFlightLine","Classification","ScanAngleRank","UserData","PointSourceId","Time","Red","Green","Blue"
    637012.24,849028.31,431.66,143,1,1,1,0,1,-9,132,7326,245381,68,77,88
    636896.33,849087.70,446.39,18,1,2,1,0,1,-11,128,7326,245381,54,66,68
    636784.74,849106.66,426.71,118,1,1,0,0,1,-10,122,7326,245382,112,97,114
    636699.38,848991.01,425.39,100,1,1,0,0,1,-6,124,7326,245383,178,138,162
    636601.87,849018.60,425.10,124,1,1,1,0,1,-4,126,7326,245383,134,104,134
    636451.97,849250.59,435.17,48,1,1,0,0,1,-9,122,7326,245384,99,85,95
    ...

The text format, of course, is the ultimate flexible-definition format -- at
least for the point data themselves. For the other header information, like
the spatial reference system, or the `ASPRS LAS`_ `UUID`_, the conversion
leaks. In short, you may need to preserve some more information as part of
your conversion to make it useful down the road.

:ref:`metadata`
..............................................................................

PDAL transmits this other information in the form of :ref:`metadata` that is
carried per-stage throughout the PDAL :ref:`processing pipeline <pipeline>`.
We can capture this metadata using the :ref:`info_command` utility.

::

    $ pdal info --metadata interesting.las

This produces metadata that looks like :ref:`this <metadatajson>`. You can use
your `JSON`_ manipulation tools to extract this information.
For formats that do not have the ability to
preserve this metadata internally, you can keep a ``.json`` file
alongside the ``.txt`` file as auxiliary information.

.. seealso::
    :ref:`metadata` contains much more detail of metadata workflow in PDAL.

A :ref:`pipeline_command` example
------------------------------------------------------------------------------

The full power of PDAL comes in the form of :ref:`pipeline_command` invocations.
While :ref:`translate_command` provides some utility as far as simple conversion of
one format to another, it does not provide much power to a user to be able
to filter or alter data as they are converted.  Pipelines are the way to take
advantage of PDAL's ability to manipulate data as they are converted. This
section will provide a basic example and demonstration of :ref:`pipeline`,
but the :ref:`pipeline` document contains more detailed exposition of the
topic.

.. note::

    The :ref:`pipeline_command` document contains detailed examples and background
    information.

The :ref:`pipeline_command` PDAL utility is one that takes in a ``.xml`` file
containing :ref:`pipeline <pipeline_command>` description that defines a PDAL
processing pipeline. Options can be given at each :cpp:class:`pdal::Stage` of
the pipeline to affect different aspects of the processing pipeline, and
stages may be chained together into multiple combinations to have varying
effects.

Simple conversion
..............................................................................

The following `XML`_ document defines a :ref:`pipeline` that takes the ``file.las``
`ASPRS LAS`_ file and converts it to a new file called ``output.las``.

::

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="writers.las">
            <Option name="filename">
                output.las
            </Option>
            <Reader type="readers.las">
                <Option name="filename">
                    ./path/to/my/file.las
                </Option>
            </Reader>
        </Writer>
    </Pipeline>

Loop a directory and filter it through a pipeline
................................................................................

This little bash script loops through a directory and pushes the las files through
a pipeline, substituting the input and output as it goes.

::

    ls *.las | cut -d. -f1 | xargs -P20 -I{} pdal pipeline -i /path/to/proj.xml --readers.las.filename={}.las --writers.las.filename=output/{}.laz

.. _`JSON`: http://www.json.org/
.. _`XML`: http://en.wikipedia.org/wiki/XML
.. _`UUID`: http://en.wikipedia.org/wiki/Universally_unique_identifier
.. _`interesting.las`: https://github.com/PDAL/PDAL/blob/master/test/data/interesting.las?raw=true
.. _`ASPRS LAS`: http://www.asprs.org/a/society/committees/standards/lidar_exchange_format.html
