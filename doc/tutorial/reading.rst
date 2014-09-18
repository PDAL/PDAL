.. _reading:

===============================================================================
Reading with PDAL
===============================================================================


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

`reStructuredText`_ output
................................................................................

By default, PDAL outputs `reStructuredText`_. This makes it convenient for 
transforming the output into more pleasing formats like PDF (using `rst2pdf`_) 
or HTML (using `rst2html`_) as part of a processing pipeline.

.. _`rst2pdf`: https://code.google.com/p/rst2pdf/
.. _`rst2html`: http://docutils.sourceforge.net/docs/user/tools.html#rst2html-py

.. _`reStructuredText`: http://docutils.sourceforge.net/rst.html

::

    $ pdal info interesting.las -p 0 

::

    Point 0
    --------------------------------------------------------------------------------

    =================== ======================================= ===================
                Name                                Value        Namespace
    =================== ======================================= ===================
    X                                        637012.23999999999  drivers.las.reader
    Y                                        849028.31000000006  drivers.las.reader
    Z                                        431.66000000000003  drivers.las.reader
    Intensity                                               143  drivers.las.reader
    ReturnNumber                                              1  drivers.las.reader
    NumberOfReturns                                           1  drivers.las.reader
    ScanDirectionFlag                                         1  drivers.las.reader
    EdgeOfFlightLine                                          0  drivers.las.reader
    Classification                                            1  drivers.las.reader
    ScanAngleRank                                            -9  drivers.las.reader
    UserData                                                132  drivers.las.reader
    PointSourceId                                          7326  drivers.las.reader
    Time                                     245380.78254962614  drivers.las.reader
    Red                                                      68  drivers.las.reader
    Green                                                    77  drivers.las.reader
    Blue                                                     88  drivers.las.reader
    =================== ======================================= ===================

JavaScript output
................................................................................

JavaScript `JSON`_ can also be output.

.. _`JSON`: http://www.json.org/

::

    $ pdal info interesting.las -p 0 --json

.. code-block:: javascript

    {
        "X": "637012.23999999999",
        "Y": "849028.31000000006",
        "Z": "431.66000000000003",
        "Intensity": "143",
        "ReturnNumber": "1",
        "NumberOfReturns": "1",
        "ScanDirectionFlag": "1",
        "EdgeOfFlightLine": "0",
        "Classification": "1",
        "ScanAngleRank": "-9",
        "UserData": "132",
        "PointSourceId": "7326",
        "Time": "245380.78254962614",
        "Red": "68",
        "Green": "77",
        "Blue": "88"
    }



XML output
................................................................................

XML output of this same point is as simple as adding the appropriate 
switch:

::

    $ pdal info interesting.las -p 0 --xml

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <point>
      <X>637012.23999999999</X>
      <Y>849028.31000000006</Y>
      <Z>431.66000000000003</Z>
      <Intensity>143</Intensity>
      <ReturnNumber>1</ReturnNumber>
      <NumberOfReturns>1</NumberOfReturns>
      <ScanDirectionFlag>1</ScanDirectionFlag>
      <EdgeOfFlightLine>0</EdgeOfFlightLine>
      <Classification>1</Classification>
      <ScanAngleRank>-9</ScanAngleRank>
      <UserData>132</UserData>
      <PointSourceId>7326</PointSourceId>
      <Time>245380.78254962614</Time>
      <Red>68</Red>
      <Green>77</Green>
      <Blue>88</Blue>
    </point>


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

    $ pdal info --metadata --xml interesting.las

This produces metadata that looks like :ref:`this <metadataxml>`. You can use
your favorite `XML`_ or `JSON`_ manipulation tools to extract this information
and do what you need with it. For formats that do not have the ability to
preserve this metadata internally, you can keep a ``.xml`` or ``.json`` file
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

The following XML document defines a :ref:`pipeline` that takes the ``file.las`` 
`ASPRS LAS`_ file and converts it to a new file called ``output.las``. 

::

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="drivers.las.writer">
            <Option name="filename">
                output.las
            </Option>
            <Reader type="drivers.las.reader">
                <Option name="filename">
                    ./path/to/my/file.las
                </Option>
            </Reader>
        </Writer>
    </Pipeline>

.. _`JSON`: http://www.json.org/
.. _`XML`: http://en.wikipedia.org/wiki/XML
.. _`UUID`: http://en.wikipedia.org/wiki/Universally_unique_identifier
.. _`interesting.las`: https://github.com/PDAL/PDAL/blob/master/test/data/interesting.las?raw=true
.. _`ASPRS LAS`: http://www.asprs.org/a/society/committees/standards/lidar_exchange_format.html
