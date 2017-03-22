.. las_tutorial:

================================================================================
LAS Reading and Writing with PDAL
================================================================================

.. include:: ../workshop/includes/substitutions.rst

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 3/20/2017

This tutorial will describe reading and writing |ASPRSLAS| data with PDAL,
discuss the capabilities that PDAL :ref:`readers.las` and :ref:`writers.las`
can provide for this format.

Introduction
-------------------------------------------------------------------------------

|ASPRSLAS| is probably the most commonly used|LiDAR| format, and PDAL's support
of LAS is important for many users of the library. This tutorial describes and
demonstrates some of the capabilities the drivers provide, points out items to
be aware of when using the drivers, and hopefully provides some examples you
can use to get what you need out of the LAS drivers.

Considerations
-------------------------------------------------------------------------------

Major Versions
................................................................................

There have five major LAS versions -- 1.0 to 1.4. Each iteration added some
complexity to the format in terms of capabilities it supports, possible data
types it stores, and metadata. Users of LAS must balance the features they need
with the use of the data by downstream applications. While LAS support in some
form is quite widespread throughout the industry, most applications do not
support each and every feature of the format. PDAL works to provide many of
these features, but it doesn't support everything either.

Spatial Reference System
................................................................................

LAS 1.0 to 1.3 used |GeoTIFF| keys for storing coordinate system information,
while LAS 1.4 uses |WellKnownText|. GeoTIFF is well supported by most softwares
that read LAS, but it is not possible to express some coordinate system
specifics with GeoTIFF. WKT is more expressive, but also presents some software
challenges.

Point Formats
................................................................................

As each revision was released, more point formats were added. A Point Format is
the fixed set of :ref:`dimensions` that a LAS file must present and store in
the file. Their definition, size, and composition all change in minor LAS
versions (ie, 1.2 point formats vs those in 1.4). It is generally true,
however, that point formats between minor revisions have similar semantic
meanings, and in many cases their storage size is the same. For example, a 1.0
file with points of type "Point Format 0" represents essentially the same data
as a 1.4 file with points of type "Point Format 0".

Extra Dimensions
................................................................................

A LAS Point Format ID defines the fixed sent of :ref:`dimensions` a file must
store, but softwares are allowed to store extra data beyond that fixed set.
This feature of the format was regularized in LAS 1.4 as something called
"extra bytes" or "extra dims", but formats with versions less than LAS 1.4 can
also store these extra per-point attributes.


Required Header Fields
................................................................................

Readers of the ASPRS LAS Specification will see there are many fields that
softwares are required to write, with their content mandated by various options
and configurations in the format. PDAL does not assume responsibility for
writing these fields and coercing meaning from the content to fit the
specification.  It is the PDAL users' responsibility to do so. Fields where
this might matter include:

* `project_id`
* `global_encoding`
* `system_id`
* `software_id`
* `filesource_id`

Coordinate Scaling
................................................................................

LAS stores coordinates as 32 bit integers. It is the user's responsibility to
ensure that the coordinate domain required by the data in the file fits within
the 32 bit integer domain. Users should scale the data in relation to the
measurement scale of the data, and they should not use the full 32 bit integer
box of precision available to them to store it.

It is usual to preserve the coordinate scale of data when translating LAS data,
but in situations that change the coordinate precision, such as reprojecting
from UTM to decimal degrees, it is not always possible to do so. Overdriven
coordinate scale also hurts `Compression`_ with |LASzip| and disrupts
communication of realistic accuracy.

Compression
................................................................................

|LASzip| is an open source, lossless compression technique for |ASPRSLAS| data.
It is supported by two different software libraries, and it can be used in both
the C/C++ and the JavaScript execution environments.  LAZ support is provided
by both :ref:`readers.las` and :ref:`writers.las` as an option, the
``compression`` one, and t is a flag to determine whether or not the data
should be compressed.  LAZ efficiency is hurt by over-specified coordinate
precision, and it is not fully compatible with LAS 1.4 as of March 2017. A
revision with 1.4 support is expected.

Variable Length Records
................................................................................

Variable Length Records, or VLRs, are blobs of binary data that the LAS format
supports to allow applications to store their own data. Coordinate system
information is one type of data stored in VLRs, and many different LAS-using
applications store data and metadata with this format capability. PDAL allows
users to access VLR information, forward it along to newly written files, and
create VLRs that store processing history information.

Forward
-------------------------------------------------------------------------------

The ``forward`` option of :ref:`writers.las` is the easiest way to get most of
what you might want in terms of header settings copied from an input to an
output file upon processing. Imagine the scenario of zero'ing out the
classification values for an LAS file in preparation for using
:ref:`filters.ground` to reassign them. During this scenario, we'd like to keep
all of the other LAS header information, such as `Variable Length Records`_,
extent information, and format settings.

.. code-block:: json
    :linenos:
    :emphasize-lines: 21

    {
        "pipeline": [
            {
                "type" : "readers.las",
                "filename" : "input.las"
            },
            {
                "type" : "filters.assign",
                "assignment" : "Classification[0:32]=0"
            },
            {
                "type" : "filters.ground",
                "cell_size" : 2.5,
                "classify" : true,
                "extract" : true,
                "approximate" : false,
                "max_distance" : 25
            },
            {
                "type" : "writers.las",
                "forward": "all",
                "filename" : "output.las"
            }
        ]
    }





Point Formats
-------------------------------------------------------------------------------

Point format or `dataformat_id` is an integer that defines the set of fixed
:ref:`dimensions` a LAS file must contain. All LAS files have at minimum a fixed
set of the following dimensions:

.. csv-table:: Base LAS :ref:`dimensions`
    :widths: auto

    "X", "Y", "Z"
    "Intensity", "ReturnNumber", "NumberOfReturns"
    "ScanDirectionFlag", "EdgeOfFlightLine", "Classification"
    "ScanAngleRank", "UserData", "PointSourceId"

These fixed formats have known
field sizes, and explicit meanings. Because LAS data are fluffy, adding 14 bytes
per point to add both GPSTime and Red/Green/Blue fields can waste a lot
of storage if there isn't any actual data in them or you would like to disregard it.

A simple ``dataformat_id`` option along with the ``forward`` option to carry
along all of the rest of the fields will be useful in our case to remove
both the time and color fields (Point format 0):

.. code-block:: json
    :linenos:
    :emphasize-lines: 10

    {
        "pipeline": [
            {
                "type" : "readers.las",
                "filename" : "input.las"
            },
            {
                "type" : "writers.las",
                "forward": "all",
                "dataformat_id": 0,
                "filename" : "output.las"
            }
        ]
    }

.. note::

    The |LASzip| storage of GPSTime and Red/Green/Blue fields with no
    data is perfectly efficient.


VLRs
-------------------------------------------------------------------------------

Variable Length Records (VLRs) are how applications can insert their own data
into LAS files. Common VLR data include:

* Coordinate system
* Metadata
* Processing history
* Indexing

.. note::

    There are VLRs that are defined by the specification, and they
    always have the VLR ``user_id`` of `LASF_Spec`. The most important
    ones are used for describing the coordinate system of the data.

VLRs are inserted into the file as a header that stores a length along with a
binary block of data. For LAS 1.0-1.3, the VLR length could be no larger than
65535 bytes. For EVLRs in LAS 1.4, this limit was increased to 4gb.


Scale and Offset
-------------------------------------------------------------------------------

Spatial Reference System
-------------------------------------------------------------------------------

PDAL Metadata
-------------------------------------------------------------------------------

