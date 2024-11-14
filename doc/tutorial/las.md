(las_tutorial)=

# LAS Reading and Writing with PDAL

```{eval-rst}

:Author: Howard Butler
:Contact: howard@hobu.co
:Date: 3/27/2017
```

```{contents} Table of Contents
:depth: 2
```

This tutorial will describe reading and writing {{ ASPRSLAS }} data with PDAL,
discuss the capabilities that PDAL {ref}`readers.las` and {ref}`writers.las`
can provide for this format.

## Introduction

{{ ASPRSLAS }} is probably the most commonly used {{ LiDAR }} format, and PDAL's support
of LAS is important for many users of the library. This tutorial describes and
demonstrates some of the capabilities the drivers provide, points out items to
be aware of when using the drivers, and hopefully provides some examples you
can use to get what you need out of the LAS drivers.

## LAS Versions

There are five LAS versions -- 1.0 to 1.4. Each iteration added some
complexity to the format in terms of capabilities it supports, possible data
types it stores, and metadata. Users of LAS must balance the features they need
with the use of the data by downstream applications. While LAS support in some
form is quite widespread throughout the industry, most applications do not
support every feature of each version.  PDAL works to provide many of
these features, but it is also incomplete.  Specifically, PDAL doesn't support
point formats that store waveform data.

### Version Example

We can use the `minor_version` option of {ref}`writers.las` to set the
version PDAL should output. The following example will write a 1.1 version LAS
file. Depending on the features you need, this may or may not be what you want.

```{code-block} json
:emphasize-lines: 8
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "writers.las",
        "minor_version": 1,
        "filename" : "output.las"
    }
]
```

```{note}
PDAL defaults to writing a LAS 1.2 version if no `minor_version` is
specified
or the `forward` option of {ref}`writers.las` is not used to carry along
a version from a previously read file.
```

## Spatial Reference System

LAS 1.0 to 1.3 use {{ GeoTIFF }} keys for storing coordinate system information,
while LAS 1.4 uses {{ WellKnownText }}. GeoTIFF is well-supported by most software
that read LAS, but it is not possible to express some coordinate system
specifics with GeoTIFF. WKT is supports
more coordinate systems than GeoTIFF, but vendor-specific and later versions
(WKT 2) may not be handled well.

### Assignment Example

The PDAL {ref}`writers.las` allows you to override or assign the coordinate
system to an explicit value if you need. Often the coordinate system defined by
a file might be incorrect or non-existent, and you can set this with PDAL.

The following example sets the `a_srs` option of the {ref}`writers.las` to
`EPSG:4326`.

```{code-block} json
:emphasize-lines: 8
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "writers.las",
        "a_srs": "EPSG:4326",
        "filename" : "output.las"
    }
]
```

```{note}
Remember to set `offset_x`, `offset_y`, `scale_x`, and
`scale_y` values to something appropriate if your are storing decimal
degree data in LAS files. The special value `auto` can be used for the
offset values, but you should set an explicit value for the scale values
to prevent overdriving the precision of the data and disrupting
[Compression] with {{ LASzip }}.
```

### Vertical Datum Example

Vertical coordinate control is important in {{ LiDAR }} and PDAL supports
assignment
and reprojection/transform of vertical coordinates using {{ Proj }} and {{ GDAL }}.
The coordinate system description magic happens in GDAL, and you assign a
compound coordinate system (both vertical and horizontal definitions) using
the following syntax:

```
EPSG:4326+3855
```

This assignment states typical 4326 horizontal coordinate system plus a vertical one that
represents [EGM08]. In {{ WellKnownText }}, this coordinate system is described by:

```
$ gdalsrsinfo "EPSG:4326+3855"
```

```
COMPD_CS["WGS 84 + EGM2008 geoid height",
    GEOGCS["WGS 84",
        DATUM["WGS_1984",
            SPHEROID["WGS 84",6378137,298.257223563,
                AUTHORITY["EPSG","7030"]],
            AUTHORITY["EPSG","6326"]],
        PRIMEM["Greenwich",0,
            AUTHORITY["EPSG","8901"]],
        UNIT["degree",0.0174532925199433,
            AUTHORITY["EPSG","9122"]],
        AUTHORITY["EPSG","4326"]],
    VERT_CS["EGM2008 geoid height",
        VERT_DATUM["EGM2008 geoid",2005,
            AUTHORITY["EPSG","1027"],
            EXTENSION["PROJ4_GRIDS","egm08_25.gtx"]],
        UNIT["metre",1,
            AUTHORITY["EPSG","9001"]],
        AXIS["Up",UP],
        AUTHORITY["EPSG","3855"]]
```

As in [Assignment Example], it is common to need to reassign the coordinate
system. The following example defines both the horizontal and vertical
coordinate system for a file to [UTM Zone 15N NAD83] for horizontal and
[NAVD88] for the vertical.

```{code-block} json
:emphasize-lines: 8
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "writers.las",
        "a_srs": "EPSG:26915+5703",
        "filename" : "output.las"
    }
]
```

```{note}
Any coordinate system description format supported by GDAL's
[SetFromUserInput]
method can be used to assign or set the coordinate system in PDAL.
This includes WKT, {{ Proj }} definitions, or OGC URNs. It is your
responsibility, however, to escape or massage any input data to
make it be valid JSON.
```

### Reprojection Example

A common desire is to transform the coordinates of an {{ ASPRSLAS }} file
from one coordinate system to another. The mechanism to do that with
PDAL is {ref}`filters.reprojection`.

```{code-block} json
:emphasize-lines: 8
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type":"filters.reprojection",
        "out_srs":"EPSG:26915"
    },
    {
        "type" : "writers.las",
        "filename" : "output.las"
    }
]
```

```{note}
If the input data doesn't specify a projection, you must specify the
`in_srs` option of {ref}`filters.reprojection`.  `in_srs` can also
be used to override an existing spatial reference attached to the input
point set.
```

## Point Formats

As each revision of LAS was released, more point formats were added. A point
format is
the fixed set of {ref}`dimensions <dimensions>` that a LAS file stores
for each point in
the file.  For any point format, the size and composition of dimensions is
consistent across versions, but users should be aware of some minor
interpretation changes based on LAS file version.  For example, a classification
value of 11 in version 1.4 indicates "Road Surface", while that value is
reserved in version 1.1.

### Point Format Example

Point format or `dataformat_id` is an integer that defines the set of fixed
{ref}`dimensions <dimensions>` stored for each point in a LAS file.
All point formats
specify the following dimensions as part of a point record:

```{eval-rst}
.. csv-table:: Base LAS :ref:`dimensions`
    :widths: auto

    "X", "Y", "Z"
    "Intensity", "ReturnNumber", "NumberOfReturns"
    "ScanDirectionFlag", "EdgeOfFlightLine", "Classification"
    "ScanAngleRank", "UserData", "PointSourceId"
```

Because LAS files have no built-in compression, it's important to use a
point format that stores the fewest fields possible that store the desired
data.  For example, point format 10 uses 45 more bytes per point than point
format zero.

If one wanted remove the Red/Green/Blue fields from a LAS file
(one using point format 2), one could simply set the `dataformat_id`
option to 0.  The `forward` option can also be set to carry forward
all possible header values from the source file to the new, smaller file.

```{code-block} json
:emphasize-lines: 9
:linenos: true

[
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
```

```{note}
The {{ LASzip }} storage of GPSTime and Red/Green/Blue fields with no
data is perfectly efficient.
```

## Extra Dimensions

A LAS Point Format ID defines the fixed set of
{ref}`dimensions <dimensions>` a file must
store, but programs are allowed to store extra data beyond that fixed set.
This feature of the format was regularized in LAS 1.4 as something called
"extra bytes" or "extra dims", but previous versions can
also store these extra per-point attributes.

### Extra Dimension Example

LAS 1.4 provides for the storage of dimensions not part of the chosen
point format by appending them to each point record.  PDAL supports this
feature when writing files with the "extra_dims" option.  The following
example will store all source dimensions in the output file and place
a description of the dimensions that aren't part of the point format in
an "extra bytes" VLR:

```{code-block} json
:emphasize-lines: 5
:linenos: true

[
    "some_non_las_file",
    {
        "type" : "writers.las",
        "extra_dims": "all",
        "minor_version" : "4",
        "filename" : "output.las"
    }
]
```

## Required Header Fields

Readers of the ASPRS LAS Specification will see there are many fields that
softwares are required to write, with their content mandated by various options
and configurations in the format. PDAL does not assume responsibility for
writing these fields and coercing meaning from the content to fit the
specification.  It is the PDAL users' responsibility to do so. Fields where
this might matter include:

- `project_id`
- `global_encoding`
- `system_id`
- `software_id`
- `filesource_id`

### Header Fields Example

The "forward" option of {ref}`writers.las` is the easiest way to get most of
what you might want in terms of header settings copied from an input to an
output file upon processing. Imagine the scenario of zero'ing out the
classification values for an LAS file in preparation for using
{ref}`filters.pmf` to reassign them. During this scenario, we'd like to keep
all of the other LAS header information, such as [Variable Length Records],
extent information, and format settings.

```{code-block} json
:emphasize-lines: 18
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "filters.assign",
        "assignment" : "Classification[0:32]=0"
    },
    {
        "type" : "filters.pmf",
        "cell_size" : 2.5,
        "approximate" : false,
        "max_distance" : 25
    },
    {
        "type" : "writers.las",
        "forward": "all",
        "filename" : "output.las"
    }
]
```

```{note}
If multiple input LAS files are being written to an output file, the
`forward` option can only preserve values when they are the same in
all input files.  If the values differ, a default will be used (as it
would if the `forward` option weren't supplied).  You can specify
specific option values for output that will also override any forwarded
data.
```

## Coordinate Scaling

LAS stores coordinates as 32 bit integers. It is the user's responsibility to
ensure that the coordinate domain required by the data in the file fits within
the 32 bit integer domain.  Most coordinate values have digits to the right
of the decimal point that must be preserved for sufficient accuracy.
Using the scale factor allows for integers to be interpreted as floating
point values when read by software.

When writing data to LAS, choosing an appropriate scale factor should take
into account not just the maximum precision that can be accommodated by the
format, but the actual precision of the data.  Using a precision greater than
the resolution of the data collection can mislead users as to the actual
measurement precision of the data.  In addition, it can lead to larger files
when writing [compressed](Compression) data with {{ LASzip }}.

### Auto Offset Example

Users can allow PDAL select scale and offset values for data with the `auto`
option.  This can have some detrimental effects on downstream processing.
`auto` for scale values will use the entire 32-bit integer domain.
This maximizes the precision available to
store the data, but this will have a detrimental effect on {{ LASzip }} storage
efficiency.  `auto` for offset calculation is just fine, however. When given
the option, choose to store {{ ASPRSLAS }} data with an explicit scale for the X,
Y, and Z dimensions that represents actual expected data precision, not
artificial storage precision or maximal storage precision.

```{code-block} json
:emphasize-lines: 8-13
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "writers.las",
        "scale_x":"0.0000001",
        "scale_y":"0.0000001",
        "scale_z":"0.01",
        "offset_x":"auto",
        "offset_y":"auto",
        "offset_z":"auto",
        "filename" : "output.las"
    }
]
```

## Compression

{{ LASzip }} is an open source, lossless compression technique for {{ ASPRSLAS }} data.
It is supported by two different software libraries, and it can be used in both
the C/C++ and the JavaScript execution environments.  LAZ support is provided
by both {ref}`readers.las` and {ref}`writers.las`.  It can be enabled by
setting the `compression` option to `laszip`.

### Compression Example

Providing a filename with a `.laz` extension will write compressed data.
Compression can be turned on explicitly as well:

```{code-block} json
:emphasize-lines: 8
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "writers.las",
        "compression":"laszip",
        "filename" : "output.laz"
    }
]
```

### Variable Length Records

Variable Length Records, or VLRs, are binary data that the LAS format
supports to allow applications to store their own data. Coordinate system
information is one type of data stored in VLRs, and many different LAS-using
applications store data and metadata with this format capability. PDAL allows
users to access VLR information, forward it along to newly written files, and
create VLRs that store processing history information.

Common VLR data include:

- Coordinate system
- Metadata
- Processing history
- Indexing

```{note}
There are VLRs that are defined by the specification, and they
have the VLR `user_id` of `LASF_Spec` or `LASF_Projection`.
`LASF_Spec` VLRs provide a description of the data beyond that
available in the header.  `LASF_Projection` VLRs store the spatial
coordinate system of the data.
```

For LAS 1.0-1.3, the VLR length could be no larger than
65535 bytes. Version 1.4 introduced extended VLRs, stored at the end of the
file, which could be up to 4gb in size.

### VLR Example

You can add your own VLRs to files to store processing information or
whatever you
want by providing a JSON block via {ref}`writers.las` `vlrs` option that
defines the `user_id` and `data` items for the VLR. The `data` option
must be [base64]-encoded string output. The data will be converted to binary
information and stored in the VLR when the file is written.

```json
[
    "input.las",
    {
        "type":"writers.las",
        "filename":"output.las",
        "vlrs": [   {
                      "description": "A description under 32 bytes",
                      "record_id": 42,
                      "user_id": "hobu",
                      "data": "dGhpcyBpcyBzb21lIHRleHQ="
                     },
                     {
                      "description": "A description under 32 bytes",
                      "record_id": 43,
                      "user_id": "hobu",
                      "data": "dGhpcyBpcyBzb21lIG1vcmUgdGV4dA=="
                      }
                    ]
    }
]
```

## PDAL Metadata

The {ref}`writers.las` driver supports an option, `pdal_metadata`, that writes
two `PDAL` VLRs to LAS files. The first is the equivalent of
{ref}`info_command`'s
`--metadata` output. The second is a copy of the output of the
`--pipeline` serialization
option that describes all stages and options of the pipeline that created
the file.  These two VLRs may
be useful in tracking down processing history of data, allow you to determine
which versions of PDAL may have written a file and what filter options were set
when it was written, and give you the ability to store metadata and other
information via pipeline `user_data` from your own applications.

### Metadata Example

The pipeline used to construct the file and all of its {ref}`metadata`
can be written into VLRs in {{ ASPRSLAS }} files under the `PDAL` [VLR key].

```{code-block} json
:emphasize-lines: 8
:linenos: true

[
    {
        "type" : "readers.las",
        "filename" : "input.las"
    },
    {
        "type" : "writers.las",
        "pdal_metadata":"true",
        "filename" : "output.laz"
    }
]
```

```{warning}
LAS versions prior to 1.4 only support VLRs of at most 64K of information.
It is possible, though improbable, that the metadata or pipeline stored
in the VLRs will not fit in that space.
```

[base64]: https://en.wikipedia.org/wiki/Base64
[egm08]: http://earth-info.nga.mil/GandG/wgs84/gravitymod/egm2008/egm08_wgs84.html
[navd88]: http://epsg.io/5703
[setfromuserinput]: https://gdal.org/en/latest/api/ogrspatialref.html#classOGRSpatialReference_1aec3c6a49533fe457ddc763d699ff8796
[utm zone 15n nad83]: http://spatialreference.org/ref/epsg/26915
[vlr key]: http://www.asprs.org/misc/las-key-list.html
