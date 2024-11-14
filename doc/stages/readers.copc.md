(readers.copc)=

# readers.copc

The **COPC Reader** supports reading from [COPC format] files. A COPC file is
a [LASzip] (compressed LAS) file that organizes its data spatially, allowing for
incremental loading and spatial filtering.

```{note}
LAS stores X, Y and Z dimensions as scaled integers.  Users converting an
input LAS file to an output LAS file will frequently want to use the same
scale factors and offsets in the output file as existed in the input
file in order to
maintain the precision of the data.  Use the `forward` option of
{ref}`writers.las` to facilitate transfer of header information from
source COPC files to destination LAS/LAZ files.
```

```{note}
COPC files can contain datatypes that are actually arrays rather than
individual dimensions.  Since PDAL doesn't support these datatypes, it
must map them into datatypes it supports.  This is done by appending the
array index to the name of the datatype.  For example, datatypes 11 - 20
are two dimensional array types and if a field had the name Foo for
datatype 11, PDAL would create the dimensions Foo0 and Foo1 to hold the
values associated with LAS field Foo.  Similarly, datatypes 21 - 30 are
three dimensional arrays and a field of type 21 with the name Bar would
cause PDAL to create dimensions Bar0, Bar1 and Bar2.  See the information
on the extra bytes VLR in the [LAS Specification] for more information
on the extra bytes VLR and array datatypes.
```

```{warning}
COPC files that use the extra bytes VLR and datatype 0 will be accepted,
but the data associated with a dimension of datatype 0 will be ignored
(no PDAL dimension will be created).
```

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::

```

## Example

```json
[
    {
        "type":"readers.copc",
        "filename":"inputfile.copc.laz"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

filename

: COPC file to read. Remote file specifications (http, AWS, Google, Azure, Dropbox) are supported.
  \[Required\]

```{include} reader_opts.md
```

bounds

: The extent of the data to select in 2 or 3 dimensions, expressed as a string,
  e.g.: `([xmin, xmax], [ymin, ymax], [zmin, zmax])`.  If omitted, the entire dataset
  will be selected. The bounds specification can be followed by a slash ('/') and a
  spatial reference specification to apply to the bounds specification.

polygon

: A clipping polygon, expressed in a well-known text string,
  e.g.: `POLYGON((0 0, 5000 10000, 10000 0, 0 0))`.  This option can be
  specified more than once. Multiple polygons will will be treated
  as a single multipolygon. The polygon specification can be followed by a slash ('/') and a
  spatial reference specification to apply to the polygon.

ogr

: A JSON object representing an OGR query to fetch polygons to use for filtering. The polygons
  fetched from the query are treated exactly like those specified in the `polygon` option.
  The JSON object is specified as follows:

  ```json
  {
      "drivers": "OGR drivers to use",
      "openoptions": "Options to pass to the OGR open function [optional]",
      "layer": "OGR layer from which to fetch polygons [optional]",
      "sql": "SQL query to use to filter the polygons in the layer [optional]",
      "options":
      {
          "geometry", "WKT or GeoJSON geomtry used to filter query [optional]"
      }
  }
  ```

requests

: The number of worker threads processing data. The optimal number depends on your system
  and your network connection, but more is not necessarily better.  A reasonably fast
  network connection can often fetch data faster than it can be processed, leading to
  memory consumption and slower performance. \[Default: 15\]

resolution

: Limit the pyramid levels of data to fetch based on the expected resolution of the data.
  Units match that of the data. \[Default: no resolution limit\]

header

: HTTP headers to forward for remote endpoints. Specify as a JSON
  object of key/value string pairs.

query

: HTTP query parameters to forward for remote endpoints. Specify as a JSON
  object of key/value string pairs.

vlr

: Read LAS VLRs and import as metadata. \[Default: false\]

keep_alive

: The number of chunks to keep active in memory while reading \[Default: 10\]

fix_dims

: Make invalid dimension names valid by converting disallowed characters to '\_'. Only
  applies to names specified in an extra-bytes VLR. \[Default: true\]

srs_vlr_order

: Preference order to read SRS VLRs (list of 'wkt1', 'wkt2', or 'projjson').
  \[Default: 'wkt1, wkt2, projjson'\]

nosrs

: Don't read the SRS VLRs. The data will not be assigned an SRS. This option is
  for use only in special cases where processing the SRS could cause performance
  issues. \[Default: false\]

[copc format]: https://copc.io/
[las specification]: https://www.asprs.org/wp-content/uploads/2019/03/LAS_1_4_r14.pdf
[laszip]: http://laszip.org
