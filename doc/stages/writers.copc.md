(writers.copc)=

# writers.copc

The **COPC Writer** supports writing to [COPC format] files. COPC
is *Cloud Optimized Point Clouds*, and it is a LAZ 1.4 file that is
organized stored as a clustered octree.

```{eval-rst}
.. embed::

```

```{note}
Visit <https://viewer.copc.io> to view COPC files in your browser.
Simply drag-n-drop the file from your desktop onto the page,
or use
```

## VLRs

VLRs can be created by providing a JSON node called `vlrs` with objects
as shown:

```json
[
    {
        "type":"readers.las",
        "filename":"inputfile.las"
    },
    {
        "type":"writers.las",
        "vlrs": [{
            "description": "A description under 32 bytes",
            "record_id": 42,
            "user_id": "hobu",
            "data": "dGhpcyBpcyBzb21lIHRleHQ="
            },
            {
            "description": "A description under 32 bytes",
            "record_id": 43,
            "user_id": "hobu",
            "filename": "path-to-my-file.input"
            },
            {
            "description": "A description under 32 bytes",
            "record_id": 44,
            "user_id": "hobu",
            "metadata": "metadata_keyname"
            }],
        "filename":"outputfile.las"
    }
]
```

```{note}
One of `data`, `filename` or `metadata` must be specified. Data must be
specified as base64 encoded strings. The content of a file is inserted as
binary. The metadata key specified must refer to a string or base64 encoded data.
```

## Example

```json
[
    "inputfile1.las",
    "inputfile2.laz",
    {
        "type":"writers.copc",
        "filename":"outputfile.copc.laz"
    }
]
```

## Options

filename

: Output filename.  \[Required\]

forward

: List of header fields whose values should be preserved from a source
  LAS file.  The option can be specified multiple times, which has the same effect as
  listing values separated by a comma.  The following values are valid:
  `filesource_id`, `global_encoding`, `project_id`, `system_id`, `software_id`,
  `creation_doy`, `creation_year`, `scale_x`, `scale_y`, `scale_z`,
  `offset_x`, `offset_y`, `offset_z`.  In addition, the special value `header`
  can be specified, which is equivalent to specifying all the values EXCEPT the scale and
  offset values.  Scale and offset values can be forwarded as a group by
  using the special values `scale` and `offset` respectively.  The special
  value `all` is equivalent to specifying `header`, `scale`, `offset` and
  `vlr` (see below).  If a header option is specified explicitly, it will override
  any forwarded header value.
  If a LAS file is the result of multiple LAS input files, the header values
  to be forwarded must match or they will be ignored and a default will
  be used instead.

  VLRs can be forwarded by using the special value `vlr`.  VLRs containing
  the following User IDs are NOT forwarded: `LASF_Projection`,
  `liblas`, `laszip encoded`.  VLRs with the User ID `LASF_Spec` and
  a record ID other than 0 or 3 are also not forwarded.  These VLRs are known
  to contain information regarding the formatting of the data and will be rebuilt
  properly in the output file as necessary.  Unlike header values, VLRs from multiple
  input files are accumulated and each is written to the output file.  Forwarded
  VLRs may contain duplicate User ID/Record ID pairs.

software_id

: String identifying the software that created this LAS file.
  \[Default: PDAL version num (build num)\]"

creation_doy

: Number of the day of the year (January 1 == 1) this file is being created.

creation_year

: Year (Gregorian) this file is being created.

system_id

: String identifying the system that created this LAS file. \[Default: "PDAL"\]

global_encoding

: Various indicators to describe the data.  See the LAS documentation.  Note
  that PDAL will always set bit four when creating LAS version 1.4 output.
  \[Default: 0\]

project_id

: UID reserved for the user \[Default: Nil UID\]

scale_x, scale_y, scale_z

: Scale to be divided from the X, Y and Z nominal values, respectively, after
  the offset has been applied.  The special value `auto` can be specified,
  which causes the writer to select a scale to set the stored values of the
  dimensions to range from \[0, 2147483647\].  \[Default: .01\]

  Note: written value = (nominal value - offset) / scale.

offset_x, offset_y, offset_z

: Offset to be subtracted from the X, Y and Z nominal values, respectively,
  before the value is scaled.  The special value `auto` can be specified,
  which causes the writer to set the offset to the minimum value of the
  dimension.  \[Default: 0\]

  Note: written value = (nominal value - offset) / scale.

filesource_id

: The file source id number to use for this file (a value between
  0 and 65535 - 0 implies "unassigned") \[Default: 0\]

pipeline

: Write a JSON representation of the running pipeline as a VLR.

vlrs

: Add VLRS specified as json. See [VLRs] above for details.

a_srs

: Spatial reference to use to write output.

threads

: Number of threads to use when writing \[Default: 10\]

extra_dims

: Extra dimensions to be written as part of each point beyond those specified
  by the LAS point format.  The format of the option is
  `<dimension_name>=<type> [, ...]`.  Any valid PDAL {ref}`type <types>`
  can be specified.

  The special value `all` can be used in place of a dimension/type list
  to request that all dimensions that can't be stored in the predefined
  LAS point record get added as extra data at the end of each point record.

enhanced_srs_vlrs

: Write WKT2 and PROJJSON as VLR \[Default: false\]

```{include} writer_opts.md
```

[copc format]: https://copc.io/
