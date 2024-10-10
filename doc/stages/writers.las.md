(writers.las)=

# writers.las

The **LAS Writer** supports writing to [LAS format] files, the standard
interchange file format for LIDAR data.

```{warning}
Scale/offset are not preserved from an input LAS file.  See below for
information on the scale/offset options and the [forward] option.
```

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
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
            "description": "Write metadata as EVLR",
            "record_id": 44,
            "user_id": "hobu",
            "evlr": true,
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
    {
        "type":"readers.las",
        "filename":"inputfile.las"
    },
    {
        "type":"writers.las",
        "filename":"outputfile.las"
    }
]
```

## Options

filename

: Output filename. The writer will accept a filename containing
  a single placeholder character (`#`).  If input to the writer consists
  of multiple PointViews, each will be written to a separate file, where
  the placeholder will be replaced with an incrementing integer.  If no
  placeholder is found, all PointViews provided to the writer are
  aggregated into a single file for output.  Multiple PointViews are usually
  the result of using {ref}`filters.splitter`, {ref}`filters.chipper` or
  {ref}`filters.divider`.
  \[Required\]

`` _`forward` ``

: List of header fields whose values should be preserved from a source
  LAS file.  The
  option can be specified multiple times, which has the same effect as
  listing values separated by a comma.  The following values are valid:
  `major_version`, `minor_version`, `dataformat_id`, `filesource_id`,
  `global_encoding`, `project_id`, `system_id`, `software_id`, `creation_doy`,
  `creation_year`, `scale_x`, `scale_y`, `scale_z`, `offset_x`, `offset_y`,
  `offset_z`.  In addition, the special value `header` can be specified,
  which is equivalent to specifying all the values EXCEPT the scale and
  offset values.  Scale and offset values can be forwarded as a group by
  using the special values `scale` and `offset` respectively.  The special
  value `all` is equivalent to specifying `header`, `scale`, `offset` and
  `vlr` (see below).
  If a header option is specified explicitly, it will override any forwarded
  header value.
  If a LAS file is the result of multiple LAS input files, the header values
  to be forwarded must match or they will be ignored and a default will
  be used instead.

  VLRs can be forwarded by using the special value `vlr`.  VLRs containing
  the following User IDs are NOT forwarded: `LASF_Projection`,
  `liblas`, `laszip encoded`.  VLRs with the User ID `LASF_Spec` and
  a record ID other than 0 or 3 are also not forwarded.  These VLRs are known
  to contain information
  regarding the formatting of the data and will be rebuilt properly in the
  output file as necessary.  Unlike header values, VLRs from multiple input
  files are accumulated and each is written to the output file.  Forwarded
  VLRs may contain duplicate User ID/Record ID pairs.

minor_version

: All LAS files are version 1, but the minor version (0 - 4) can be specified
  with this option. \[Default: 4 (was 2 for PDAL 1.0 - 2.7)\]

software_id

: String identifying the software that created this LAS file.
  \[Default: PDAL version num (build num)\]"

creation_doy

: Number of the day of the year (January 1 == 1) this file is being created.

creation_year

: Year (Gregorian) this file is being created.

dataformat_id

: Controls whether information about color and time are stored with the point
  information in the LAS file. \[Default: 3\]

  - 0 == no color or time stored
  - 1 == time is stored
  - 2 == color is stored
  - 3 == color and time are stored
  - 4 \[Not Currently Supported\]
  - 5 \[Not Currently Supported\]
  - 6 == time is stored (LAS version 1.4+ only)
  - 7 == time and color are stored (LAS version 1.4+ only)
  - 8 == time, color and near infrared are stored (LAS version 1.4+ only)
  - 9 \[Not Currently Supported\]
  - 10 \[Not Currently Supported\]

system_id

: String identifying the system that created this LAS file. \[Default: "PDAL"\]

a_srs

: The spatial reference system of the file to be written. Can be an EPSG string
  (e.g. "EPSG:26910") or a WKT string. \[Default: Not set\]

global_encoding

: Various indicators to describe the data.  See the LAS documentation.  Note
  that PDAL will always set bit four when creating LAS version 1.4 output.
  \[Default: 0\]

project_id

: UID reserved for the user \[Default: Nil UID\]

compression

: Set to "true" to apply compression to the output, creating a LAZ file (using
  the LazPerf compressor) instead of a LAS file.
  For backwards compatibility, "lazperf" or "laszip" are still accepted, but
  those values are treated as "true". \[Default: "false"\]

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

discard_high_return_numbers

: If true, discard all points with a return number greater than the maximum
  supported by the point format (5 for formats 0-5, 15 for formats 6-10).
  \[Default: false\]

extra_dims

: Extra dimensions to be written as part of each point beyond those specified
  by the LAS point format.  The format of the option is
  `<dimension_name>=<type> [, ...]`.  Any valid PDAL {ref}`type <types>`
  can be specified.

  The special value `all` can be used in place of a dimension/type list
  to request that all dimensions that can't be stored in the predefined
  LAS point record get added as extra data at the end of each point record.

  PDAL writes an extra bytes VLR (User ID: LASF_Spec, Record ID: 4) when
  extra dims are written.  The VLR describes the extra dimensions specified by
  this option.  Note that reading of this VLR is only specified for LAS
  version 1.4, though some systems will honor it for earlier file formats.
  The {ref}`LAS reader <readers.las>` requires the option
  use_eb_vlr in order to
  read the extra bytes VLR for files written with 1.1 - 1.3 LAS format.

  Setting --verbose=Info will provide output on the names, types and order
  of dimensions being written as part of the LAS extra bytes.

pdal_metadata

: Write two VLRs containing [JSON] output with both the {ref}`metadata` and
  {ref}`pipeline` serialization. \[Default: false\]

vlrs

: Add VLRS specified as json. See [VLRs] above for details.

```{include} writer_opts.md
```

[json]: http://www.json.org/
[las format]: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
