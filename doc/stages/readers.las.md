(readers.las)=

# readers.las

The **LAS Reader** supports reading from [LAS format] files, the standard
interchange format for LIDAR data.  The reader does NOT support point formats
containing waveform data (4, 5, 9 and 10).

The reader also supports compressed LAS files, known as LAZ files or
[LASzip] files.

```{note}
LAS stores X, Y and Z dimensions as scaled integers.  Users converting an
input LAS file to an output LAS file will frequently want to use the same
scale factors and offsets in the output file as existed in the input
file in order to
maintain the precision of the data.  Use the `forward` option on the
{ref}`writers.las` to facilitate transfer of header information from
source to destination LAS/LAZ files.
```

```{note}
LAS 1.4 files can contain datatypes that are actually arrays rather than
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
LAS 1.4 files that use the extra bytes VLR and datatype 0 will be accepted,
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
        "type":"readers.las",
        "filename":"inputfile.las"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

`` _`filename` ``

: LAS file to read \[Required\]

```{include} reader_opts.md
```

`` _`start` ``

: Point at which reading should start (0-indexed). Useful in combination
  with 'count' option to read a subset of points. \[Default: 0\]

`` _`extra_dims` ``

: Extra dimensions to be read as part of each point beyond those specified by
  the LAS point format.  The format of the option is
  `<dimension_name>=<type>[, ...]`.  Any valid PDAL {ref}`type <types>` can be
  specified.

  ```{note}
  The presence of an extra bytes VLR when reading a version
  1.4 file or a version 1.0 - 1.3 file with **use_eb_vlr** set
  causes this option to be ignored.
  ```

`` _`use_eb_vlr` ``

: If an extra bytes VLR is found in a version 1.0 - 1.3 file, use it as if it
  were in a 1.4 file. This option has no effect when reading a version 1.4 file.
  \[Default: false\]

compression

: \[Deprecated\]

ignore_vlr

: A comma-separated list of "userid/record_id" pairs specifying VLR records that should
  not be loaded.

fix_dims

: Make invalid dimension names valid by converting disallowed characters to '\_'. Only
  applies to names specified in an extra-bytes VLR. \[Default: true\]

nosrs

: Don't read the SRS VLRs. The data will not be assigned an SRS. This option is
  for use only in special cases where processing the SRS could cause performance
  issues. \[Default: false\]

threads

: Thread pool size. Number of threads used to decode laz chunk tables (Default: 7)

[las format]: http://asprs.org/Committee-General/LASer-LAS-File-Format-Exchange-Activities.html
[las specification]: http://www.asprs.org/a/society/committees/standards/LAS_1_4_r13.pdf
[laszip]: http://laszip.org
[lazperf]: https://github.com/verma/laz-perf
