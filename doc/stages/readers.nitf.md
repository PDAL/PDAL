(readers.nitf)=

# readers.nitf

The [NITF] format is used primarily by the US Department of Defense and
supports many kinds of data inside a generic wrapper. The [NITF 2.1] version
added support for LIDAR point cloud data, and the **NITF file reader** supports
reading that data, if the NITF file supports it.

- The file must be NITF 2.1
- There must be at least one Image segment ("IM").
- There must be at least one [DES segment] ("DE") named "LIDARA".
- Only LAS or LAZ data may be stored in the LIDARA segment

The dimensions produced by the reader match exactly to the LAS dimension names
and types for convenience in file format transformation.

```{note}
Only LAS or LAZ data may be stored in the LIDARA segment. PDAL uses
the {ref}`readers.las` and {ref}`writers.las`
to actually read and write the data.
```

```{note}
PDAL uses a fork of the [NITF Nitro] library available at
<https://github.com/hobu/nitro> for NITF read and write support.
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
        "type":"readers.nitf",
        "filename":"mynitf.nitf"
    },
    {
        "type":"writers.las",
        "filename":"outputfile.las"
    }
]
```

## Options

filename

: Filename to read from \[Required\]

```{include} reader_opts.md
```

extra_dims

: Extra dimensions to be read as part of each point beyond those specified by
  the LAS point format.  The format of the option is
  `<dimension_name>=<type>[, ...]`.  Any PDAL {ref}`type <types>` can
  be specified.

  ```{note}
  The presence of an extra bytes VLR when reading a version
  1.4 file or a version 1.0 - 1.3 file with **use_eb_vlr** set
  causes this option to be ignored.
  ```

use_eb_vlr

: If an extra bytes VLR is found in a version 1.0 - 1.3 file, use it as if it
  were in a 1.4 file. This option has no effect when reading a version 1.4 file.
  \[Default: false\]

compression

: May be set to "lazperf" or "laszip" to choose either the LazPerf decompressor
  or the LASzip decompressor for LAZ files.  PDAL must have been built with
  support for the decompressor being requested.  The LazPerf decompressor
  doesn't support version 1 LAZ files or version 1.4 of LAS.
  \[Default: "none"\]

[des segment]: https://nsgreg.nga.mil/doc/view?i=5402
[nitf]: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format
[nitf 2.1]: https://gwg.nga.mil/gwg/focus-groups/NITFS_NTB_Documentation.html
[nitf nitro]: https://github.com/mdaus/nitro
