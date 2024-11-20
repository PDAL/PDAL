(writers.pcd)=

# writers.pcd

The **PCD Writer** supports writing to [Point Cloud Data (PCD)] formatted
files, which are used by the [Point Cloud Library (PCL)].

By default, compression is not enabled, and the PCD writer will output ASCII
formatted data.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

```{note}
X, Y, and Z dimensions will be written as single-precision floats by
default to be compatible with most of the existing PCL point types. These
dimensions can be forced to double-precision using the `order` option, but
the PCL code reading this data must be capable of reading double-precision
fields (i.e., it is not the responsibility of PDAL to ensure this
compatibility).
```

```{note}
When working with large coordinate values it is recommended that users
first translate the coordinate values using {ref}`filters.transformation`
to avoid loss of precision when writing single-precision XYZ data.
```

## Example

```json
[
    {
        "type":"readers.pcd",
        "filename":"inputfile.pcd"
    },
    {
        "type":"writers.pcd",
        "filename":"outputfile.pcd"
    }
]
```

## Options

filename

: PCD file to write \[Required\]

compression

: Level of PCD compression to use (ascii, binary, compressed) \[Default:
  "ascii"\]

precision

: Decimal Precision for output of values. This can be overridden for individual
  dimensions using the order option. \[Default: 2\]

order

: Comma-separated list of dimension names in the desired output order. For
  example "X,Y,Z,Red,Green,Blue". Dimension names can optionally be followed
  by a PDAL type (e.g., Unsigned32) and dimension-specific precision (used only
  with "ascii" compression).  Ex: "X=Float:2, Y=Float:2, Z=Float:3,
  Intensity=Unsigned32" If no precision is specified the value provided with
  the [precision] option is used.  The default dimension type is double
  precision float. \[Default: none\]

keep_unspecified

: If true, writes all dimensions. Dimensions specified with the [order] option
  precede those not specified. \[Default: **true**\]

```{include} writer_opts.md
```

[point cloud data (pcd)]: https://pcl-tutorials.readthedocs.io/en/latest/pcd_file_format.html
[point cloud library (pcl)]: http://pointclouds.org
