(readers.ptx)=

# readers.ptx

The **PTX reader** reads data from [Leica Cyclone PTX] files. It infers
dimensions from points stored in a text file.

```{note}
PTX files can contain multiple point clouds stored in a single
file.  If that is the case, the reader will read all the points
from all of the internal point clouds as one.
:::

```{eval-rst}
.. embed::

```

## Example Pipeline

```json
[
    {
        "type":"readers.ptx",
        "filename":"test.ptx"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

filename

: File to read. \[Required\]

```{include} reader_opts.md
```

discard_missing_points

: Each point cloud in a PTX file is "fully populated", in that the point cloud
  will contain missing points with XYZ values of "0 0 0". When this option is
  enabled, we will skip over any missing input points.
  \[Default: true\]

[leica cyclone ptx]: http://paulbourke.net/dataformats/ptx/
