(writers.e57)=

# writers.e57

The **E57 Writer** supports writing to E57 files.

The writer supports E57 files with Cartesian point clouds.

```{note}
E57 files can contain multiple point clouds stored in a single
file.  The writer will only write a single cloud per file.
```

```{note}
Spherical format points are not supported.
```

```{note}
The E57 `cartesianInvalidState` dimension is mapped to the Omit
PDAL dimension.  A range filter can be used to filter out the
invalid points.
```

```{eval-rst}
.. plugin::
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
        "type":"writers.e57",
        "filename":"outputfile.e57",
          "double_precision":false
    }
]
```

## Options

filename

: E57 file to write \[Required\]

double_precision

: Use double precision for storage (false by default).

```{include} writer_opts.md
```
