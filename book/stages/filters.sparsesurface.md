(filters-sparsesurface)=

# filters.sparsesurface

The **Sparse Surface filter** segments input points into two classes: ground or
low point. It does this by adding ground points in ascending elevation order,
and masking all neighbor points within a specified radius as low points. This
process creates a sparse sampling of the ground estimate akin to the Poisson
disk sampling available in {ref}`filters.sample` and marks all other points as
low noise. It is expected that the input point cloud will either only include
points labeled as ground or the `where` option will be employed to limit
points to those marked as ground.

```{eval-rst}
.. embed::
```

## Example #1

The sample pipeline below uses the SMRF filter to segment ground and non-ground
returns, uses the expression filter to retain only ground returns, and then the
sparse surface filter to segment ground and low noise.

```json
[
    "input.las",
    {
        "type":"filters.smrf"
    },
    {
        "type":"filters.expression",
        "expression":"Classification==2"
    },
    {
        "type":"filters.sparsesurface"
    },
    "output.laz"
]
```

## Example #2

This sample pipeline is nearly identical to the previous one, but retains all
points (including non-ground) while still only operating on ground returns when
computing the sparse surface. It also sets the only option unique to the sparse
sample filter, which is the sampling radius--no two ground points will be
closer than 3.0 meters (horizontally).

```json
[
    "input.las",
    {
        "type":"filters.smrf"
    },
    {
        "type":"filters.sparsesurface",
        "radius":3.0,
        "where":"Classification==2"
    },
    "output.laz"
]
```

## Options

radius

: Mask neighbor points as low noise. \[Default: **1.0**\]

```{eval-rst}
.. include:: filter_opts.rst
```
