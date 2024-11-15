(filters.splitter)=

# filters.splitter

The **Splitter Filter** breaks a point cloud into square tiles of a
specified size.  The origin of the tiles is chosen arbitrarily unless specified
with the [origin_x] and [origin_y] option.

The splitter takes a single `PointView` as its input and creates a
`PointView` for each tile as its output.

Splitting is usually applied to data read from files (which produce one large
stream of points) before the points are written to a database (which prefer
data segmented into smaller blocks).

```{eval-rst}
.. embed::
```

## Example

```json
[
    "input.las",
    {
        "type":"filters.splitter",
        "length":"100",
        "origin_x":"638900.0",
        "origin_y":"835500.0"
    },
    {
        "type":"writers.pgpointcloud",
        "connection":"dbname='lidar' user='user'"
    }
]
```

## Options

length

: Length of the sides of the tiles that are created to hold points.
  \[Default: 1000\]

origin_x

: X Origin of the tiles.  \[Default: none (chosen arbitrarily)\]

origin_y

: Y Origin of the tiles.  \[Default: none (chosen arbitrarily)\]

buffer

: Amount of overlap to include in each tile. This buffer is added onto
  length in both the x and the y direction.  \[Default: 0\]

```{include} filter_opts.md
```
