(filters.head)=

# filters.head

The **Head filter** returns a specified number of points from the beginning
of a `PointView`.

```{note}
If the requested number of points exceeds the size of the point cloud, all
points are passed with a warning.
```

```{eval-rst}
.. embed::

```

## Example #1

Thin a point cloud by first shuffling the point order with
{ref}`filters.randomize` and then picking the first 10000 using the HeadFilter.

```json
[
    {
        "type":"filters.randomize"
    },
    {
        "type":"filters.head",
        "count":10000
    }
]
```

## Example #2

Compute height above ground and extract the ten highest points.

```json
[
    {
        "type":"filters.smrf"
    },
    {
        "type":"filters.hag_nn"
    },
    {
        "type":"filters.sort",
        "dimension":"HeightAboveGround",
        "order":"DESC"
    },
    {
        "type":"filters.head",
        "count":10
    }
]
```

```{seealso}
{ref}`filters.tail` is the dual to {ref}`filters.head`.
```

## Options

count

: Number of points to return. \[Default: 10\]

```{include} filter_opts.md
```
