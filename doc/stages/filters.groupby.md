(filters.groupby)=

# filters.groupby

The **Groupby Filter** takes a single `PointView` as its input and
creates a `PointView` for each category in the named [dimension] as
its output.

```{eval-rst}
.. embed::
```

## Example

The following pipeline will create a set of LAS files, where each file contains
only points of a single `Classification`.

```json
[
    "input.las",
    {
        "type":"filters.groupby",
        "dimension":"Classification"
    },
    "output_#.las"
]
```

By default the groups are ordered according to the order of first occurrence within the input. To control the order of output groups, use {ref}`filters.sort` first to order the points according to `dimension`. For example, the pipeline below will create a file for each PointSourceId-Classification combination in `input.las`. The first output will contain the minimum source and class, and the final output will contain the maximum source and class.

```json
[
    "input.las",
    {
        "type": "filters.sort",
        "dimension": "PointSourceId,Classification"
    },
    {
        "type": "filters.groupby",
        "dimension": "PointSourceId,Classification"
    },
    "output_#.las"
]
```

## Options

dimension

: A list of 1 or more dimensions by which to group points.

```{include} filter_opts.md
```
