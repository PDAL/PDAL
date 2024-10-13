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

```{note}
By default the groups are ordered according to the order of first occurance within the input. To change this, use `filters.sort` first to order the points according to `dimension`.
```

## Options

dimension

: The dimension containing data to be grouped.

```{include} filter_opts.md
```
