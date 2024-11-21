(filters.divider)=

# filters.divider

The **Divider Filter** breaks a point view into a set of smaller point views
based on simple criteria.  The number of subsets can be specified explicitly,
or one can specify a maximum point count for each subset.  Additionally,
points can be placed into each subset sequentially (as they appear in the
input) or in round-robin fashion.

Normally points are divided into subsets to facilitate output by writers
that support creating multiple output files with a template (LAS and BPF
are notable examples).

```{eval-rst}
.. embed::
```

## Example

This pipeline will create 10 output files from the input file readers.las.

```json
[
    "example.las",
    {
        "type":"filters.divider",
        "count":"10"
    },
    {
        "type":"writers.las",
        "filename":"out_#.las"
    }
]
```

## Options

mode

: A mode of "partition" will write sequential points to an output view until
  the view meets its predetermined size. "round_robin" mode will iterate
  through the output views as it writes sequential points.
  \[Default: "partition"\]

count

: Number of output views.  \[Default: none\]

capacity

: Maximum number of points in each output view.  Views will contain
  approximately equal numbers of points.  \[Default: none\]

```{include} filter_opts.md
```

```{warning}
You must specify exactly one of either [count] or [capacity].
```
