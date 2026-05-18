(filters.divider)=

# filters.divider

The **Divider Filter** breaks a point view into a set of smaller point views
based on criteria.  The filter supports three modes – `partition`, `round_robin`, and
`expression`. With `partition` mode, the number of subsets can be specified explicitly.
`round_robin` allows you to specify a maximum point count for each subset. Finally,
`expression` mode breaks up the point view based on a given `expression` and `count`.

Points can be divided into subsets to facilitate output by writers
that support creating multiple output files with a template (LAS and BPF
are notable examples).

```{eval-rst}
.. embed::
```

## `partition` Example

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

## `round_robin` Example

This pipeline will create 10 output files where each consecutive point is
inserted into a new view.

```json
[
    "example.las",
    {
        "type":"filters.divider",
        "mode":"round_robin",
        "count":"10"
    },
    {
        "type":"writers.las",
        "filename":"out_#.las"
    }
]
```

## `expression` Example

This pipeline will create a new view every time a point that satisfies the
expression `UserData == 122` is encountered. Use in combination with
{ref}`filters.sort` to control ordering of points to determine split locations.

```json
[
    "example.las",
    {
        "type":"filters.divider",
        "expression":"UserData == 122"
    },
    {
        "type":"writers.las",
        "filename":"out_#.las"
    }
]
```

## Options

mode

: A mode of `partition` will write sequential points to an output view until
  the view meets its predetermined size. `round_robin` mode will iterate
  through the output views as it writes sequential points. `expression` will
  iterate write sequential points into an output view until the view meets
  the expression and `count` size.
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
