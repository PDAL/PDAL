(filters.separateflightline)=

# filters.separateflightline

The **separate flight line filter** takes a single `PointView` as its input and
creates a new `PointView` for each pass of an aircraft based on `GpsTime`. 
`PointView` must contain the `GpsTime` dimension. If two sequential returns are
separated by a difference in `GpsTime` greater than the threshold `time_gap`, it
is assumed that the returns came from different flight lines. Points must first
be sorted by ascending `GpsTime`.

```{eval-rst}
.. embed::
```

## Example

```json
[
    "input.las",
    {
        "type": "filters.sort",
        "dimension": "GpsTime",
        "order": "ASC"
    },
    {
        "type":"filters.separateflightline",
        "time_gap":5
    },
    "output_#.las"
]
```

## Options

time_gap

: Minimum amount of GpsTime that separates two flight lines. \[Default : 5.0\]

```{include} filter_opts.md
```
