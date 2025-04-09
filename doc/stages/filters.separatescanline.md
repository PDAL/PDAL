(filters.separatescanline)=

# filters.separatescanline

The **Separate scan line Filter** takes a single `PointView` as its input and
creates a `PointView` for each scan line (or group of san lines) as its output. 
`PointView` must contain the `EdgeOfFlightLine` dimension. A scan lines is defined 
as the set of returns between two returns marked as EdgeOfFlightLine. Scan lines are
perpendicular to the direction of the aircraft's travel in typical ALS data collection. 
This filter will likely produce a large number of output files if `groupby` is small.

```{eval-rst}
.. embed::
```

## Example

The following pipeline will create a set of text files, where each file contains
only 10 scan lines.

```json
[
    "input.text",
    {
        "type":"filters.separatescanline",
        "groupby":10
    },
    "output_#.text"
]
```

## Options

groupby

: The number of lines to be grouped by. \[Default : 1\]

```{include} filter_opts.md
```
