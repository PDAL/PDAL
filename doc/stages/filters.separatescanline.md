(filters.separatescanline)=

# filters.separatescanline

The **Separate scan line Filter** takes a single `PointView` as its input and
creates a `PointView` for each scan line as its output. `PointView` must contain
the `EdgeOfFlightLine` dimension.

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
