(filters.griddecimation)=

# filters.griddecimation

The **grid decimation filter** transform only one point in each cells of a grid calculated from the points cloud and a resolution therm. The transformation is done by the value information. The selected point could be the highest or the lowest point on the cell. It can be used, for exemple, to quickly filter vegetation points in order to keep only the canopy points.

```{eval-rst}
.. embed::
```

## Example

This example transform highest points of classification 5 in classification 9, on a grid of 0.75m square.

```json
[
   "file-input.las",
  {
      "type": "filters.gridDecimation",
      "output_type":"max",
      "resolution": "0.75",
      "where":"Classification==5",
      "value":"Classification=9"
  },
  {
        "type":"writers.las",
        "filename":"file-output.las"
  }
]
```

## Options

output_type

: The type of points transform by the value information. The value should be `"max"` for transform the highest point, or `"min"` for the lowest. \[Default: false\]

resolution

: The resolution of the cells in meter. \[Default: 1.\]

value

: A list of {ref}`assignment expressions <assignment_expressions>` to be applied to points.
  The list of values is evaluated in order. \[Default: none\]

```{include} filter_opts.md
```
