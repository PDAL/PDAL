(filters-expressionstats)=

# filters.expressionstats

The {ref}`filters.expressionstats` stage computes counting summary for a single
dimension for a given set of expressions. This is useful for summarizing dimensions
that are conveniently countable.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

:::{warning}
The `dimension` selected should be an integer, not floating point dimension.
Additionally, a dimension with lots of unique values is likely to generate a
many entries in the map. This may not be what you want.
:::

## Example

```json
{
    "pipeline": [{
        "bounds": "([-10190065.06156413, -10189065.06156413], [5109498.61041016, 5110498.61041016])",
        "filename": "https://s3-us-west-2.amazonaws.com/usgs-lidar-public/IA_Eastern_1_2019/ept.json",
        "requests": "16",
        "type": "readers.ept"
    },
    {
        "type": "filters.stats"
    },
    {
        "type": "filters.expressionstats",
        "dimension":"Classification",
        "expressions":["Withheld == 1", "Keypoint == 1", "Overlap == 1", "Synthetic == 1"]
    },
    {
        "filename": "hobu-office.laz",
        "type": "writers.copc"
    }]
}
```

### Output

```json
{
  "dimension": "Classification",
  "statistic":
  [
    {
      "expression": "(Keypoint==1.000000)",
      "position": 0
    },
    {
      "expression": "(Overlap==1.000000)",
      "position": 1
    },
    {
      "bins":
      [
        {
          "count": 154,
          "value": 1
        }
      ],
      "expression": "(Synthetic==1.000000)",
      "position": 2
    },
    {
      "bins":
      [
        {
          "count": 313615,
          "value": 1
        },
        {
          "count": 6847,
          "value": 7
        },
        {
          "count": 4425,
          "value": 18
        }
      ],
      "expression": "(Withheld==1.000000)",
      "position": 3
    }
  ]
}
```

#### Options

dimension

: The dimension on which to apply the expressions.

expressions

: An array of expressions to apply.

```{eval-rst}
.. include:: filter_opts.rst
```
