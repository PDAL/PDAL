(filters.returns)=

# filters.returns

The **Returns Filter** takes a single PointView as its input and creates a
`PointView` for each of the user-specified [groups] defined below.

"first" is defined as those points whose `ReturnNumber` is 1 when the `NumberOfReturns` is greater than 1.

"intermediate" is defined as those points whose `ReturnNumber` is greater than 1 and less than `NumberOfReturns` when `NumberOfReturns` is greater than 2.

"last" is defined as those points whose `ReturnNumber` is equal to `NumberOfReturns` when `NumberOfReturns` is greater than 1.

"only" is defined as those points whose `NumberOfReturns` is 1.

```{eval-rst}
.. embed::
```

## Example

This example creates two separate output files for the "last" and "only"
returns.

```json
[
    "input.las",
    {
        "type":"filters.returns",
        "groups":"last,only"
    },
    "output_#.las"
]
```

## Options

groups

: Comma-separated list of return number groupings. Valid options are "first",
  "last", "intermediate" or "only". \[Default: "last"\]

```{include} filter_opts.md
```
