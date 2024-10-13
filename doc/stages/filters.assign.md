(filters.assign)=

# filters.assign

The assign filter allows you set the value of a dimension for all points
to a provided value that pass a range filter.

```{embed}
```

```{streamable}
```

```{note}
The `assignment` and `condition` options are deprecated and may be removed in a
future release.
```



(assignment_expressions)=

# Assignment Expressions

The assignment expression syntax is an expansion on the {ref}`PDAL expression <pdal_expression>` syntax
that provides for assignment of values to points. The generic expression is:

```
"value" : "Dimension = ValueExpression [WHERE ConditionalExpression)]"
```

`Dimension` is the name of a PDAL dimension.

A `ValueExpression` consists of constants, dimension names and mathematical operators
that evaluates to a numeric value.  The supported mathematical operations are addition(`+`),
subtraction(`-`), multiplication(`*`) and division(`\\`).

A {ref}`ConditionalExpression <pdal_expression>` is an optional boolean value that must
evaluate to `true` for the `ValueExpression` to be applied.

```{note}
As of PDAL 2.7.0, assignment to a dimension that does not exist will cause
it to be created. It will always be created with type double, however.
```

# Example 1

```json
[
    "input.las",
    {
        "type": "filters.assign",
        "value" : "Red = Red / 256"
    },
    "output.laz"
]
```

This scales the `Red` value by 1/256. If the input values are in the range 0 - 65535, the output
value will be in the range 0 - 255.

# Example 2

```json
[
    "input.las",
    {
        "type": "filters.assign",
        "value" : [
            "Red = Red * 256",
            "Green = Green * 256",
            "Blue = Blue * 256"
        ]
    },
    "output.laz"
]
```

This scales the values of Red, Green and Blue by 256. If the input values are in the range 0 - 255, the output
value will be in the range 0 - 65535. This can be handy when creating a {ref}`COPC <writers.copc>` file which
(as defined in LAS 1.4) needs color values scaled in that range.

# Example 3

```json
[
    "input.las",
    {
        "type": "filters.assign",
        "value": [
            "Classification = 2 WHERE HeightAboveGround < 5",
            "Classification = 1 WHERE HeightAboveGround >= 5"
        ]
    },
    "output.laz"
]
```

This sets the classification of points to either `Ground` or `Unassigned` depending on the
value of the `HeightAboveGround` dimension.

# Example 4

```json
[
    "input.las",
    {
        "type": "filters.assign",
        "value": [
            "X = 1",
            "X = 2 WHERE X > 10"
        ]
    },
    "output.laz"
]
```

This sets the value of `X` for all points to 1. The second statement is essentially ignored
since the first statement sets the `X` value of all points to 1 and therefore no points
the `ConditionalExpression` of the second statement.

## Options

assignment

: A {ref}`range <ranges>` followed by an assignment of a value (see example).
  Can be specified multiple times.  The assignments are applied sequentially
  to the dimension value as set when the filter began processing. \[Required\]

condition

: A single {ref}`ranges <ranges>` that a point's values must pass in order
  for the assignment to be performed. \[Default: none\] \[Deprecated - use 'value'\]

value

: A list of {ref}`assignment expressions <assignment_expressions>` to be applied to points.
  The list of values is evaluated in order. \[Default: none\]

```{include} filter_opts.md
```
