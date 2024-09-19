(filters.expression)=

# filters.expression

The **Expression Filter** applies filtering to the input point cloud
based on a set of criteria on the given dimensions.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example

This example passes through all points whose `Z` value is in the
range \[0,100\]
and whose `Classification` equals 2 (corresponding to ground in LAS).

```json
[
    "input.las",
    {
        "type":"filters.expression",
        "expression":"(Z >= 0 && Z <= 100) && Classifcation == 2"
    },
    {
        "type":"writers.las",
        "filename":"filtered.las"
    }
]
```

The equivalent pipeline invoked via the PDAL `translate` command would be

```bash
$ pdal translate -i input.las -o filtered.las -f range --filters.expression.expression="(Z >= 0 && Z <= 100) && Classifcation == 2"
```

## Options

expression

: An {ref}`expression <PDAL expression>` that limits points passed to a filter.
