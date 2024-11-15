(filters.radialdensity)=

# filters.radialdensity

The **Radial Density filter** creates a new attribute `RadialDensity` that
contains the density of points in a sphere of given radius.

The density at each point is computed by counting the number of points falling
within a sphere of given [radius] (default is 1.0) and centered at the current
point. The number of neighbors (including the query point) is then normalized
by the volume of the sphere, defined as

$$
V = \frac{4}{3} \pi r^3
$$

The radius $r$ can be adjusted by changing the [radius] option.

```{eval-rst}
.. embed::
```

## Example

```json
[
    "input.las",
    {
        "type":"filters.radialdensity",
        "radius":2.0
    },
    {
        "type":"writers.bpf",
        "filename":"output.bpf",
        "output_dims":"X,Y,Z,RadialDensity"
    }
]
```

## Options

radius

: Radius. \[Default: 1.0\]

```{include} filter_opts.md
```
