(filters.trajectory)=

# filters.trajectory

The **trajectory filter** computes an estimate the the sensor location based
on the position of multiple returns and the sensor scan angle. It is primarily
useful for LAS input as it requires scan angle and return counts in order to
work. Points must be sorted by `GpsTime` and should be separated by independent 
flight lines (trajectories) before applying this filter.

The method is described in detail [here]. It extends the method of {cite}`Gatziolis2019`.

```{note}
This filter creates a new dataset describing the trajectory of the sensor,
replacing the input dataset.
```

## Examples

```json
[
    "input.las",
    {
        "type": "filters.sort",
        "dimension": "GpsTime",
        "order": "ASC"
    },
    {
        "type": "filters.separateflightline"
    },
    {
        "type": "filters.trajectory"
    },
    "trajectories.las"
]
```

## Options

dtr

: Multi-return sampling interval in seconds. \[Default: .001\]

dst

: Single-return sampling interval in seconds. \[Default: .001\]

minsep

: Minimum separation of returns considered in meters. \[Default: .01\]

tblock

: Block size for cublic spline in seconds. \[Default: 1.0\]

tout

: Output data interval in seconds. \[Default: .01\]

```{include} filter_opts.md
```

[here]: ../papers/lidar-traj.pdf
