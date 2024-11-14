(filters.mad)=

# filters.mad

The **MAD filter** filter crops the input point cloud based on
the distribution of points in the specified [dimension]. Specifically, we choose
the method of median absolute deviation from the median (commonly referred to
as
MAD), which is robust to outliers (as opposed to mean and standard deviation).

```{note}
This method can remove real data, especially ridges and valleys in rugged
terrain, or tall features such as towers and rooftops in flat terrain. While
the number of deviations can be adjusted to account for such content-specific
considerations, it must be used with care.
```

```{eval-rst}
.. embed::
```

## Example

The sample pipeline below uses filters.mad to automatically crop the `Z`
dimension and remove possible outliers. The number of deviations from the
median has been adjusted to be less aggressive and to only crop those outliers
that are greater than four deviations from the median.

```json
[
    "input.las",
    {
        "type":"filters.mad",
        "dimension":"Z",
        "k":4.0
    },
    "output.laz"
]
```

## Options

k

: The number of deviations from the median. \[Default: 2.0\]

dimension

: The name of the dimension to filter.

```{include} filter_opts.md
```
