(filters.stats)=

# filters.stats

The **Stats Filter** calculates the minimum, maximum and average (mean) values
of dimensions.  On request it will also provide an enumeration of values of
a dimension and skewness and kurtosis.

The output of the stats filter is metadata that can be stored by writers or
used through the PDAL API.  Output from the stats filter can also be
quickly obtained in JSON format by using the command "pdal info --stats".

```{note}
The filter can compute both sample and population statistics.  For kurtosis,
the filter can also compute standard and excess kurtosis.  However, only
a single value is reported for each statistic type in metadata, and that is
the sample statistic, rather than the population statistic.  For kurtosis
the sample excess kurtosis is reported.  This seems to match the behavior
of many other software packages.
```

## Example

```json
[
    "input.las",
    {
        "type":"filters.stats",
        "dimensions":"X,Y,Z,Classification",
        "enumerate":"Classification"
    },
    {
        "type":"writers.las",
        "filename":"output.las"
    }
]
```

### Options

(stats-dimensions)=

dimensions

: A comma-separated list of dimensions whose statistics should be
  processed.  If not provided, statistics for all dimensions are calculated.

enumerate

: A comma-separated list of dimensions whose values should be enumerated.
  Note that this list does not add to the list of dimensions that may be
  provided in the {ref}`dimensions <stats-dimensions>` option.

count

: Identical to the [enumerate] option, but provides a count of the number
  of points in each enumerated category.

global

: A comma-separated list of dimensions for which global statistics (median,
  mad, mode) should be calculated.

advanced

: Calculate advanced statistics (skewness, kurtosis). \[Default: false\]

```{include} filter_opts.md
```
