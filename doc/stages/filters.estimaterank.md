(filters.estimaterank)=

# filters.estimaterank

The **rank estimation filter** uses singular value decomposition (SVD) to
estimate the rank of a set of points. Point sets with rank 1 correspond
to linear features, while sets with rank 2 correspond to planar features.
Rank 3 corresponds to a full 3D feature. In practice this can be used alone, or
possibly in conjunction with other filters to extract features (e.g.,
buildings, vegetation).

Two parameters are required to estimate rank (though the default values will be
suitable in many cases). First, the [knn] parameter defines the number of
points to consider when computing the SVD and estimated rank. Second, the
[thresh] parameter is used to determine when a singular value shall be
considered non-zero (when the absolute value of the singular value is greater
than the threshold).

The rank estimation is performed on a pointwise basis, meaning for each point
in the input point cloud, we find its [knn] neighbors, compute the SVD, and
estimate rank. The filter creates a new dimension called `Rank`
that can be used downstream of this filter stage in the pipeline. The type of
writer used will determine whether or not the `Rank` dimension itself can be
saved to disk.

```{eval-rst}
.. embed::
```

## Example

This sample pipeline estimates the rank of each point using this filter
and then filters out those points where the rank is three using
{ref}`filters.range`.

```json
[
    "input.las",
    {
        "type":"filters.estimaterank",
        "knn":8,
        "thresh":0.01
    },
    {
        "type":"filters.range",
        "limits":"Rank![3:3]"
    },
    "output.laz"
]
```

## Options

knn

: The number of k-nearest neighbors. \[Default: 8\]

thresh

: The threshold used to identify nonzero singular values. \[Default: 0.01\]

```{include} filter_opts.md
```
