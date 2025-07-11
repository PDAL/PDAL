(filters.supervoxel)=

# filters.supervoxel

The filter performs an unsupervised over-segmentation of the input with the aim
 of preserving boundries. The approach is outlined in {cite:p}`lin2018supervoxel`.
The filter adds a new dimension ClusterID that contains the supervoxel ID. The
ClusterID starts at zero.

```{figure} filters.supervoxel.img1.jpg
:alt: Points after supervoxel segmentation

Points after supervoxel segmentation
```

```{eval-rst}
.. embed::
```

## Example

The sample pipeline below uses CSF to segment ground and non-ground returns,
using default options, and writing only the ground returns to the output file.

```json
[
    "input.las",
    {
        "type": "filters.normal",
        "knn": 16
    },
    {
        "type": "filters.supervoxel"
    },
    {
        "type":"writers.las",
        "filename": "output.laz"
        "minor_version": 4,
        "extra_dims": "ClusterID=uint64"
    }
]
```

## Options

knn

: Neighbours to consider. \[Default: 32\]

resolution

: Resolution. This is used to estimate the number of clusters. \[Default: 1.0\]

```{include} filter_opts.md
```
