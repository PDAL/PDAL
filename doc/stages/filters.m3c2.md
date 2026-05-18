(filters.m3c2)=

# filters.m3c2

The **M3C2 filter** uses the Multiscale Model to Model Cloud Comparison (M3C2)
algorithm, introduced in {cite:p}`lague2013`, to find the 3D distance between two 
point clouds. The filter takes three inputs; the first is the 'reference' cloud,
and the second is the 'compared' cloud whose change is being calculated. The third input
is a set of "core points", generally a subset of the reference cloud, for which distance 
metrics are calculated. For example, these could be spaced in a regular grid created by 
{ref}`filters.voxelcenternearestneighbor` to allow for easier rasterization.

M3C2 creates seven new dimensions: `m3c2_distance`, `m3c2_uncertainty`, `m3c2_significant`, 
`m3c2_std_dev1`, `m3c2_std_dev2`, `m3c2_count1` & `m3c2_count2`. The filter always returns 
a single PointView containg core points.

```{note}
For best results, the input point sets can be aligned using a registration algorithm like 
{ref}`filters.icp`.
```

## Examples

```json
[
    "fixed.las",
    "compare.las",
    "cores.las",
    {
        "type": "filters.m3c2",
        "normal_radius": 5,
        "cyl_radius": 10,
        "cyl_halflen": 5
    },
    "output.las"
]
```

```json
[
    {
        "filename": "fixed.las",
        "tag": "A"
    },
    {
        "type": "filters.sample",
        "cell": 5.0,
        "tag": "Sampled"
    },
    {
        "filename": "compare.las",
        "tag": "B"
    },
    {
        "type": "filters.m3c2",
        "normal_radius": 5,
        "cyl_radius": 10,
        "cyl_halflen": 5,
        "inputs": [
            "A",
            "B",
            "Sampled"
        ]
    },
        "output.las"
]
```

## Options

normal_radius

: Radius of the sphere around each core point that defines the neighbors from which normals are 
  calculated. These normals are the orientation of the cylinder described below. \[Default: 2\]

cyl_radius

: Radius of the cylinder inside of which points are searched for when calculating change. \[Default: 2\]

cyl_halflen

: The half-length of the cylinder of neighbors used for calculating change. \[Default: 5\]

reg_error

: Registration error; used for calculating the confidence interval (`m3c2_uncertainty` dimension), 
  where a higher registration error gives higher uncertainty. \[Default: 0\]

orientation

: Which direction to orient the cylinder/normal vector used for comparison between the two point clouds. 
  Can be in the direction of "up" (+Z), "down" (-Z) or "none". \[Default: "up"\]

min_points

: The minimum number of points in a neighborhood to use for calculating statistics. \[Default: 1\]

