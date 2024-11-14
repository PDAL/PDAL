(filters.greedyprojection)=

# filters.greedyprojection

The **Greedy Projection Filter** creates a mesh (triangulation) in
an attempt to reconstruct the surface of an area from a collection of points.

GreedyProjectionTriangulation is an implementation of a greedy triangulation
algorithm for 3D points based on local 2D projections. It assumes locally
smooth
surfaces and relatively smooth transitions between areas with different point
densities.  The algorithm itself is identical to that used in the [PCL]
library.

```{eval-rst}
.. embed::
```

## Example

```json
[
    "input.las",
    {
        "type": "filters.greedyprojection",
        "multiplier": 2,
        "radius": 10
    },
    {
        "type":"writers.ply",
        "faces":true,
        "filename":"output.ply"
    }
]
```

## Options

multiplier

: Nearest neighbor distance multiplier. \[Required\]

radius

: Search radius for neighbors. \[Required\]

num_neighbors

: Number of nearest neighbors to consider. \[Required\]

min_angle

: Minimum angle for created triangles. \[Default: 10 degrees\]

max_angle

: Maximum angle for created triangles. \[Default: 120 degrees\]

eps_angle

: Maximum normal difference angle for triangulation consideration. \[Default: 45 degrees\]

```{include} filter_opts.md
```

[pcl]: https://pcl.readthedocs.io/projects/tutorials/en/master/greedy_projection.html
