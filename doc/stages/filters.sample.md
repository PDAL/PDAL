(filters.sample)=

# filters.sample

The **Sample Filter** performs Poisson sampling of the input `PointView`. The
practice of performing Poisson sampling via "Dart Throwing" was introduced
in the mid-1980's by {cite:p}`cook1986stochastic` and {cite:p}`dippe1985antialiasing`, and has been applied to
point clouds in other software {cite:p}`cite_mesh2009`.

Our implementation of Poisson sampling is made streamable by voxelizing the
space and only adding points to the output `PointView` if they do not violate
the minimum distance criterion (as specified by `radius`). The voxelization
allows several optimizations, first by checking for existing points within the
same voxel as the point under consideration, which are mostly likely to
violate the minimum distance criterion. Furthermore, we can easily visit
neighboring voxels (limiting the search to those that are populated) without
the need to create a KD-tree from the entire input `PointView` first and
performing costly spatial searches.

```{seealso}
{ref}`filters.decimation`, {ref}`filters.fps`,
{ref}`filters.relaxationdartthrowing`,
{ref}`filters.voxelcenternearestneighbor`,
{ref}`filters.voxelcentroidnearestneighbor`, and {ref}`filters.voxeldownsize` also
perform decimation.
```

```{note}
Starting with PDAL v2.3, the `filters.sample` now supports streaming
mode. As a result, there is no longer an option to `shuffle` points (or
to provide a `seed` for the shuffle).
```

```{note}
Starting with PDAL v2.3, a `cell` option has been added that works with
the existing `radius`. The user must provide one or the other, but not
both. The provided option will be used to automatically compute the other.
The relationship between `cell` and `radius` is such that the
`radius` defines the radius of a sphere that circumscribes a voxel with
edge length defined by `cell`.
```

```{note}
Care must be taken with selection of the `cell`/`radius` option.
Although the filter can now operate in streaming mode, if the extents of
the point cloud are large (or conversely, if the cell size is small) the
voxel occupancy map which grows as a function of these variables can still
require a large memory footprint.
```

```{note}
To operate in streaming mode, the filter will typically retain the first
point to occupy a voxel (subject to the minimum distance criterion set
forth earlier). This means that point ordering matters, and in fact, it is
quite possible that points in the incoming stream can be ordered in such a
way as to introduce undesirable artifacts (e.g., related to previous tiling
of the data). In our experience, processing data that is still in scan
order (ordered by GpsTime, if available) does produce reliable results,
although to require this sort either internally or by inserting
{ref}`filters.sort` prior to sampling would break our ability to stream the
data.
```

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Options

cell

: Voxel cell size. If `radius` is set, `cell` is automatically computed
  such that the cell is circumscribed by the sphere defined by `radius`.

dimension

: Instead of culling points, create a new `uint8_t` dimension with this name and
  write a `1` if the point was sampled and a `0` if it was not sampled.

origin_x

: X origin of the voxelization for sampling.  \[Default: X of first point\]

origin_y

: Y origin of the voxelization for sampling.  \[Default: Y of first point\]

origin_z

: Z origin of the voxelization for sampling.  \[Default: Z of first point\]

radius

: Minimum distance between samples. If `cell` is set, `radius` is
  automatically computed to defined a sphere that circumscribes the voxel cell.
  Whether specified or derived, `radius` defines the minimum allowable
  distance between points.

```{include} filter_opts.md
```
