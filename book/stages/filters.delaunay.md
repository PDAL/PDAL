(filters-delaunay)=

# filters.delaunay

The **Delaunay Filter** creates a triangulated mesh fulfilling the Delaunay
condition from a collection of points.

The filter is implemented using the [delaunator-cpp] library, a C++ port of
the JavaScript [Delaunator] library.

The filter currently only supports 2D Delaunay triangulation, using the `X`
and `Y` dimensions of the point cloud.

```{eval-rst}
.. embed::
```

## Example

```json
[
    "input.las",
    {
        "type": "filters.delaunay"
    },
    {
        "type": "writers.ply",
        "filename": "output.ply",
        "faces": true
    }
]
```

## Options

```{eval-rst}
.. include:: filter_opts.rst
```

[delaunator]: https://github.com/mapbox/delaunator
[delaunator-cpp]: https://github.com/delfrrr/delaunator-cpp
