(filters.crop)=

# filters.crop

The **crop filter** removes points that fall outside or inside a
cropping bounding
box (2D or 3D), polygon, or point+distance.  If more than one bounding region is
specified, the filter will pass all input points through each bounding region,
creating an output point set for each input crop region.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

The provided bounding regions are assumed to have the same spatial reference
as the points unless the option [a_srs] provides an explicit spatial reference
for bounding regions.
If the point input consists of multiple point views with differing
spatial references, one is chosen at random and assumed to be the
spatial reference of the input bounding region.  In this case a warning will
be logged.

## Example 1

This example crops an input point cloud using a square polygon.

```json
[
    "file-input.las",
    {
        "type":"filters.crop",
        "bounds":"([0,1000000],[0,1000000])"
    },
    {
        "type":"writers.las",
        "filename":"file-cropped.las"
    }
]
```

## Example 2

This example crops all points more than 500 units in any direction from a point.

```json
[
    "file-input.las",
    {
        "type":"filters.crop",
        "point":"POINT(0 0 0)",
        "distance": 500
    },
    {
        "type":"writers.las",
        "filename":"file-cropped.las"
    }
]
```

## Options

bounds

: The extent of the clipping rectangle in the format
  `"([xmin, xmax], [ymin, ymax])"`.  This option can be specified more than
  once by placing values in an array.

  ```{note}
  3D bounds can be given in the form `([xmin, xmax], [ymin, ymax], [zmin, zmax])`.
  ```

  ```{warning}
  If a 3D bounds is given to the filter, a 3D crop will be attempted, even
  if the Z values are invalid or inconsistent with the data.
  ```

polygon

: The clipping polygon, expressed in a well-known text string,
  eg: `"POLYGON((0 0, 5000 10000, 10000 0, 0 0))"`.  This option can be
  specified more than once by placing values in an array.

outside

: Invert the cropping logic and only take points outside the cropping
  bounds or polygon. \[Default: false\]

point

: An array of WKT or GeoJSON 2D or 3D points (eg: `"POINT(0 0 0)"`). Requires [distance].

distance

: Distance (radius) in units of common X, Y, and Z {ref}`dimensions` in combination with [point]. Passing a 2D point will crop using a circle. Passing a 3D point will crop using a sphere.

a_srs

: Indicates the spatial reference of the bounding regions.  If not provided,
  it is assumed that the spatial reference of the bounding region matches
  that of the points.

```{include} filter_opts.md
```

## Notes

1. See {ref}`workshop-clipping`: and {ref}`clipping` for example usage scenarios for {ref}`filters.crop`.
