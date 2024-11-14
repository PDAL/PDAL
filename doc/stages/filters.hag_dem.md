(filters.hag_dem)=

# filters.hag_dem

The **Height Above Ground (HAG) Digital Elevation Model (DEM) filter** loads
a GDAL-readable raster image specifying the DEM. The `Z` value of each point
in the input is compared against the value at the corresponding X,Y location
in the DEM raster. It creates a new dimension, `HeightAboveGround`, that
contains the normalized height values.

Normalized heights are a commonly used attribute of point cloud data. This can
also be referred to as *height above ground* (HAG) or *above ground level* (AGL)
heights. In the end, it is simply a measure of a point's relative height as
opposed to its raw elevation value.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example #1

Using the autzen dataset (here shown colored by elevation)

```{image} ./images/autzen-elevation.png
:height: 400px
```

we generate a DEM based on the points already classified as ground

```
$ pdal translate autzen.laz autzen_dem.tif range \
    --filters.range.limits="Classification[2:2]" \
    --writers.gdal.output_type="idw" \
    --writers.gdal.resolution=6 \
    --writers.gdal.window_size=24
```

and execute the following pipeline

```json
[
    "autzen.laz",
    {
        "type":"filters.hag_dem",
        "raster": "autzen_dem.tif"
    },
    {
        "type":"writers.las",
        "filename":"autzen_hag_dem.laz",
        "extra_dims":"HeightAboveGround=float32"
    }
]
```

which is equivalent to the `pdal translate` command

```
$ pdal translate autzen.laz autzen_hag_dem.laz hag_dem \
    --filters.hag_dem.raster=autzen_dem.tif \
    --writers.las.extra_dims="HeightAboveGround=float32"
```

In either case, the result, when colored by the normalized height instead of
elevation is

```{image} ./images/autzen-hag-dem.png
:height: 400px
```

## Options

raster

: GDAL-readable raster to use for DEM.

band

: GDAL Band number to read (count from 1).
  \[Default: 1\]

zero_ground

: If true, set HAG of ground-classified points to 0 rather than comparing
  `Z` value to raster DEM.
  \[Default: true\]

```{include} filter_opts.md
```
