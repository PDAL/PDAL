(filters.projpipeline)=

# filters.projpipeline

The projpipeline filter applies a coordinates transformation pipeline. The pipeline could be specified as PROJ string (single step operation or multiple step string starting with +proj=pipeline), a WKT2 string describing a CoordinateOperation, or a `<urn:ogc:def:coordinateOperation:EPSG::XXXX>` URN.

```{note}
The projpipeline filter does not consider any spatial reference information.
However user could specify an output srs, but no check is done to ensure
the compliance with the provided transformation pipeline.
```

```{note}
The projpipeline filter is enabled if the version of GDAL is superior or equal to 3.0
```

```{eval-rst}
.. streamable::
```

## Example

This example shift point on the z-axis.

```json
[
    "untransformed.las",
    {
        "type":"filters.projpipeline",
        "coord_op":"+proj=affine +zoff=100"
    },
    {
        "type":"writers.las",
        "filename":"transformed.las"
    }
]
```

This example apply a shift on the z-axis then reproject from utm 10
to WGS84, using the `reverse_transfo` flag. It also set the output srs

```json
[
    "utm10.las",
    {
        "type":"filters.projpipeline",
        "coord_op":"+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad +step +proj=utm +zone=10 +step +proj=affine +zoff=100",
        "reverse_transfo": "true",
        "out_srs": "EPSG:4326"
    },
    {
        "type":"writers.las",
        "filename":"wgs84.las"
    }
]
```

```{note}
PDAL use the GDAL `OGRCoordinateTransformation` class to transform coordinates.
By default output angular unit are in radians. To change to degrees we need to
apply a unit conversion step.
```

## Options

coord_op

: The coordinate operation string.
  Could be specified as PROJ string (single step operation or
  multiple step string starting with +proj=pipeline),
  a WKT2 string describing a CoordinateOperation,
  or a “<urn:ogc:def:coordinateOperation:EPSG::XXXX>” URN.

reverse_transfo

: Boolean, Whether the coordinate operation should be evaluated
  in the reverse path \[Default: false\]

out_srs

: The spatial reference system of the file to be written.
  Can be an EPSG string (e.g. “EPSG:26910”) or a WKT string.
  No check is done to ensure the compliance with the specified coordinate
  operation \[Default: Not set\]

```{include} filter_opts.md
```
