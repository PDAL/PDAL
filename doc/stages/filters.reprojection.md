(filters.reprojection)=

# filters.reprojection

The **reprojection filter** converts the X, Y and/or Z dimensions to a
new spatial
reference system. The old coordinates are replaced by the new ones.
If you want to preserve the old coordinates for future processing, use a
{ref}`filters.ferry` to create copies of the original dimensions before
reprojecting.

```{note}
When coordinates are reprojected, it may significantly change the precision
necessary to represent the values in some output formats.  Make sure
that you're familiar with any scaling necessary for your output format
based on the projection you've used.
```

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example 1

This pipeline reprojects terrain points with Z-values between 0 and 100 by first
applying a range filter and then specifying both the input and output spatial
reference as EPSG-codes. The X and Y dimensions are scaled to allow enough
precision in the output coordinates.

```json
[
    {
        "filename":"input.las",
        "type":"readers.las",
        "spatialreference":"EPSG:26916"
    },
    {
        "type":"filters.range",
        "limits":"Z[0:100],Classification[2:2]"
    },
    {
        "type":"filters.reprojection",
        "in_srs":"EPSG:26916",
        "out_srs":"EPSG:4326"
    },
    {
        "type":"writers.las",
        "scale_x":"0.0000001",
        "scale_y":"0.0000001",
        "scale_z":"0.01",
        "offset_x":"auto",
        "offset_y":"auto",
        "offset_z":"auto",
        "filename":"example-geog.las"
    }
]
```

## Example 2

In some cases it is not possible to use a EPSG-code as a spatial reference.
Instead {{ PROJ }} parameters can be used to define a spatial
reference.  In this example the vertical component of points in a laz file is
converted from geometric (ellipsoidal) heights to orthometric heights by using
the `geoidgrids` parameter from PROJ.  Here we change the vertical datum
from the GRS80 ellipsoid to DVR90, the vertical datum in Denmark. In the
writing stage of the pipeline the spatial reference of the file is set to
EPSG:7416. The last step is needed since PDAL will otherwise reference the
vertical datum as "Unnamed Vertical Datum" in the spatial reference VLR.

```json
[
    "./1km_6135_632.laz",
    {
        "type":"filters.reprojection",
        "in_srs":"EPSG:25832",
        "out_srs":"+init=epsg:25832 +geoidgrids=C:/data/geoids/dvr90.gtx"
    },
    {
        "type":"writers.las",
        "a_srs":"EPSG:7416",
        "filename":"1km_6135_632_DVR90.laz"
    }
]
```

## Options

in_srs

: Spatial reference system of the input data. Express as an EPSG string (eg
  "EPSG:4326" for WGS84 geographic), PROJ string or a well-known text
  string. \[Required if not part of the input data set\]

out_srs

: Spatial reference system of the output data. Express as an EPSG string (eg
  "EPSG:4326" for WGS84 geographic), PROJ string or a well-known text
  string. \[Required\]

in_axis_ordering

: An array of numbers that override the axis order for the in_srs (or if
  not specified, the inferred SRS from the previous Stage). "2, 1" for
  example would swap X and Y, which may be commonly needed for
  something like "EPSG:4326".

in_coord_epoch

: Coordinate epoch for the input coordinate system as a double. \[Default: 0\]

out_axis_ordering

: An array of numbers that override the axis order for the out_srs.
  "2, 1" for example would swap X and Y, which may be commonly needed for
  something like "EPSG:4326".

out_coord_epoch

: Coordinate epoch for the output coordinate system as a double. \[Default: 0\]

error_on_failure

: If true and reprojection of any point fails, throw an exception that terminates
  PDAL . \[Default: false\]
