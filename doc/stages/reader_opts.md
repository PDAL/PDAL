count

: Maximum number of points to read. \[Default: unlimited\]

override_srs

: Spatial reference to apply to the data.  Overrides any SRS in the input
  itself. Can be specified as a [WKT](https://www.ogc.org/standard/wkt-crs/),
  [PROJ](https://proj.org) or [EPSG](https://spatialreference.org) string.
  Can't use with 'default_srs'. \[Default: none\]

default_srs

: Spatial reference to apply to the data if the input does not specify
  one. Can be specified as a [WKT](https://www.ogc.org/standard/wkt-crs/),
  [PROJ](https://proj.org) or [EPSG](https://spatialreference.org) string.
  Can't use with 'override_srs'. \[Default: none\]
