(writers.ogr)=

# writers.ogr

The **OGR Writer** will create files of various [vector formats] as supported
by the OGR library.  PDAL points are generally stored as point geometries in the
output format, though PDAL will create multipoint geometries instead if the
[multicount] option is set to a value greater than 1.
Points can be written with a additional measure value (POINTZM) if [measure_dim]
specifies a valid PDAL dimension, and dimensions can be set as feature
attributes using the [attr_dims] option.

By default, the OGR writer will create ESRI shapefiles.  The particular OGR
driver can be specified with the `ogrdriver` option.

## Example

```json
[
    "inputfile.las",
    {
        "type": "writers.ogr",
        "filename": "outfile.geojson",
        "measure_dim": "Intensity",
        "attr_dims": "Classification"
    }
]
```

## Options

filename

: Output file to write.  The writer will accept a filename containing
  a single placeholder character (`#`).  If input to the writer consists
  of multiple PointViews, each will be written to a separate file, where
  the placeholder will be replaced with an incrementing integer.  If no
  placeholder is found, all PointViews provided to the writer are
  aggregated into a single file for output.  Multiple PointViews are usually
  the result of multiple input files, or using {ref}`filters.splitter`,
  {ref}`filters.chipper` or {ref}`filters.divider`.

  The driver will use the OGR GeoJSON driver if the output filename
  extension is `.geojson`, and the ESRI Shapefile driver if the output
  filename extension is `.shp`.
  If neither extension is recognized, the filename is taken
  to represent a directory in which ESRI Shapefiles are written.  The
  driver can be explicitly specified by using the [ogrdriver] option.

multicount

: If 1, point features will be written.  If greater than 1, specifies the
  number of points to group into a feature with a multipoint geometry.  Not all
  OGR drivers support multipoint geometries. \[Default: 1\]

measure_dim

: If specified, points will be written with an extra data field, the dimension
  of which is specified by this option. Not all output formats support
  measure data. \[Default: None\]

attr_dims

: List of dimensions to write as feature attributes. Separate multiple values
  with `,` or repeat the option. Use `all` to write all dimensions.
  `X`, `Y`, `Z`, and any [measure_dim] are never written as attributes.
  This option is incompatible with the [multicount] option. \[Default: None\]

ogrdriver

: The OGR driver to use for output.  This option overrides any inference made
  about output drivers from [filename].

ogr_options

: List of OGR driver-specific layer creation options, formatted as an
  `OPTION=VALUE` string. Separate multiple values with `,` or repeat the
  option. \[Default: None\]

```{include} writer_opts.md
```

[vector formats]: https://gdal.org/drivers/vector/index.html
