(writers.pgpointcloud)=

# writers.pgpointcloud

The **PostgreSQL Pointcloud Writer** allows you to write to PostgreSQL database
that have the [PostgreSQL Pointcloud] extension enabled. The Pointcloud
extension stores point cloud data in tables that contain rows of patches. Each
patch in turn contains a large number of spatially nearby points.

While you can theoretically store the contents of a whole file of points in a
single patch, it is more practical to store a table full of smaller patches,
where the patches are under the PostgreSQL page size (8KB). For most LIDAR
data, this practically means a patch size of between 400 and 600 points.

In order to create patches of the right size, the Pointcloud writer should be
preceded in the pipeline file by {ref}`filters.chipper`.

The pgpointcloud format does not support WKT spatial reference specifications.  A subset of spatial references can be stored by using the 'srid' option, which
allows storage of an [EPSG code] that covers many common spatial references.
PDAL makes no attempt to reproject data to your specified srid.  Use
{ref}`filters.reprojection` for this purpose.

```{eval-rst}
.. plugin::
```

## Example

```json
[
    {
        "type":"readers.las",
        "filename":"inputfile.las",
        "spatialreference":"EPSG:26916"
    },
    {
        "type":"filters.chipper",
        "capacity":400
    },
    {
        "type":"writers.pgpointcloud",
        "connection":"host='localhost' dbname='lidar' user='pramsey'",
        "table":"example",
        "compression":"dimensional",
        "srid":"26916"
    }
]
```

## Options

connection

: PostgreSQL connection string. In the form *"host=hostname dbname=database user=username password=pw port=5432"* \[Required\]

table

: Database table to write to. \[Required\]

schema

: Database schema to write to. \[Default: "public"\]

column

: Table column to put patches into. \[Default: "pa"\]

compression

: Patch compression type to use. \[Default: ""dimensional""\]

  - **none** applies no compression
  - **dimensional** applies dynamic compression to each dimension separately
  - **lazperf** applies a "laz" compression (using the [laz-perf] library in PostgreSQL Pointcloud)

overwrite

: To drop the table before writing set to 'true'. To append to the table
  set to 'false'. \[Default: false\]

srid

: Spatial reference ID (relative to the `spatial_ref_sys` table in PostGIS)
  to store with the point cloud schema. \[Default: 4326\]

pcid

: An optional existing PCID to use for the point cloud schema. If specified,
  the schema must be present. If not specified, a match will still be
  looked for, or a new schema will be inserted. \[Default: 0\]

pre_sql

: SQL to execute *before* running the translation. If the value
  references a file, the file is read and any SQL inside is executed.
  Otherwise the value is executed as SQL itself. \[Optional\]

post_sql

: SQL to execute *after* running the translation. If the value references
  a file, the file is read and any SQL inside is executed. Otherwise the
  value is executed as SQL itself. \[Optional\]

scale_x, scale_y, scale_z / offset_x, offset_y, offset_z

: If ANY of these options are specified the X, Y and Z dimensions are adjusted
  by subtracting the offset and then dividing the values by the specified
  scaling factor before being written as 32-bit integers (as opposed to double
  precision values).  If any of these options is specified, unspecified
  scale\_\<x,y,x> options are given the value of 1.0 and unspecified
  offset\_\<x,y,z> are given the value of 0.0.

output_dims

: If specified, limits the dimensions written for each point.  Dimensions
  are listed by name and separated by commas.

```{include} writer_opts.md
```

[epsg code]: https://www.iogp.org/bookstore/product/epsg-geodetic-parameter-relational-database-developers-guide/
[laz-perf]: https://github.com/hobu/laz-perf
[postgresql pointcloud]: http://github.com/pramsey/pointcloud
