(stacindex_command)=

# stacindex

```{eval-rst}
.. plugin::
```

The `stacindex` command is an adaptation of the PDAL {ref}`tindex <tindex_command>`
program, which creates a spatial index file for a set of point cloud files. 
`stacindex` creates a file that is conformant with the [STAC-Geoparquet] 
specification, similar to the [gdal raster index] `--profile=STAC-GeoParquet` option.

It shares most of its options with {ref}`tindex <tindex_command>`, but overrides
the `--ogrdriver` (always `Parquet`) and `--t_srs` (always `EPSG:4326`) options.

This program requires the PDAL Arrow plugin, and your GDAL installation must be 
built with Arrow/Parquet support to build an index with the Parquet driver.

## Creation mode

```
/** STAC-GeoParquet creation **/
--statistics           Write detailed statistics on all points
--pc_type              Pointcloud type for STAC generation (lidar, eopc, radar, 
                       sonar, other)
--static_fields        JSON array of field names and values to include "
                       in tile index; specified as { \"fieldName\": value, ...}

/** Shared options with `pdal tindex` **/
--tindex               OGR-readable/writeable tile index output
--glob                 Glob pattern of files to index
--stdin, -s            Read list of input files from standard input
--filelist             Text file to read list of input files from (newline delimited)
--lyr_name             OGR layer name to write into datasource
--tindex_name          Tile index column name
--lco                  Driver-specific NAME=VALUE OGR layer creation options
--a_srs                Assign SRS of tile with no SRS to this value
--write_absolute_path  Write absolute rather than relative file paths
--skip_different_srs   Reject files to be indexed with different SRS values
--threads              Number of threads to use for file boundary creation

/** Boundary Generation **/
--fast_boundary        Use extent instead of exact boundary
--simplify             Simplify the file's exact boundary
--threshold            Number of points a cell must contain to be declared 
                       positive space, when creating exact boundaries
--resolution           Cell edge length to be used when creating exact boundaries
--sample_size          Sample size for auto-edge length calculation in internal
                       hexbin filter (exact boundary)
--where                Expression describing points to be processed for exact
                       boundary creation
```

The `--statistics` option controls whether `pc:statistics` and `proj:bbox` fields
are written to the output. Since these require {ref}`filters.stats` to be run
(reading all points), index creation may be slower.

Additional fields other than those created by default, containing static values, 
can be written using the `--static_fields` option. By specifying a JSON object 
containing pairs of `"fieldName": value` to write, values can be interpreted as 
`String`, `StringList`, `Integer64` or `Real` OGR field types. 

For example, the STAC asset media type for an index composed of LAZ files can be 
specified as follows:

```
$ pdal tindex create <tindex> <glob> \
    --stac-geoparquet \
    --static_fields={\"assets.data.type\":\"application/vnd.laszip\"}
```

[gdal raster index]: https://gdal.org/en/latest/programs/gdal_raster_index.html
[STAC-Geoparquet] https://radiantearth.github.io/stac-geoparquet-spec/latest/