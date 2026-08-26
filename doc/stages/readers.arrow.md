(readers.arrow)=

# readers.arrow

```{eval-rst}
.. plugin::
```

```{eval-rst}
.. streamable::
```

The Arrow reader supports reading Parquet-formatted data as written by
{ref}`writers.arrow`, although it should support point clouds written by other
writers too if they follow [GeoParquet](https://github.com/opengeospatial/geoparquet/) specification or contain point data encoded in Parquet's native [geospatial types].

## Options

filename

: Parquet or GeoParquet file to read \[Required\]

geometry_column

: Name of the column from which to read Parquet point geometry. Defaults to
  the value of 'primary_column' from GeoParquet's `geo` metadata if the 
  option is not set.

```{include} reader_opts.md
```

[geospatial types]: https://parquet.apache.org/docs/file-format/types/geospatial/