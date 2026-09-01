(writers.arrow)=

# writers.arrow

The **Arrow Writer** supports writing to [Apache Arrow] [GeoParquet] files.

Points are always written as WKB. The `geoparquet_version` option controls which
[GeoParquet version] the file is written as; version `2.0.0` allows the geometry
field to be created as a Parquet Geometry or Geography [logical type]. 
GeoParquet's `geo` metadata is always written, regardless of version.

```{eval-rst}
.. plugin::
```

```{eval-rst}
.. streamable::


```

## Example

```json
[
    {
        "type":"readers.las",
        "filename":"inputfile.las"
    },
    {
        "type":"writers.arrow",
        "geometry_name":"wkb",
        "geoparquet_version":"1.1.0",
        "filename":"outputfile.parquet"
    }
]
```

## Options

batch_size

: Number of rows to write as a batch. Also sets the Parquet row group size. 
\[Default: 65536\*4 \]

filename

: Output file to write \[Required\]

geometry_name

: Column name in which WKB points will be written \[Default: geometry\]. Always
  written as WKB, rather than GeoArrow XYZ structs (supported in GeoParquet 1.1.0).

geoparquet_version

: GeoParquet version to write; supports '1.0.0', '1.1.0' and '2.0.0'. 
  \[Default: 1.1.0\]

write_pipeline_metadata

: Write PDAL pipeline metadata into a `PDAL:pipeline:metadata` key in file-level
  key-value metadata. \[Default: true\]

```{include} writer_opts.md
```

[apache arrow]: https://arrow.apache.org/
[parquet]: https://arrow.apache.org/docs/cpp/parquet.html
[geoparquet]: https://geoparquet.org/
[geoparquet version]: https://geoparquet.org/releases/
[logical type]: https://parquet.apache.org/docs/file-format/types/geospatial/
