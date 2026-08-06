(writers.arrow)=

# writers.arrow

The **Arrow Writer** supports writing to [Apache Arrow] [Feather]
and [Parquet] file types.

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
        "format":"feather",
        "filename":"outputfile.feather"
    }
]
```

```json
[
    {
        "type":"readers.las",
        "filename":"inputfile.las"
    },
    {
        "type":"writers.arrow",
        "format":"parquet",
        "geoparquet":"true",
        "filename":"outputfile.parquet"
    }
]
```

## Options

batch_size

: Number of rows to write as a batch \[Default: 65536\*4 \]

filename

: Output file to write \[Required\]

format

: File type to write (feather, parquet) \[Default: "feather"\]

geoarrow_dimension_name

: Dimension name to write GeoArrow struct \[Default: xyz\]. Only
  applies to Feather output. Parquet output always uses a `wkb`
  column (WKB point geometry) to satisfy the GeoParquet geometry
  requirement; the packed GeoArrow struct is not written since it
  would be redundant.

geoparquet

: Write WKB column and GeoParquet metadata when writing parquet output

write_pipeline_metadata

: Write PDAL pipeline metadata into `PDAL:pipeline:metadata` of
  `geoarrow_dimension_name`

```{include} writer_opts.md
```

[apache arrow]: https://arrow.apache.org/
[feather]: https://arrow.apache.org/docs/python/feather.html
[parquet]: https://arrow.apache.org/docs/cpp/parquet.html
