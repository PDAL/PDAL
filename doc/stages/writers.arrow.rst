.. _writers.arrow:

writers.arrow
===============

The **Arrow Writer** supports writing to `Apache Arrow`_ `Feather`_
and `Parquet`_ file types.

.. plugin::

.. streamable::



Example
-------

.. code-block:: json

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

.. code-block:: json

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

Options
-------

batch_size
  Number of rows to write as a batch [Default: 65536*4 ]

filename
  Output file to write [Required]
format
  File type to write (feather, parquet) [Default: "feather"]

geoarrow_dimension_name
  Dimension name to write GeoArrow struct [Default: xyz]

geoparquet
  Write WKB column and GeoParquet metadata when writing parquet output

write_pipeline_metadata
  Write PDAL pipeline metadata into `PDAL:pipeline:metadata` of
  `geoarrow_dimension_name`

.. include:: writer_opts.rst

.. _Apache Arrow: https://arrow.apache.org/
.. _Feather: https://arrow.apache.org/docs/python/feather.html
.. _Parquet: https://arrow.apache.org/docs/cpp/parquet.html

