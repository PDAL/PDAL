.. _readers.arrow:

readers.arrow
==============


.. plugin::

.. streamable::

The Arrow reader supports reading Arrow and Parquet -formatted data as written by 
:ref:`writers.arrow`, although it should support point clouds written by other 
writers too if they follow either the `GeoArrow <https://github.com/geoarrow/geoarrow/>`__ 
or `GeoParquet <https://github.com/opengeospatial/geoparquet/>`__ specification. 

Caveats:

* Which schema is read is chosen by the file name extension, but can be 
  overridden with the `format` option set to `geoarrow` or `geoparquet`
* 

Options
-------

filename
  Arrow GeoArrow or GeoParquet file to read [Required]
format
  `geoarrow` or `geoparquet` option to override any filename extension 
  hinting of data type [Optional]

.. include:: reader_opts.rst

