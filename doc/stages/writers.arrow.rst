.. _writers.arrow:

writers.arrow
===========

The **Arrow Writer** supports writing to `Apache Arrow`_ `Feather`_
and `Parquet`_ file types.

.. plugin::

.. streamable::

.. note::



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
          "filename":"outputfile.parquet"
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
          "format":"orc",
          "filename":"outputfile.orc"
      }
  ]

Options
-------

filename
  Output file to write [Required]

format
  File type to write (feather, parquet, orc) [Default: "feather"]

.. include:: writer_opts.rst

.. _Apache Arrow: https://arrow.apache.org/
.. _Feather: https://arrow.apache.org/docs/python/feather.html

