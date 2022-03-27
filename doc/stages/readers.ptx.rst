.. _readers.ptx:

readers.ptx
============

The **PTX reader** reads data from `Leica Cyclone PTX`_ files. It infers
dimensions from points stored in a text file.

.. note::

   PTX files can contain multiple point clouds stored in a single
   file.  If that is the case, the reader will read all the points
   from all of the internal point clouds as one.

.. embed::


Example Pipeline
----------------

.. code-block:: json

  [
      {
          "type":"readers.ptx",
          "filename":"test.ptx"
      },
      {
          "type":"writers.text",
          "filename":"outputfile.txt"
      }
  ]

Options
-------

filename
  File to read. [Required]

.. include:: reader_opts.rst

_`discard_empty`
  Discard empty input points. [Default: true]

.. _Leica Cyclone PTX: http://paulbourke.net/dataformats/ptx/