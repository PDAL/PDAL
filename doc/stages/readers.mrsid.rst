  .. _readers.mrsid:

readers.mrsid
=============

.. note::

    The MrSID reader is deprecated and will be removed in a future release.

Implements MrSID 4.0 LiDAR Compressor. It requires the `Lidar_DSDK`_ to be able to
decompress and read data.

.. plugin::

Example
-------

.. code-block:: json

  [
      {
          "type":"readers.mrsid",
          "filename":"myfile.sid"
      },
      {
          "type":"writers.las",
          "filename":"outputfile.las"
      }
  ]


Options
-------

filename
  Filename to read from. [Required]

.. include:: reader_opts.rst

.. _Lidar_DSDK: https://www.extensis.com/support/developers

