  .. _readers.mrsid:

readers.mrsid
=============

Implements MrSID 4.0 LiDAR Compressor. It requires the `Lidar_DSDK`_ to be able to
decompress and read data.

.. plugin::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.mrsid",
          "filename":"myfile.sid"
        },
        {
          "type":"writers.las",
          "filename":"outputfile.las"
        }
      ]
    }


Options
-------

filename
  Filename to read from. [Required]

.. include:: reader_opts.rst

.. _Lidar_DSDK: https://www.lizardtech.com/developer/

.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx
