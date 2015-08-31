.. _readers.mrsid:

readers.mrsid
=============

Implements MrSID 4.0 LiDAR Compressor. It requires the `Lidar_DSDK`_ to be able to
decompress and read data.

Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">outputfile.las</Option>
      <Reader type="readers.mrsid">
        <Option name="filename">myfile.sid</Option>
      </Reader>
    </Writer>
  </Pipeline>


Options
-------

filename
  Filename to read from [Required]


.. _Lidar_DSDK: https://www.lizardtech.com/developer/

.. _NITF: http://en.wikipedia.org/wiki/National_Imagery_Transmission_Format

.. _NITF 2.1: http://www.gwg.nga.mil/ntb/baseline/docs/2500c/index.html

.. _DES segment: http://jitc.fhu.disa.mil/cgi/nitf/registers/desreg.aspx
