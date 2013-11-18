.. _drivers.qfit.reader:

drivers.qfit.reader
===================

The **QFIT reader** read from files in the `QFIT format`_ originated for the Airborne Topographic Mapper (ATM) project at NASA Goddard Space Flight Center.


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="drivers.las.writer">
      <Option name="filename">output.las</Option>
      <Reader type="drivers.qfit.reader">
        <Option name="filename">
          qfitfile.qi
        </Option>
        <Option name="flip_coordinates">
          false
        </Option>
        <Option name="scale_z">
          1.0
        </Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  File to read from [Required]

flip_coordinates
  Flip coordinates from 0-360 to -180-180 [Default: **true**] 

scale_z
  Z scale. Use 0.001 to go from mm to m. [Default: **1**] 
  
little_endian
  Are data in little endian format? This should be automatically detected by the driver.


.. _QFIT format: http://nsidc.org/data/docs/daac/icebridge/ilatm1b/docs/ReadMe.qfit.txt


