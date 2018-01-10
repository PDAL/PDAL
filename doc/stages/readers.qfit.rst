.. _readers.qfit:

******************************************************************************
readers.qfit
******************************************************************************

The **QFIT reader** read from files in the `QFIT format`_ originated for the
Airborne Topographic Mapper (ATM) project at NASA Goddard Space Flight Center.

.. embed::


Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.qfit",
          "filename":"inputfile.qi",
          "flip_coordinates":"false",
          "scale_z":"1.0"
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
  File to read from [Required]

flip_coordinates
  Flip coordinates from 0-360 to -180-180 [Default: **true**]

scale_z
  Z scale. Use 0.001 to go from mm to m. [Default: **1**]

little_endian
  Are data in little endian format? This should be automatically detected by the driver.

count 
  Maximum number of points to read [Optional]

.. _QFIT format: http://nsidc.org/data/docs/daac/icebridge/ilatm1b/docs/ReadMe.qfit.txt


