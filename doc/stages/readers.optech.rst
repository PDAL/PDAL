.. _readers.optech:

readers.optech
==============

The **Optech reader** reads Corrected Sensor Data (.csd) files.  These files
contain scan angles, ranges, IMU and GNSS information, and boresight
calibration values, all of which are combined in the reader into XYZ points
using the WGS84 reference frame.


.. embed::

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.optech",
          "filename":"input.csd"
        },
        {
          "type":"writers.text",
          "filename":"outputfile.txt"
        }
      ]
    }


Options
-------

filename
  csd file to read [Required]

.. include:: reader_opts.rst
