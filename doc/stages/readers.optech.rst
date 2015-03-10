.. _readers.optech:

readers.optech
==============

The **Optech reader** reads Corrected Sensor Data (.csd) files.
These files contain scan angles, ranges, IMU and GNSS information, and boresight calibration values, all of which are combined in the reader into XYZ points using the WGS84 reference frame.


Example
-------

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.text">
      <Option name="filename">outputfile.txt</Option>
      <Reader type="readers.optech">
        <Option name="filename">inputfile.csd</Option>
      </Reader>
    </Writer>
  </Pipeline>

Options
-------

filename
  csd file to read [Required] 
