.. _readers.rxp:

readers.rxp
===========

The **RXP reader** read from files in the RXP format, the in-house streaming format used by `RIEGL Laser Measurement Systems`_.

.. warning::
   This software has not been developed by RIEGL, and RIEGL will not provide any support for this driver.
   Please do not contact RIEGL with any questions or issues regarding this driver.
   RIEGL is not responsible for damages or other issues that arise from use of this driver.
   This driver has been tested against RiVLib version 1.39 on a Ubuntu 14.04 using gcc43.


Installation
------------

To build PDAL with rxp support, set RiVLib_DIR to the path of your local RiVLib installation.
RiVLib can be obtained from the `RIEGL download pages`_ with a properly enabled user account.
The RiVLib files do not need to be in a system-level directory, though they could be (e.g. they could be in ``/usr/local``, or just in your home directory somewhere).
For help building PDAL with optional libraries, see `the optional library documentation`_.


Example
-------

This example rescales the points, given in the scanner's own coordinate system, to values that can be written to a las file.
Only points with a valid gps time, as determined by a pps pulse, are read from the rxp, since the ``sync_to_pps`` option is "true".

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <Pipeline version="1.0">
        <Writer type="writers.las">
            <Option name="filename">output.las</Option>
            <Option name="discard_high_return_numbers">true</Option>
            <Filter type="filters.scaling">
                <Option name="dimension">
                    X
                    <Options>
                        <Option name="scale">0.001</Option>
                        <Option name="offset">0</Option>
                    </Options>
                </Option>
                <Option name="dimension">
                    Y
                    <Options>
                        <Option name="scale">0.001</Option>
                        <Option name="offset">0</Option>
                    </Options>
                </Option>
                <Option name="dimension">
                    Z
                    <Options>
                        <Option name="scale">0.001</Option>
                        <Option name="offset">0</Option>
                    </Options>
                </Option>
                <Filter type="filters.programmable">
                    <Option name="source">
    import numpy
    def reflectance_to_intensity(ins, outs):
        ref = ins["Reflectance"]
        min = numpy.amin(ref)
        max = numpy.amax(ref)
        outs["Intensity"] = (65535 * (ref - min) / (max - min)).astype(numpy.uint16)
        return True
                    </Option>
                    <Option name="function">reflectance_to_intensity</Option>
                    <Option name="module">pyrxp</Option>
                    <Option name="add_dimension">Intensity</Option>
                    <Reader type="readers.rxp">
                        <Option name="filename">120304_204030.rxp</Option>
                        <Option name="sync_to_pps">true</Option>
                    </Reader>
                </Filter>
            </Filter>
        </Writer>
    </Pipeline>

A few things to note:

- We use a ``filters.programmable`` to remap Reflectance values to Intensity values, scaling them so the entire range of Reflectance values fit into the Intensity field.
  This is analogous to the "Export reflectance as intensity" option in RiSCAN Pro.
  You could also explicitly set the minimum and maximum Reflectance values as you would in RiSCAN Pro using the same programmable filter.
  You could also use "Amplitude" instead of "Reflectance".
  If you do not need Intensity values in your output file, you can delete the programmable filter.
- We set the ``discard_high_return_numbers`` option to ``true`` on the las writer.
  RXP files can contain more returns per shot than is supported by las, and so we need to explicitly tell the las writer to ignore those high return number points.
  You could also use ``filters.predicate`` to filter those points earlier in the pipeline, or modify the return values with a ``filters.programmable``.


Options
-------

filename
  File to read from [Required if rdtp is not provided]

rdtp
  URI for a network-assessable scanner [Required if filename is not provided]

sync_to_pps
  If "true", ensure all incoming points have a valid pps timestamp, usually provided by some sort of GPS clock.
  If "false", use the scanner's internal time.
  Defaults to "true"

minimal
  If "true", only write X, Y, Z, and time values to the data stream.
  If "false", write all available values as derived from the rxp file.
  Use this feature to reduce the memory footprint of a PDAL run, if you don't need any values but the points themselves.
  Defaults to "false".

inclination_fix
  *EXPERIMENTAL*: If "true", use inclination values in the rxp file to dynamically correct for inclination changes throughout the scan, using a moving average of 2 * ``inclination_fix_window`` inclination readings (see below).
  This is an experimental feature that will remove some points from the data stream and modify many others.
  Use with caution.
  If "false", disable this feature.
  Defaults to "false".

inclination_fix_window
  *EXPERIMENTAL*: Sets the half-size of the inclination fix window (see above).
  Use of this feature should be considered highly experimental.


.. _RIEGL Laser Measurement Systems: http://www.riegl.com
.. _RIEGL download pages: http://www.riegl.com/members-area/software-downloads/libraries/
.. _the optional library documentation: http://www.pdal.io/compilation/unix.html#configure-your-optional-libraries
