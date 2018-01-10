.. _readers.rxp:

readers.rxp
===========

The **RXP reader** read from files in the RXP format, the in-house streaming format used by `RIEGL Laser Measurement Systems`_.

.. warning::
   This software has not been developed by RIEGL, and RIEGL will not provide any support for this driver.
   Please do not contact RIEGL with any questions or issues regarding this driver.
   RIEGL is not responsible for damages or other issues that arise from use of this driver.
   This driver has been tested against RiVLib version 1.39 on a Ubuntu 14.04 using gcc43.

.. plugin::

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
Reflectance values are mapped to intensity values using sensible defaults.

.. code-block:: json

    {
      "pipeline":[
        {
          "type": "readers.rxp",
          "filename": "120304_204030.rxp",
          "sync_to_pps": "true",
          "reflectance_as_intensity": "true"
        },
        {
          "type": "writers.las",
          "filename": "outputfile.las",
          "discard_high_return_numbers": "true"
        }
      ]
    }


We set the ``discard_high_return_numbers`` option to ``true`` on the :ref:`writers.las`.
RXP files can contain more returns per shot than is supported by las, and so we need to explicitly tell the las writer to ignore those high return number points.
You could also use :ref:`filters.python` to filter those points earlier in the pipeline.


Options
-------

filename
  File to read from, or rdtp URI for network-accessible scanner. [Required]

rdtp
  Boolean to switch from file-based reading to RDTP-based. [default: false]

sync_to_pps
  If "true", ensure all incoming points have a valid pps timestamp, usually provided by some sort of GPS clock.
  If "false", use the scanner's internal time.
  [default: true]

minimal
  If "true", only write X, Y, Z, and time values to the data stream.
  If "false", write all available values as derived from the rxp file.
  Use this feature to reduce the memory footprint of a PDAL run, if you don't need any values but the points themselves.
  [default: false]

reflectance_as_intensity
  If "true", maps reflectance values onto intensity values using a range from -25dB to 5dB.
  [default: true]

min_reflectance
  The low end of the reflectance-to-intensity map.
  [default: -25.0]

max_reflectance
  The high end of the reflectance-to-intensity map.
  [default: 5.0]

.. _RIEGL Laser Measurement Systems: http://www.riegl.com
.. _RIEGL download pages: http://www.riegl.com/members-area/software-downloads/libraries/
.. _the optional library documentation: http://www.pdal.io/compilation/unix.html#configure-your-optional-libraries
