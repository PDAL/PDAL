.. _readers.rxp:

readers.rxp
===========

The **RXP reader** read from files in the RXP format, the in-house streaming format used by `RIEGL Laser Measurement Systems GmbH`_.

.. warning::
   This software has not been developed by RIEGL, and RIEGL will not provide
   any support for this driver.  Please do not contact RIEGL with any
   questions or issues regarding this driver.  RIEGL is not responsible
   for damages or other issues that arise from use of this driver.
   This driver has been tested against RiVLib version 1.39 on a Ubuntu
   14.04 using gcc43.

.. plugin::

.. streamable::

Installation
------------

To build PDAL with rxp support, set RiVLib_DIR to the path of your local
RiVLib installation.  RiVLib can be obtained from the `RIEGL download pages`_
with a properly enabled user account.  The RiVLib files do not need to be
in a system-level directory, though they could be (e.g. they could be
in ``/usr/local``, or just in your home directory somewhere).  For help
building PDAL with optional libraries, see :ref:`the optional library documentation <optional-libraries>`.


Example
-------

This example rescales the points, given in the scanner's own coordinate
system, to values that can be written to a las file.  Only points with a
valid gps time, as determined by a pps pulse, are read from the rxp, since
the ``sync_to_pps`` option is "true".  Reflectance values are mapped to
intensity values using sensible defaults.

.. code-block:: json

  [
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


We set the ``discard_high_return_numbers`` option to ``true`` on the
:ref:`writers.las`.  RXP files can contain more returns per shot than is
supported by las, and so we need to explicitly tell the las writer to ignore
those high return number points.  You could also use :ref:`filters.python`
to filter those points earlier in the pipeline.


Options
-------

filename
  File to read from, or rdtp URI for network-accessible scanner. [Required]

.. include:: reader_opts.rst

rdtp
  Boolean to switch from file-based reading to RDTP-based. [Default: false]

sync_to_pps
  If "true", ensure all incoming points have a valid pps timestamp, usually
  provided by some sort of GPS clock.  If "false", use the scanner's internal
  time.  [Default: true]

reflectance_as_intensity
  If "true", in addition to storing reflectance values directly, also
  stores the values as Intensity by mapping the reflectance values in the
  range from `min_reflectance` to `max_reflectance` to the range 0-65535.
  Values less than `min_reflectance` are assigned the value 0.
  Values greater `max_reflectance` are assigned the value 65535.
  [Default: true]

min_reflectance
  The low end of the reflectance-to-intensity map.  [Default: -25.0]

max_reflectance
  The high end of the reflectance-to-intensity map.  [Default: 5.0]

.. _RIEGL Laser Measurement Systems GmbH: http://www.riegl.com
.. _RIEGL download pages: http://www.riegl.com/members-area/software-downloads/libraries/

