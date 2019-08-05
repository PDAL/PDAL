.. _writers.sbet:

writers.sbet
============

The **SBET writer** writes files in the SBET format, used for exchange data from inertial measurement units (IMUs).

.. embed::

.. streamable::

Example
-------

.. code-block:: json

  [
      "input.sbet",
      "output.sbet"
  ]


Options
-------

filename
  File to write. [Required]

angles_are_degrees
  Convert all angular values from degrees to radians before write.
  [Default: true]
