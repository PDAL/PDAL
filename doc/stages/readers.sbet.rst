.. _readers.sbet:

readers.sbet
============

The **SBET reader** read from files in the SBET format, used for exchange data from interital measurement units (IMUs).
SBET files store angles as radians, but by default this reader converts all angle-based measurements to degrees.
Set ``angles_as_degrees`` to ``false`` to disable this conversion.

.. embed::

.. streamable::

Example
-------


.. code-block:: json

    {
      "pipeline":[
        "sbetfile.sbet",
        "output.las"
      ]
    }


Options
-------

filename
  File to read from [Required]

angles_as_degrees
  Convert all angles to degrees. If false, angles are read as radians. [Default: true]

.. include:: reader_opts.rst

