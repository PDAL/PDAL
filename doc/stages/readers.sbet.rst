.. _readers.sbet:

readers.sbet
============

The **SBET reader** read from files in the SBET format, used for exchange data from interital measurement units (IMUs).

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

.. include:: reader_opts.rst

