.. _writers.sbet:

writers.sbet
============

The **SBET writer** writes files in the SBET format, used for exchange data from interital measurement units (IMUs).

.. embed::

.. streamable::

Example
-------


.. code-block:: json

    {
      "pipeline":[
        "input.sbet",
        "output.sbet"
      ]
    }


Options
-------

filename
  File to write. [Required]
