.. _readers.sbet:

readers.sbet
============

The **SBET reader** read from files in the SBET format, used for exchange data from interital measurement units (IMUs).


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
