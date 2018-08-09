.. _readers.terrasolid:

readers.terrasolid
==================

The **Terrasolid Reader** loads points from terrasolid files (.bin).
It supports boths Terrasolid format 1 and format 2.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        {
          "type":"readers.terrasolid",
          "filename":"autzen.bin"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }

Options
-------

filename
  Input file name [Required]

count
  Maximum number of points to read. [Default: read all points]
