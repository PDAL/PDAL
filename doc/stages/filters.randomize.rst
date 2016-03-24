.. _filters.randomize:

filters.randomize
=================

The randomize filter reorders the points in a point view randomly.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.randomize"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }


