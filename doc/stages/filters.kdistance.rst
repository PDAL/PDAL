.. _filters.kdistance:

===============================================================================
filters.kdistance
===============================================================================

The K-Distance filter creates a new attribute ``KDistance`` that contains the
Euclidean distance to a point's k-th nearest neighbor. It can also be set to compute the average distance of its k nearest neighbors.

.. embed::

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.kdistance",
          "k":8
        },
        {
          "type":"writers.bpf",
          "filename":"output.las",
          "output_dims":"X,Y,Z,KDistance"
        }
      ]
    }


Options
-------------------------------------------------------------------------------

k
  The number of k nearest neighbors. [Default: **10**]
average
  Compute the average distance of the point's k nearest neighbors. [Default: **false**]

