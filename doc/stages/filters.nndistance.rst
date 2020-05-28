.. _filters.nndistance:

===============================================================================
filters.nndistance
===============================================================================

The NNDistance filter runs a 3-D nearest neighbor algorithm on the input
cloud and creates a new dimension, ``NNDistance``, that contains a distance
metric described by the mode_ of the filter.

.. embed::

Example
-------------------------------------------------------------------------------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.nndistance",
          "k":8
      },
      {
          "type":"writers.bpf",
          "filename":"output.las",
          "output_dims":"X,Y,Z,NNDistance"
      }
  ]


Options
-------------------------------------------------------------------------------

_`mode`
  The mode of operation.  Either "kth", in which the distance is the euclidian
  distance of the subject point from the kth remote point or "avg" in which
  the distance is the average euclidian distance from the k_ nearest points.
  [Default: 'kth']

_`k`
  The number of k nearest neighbors to consider. [Default: **10**]

