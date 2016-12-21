.. _filters.cluster:

===============================================================================
filters.cluster
===============================================================================

The Cluster filter first performs Euclidean Cluster Extraction on the input
``PointView`` and then labels each point with its associated cluster ID.

Example
-------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.cluster"
        },
        {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,ClusterID"
        }
      ]
    }

Options
-------

min_points
  Minimum number of points to be considered a cluster. [Default: **1**]

max_points
  Maximum number of points to be considered a cluster. [Default: **UINT64_MAX**]

tolerance
  Cluster tolerance - maximum Euclidean distance for a point to be added to the
  cluster. [Default: **1.0**]
  
