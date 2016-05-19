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
