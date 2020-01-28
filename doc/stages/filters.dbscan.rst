.. _filters.dbscan:

===============================================================================
filters.dbscan
===============================================================================

The DBSCAN filter performs Density-Based Spatial Clustering of Applications
with Noise (DBSCAN) [Ester1996]_ and labels each point with its associated
cluster ID. Points that do not belong to a cluster are given a Cluster ID of
-1. The remaining clusters are labeled as integers starting from 0.

.. embed::

.. versionadded:: 2.1

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.dbscan",
          "min_points":10,
          "eps":2.0,
          "dimensions":"X,Y,Z"
      },
      {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,ClusterID"
      }
  ]

Options
-------

min_points
  The minimum cluster size ``min_points`` should be greater than or equal to
  the number of dimensions (e.g., X, Y, and Z) plus one. As a rule of thumb,
  two times the number of dimensions is often used. [Default: 6]

eps
  The epsilon parameter can be estimated from a k-distance graph (for k =
  ``min_points`` minus one). ``eps`` defines the Euclidean distance that will
  be used when searching for neighbors. [Default: 1.0]

dimensions
  Comma-separated string indicating dimensions to use for clustering. [Default: X,Y,Z]

