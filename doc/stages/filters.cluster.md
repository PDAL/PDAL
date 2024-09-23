.. _filters.cluster:

===============================================================================
filters.cluster
===============================================================================

The Cluster filter first performs Euclidean Cluster Extraction on the input
``PointView`` and then labels each point with its associated cluster ID.
It creates a new dimension ``ClusterID`` that contains the cluster ID value.
Cluster IDs start with the value 1.  Points that don't belong to any
cluster will are given a cluster ID of 0.

.. embed::

Example
-------

.. code-block:: json

  [
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

Options
-------

min_points
  Minimum number of points to be considered a cluster. [Default: 1]

max_points
  Maximum number of points to be considered a cluster. [Default: 2^64 - 1]

tolerance
  Cluster tolerance - maximum Euclidean distance for a point to be added to the
  cluster. [Default: 1.0]

is3d
  By default, clusters are formed by considering neighbors in a 3D sphere, but
  if ``is3d`` is set to false, it will instead consider neighbors in a 2D
  cylinder (XY plane only). [Default: true]

.. include:: filter_opts.rst
