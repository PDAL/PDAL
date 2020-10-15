.. _filters.lloydkmeans:

===============================================================================
filters.lloydkmeans
===============================================================================

K-means clustering using Lloyd's algorithm labels each point with its
associated cluster ID (starting at 0).

.. embed::

.. versionadded:: 2.1

Example
-------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.lloydkmeans",
          "k":10,
          "maxiters":20,
          "dimensions":"X,Y,Z"
      },
      {
          "type":"writers.las",
          "filename":"output.laz",
          "minor_version":4,
          "extra_dims":"all"
      }
  ]

Options
-------

k
  The desired number of clusters. [Default: 10]

maxiters
  The maximum number of iterations. [Default: 10]

dimensions
  Comma-separated string indicating dimensions to use for clustering.
  [Default: X,Y,Z]

.. include:: filter_opts.rst

