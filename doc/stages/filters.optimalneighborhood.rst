.. _filters.optimalneighborhood:

===============================================================================
filters.optimalneighborhood
===============================================================================

The **Optimal Neighborhood filter** computes the eigenentropy (defined as the
Shannon entropy of the normalized eigenvalues) for a neighborhood of points in
the range ``min_k`` to ``max_k``. The neighborhood size that minimizes the
eigenentropy is saved to a new dimension ``OptimalKNN``. The corresponding
radius of the neighborhood is saved to ``OptimalRadius``. These dimensions can
be written to an output file or utilized directly by
:ref:`filters.covariancefeatures`.

.. embed::

Example
-------------------------------------------------------------------------------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.optimalneighborhood",
          "min_k":8,
          "max_k": 50
      },
      {
          "type":"writers.las",
          "minor_version":4,
          "extra_dims":"all",
          "forward":"all",
          "filename":"output.las"
      }
  ]

Options
-------------------------------------------------------------------------------

min_k
  The minimum number of k nearest neighbors to consider for optimal
  neighborhood selection. [Default: 10]

max_k
  The maximum number of k nearest neighbors to consider for optimal
  neighborhood selection. [Default: 100]
