.. _filters.eigenvalues:

filters.eigenvalues
===============================================================================

The **eignvalue filter** returns the eigenvalues for a given point,
based on its k-nearest neighbors.

The filter produces three new dimensions (``Eigenvalue0``, ``Eigenvalue1``, and
``Eigenvalue2``), which can be analyzed directly, or consumed by downstream
stages for more advanced filtering. The eigenvalues are sorted in ascending
order.

The eigenvalue decomposition is performed using Eigen's
SelfAdjointEigenSolver_.

.. _SelfAdjointEigenSolver: https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html

.. embed::


Example
-------

This pipeline demonstrates the calculation of the eigenvalues. The newly created
dimensions are written out to BPF for further inspection.

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.eigenvalues",
          "knn":8
      },
      {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,Eigenvalue0,Eigenvalue1,Eigenvalue2"
      }
  ]

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: 8]

normalize
  Normalize eigenvalues such that the sum is 1. [Default: false]
