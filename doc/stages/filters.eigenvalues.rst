.. _filters.eigenvalues:

filters.eigenvalues
===============================================================================

``filters.eigenvalues`` returns the eigenvalues for a given point, based on its
k-nearest neighbors.

The filter produces three new dimensions (``Eigenvalue0``, ``Eigenvalue1``, and
``Eigenvalue2``), which can be analyzed directly, or consumed by downstream
stages for more advanced filtering. The eigenvalues are sorted in ascending
order.

The eigenvalue decomposition is performed using Eigen's
``SelfAdjointEigenSolver``. For more information see
https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html.

.. embed::


Example
-------

This pipeline demonstrates the calculation of the eigenvalues. The newly created
dimensions are written out to BPF for further inspection.

.. code-block:: json

    {
      "pipeline":[
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
    }

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: **8**]
