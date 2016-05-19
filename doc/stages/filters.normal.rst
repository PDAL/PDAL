.. _filters.normal:

filters.normal
===============================================================================

``filters.normal`` returns the estimated normal and curvature for a collection
of points. The algorithm first computes the eigenvalues and eigenvectors of the
collection of points, which is comprised of the k-nearest neighbors. The normal
is taken as the eigenvector corresponding to the smallest eigenvalue. The
curvature is computed as

.. math::
  
  curvature = \frac{\lambda_0}{\lambda_0 + \lambda_1 _+ \lambda_2}
  
where :math:`\lambda_i` are the eigenvalues sorted in ascending order.

The filter produces four new dimensions (``NormalX``, ``NormalY``, ``NormalZ``,
and ``Curvature``), which can be analyzed directly, or consumed by downstream
stages for more advanced filtering.

The eigenvalue decomposition is performed using Eigen's
``SelfAdjointEigenSolver``. For more information see
https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html.

Example
-------

This pipeline demonstrates the calculation of the normal values (along with
curvature). The newly created dimensions are written out to BPF for further
inspection.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.normal",
          "knn":8
        },
        {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,NormalX,NormalY,NormalZ,Curvature"
        }
      ]
    }

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: **8**]
