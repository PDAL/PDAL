.. _filters.approximatecoplanar:

filters.approximatecoplanar
===============================================================================

``filters.approximatecoplanar`` filter estimates the planarity of a neighborhood
of points by first computing eigenvalues for the points and then tagging those
points for which the following is true:

.. math::
  
  \lambda_1 > (thresh_1 * \lambda_0) \text{ && } (\lambda_1 * thresh_2) > \lambda2
  
where :math:`\lambda_0`, :math:`\lambda_1`, :math:`\lambda_2` are the
eigenvalues in ascending order. The threshold values :math:`thresh_1` and
:math:`thresh_2` are user-defined and default to 25 and 6 respectively.

The filter returns a point cloud with a new dimension  ``Coplanar`` that 
indicates those points that are part of a neighborhood that is approximately
coplanar (1) or not (0).

Eigenvalue estimation is performed using Eigen's ``SelfAdjointEigenSolver``. For
more information see
https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html.

Example
-------

The sample pipeline presented below estimates the planarity of a point based on
its eight nearest neighbors using the ``filters.approximatecoplanar`` filter. A
``filters.range`` stage then filters out any points that were not deemed to be
coplanar before writing the result in compressed LAZ.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.approximatecoplanar",
          "knn":8,
          "thresh1":25,
          "thresh2":6
        },
        {
          "type":"filters.range",
          "limits":"Coplanar[1:1]"
        },
        "output.laz"
      ]
    }

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: **8**]
  
thresh1
  The threshold to be applied to the smallest eigenvalue. [Default: **25**]
  
thresh2
  The threshold to be applied to the second smallest eigenvalue. [Default: **6**]
