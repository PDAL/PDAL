.. _filters.approximatecoplanar:

filters.approximatecoplanar
===============================================================================

The **approximate coplanar filter** implements a portion of the algorithm
presented
in [Limberger2015]_. Prior to clustering points, the authors first apply an
approximate coplanarity test, where points that meet the following criteria are
labeled as approximately coplanar.

.. math::

  \lambda_2 > (s_{\alpha}\lambda_1) \&\& (s_{\beta}\lambda_2) > \lambda_3

:math:`\lambda_1`, :math:`\lambda_2`, :math:`\lambda_3` are the eigenvalues of
a neighborhood of points (defined by ``knn`` nearest neighbors) in ascending
order. The threshold values :math:`s_{\alpha}` and :math:`s_{\beta}` are
user-defined and default to 25 and 6 respectively.

The filter returns a point cloud with a new dimension ``Coplanar`` that
indicates those points that are part of a neighborhood that is approximately
coplanar (1) or not (0).

.. embed::

Example
-------

The sample pipeline presented below estimates the planarity of a point based on
its eight nearest neighbors using the approximate coplanar filter. A
:ref:`filters.range` stage then filters out any points that were not
deemed to be coplanar before writing the result in compressed LAZ.

.. code-block:: json

  [
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

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: 8]

thresh1
  The threshold to be applied to the smallest eigenvalue. [Default: 25]

thresh2
  The threshold to be applied to the second smallest eigenvalue. [Default: 6]
