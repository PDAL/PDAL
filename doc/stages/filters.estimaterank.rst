.. _filters.estimaterank:

filters.estimaterank
===============================================================================

``filters.estimaterank`` computes the rank (i.e., the number of nonzero singular
values) of a neighborhood of points.

This method uses Eigen's JacobiSVD class to solve the singular value
decomposition and to estimate the rank using the user-specified threshold. A
singular value will be considered nonzero if its absolute value is greater than
the product of the user-supplied threshold and the absolute value of the maximum
singular value.

More on JacobiSVD can be found at
https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html.

.. embed::

Example
-------

This sample pipeline estimates the rank of each point using
``filters.estimaterank`` and then filters out those points where the rank is
three using ``filters.range``.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.estimaterank",
          "knn":8,
          "thresh":0.01
        },
        {
          "type":"filters.range",
          "limits":"Rank![3:3]"
        },
        "output.laz"
      ]
    }

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: **8**]

thresh
  The threshold used to identify nonzero singular values. [Default: **0.01**]
