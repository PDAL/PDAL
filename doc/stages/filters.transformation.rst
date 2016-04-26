.. _filters.transformation:

filters.transformation
======================

The transformation filter applies an arbitrary rotation+translation
transformation, represented as a 4x4 matrix, to each xyz triplet.

The filter does *no* checking to ensure the matrix is a valid affine
transformation — buyer beware.

.. note::

    The transformation filter does not apply any spatial reference information
    — if spatial reference information is desired, it must be specified on
    another filter.

Example
-------

This example rotates the points around the z-axis while translating them.

.. code-block:: json

    {
      "pipeline":[
        "untransformed.las",
        {
          "type":"filters.transformation",
          "matrix":"0 -1  0  1  1  0  0  2  0  0  1  3  0  0  0  1",
        },
        {
          "type":"writers.las",
          "filename":"transformed.las"
        }
      ]
    }



Options
-------

matrix
  A whitespace-delimited transformation matrix.
  The matrix is assumed to be presented in row-major order.
  Only matrices with sixteen elements are allowed.

Notes
-----

The transformation filter does not apply any spatial reference information — if spatial reference information is desired, it must be specified on another filter.
