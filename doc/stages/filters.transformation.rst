.. _filters.transformation:

filters.transformation
======================

The transformation filter applies an arbitrary rotation+translation transformation, represented as a 4x4 matrix, to each xyz triplet.
The filter does *no* checking to ensure the matrix is a valid affine transformation — buyer beware.


Example
-------

This example rotates the points around the z-axis while translating them.

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <Pipeline version="1.0">
    <Writer type="writers.las">
      <Option name="filename">
        transformed.las
      </Option>
      <Filter type="filters.transformation">
        <Option name="matrix">
          0 -1  0  1
          1  0  0  2
          0  0  1  3
          0  0  0  1
        </Option>
        <Reader type="readers.las">
          <Option name="filename">
            untransformed.las
          </Option>
        </Reader>
      </Filter>
    </Writer>
  </Pipeline>


Options
-------

matrix
  A whitespace-delimited transformation matrix.
  The matrix is assumed to be presented in row-major order.
  Only matrices with sixteen elements are allowed.

Notes
-----

The transformation filter does not apply any spatial reference information — if spatial reference information is desired, it must be specified on another filter.
