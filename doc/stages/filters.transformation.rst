.. _filters.transformation:

filters.transformation
======================

The transformation filter applies an arbitrary rotation+translation
transformation, represented as a 4x4 matrix_, to each xyz triplet.

The filter does *no* checking to ensure the matrix is a valid affine
transformation.

.. note::

    The transformation filter does not apply or consider any spatial
    reference information.

.. embed::

.. streamable::

Example
-------

This example rotates the points around the z-axis while translating them.

.. code-block:: json

  [
      "untransformed.las",
      {
          "type":"filters.transformation",
          "matrix":"0 -1  0  1  1  0  0  2  0  0  1  3  0  0  0  1"
      },
      {
          "type":"writers.las",
          "filename":"transformed.las"
      }
  ]


Options
-------

invert
  If set to true, applies the inverse of the provided transformation matrix.
  [Default: false]

_`matrix`
  A whitespace-delimited transformation matrix.
  The matrix is assumed to be presented in row-major order.
  Only matrices with sixteen elements are allowed.

.. include:: filter_opts.rst

Further details
---------------

A full tutorial about transformation matrices is beyond the scope of this
documentation. Instead, we will provide a few pointers to introduce core
concepts, especially as pertains to PDAL's handling of the ``matrix`` argument.

Transformations in a 3-dimensional coordinate system can be represented as an
affine transformation using homogeneous coordinates. This 4x4 matrix can
represent transformations describing operations like translation, rotation, and
scaling of coordinates.

The transformation filter's ``matrix`` argument is a space delimited, 16
element string. This string is simply a row-major representation of the 4x4
matrix (i.e., first four elements correspond to the top row of the
transformation matrix and so on).

In the event that readers are accustomed to an alternate representation of the
transformation matrix, we provide some simple examples in the form of pure
translations, rotations, and scaling, and show the corresponding ``matrix``
string.

Translation
...........

A pure translation by :math:`t_x`, :math:`t_y`, and :math:`t_z` in the X, Y,
and Z dimensions is represented by the following matrix.

.. math::

    \begin{matrix}
        1 & 0 & 0 & t_x \\
        0 & 1 & 0 & t_y \\
        0 & 0 & 1 & t_z \\
        0 & 0 & 0 & 1
    \end{matrix}

The JSON syntax required for such a translation is written as follows for :math:`t_x=7`, :math:`t_y=8`, and :math:`t_z=9`.

.. code-block:: json

  [
      {
          "type":"filters.transformation",
          "matrix":"1  0  0  7  0  1  0  8  0  0  1  9  0  0  0  1"
      }
  ]

Scaling
.......

Scaling of coordinates is also possible using a transformation matrix. The
matrix shown below will scale the X coordinates by :math:`s_x`, the Y
coordinates by :math:`s_y`, and Z by :math:`s_z`.

.. math::

    \begin{matrix}
        s_x &   0 &   0 & 0 \\
          0 & s_y &   0 & 0 \\
          0 &   0 & s_z & 0 \\
          0 &   0 &   0 & 1
    \end{matrix}

We again provide an example JSON snippet to demonstrate the scaling
transformation. In the example, X and Y are not scaled at all (i.e.,
:math:`s_x=s_y=1`) and Z is magnified by a factor of 2 (:math:`s_z=2`).

.. code-block:: json

  [
      {
          "type":"filters.transformation",
          "matrix":"1  0  0  0  0  1  0  0  0  0  2  0  0  0  0  1"
      }
  ]

Rotation
........

A rotation of coordinates by :math:`\theta` radians counter-clockwise about
the z-axis is accomplished with the following matrix.

.. math::

    \begin{matrix}
        \cos{\theta} & -\sin{\theta} & 0 & 0 \\
        \sin{\theta} &  \cos{\theta} & 0 & 0 \\
                   0 &             0 & 1 & 0 \\
                   0 &             0 & 0 & 1
    \end{matrix}

In JSON, a rotation of 90 degrees (:math:`\theta=1.57` radians) takes the form
shown below.

.. code-block:: json

  [
      {
          "type":"filters.transformation",
          "matrix":"0  0  -1  0  1  0  0  0  0  0  1  0  0  0  0  1"
      }
  ]

Similarly, a rotation about the x-axis by :math:`\theta` radians is represented
as

.. math::

    \begin{matrix}
        1 &            0 &             0 & 0 \\
        0 & \cos{\theta} & -\sin{\theta} & 0 \\
        0 & \sin{\theta} &  \cos{\theta} & 0 \\
        0 &            0 &             0 & 1
    \end{matrix}

which takes the following form in JSON for a rotation of 45 degrees (:math:`\theta=0.785` radians)

.. code-block:: json

  [
      {
          "type":"filters.transformation",
          "matrix":"1  0  0  0  0  0.707  -0.707  0  0  0.707  0.707  0  0  0  0  1"
      }
  ]

Finally, a rotation by :math:`\theta` radians about the y-axis is accomplished
with the matrix

.. math::

    \begin{matrix}
         \cos{\theta} & 0 & \sin{\theta} & 0 \\
                    0 & 1 &            0 & 0 \\
        -\sin{\theta} & 0 & \cos{\theta} & 0 \\
                    0 & 0 &            0 & 1
    \end{matrix}

and the JSON string for a rotation of 10 degrees (:math:`\theta=0.175` radians) becomes

.. code-block:: json

  [
      {
          "type":"filters.transformation",
          "matrix":"0.985  0  0.174  0  0  1  0  0  -0.174  0  0.985  0  0  0  0  1"
      }
  ]
