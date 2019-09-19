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

_`matrix`
  A whitespace-delimited transformation matrix.
  The matrix is assumed to be presented in row-major order.
  Only matrices with sixteen elements are allowed.

Further details
---------------

.. note::
    This description should only give a high level overview about 3D 
    transformation abilities offered by PDAL.

Normally transformation of coordinates in a 3 dimensional coordinate system
is expressed in a 4x4 matrix. Whenever you will look for it you will find 
several sources out there in the net or in papers. But sometimes this 
4x4 matrix is ordered in different way. To avoid confusion about how this 
filter expects values to perform transformation, you will find in the 
sections below some simple examples to show you how the linearisation to 
the pdal pipeline should be done. So in the left 4x4 matirx shows the order 
of transformation components how they are expected and the line of 
elements right to it shows how the 4x4 matrix is linearised.

Simple translation
..................

.. figure:: ../images/filters.transformation.translation.svg.png

   Can be used to move all points of the pointcloud along the desired
   axis in the ammount of t:sub:`x` t:sub:`z` and t:sub:`z`.

Regarding to the figure above the following matrix will shift the 
pointcloud:

* 10 units along x-axis
* 10 units along y-axis
* 10 units along z-axis

Or in other words: This sums 10 units to each coordinate triple element.

.. code-block:: json

  [
      "untransformed.las",
      {
          "type":"filters.transformation",
          "matrix":"1  0  0  10  0  1  0  10  0  0  1  10  0  0  0  1"
      },
      {
          "type":"writers.las",
          "filename":"transformed.las"
      }
  ]
   
Simple scaling
..............

.. figure:: ../images/filters.transformation.scaling.svg.png

   Can be used to scale all points of the pointcloud along the desired
   axis times of s:sub:`x` s:sub:`z` and s:sub:`z`.

Regarding to the figure above the following matrix will scale the 
pointcloud:

* 2 times x-axis values
* 2 times y-axis values
* 2 times z-axis values

Or in other words: This multiplies 2 with each coordinate triple element.

.. code-block:: json

  [
      "untransformed.las",
      {
          "type":"filters.transformation",
          "matrix":"2  0  0  0  0  2  0  0  0  0  2  0  0  0  0  1"
      },
      {
          "type":"writers.las",
          "filename":"transformed.las"
      }
  ]

Typical usecase might be the exaggeration of height to make differences 
more visible.

Rotation in general
...................

Please keep in mind that rotation is always be done around native axis 
of your used CRS. So you will end up with really big transformed 
coordinates. If you want to rotate the pointcloud right in place you 
need to translate it first to your desired rotation axis.

Simple z-axis rotation (counter-clockwise)
.................................................

.. figure:: ../images/filters.transformation.rotation_z_axis_counter-clockwise.svg.png

   Can be used to rotate all points of the pointcloud around the z-axis with 
   the ammount of calculated SINUS and COSINUS of Φ.

Regarding to the figure above the following matrix will rotate the 
pointcloud 90° around the z-axis:

.. code-block:: json

  [
      "untransformed.las",
      {
          "type":"filters.transformation",
          "matrix":"0  -1  0  0  1  0  0  0  0  0  1  0  0  0  0  1"
      },
      {
          "type":"writers.las",
          "filename":"transformed.las"
      }
  ]

Simple x-axis rotation (counter-clockwise)
.................................................

.. figure:: ../images/filters.transformation.rotation_x_axis_counter-clockwise.svg.png

   Can be used to rotate all points of the pointcloud around the x-axis with 
   the ammount of calculated SINUS and COSINUS of Φ.

Regarding to the figure above the following matrix will rotate the 
pointcloud 90° around the x-axis:

.. code-block:: json

  [
      "untransformed.las",
      {
          "type":"filters.transformation",
          "matrix":"1  0  0  0  0  0  -1  0  0  1  0  0  0  0  0  1"
      },
      {
          "type":"writers.las",
          "filename":"transformed.las"
      }
  ]

Simple y-axis rotation (counter-clockwise)
.................................................

.. figure:: ../images/filters.transformation.rotation_y_axis_counter-clockwise.svg.png

   Can be used to rotate all points of the pointcloud around the y-axis with 
   the ammount of calculated SINUS and COSINUS of Φ.

Regarding to the figure above the following matrix will rotate the 
pointcloud 90° around the y-axis:

.. code-block:: json

  [
      "untransformed.las",
      {
          "type":"filters.transformation",
          "matrix":"0  0  1  0  0  1  0  0  -1  0  0  0  0  0  0  1"
      },
      {
          "type":"writers.las",
          "filename":"transformed.las"
      }
  ]
