.. _filters.movingleastsquares:

===============================================================================
filters.movingleastsquares
===============================================================================

The Moving Least Squares filter passes data through the Point Cloud Library
`moving least squares <http://docs.pointclouds.org/trunk/classpcl_1_1_moving_least_squares.html>`_ algorithm.

Moving least squares is intended to smooth data and improve normal estiation
as described in [Alexa2003]_. It also contains methods for upsampling
the resulting cloud based on the parametric fit.

.. _`PCL`: http://www.pointclouds.org

.. plugin::

Example
-------------------------------------------------------------------------------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.movingleastsquares"
      },
      {
          "type":"writers.las",
          "filename":"output.las"
      }
  ]


Options
-------------------------------------------------------------------------------

None.
