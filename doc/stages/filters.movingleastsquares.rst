.. _filters.movingleastsquares:

===============================================================================
filters.movingleastsquares
===============================================================================

The Moving Least Squares filter passes data through the Point Cloud Library
(`PCL`_) MovingLeastSquares algorithm.

MovingLeastSquares is an implementation of the MLS (Moving Least Squares)
algorithm for data smoothing and improved normal estimation described in
[Alexa2003]_. It also contains methods for upsampling the resulting cloud based
on the parametric fit.

.. [Alexa2003] Alexa, Marc, et al. "Computing and rendering point set surfaces." Visualization and Computer Graphics, IEEE Transactions on 9.1 (2003): 3-15.

.. _`PCL`: http://www.pointclouds.org

.. plugin::

Example
-------------------------------------------------------------------------------

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.movingleastsquares"
        },
        {
          "type":"writers.las",
          "filename":"output.las"
        }
      ]
    }



Options
-------------------------------------------------------------------------------

None at the moment. Relying on defaults within PCL.
