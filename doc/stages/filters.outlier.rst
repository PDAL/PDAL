.. _filters.outlier:

===============================================================================
filters.outlier
===============================================================================

The Outlier filter provides two outlier filtering methods: radius and
statistical. These two approaches are discussed in further detail below.

Statistical Method
-------------------------------------------------------------------------------

The default method for identifying outlier points is the statistical outlier method. This method requires two passes through the input ``PointView``, first to compute a threshold value based on global statistics, and second to identify outliers using the computed threshold.

In the first pass, for each point :math:`p_i` in the input ``PointView``, compute the mean distance :math:`\mu_i` to each of the :math:`k` nearest neighbors (where :math:`k` is configurable and specified by ``mean_k``). Then,

.. math::

  \overline{\mu} = \frac{1}{N} \sum_{i=1}^N \mu_i

.. math::

  \sigma = \sqrt{\frac{1}{N-1} \sum_{i=1}^N (\mu_i - \overline{\mu})^2}

A global mean :math:`\overline{\mu}` of these mean distances is then computed along with the standard deviation :math:`\sigma`. From this, the threshold is computed as

.. math::

  t = \mu + m\sigma

where :math:`m` is a user-defined multiplier specified by ``multiplier``.

We now interate over the pre-computed mean distances :math:`\mu_i` and compare to computed threshold value. If :math:`\mu_i` is greater than the threshold, it is marked as an outlier.

.. math::

  outlier_i = \begin{cases}
      \text{true,} \phantom{false,} \text{if } \mu_i >= t \\
      \text{false,} \phantom{true,} \text{otherwise} \\
  \end{cases}

The ``classify`` and ``extract`` options are used to control whether outlier
points are labeled as noise, or removed from the output ``PointView``
completely.

.. figure:: filters.statisticaloutlier.img1.png
    :scale: 70 %
    :alt: Points before outlier removal

Before outlier removal, noise points can be found both above and below the
scene.


.. figure:: filters.statisticaloutlier.img2.png
    :scale: 60 %
    :alt: Points after outlier removal

After outlier removal, the noise points are removed.

See [Rusu2008]_ for more information.


Example
................................................................................

In this example, points are marked as outliers if the average distance to each
of the 12 nearest neighbors is below the computed threshold.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.outlier",
          "method":"statistical",
          "mean_k":12,
          "multiplier":2.2
        },
        "output.las"
      ]
    }

Radius Method
-------------------------------------------------------------------------------

For each point :math:`p_i` in the input ``PointView``, this method counts the
number of neighboring points :math:`k_i` within radius :math:`r` (specified by
``radius``). If :math:`k_i<k_{min}`, where :math:`k_{min}` is the minimum number
of neighbors specified by ``min_k``, it is marked as an outlier.

.. math::

  outlier_i = \begin{cases}
      \text{true,} \phantom{false,} \text{if } k_i < k_{min} \\
      \text{false,} \phantom{true,} \text{otherwise} \\
  \end{cases}

The ``classify`` and ``extract`` options are used to control whether outlier
points are labeled as noise, or removed from the output ``PointView``
completely.

Example
...............................................................................

The following example will mark points as outliers when there are fewer than
four neighbors within a radius of 1.0.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.outlier",
          "method":"radius",
          "radius":1.0,
          "min_k":4
        },
        "output.las"
      ]
    }

Options
-------------------------------------------------------------------------------

method
  The outlier removal method. [Default: **statistical**]

min_k
  Minimum number of neighbors in radius (radius method only). [Default: **2**]

radius
  Radius (radius method only). [Default: **1.0**]

mean_k
  Mean number of neighbors (statistical method only). [Default: **8**]

multiplier
  Standard deviation threshold (statistical method only). [Default: **2.0**]

classify
  Apply classification value of 18 (LAS high noise)? [Default: **true**]

extract
  Extract inlier returns only? [Default: **false**]
