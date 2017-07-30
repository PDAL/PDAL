.. _filters.lof:

filters.lof
===============================================================================

Local Outlier Factor (LOF) was introduced as a method of determining the degree
to which an object is an outlier. This filter is an implementation of the method
described in [Breunig2000]_.

The filter creates three new dimensions, all of which are doubles. The
``KDistance`` dimension records the Euclidean distance between a point and it's
``k-th`` nearest neighbor (the number of ``k`` neighbors is set with the
``minpts`` option). The ``LocalReachabilityDistance`` is the inverse of the mean
of all reachability distances for a neighborhood of points. This reachability
distance is defined as the max of the Euclidean distance to a neighboring point
and that neighbor's own previously computed ``KDistance``. Finally, each point
has a ``LocalOutlierFactor`` which is the mean of all
``LocalReachabilityDistance`` values for the neighborhood. In each case, the
neighborhood is the set of ``k`` nearest neighbors, where ``k`` is set with the
``minpts`` option.

In practice, setting the ``minpts`` parameter appropriately and subsequently
filtering outliers based on the computed ``LocalOutlierFactor`` can be
difficult. The authors present some work on establishing upper and lower bounds
on LOF values, and provide some guidelines on selecting ``minpts`` values, which
users of ``filters.lof`` should find instructive.

.. note::

  To inspect the newly created, non-standard dimensions, be sure to write to an
  output format that can support arbitrary dimensions, such as BPF.

.. [Breunig2000] Breunig, M.M., Kriegel, H.-P., Ng, R.T., Sander, J., 2000. LOF: Identifying Density-Based Local Outliers. Proc. 2000 Acm Sigmod Int. Conf. Manag. Data 1–12.

.. embed::

Example
-------

The sample pipeline below uses ``filters.lof`` to compute the LOF with a neighborhood of 20 neighbors, followed by a range filter to crop out points whose ``LocalOutlierFactor`` exceeds 1.2, before writing the output.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.lof",
          "minpts":20
        },
        {
          "type":"filters.range",
          "limits":"LocalOutlierFactor[:1.2]"
        },
        "output.laz"
      ]
    }

Options
-------------------------------------------------------------------------------

minpts
  The number of k nearest neighbors. [Default: **10**]
