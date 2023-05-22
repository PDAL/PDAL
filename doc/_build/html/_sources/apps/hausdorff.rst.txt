.. _hausdorff_command:

********************************************************************************
hausdorff
********************************************************************************

The ``hausdorff`` command is used to compute the Hausdorff distance between two
point clouds. In this context, the Hausdorff distance is the greatest of all
Euclidean distances from a point in one point cloud to the closest point in the
other point cloud.

More formally, for two non-empty subsets :math:`X` and :math:`Y`, the Hausdorff
distance :math:`d_H(X,Y)` is

.. math::

  d_H(X,Y) = \operatorname*{max} \big\{ \operatorname*{sup}_{x \in X} \operatorname*{inf}_{y \in Y} d(x,y), \operatorname*{sup}_{y \in Y} \operatorname*{inf}_{x \in X} d(x,y)\big\}
  
where :math:`\operatorname*{sup}` and :math:`\operatorname*{inf}` are the
supremum and infimum respectively.

::

    $ pdal hausdorff <source> <candidate>

::

    --source arg     Source filename
    --candidate arg  Candidate filename

The algorithm makes no distinction between source and candidate files (i.e.,
they can be transposed with no affect on the computed distance).

The command returns 0 along with a JSON-formatted message summarizing the PDAL
version, source and candidate filenames, and the Hausdorff distance. Identical
point clouds will return a Hausdorff distance of 0.

::

    $ pdal hausdorff source.las candidate.las
    {
      "filenames":
      [
        "\/path\/to\/source.las",
        "\/path\/to\/candidate.las"
      ],
      "hausdorff": 1.303648726,
      "pdal_version": "1.3.0 (git-version: 191301)"
    }

.. note::
  
  The ``hausdorff`` is computed for XYZ coordinates only and as such says
  nothing about differences in other dimensions or metadata.
