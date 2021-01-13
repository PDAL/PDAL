.. _chamfer_command:

********************************************************************************
chamfer
********************************************************************************

The ``chamfer`` command is used to compute the Chamfer distance between two
point clouds. The Chamfer distance is computed by summing the squared distances
between nearest neighbor correspondences of two point clouds.

More formally, for two non-empty subsets :math:`X` and :math:`Y`, the Chamfer
distance :math:`d_{CD}(X,Y)` is

.. math::

  d_{CD}(X,Y) = \sum_{x \in X} \operatorname*{min}_{y \in Y} ||x-y||^2_2 + \sum_{y \in Y} \operatorname*{min}_{x \in X} ||x-y||^2_2
  
::

    $ pdal chamfer <source> <candidate>

::

    --source arg     Non-positional option for specifying filename of source file.
    --candidate arg  Non-positional option for specifying filename to test against source.

The algorithm makes no distinction between source and candidate files (i.e.,
they can be transposed with no affect on the computed distance).

The command returns 0 along with a JSON-formatted message summarizing the PDAL
version, source and candidate filenames, and the Chamfer distance. Identical
point clouds will return a Chamfer distance of 0.

::

    $ pdal chamfer source.las candidate.las
    {
      "filenames":
      [
        "\/path\/to\/source.las",
        "\/path\/to\/candidate.las"
      ],
      "chamfer": 1.303648726,
      "pdal_version": "1.3.0 (git-version: 191301)"
    }

.. note::
  
  The Chamfer distance is computed for XYZ coordinates only and as such says
  nothing about differences in other dimensions or metadata.
