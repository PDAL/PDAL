.. _hausdorff_command:

********************************************************************************
hausdorff
********************************************************************************

.. warning::

  As of PDAL v2.6.0, the ``hausdorff`` command is marked as DEPRECATED. It will
  be removed from the default install in PDAL v2.7 and removed completely in
  PDAL v2.8.

  The following Python code can be used with the PDAL Python bindings to compute
  the Hausdorff distance.

  ::

      import pdal
      import numpy as np

      def hausdorff_distance(arr1, arr2):
          max_min_distance_1_to_2 = 0
          max_min_distance_2_to_1 = 0

          points1 = np.column_stack((arr1['X'], arr1['Y'], arr2['Z']))
          points2 = np.column_stack((arr2['X'], arr2['Y'], arr2['Z']))
          
          # Compute distance from each point in arr1 to arr2
          for p1 in points1:
              distances = np.sqrt(np.sum((points2 - p1)**2, axis=1))
              min_distance = np.min(distances)
              max_min_distance_1_to_2 = max(max_min_distance_1_to_2, min_distance)
          
          # Compute distance from each point in arr2 to arr1
          for p2 in points2:
              distances = np.sqrt(np.sum((points1 - p2)**2, axis=1))
              min_distance = np.min(distances)
              max_min_distance_2_to_1 = max(max_min_distance_2_to_1, min_distance)
          
          return max(max_min_distance_1_to_2, max_min_distance_2_to_1)

      pipeline1 = pdal.Reader("/path/to/input1.laz").pipeline()
      pipeline1.execute()
      arr1 = pipeline1.array[0]

      pipeline2 = pdal.Reader("/path/to/input2.laz").pipeline()
      pipeline2.execute()
      arr2 = pipeline2.array[0]

      # Compute Hausdorff distance
      result = hausdorff_distance(arr1, arr2)
      print("Hausdorff Distance:", result)

  SciPy can be used to simplify this function even further, as shown below.

  ::

      import pdal
      import numpy as np
      from scipy.spatial.distance import directed_hausdorff

      def hausdorff_distance(arr1, arr2):
          points1 = np.column_stack((arr1['X'], arr1['Y'], arr2['Z']))
          points2 = np.column_stack((arr2['X'], arr2['Y'], arr2['Z']))
          
          # Compute directed Hausdorff distances
          d1 = directed_hausdorff(points1, points2)[0]
          d2 = directed_hausdorff(points2, points1)[0]
          
          return max(d1, d2)

      pipeline1 = pdal.Reader("/path/to/input1.laz").pipeline()
      pipeline1.execute()
      arr1 = pipeline1.array[0]

      pipeline2 = pdal.Reader("/path/to/input2.laz").pipeline()
      pipeline2.execute()
      arr2 = pipeline2.array[0]

      # Compute Hausdorff distance
      result = hausdorff_distance(arr1, arr2)
      print("Hausdorff Distance:", result)

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
