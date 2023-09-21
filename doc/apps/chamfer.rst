.. _chamfer_command:

********************************************************************************
chamfer
********************************************************************************

.. warning::

  As of PDAL v2.6.0, the ``chamfer`` command is marked as DEPRECATED. It will be
  removed from the default install in PDAL v2.7 and removed completely in PDAL
  v2.8.

  The following Python code can be used with the PDAL Python bindings to compute
  the chamfer distance.

  ::

      import pdal
      import numpy as np
      from scipy.spatial.distance import cdist

      def chamfer_distance(arr1, arr2):
          distance_1_to_2 = 0
          distance_2_to_1 = 0

          points1 = np.column_stack((arr1['X'], arr1['Y'], arr2['Z']))
          points2 = np.column_stack((arr2['X'], arr2['Y'], arr2['Z']))
          
          # Compute distance from each point in arr1 to arr2
          for p1 in points1:
              distances = np.sqrt(np.sum((points2 - p1)**2, axis=1))
              min_distance = np.min(distances)
              distance_1_to_2 += min_distance
          
          # Compute distance from each point in arr2 to arr1
          for p2 in points2:
              distances = np.sqrt(np.sum((points1 - p2)**2, axis=1))
              min_distance = np.min(distances)
              distance_2_to_1 += min_distance
          
          return (distance_1_to_2 + distance_2_to_1) / (len(arr1) + len(arr2))

      pipeline1 = pdal.Reader("/path/to/input1.laz").pipeline()
      pipeline1.execute()
      arr1 = pipeline1.array[0]

      pipeline2 = pdal.Reader("/path/to/input2.laz").pipeline()
      pipeline2.execute()
      arr2 = pipeline2.array[0]

      # Compute Chamfer distance
      result = chamfer_distance(arr1, arr2)
      print("Chamfer Distance:", result)

  Popular Python packages such as scipy and sklearn have functions to compute
  the pairwise distance between points and can be used to simplify the above
  somewhat.

  Note that the provided code does not match exactly the output of PDAL's
  original implementation, which summed the square of the distance to the
  nearest neighbor. We have elected not to update the PDAL implementation at
  this time.
      

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

    --source arg     Source filename
    --candidate arg  Candidate filename

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
