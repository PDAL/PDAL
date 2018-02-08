.. _filters.knnassign:

filters.knnassign
===================

The knnassign filter allows you update the value of a dimension for specific points
to a value determined by a K-nearest neighbors vote. For each point, the k 
nearest neighbors are queried and if more than half of them have the same 
value for the specified dimension, the filter updates the selected point 
accordingly

For example, if an automated classification procedure put/left erroneous 
vegetation points near the edges of buildings which were largely classified 
correctly, you could try using this filter to fix that problem.

Similiarly, some automated classification processes result in prediction for
only a subset of the original point cloud.  This filter could be used to
extrapolate those predictions to the original.

.. embed::

Example 1
---------

This pipeline updates the Classification of all points with classification
1 (unclassified) based on the consensus (majority) of its nearest 10 neighbors.

.. code-block:: json
    {
      "pipeline":[
        "autzen_class.las",
        {
          "type" : "filters.knnassign",
          "domain" : "Classification[1:1]",
          "dimension" : "Classification",
          "k" : 10
        },
        {
          "filename":"autzen_class_refined.las"
        }
      ]
    } 

Example 2
---------

This pipeline moves all the classifications from "pred.txt"
to src.las.  Any points in src.las that are not in pred.txt will be
assigned based on the closest point in pred.txt.

.. code-block:: json
    {
      "pipeline":[
        "src.las",
        {
          "type" : "filters.knnassign",
          "dimension" : "Classification",
          "k" : 1,
          "candidate" : "pred.txt"
        },
        {
          "filename":"dest.las"
        }
      ]
    } 

Options
-------

candidate
  A filename which points to the point cloud containing the points which
  will do the voting.  If not specified, defaults to the input of the filter.

domain
  A :ref:`range <ranges>` which selects points to be processed by the filter.
  Can be specified multiple times.  Points satisfying any range will be
  processed

dimension
  A :ref:`dimension` which specifies which dimension to be updated by the filter.

k
  An integer which specifies the number of neighbors which vote on each
  selected point.  
