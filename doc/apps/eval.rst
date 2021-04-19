.. _eval_command:

********************************************************************************
eval
********************************************************************************

The ``eval`` command is used to compare the ``Classification`` dimensions of two
point clouds.
::

    $ pdal eval <predicted> <truth> --labels <labels>

::

    --predicted arg     Positional argument specifying point cloud filename containing predicted labels.
    --truth arg         Positional argument specifying point cloud filename containing truth labels.
    --labels arg        Comma-separated list of classification labels to evaluate.

The command returns 0 along with a JSON-formatted classification report
summarizing various classification metrics.

In the provided example below, the truth and predicted point clouds contain
points classified as ground (class 2) and medium vegetation (class 4) in
accordance with the LAS specification. Both point clouds also contain some
number of classifications that are either unlabeled or do not fall into the
specificied classes.

::

    $ pdal eval predicted.las truth.las --labels 2,4
    {
      "confusion_matrix": "[[5240537,3860,24102],[268015,3179304,326677],[111453,115516,2950315]]",
      "f1_score": 0.944,
      "labels": [
        {
          "accuracy": 0.967,
          "f1_score": 0.973,
          "intersection_over_union": 0.947,
          "label": "1",
          "precision": 0.951,
          "sensitivity": 0.995,
          "specificity": 0.929,
          "support": 5268499
        },
        {
          "accuracy": 0.934,
          "f1_score": 0.914,
          "intersection_over_union": 0.842,
          "label": "2",
          "precision": 0.999,
          "sensitivity": 0.842,
          "specificity": 0.999,
          "support": 3773996
        }
      ],
      "mean_intersection_over_union": 0.894,
      "overall_accuracy": 0.931,
      "pdal_version": "2.2.0 (git-version: 6e80b9)",
      "predicted_file": "predicted.las",
      "truth_file": "truth.las"
    }

Most of the returned metrics will be self explanatory, with scores reported
both for individual classes and at a summary level. The returned confusion
matrix is presented in row-major order, where each row corresponds to a truth
label (the last row is a catch-all for any unlabeled or ignored entries).
Similarly, confusion matrix columns correspond to predicted labels where the
last column is once again a catch-all for unlabeled entries. Although
unlabeled/ignored truth labels are reported in the confusion matrix, they are
excluded from all computed scores.
