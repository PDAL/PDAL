.. _filters.covariancefeatures:

===============================================================================
filters.covariancefeatures
===============================================================================

This filter implements various local feature descriptors that are based on the
covariance matrix of a point's neighborhood.

The user can pick a set of feature descriptors by setting the ``feature_set``
option. Currently, the only supported feature is the dimensionality_ set of
feature descriptors introduced below.

Alternately, the user can provide a comma-separated list of ``features`` to
explicitly itemize those covariance features they wish to be computed.

Supported features include:

* Anisotropy
* DemantkeVerticality
* Density
* Eigenentropy
* Linearity
* Omnivariance
* Planarity
* Scattering
* Sum
* SurfaceVariation
* Verticality

Example #1
-------------------------------------------------------------------------------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.covariancefeatures",
          "knn":8,
          "threads": 2,
          "feature_set": "Dimensionality"
      },
      {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,Linearity,Planarity,Scattering,Verticality"
      }
  ]

Example #2
-------------------------------------------------------------------------------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.optimalneighborhood"
      },
      {
          "type":"filters.covariancefeatures",
          "knn":8,
          "threads": 2,
          "optimized":true,
          "features": "Linearity,Omnivariance,Density"
      },
      {
          "type":"writers.las",
          "minor_version":4,
          "extra_dims":"all",
          "forward":"all",
          "filename":"output.las"
      }
  ]

Options
-------------------------------------------------------------------------------

knn
  The number of k nearest neighbors used for calculating the covariance matrix.
  [Default: 10]

threads
  The number of threads used for computing the feature descriptors. [Default: 1]

feature_set
  The features to be computed. Currently only supports ``Dimensionality``.

stride
  When finding k nearest neighbors, stride determines the sampling rate. A
  stride of 1 retains each neighbor in order. A stride of two selects every
  other neighbor and so on. [Default: 1]

min_k
  Minimum number of neighbors in radius (radius search only). [Default: 3]

radius
  If radius is specified, neighbors will be obtained by radius search rather
  than k nearest neighbors, subject to meeting the minimum number of neighbors
  (``min_k``).

features
  A comma-separated list of individual features to be computed. [Default: "all"]

mode
  By default, features are computed using the standard deviation along each
  eigenvector, i.e., using the square root of the computed eigenvalues
  (``mode="SQRT"``). ``mode`` also accepts "Normalized" which normalizes
  eigenvalue such that they sum to one, or "Raw" such that the eigenvalues are
  used directly. [Default: "SQRT"]

optimized
  ``optimized`` can be set to ``true`` to enable computation of features using
  precomputed optimal neighborhoods (see :ref:`filters.optimalneighborhood`).
  Enables computation of ``Density`` feature. [Default: false]

.. _dimensionality:

Dimensionality feature set
................................................................................
The features introduced in [Demantke2011]_ describe the shape of the
neighborhood, indicating whether the local geometry is more linear (1D), planar
(2D) or volumetric (3D) while the one introduced in [Guinard2017]_ adds the
idea of a structure being vertical.

The dimensionality filter introduces the following four descriptors that are
computed from the covariance matrix of a point's neighbors (as defined by
``knn`` or ``radius``):

* linearity - higher for long thin strips
* planarity - higher for planar surfaces
* scattering - higher for complex 3d neighbourhoods
* verticality - higher for vertical structures, highest for thin vertical strips

It introduces four new dimensions that hold each one of these values:
``Linearity``, ``Planarity``, ``Scattering`` and ``Verticality``.

