.. _filters.covariancefeatures:

===============================================================================
filters.covariancefeatures
===============================================================================

This filter implements various local feature descriptors that are based on the
covariance matrix of a point's neighborhood.

The user can pick a set of feature descriptors by setting the ``feature_set``
option. The dimensionality_ set of feature descriptors introduced below is the
default. The user can also provide a comma-separated list of features to
explicitly itemize those covariance features they wish to be computed. This can
be combined with any suppported presets like "Dimensionality".  Specifying "all"
will compute all available features.

Supported features include:

* Anisotropy
* DemantkeVerticality
* Density
* Eigenentropy
* Linearity
* Omnivariance
* Planarity
* Scattering
* EigenvalueSum
* SurfaceVariation
* Verticality

.. note::

    Density requires both ``OptimalKNN`` and ``OptimalRadius`` which can be
    computed by running :ref:`filters.optimalneighborhood` prior to
    ``filters.covariancefeatures``.

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
          "feature_set": "Linearity,Omnivariance,Density"
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
  A comma-separated list of individual features or feature presets (e.g.,
  "Dimensionality") to be computed. To compute all available features, specify
  "all". [Default: "Dimensionality"]

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

mode
  By default, features are computed using the standard deviation along each
  eigenvector, i.e., using the square root of the computed eigenvalues
  (``mode="SQRT"``). ``mode`` also accepts "Normalized" which normalizes
  eigenvalue such that they sum to one, or "Raw" such that the eigenvalues are
  used directly. [Default: "SQRT"]

optimized
  ``optimized`` can be set to ``true`` to enable computation of features using
  precomputed optimal neighborhoods (found in the ``OptimalKNN`` dimension).
  Requires :ref:`filters.optimalneighborhood` be run prior to this stage.
  [Default: false]

.. include:: filter_opts.rst

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

