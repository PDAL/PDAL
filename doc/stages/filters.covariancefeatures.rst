.. _filters.covariancefeatures:

===============================================================================
filters.covariancefeatures
===============================================================================

This filter implements various local feature descriptors introduced that are based on the covariance matrix of a
point's neighborhood. The user can pick a set of feature descriptors by
setting the ``feature_set`` option. Currently, the only supported feature is the dimensionality_
set of feature descriptors introduced below.

Example
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
          "filename":"output.las",
          "output_dims":"X,Y,Z,Linearity,Planarity,Scattering,Verticality"
      }
  ]

Options
-------------------------------------------------------------------------------

knn
  The number of k nearest neighbors used for calculating the covariance matrix. [Default: 10]

threads
  The number of threads used for computing the feature descriptors. [Default: 1]

feature_set
  The features to be computed. Currently only supports ``Dimensionality``. [Default: "Dimensionality"]

stride
  When finding k nearest neighbors, stride determines the sampling rate. A
  stride of 1 retains each neighbor in order. A stride of two selects every
  other neighbor and so on. [Default: 1]

.. _dimensionality:

Dimensionality feature set
................................................................................
The features introduced in [Demantke2011]_ describe the shape
of the neighborhood, indicating whether
the local geometry is more linear (1D), planar (2D) or volumetric (3D) while the one introduced in
[Guinard2017]_ adds the idea of a structure being vertical.


The dimensionality filter introduces the following four descriptors that are computed from the covariance matrix of the ``knn`` neighbors:

* linearity - higher for long thin strips
* planarity - higher for planar surfaces
* scattering - higher for complex 3d neighbourhoods
* verticality - higher for vertical structures, highest for thin vertical strips

It introduces four new dimensions that hold each one of these values: ``Linearity``  ``Planarity``  ``Scattering``
and  ``Verticality``.



