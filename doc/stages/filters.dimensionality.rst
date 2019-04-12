.. _filters.dimensionality:

===============================================================================
filters.dimensionality
===============================================================================

This filter implements the local feature descriptors introduced in [Demantke2011]_ and [Guinard2017]_.
The features introduced in [Demantke2011]_ describe the shape of the neighborhood, indicating whether
the local geometry is more linear (1D), planar (2D) or volumetric (3D) while the one introduced in
[Guinard2017]_ adds the idea of a structure being vertical.

The filter introduces the following four descriptors that are computed from the covariance matrix of the `knn` neighbors:

    linearity - higher for long thin strips
    planarity - higher for planar surfaces
    scattering - higher for complex 3d neighbourhoods
    verticality - higher for thin vertical strips

It introduces four new dimensions that hold each one of these values: ``Linearity``  ``Planarity``  ``Scattering``
and  ``Verticality``

Example
-------------------------------------------------------------------------------

.. code-block:: json

  [
      "input.las",
      {
          "type":"filters.dimensionality",
          "knn":8,
          "threads": 2
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
  The number of threads used for computing the feature descriptors

