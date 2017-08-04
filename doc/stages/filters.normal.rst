.. _filters.normal:

filters.normal
===============================================================================

``filters.normal`` returns the estimated normal and curvature for a collection
of points. The algorithm first computes the eigenvalues and eigenvectors of the
collection of points, which is comprised of the k-nearest neighbors. The normal
is taken as the eigenvector corresponding to the smallest eigenvalue. The
curvature is computed as

.. math::

  curvature = \frac{\lambda_0}{\lambda_0 + \lambda_1 + \lambda_2}

where :math:`\lambda_i` are the eigenvalues sorted in ascending order.

The filter produces four new dimensions (``NormalX``, ``NormalY``, ``NormalZ``,
and ``Curvature``), which can be analyzed directly, or consumed by downstream
stages for more advanced filtering.

The eigenvalue decomposition is performed using Eigen's
``SelfAdjointEigenSolver``. For more information see
https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html.

Normals will be automatically flipped towards the viewpoint to be consistent. By
default the viewpoint is located at the midpoint of the X and Y extents, and
1000 units above the max Z value. Users can override any of the XYZ coordinates,
or set them all to zero to effectively disable the normal flipping.

.. note::

  By default, the Normal filter will invert normals such that they are always
  pointed "up" (positive Z). If the user provides a ``viewpoint``, normals will
  instead be inverted such that they are oriented towards the viewpoint,
  regardless of the ``always_up`` flag. To disable all normal flipping, do not
  provide a ``viewpoint`` and set ``always_up=false``.

.. embed::

Example
-------

This pipeline demonstrates the calculation of the normal values (along with
curvature). The newly created dimensions are written out to BPF for further
inspection.

.. code-block:: json

    {
      "pipeline":[
        "input.las",
        {
          "type":"filters.normal",
          "knn":8
        },
        {
          "type":"writers.bpf",
          "filename":"output.bpf",
          "output_dims":"X,Y,Z,NormalX,NormalY,NormalZ,Curvature"
        }
      ]
    }

Options
-------------------------------------------------------------------------------

knn
  The number of k-nearest neighbors. [Default: **8**]

viewpoint
  A single WKT or GeoJSON 3D point. Normals will be inverted such that they are
  all oriented towards the viewpoint.

always_up
  A flag indicating whether or not normals should be inverted only when the Z
  component is negative. [Default: **true**]
