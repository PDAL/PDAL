(filters.normal)=

# filters.normal

The **normal filter** returns the estimated normal and curvature for
a collection
of points. The algorithm first computes the eigenvalues and eigenvectors of the
collection of points, which is comprised of the k-nearest neighbors. The normal
is taken as the eigenvector corresponding to the smallest eigenvalue. The
curvature is computed as

$$
curvature = \frac{\lambda_0}{\lambda_0 + \lambda_1 + \lambda_2}
$$

where $\lambda_i$ are the eigenvalues sorted in ascending order.

The filter produces four new dimensions (`NormalX`, `NormalY`, `NormalZ`,
and `Curvature`), which can be analyzed directly, or consumed by downstream
stages for more advanced filtering.

The eigenvalue decomposition is performed using Eigen's
[SelfAdjointEigenSolver](https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html).

Normals will be automatically flipped towards positive Z, unless the [always_up]
flag is set to `false`. Users can optionally set any of the XYZ coordinates to
specify a custom [viewpoint] or set them all to zero to effectively disable the
normal flipping.

```{note}
By default, the Normal filter will invert normals such that they are always
pointed "up" (positive Z). If the user provides a [viewpoint], normals will
instead be inverted such that they are oriented towards the viewpoint,
regardless of the [always_up] flag. To disable all normal flipping, do not
provide a [viewpoint] and set [always_up] to false.
```

In addition to [always_up] and [viewpoint], users can run a refinement step (off
by default) that propagates normals using a minimum spanning tree. The
propagated normals can lead to much more consistent results across the dataset.

```{note}
To enable normal propagation, users can set [refine] to `true`.
```

```{eval-rst}
.. embed::
```

## Example

This pipeline demonstrates the calculation of the normal values (along with
curvature). The newly created dimensions are written out to BPF for further
inspection.

```json
[
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
```

## Options

knn

: The number of k-nearest neighbors. \[Default: 8\]

viewpoint

: A single WKT or GeoJSON 3D point. Normals will be inverted such that they are
  all oriented towards the viewpoint.

always_up

: A flag indicating whether or not normals should be inverted only when the Z
  component is negative. \[Default: true\]

refine

: A flag indicating whether or not to reorient normals using minimum spanning
  tree propagation. \[Default: false\]

```{include} filter_opts.md
```
