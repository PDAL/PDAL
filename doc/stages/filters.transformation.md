(filters.transformation)=

# filters.transformation

The transformation filter applies an arbitrary homography
transformation, represented as a 4x4 [matrix], to each xyz triplet.

```{note}
The transformation filter does not apply or consider any spatial
reference information.
```

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example

This example rotates the points around the z-axis while translating them.

```json
[
    "untransformed.las",
    {
        "type":"filters.transformation",
        "matrix":"0 -1  0  1  1  0  0  2  0  0  1  3  0  0  0  1"
    },
    {
        "type":"writers.las",
        "filename":"transformed.las"
    }
]
```


## Further details

A full tutorial about transformation matrices is beyond the scope of this
documentation. Instead, we will provide a few pointers to introduce core
concepts, especially as pertains to PDAL's handling of the `matrix` argument.

Transformations in a 3-dimensional coordinate system can be represented
as a homography transformation using homogeneous coordinates. This 4x4
matrix can represent affine transformations describing operations like
translation, rotation, and scaling of coordinates.  In addition it can
represent perspective transformations modeling a pinhole camera.

The transformation filter's `matrix` argument is a space delimited, 16
element string. This string is simply a row-major representation of the 4x4
matrix (i.e., first four elements correspond to the top row of the
transformation matrix and so on).

In the event that readers are accustomed to an alternate representation of the
transformation matrix, we provide some simple examples in the form of pure
translations, rotations, and scaling, and show the corresponding `matrix`
string.

### Translation

A pure translation by $t_x$, $t_y$, and $t_z$ in the X, Y,
and Z dimensions is represented by the following matrix.

$$
\begin{matrix}
    1 & 0 & 0 & t_x \\
    0 & 1 & 0 & t_y \\
    0 & 0 & 1 & t_z \\
    0 & 0 & 0 & 1
\end{matrix}
$$

The JSON syntax required for such a translation is written as follows for $t_x=7$, $t_y=8$, and $t_z=9$.

```json
[
    {
        "type":"filters.transformation",
        "matrix":"1  0  0  7  0  1  0  8  0  0  1  9  0  0  0  1"
    }
]
```

### Scaling

Scaling of coordinates is also possible using a transformation matrix. The
matrix shown below will scale the X coordinates by $s_x$, the Y
coordinates by $s_y$, and Z by $s_z$.

$$
\begin{matrix}
    s_x &   0 &   0 & 0 \\
      0 & s_y &   0 & 0 \\
      0 &   0 & s_z & 0 \\
      0 &   0 &   0 & 1
\end{matrix}
$$

We again provide an example JSON snippet to demonstrate the scaling
transformation. In the example, X and Y are not scaled at all (i.e.,
$s_x=s_y=1$) and Z is magnified by a factor of 2 ($s_z=2$).

```json
[
    {
        "type":"filters.transformation",
        "matrix":"1  0  0  0  0  1  0  0  0  0  2  0  0  0  0  1"
    }
]
```

### Rotation

A rotation of coordinates by $\theta$ radians counter-clockwise about
the z-axis is accomplished with the following matrix.

$$
\begin{matrix}
    \cos{\theta} & -\sin{\theta} & 0 & 0 \\
    \sin{\theta} &  \cos{\theta} & 0 & 0 \\
               0 &             0 & 1 & 0 \\
               0 &             0 & 0 & 1
\end{matrix}
$$

In JSON, a rotation of 90 degrees ($\theta=1.57$ radians) takes the form
shown below.

```json
[
    {
        "type":"filters.transformation",
        "matrix":"0  -1  0  0  1  0  0  0  0  0  1  0  0  0  0  1"
    }
]
```

Similarly, a rotation about the x-axis by $\theta$ radians is represented
as

$$
\begin{matrix}
    1 &            0 &             0 & 0 \\
    0 & \cos{\theta} & -\sin{\theta} & 0 \\
    0 & \sin{\theta} &  \cos{\theta} & 0 \\
    0 &            0 &             0 & 1
\end{matrix}
$$

which takes the following form in JSON for a rotation of 45 degrees ($\theta=0.785$ radians)

```json
[
    {
        "type":"filters.transformation",
        "matrix":"1  0  0  0  0  0.707  -0.707  0  0  0.707  0.707  0  0  0  0  1"
    }
]
```

Finally, a rotation by $\theta$ radians about the y-axis is accomplished
with the matrix

$$
\begin{matrix}
     \cos{\theta} & 0 & \sin{\theta} & 0 \\
                0 & 1 &            0 & 0 \\
    -\sin{\theta} & 0 & \cos{\theta} & 0 \\
                0 & 0 &            0 & 1
\end{matrix}
$$

and the JSON string for a rotation of 10 degrees ($\theta=0.175$ radians) becomes

```json
[
    {
        "type":"filters.transformation",
        "matrix":"0.985  0  0.174  0  0  1  0  0  -0.174  0  0.985  0  0  0  0  1"
    }
]
```


## Options

invert

: If set to true, applies the inverse of the provided transformation matrix.
  \[Default: false\]

matrix

: A whitespace-delimited transformation matrix.
  The matrix is assumed to be presented in row-major order.
  Only matrices with sixteen elements are allowed.

```{include} filter_opts.md
```
