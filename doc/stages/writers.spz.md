(writers.spz)=

# writers.spz

The **SPZ writer** writes files in the [spz] format, designed for
storing compressed [gaussian splat] data. The format stores compressed
points, with their associated scale, rotation, color and spherical harmonics.

```{eval-rst}
.. plugin::
```

```{note}
The SPZ writer expects valid gaussian splat data. If scale, rotation, color, 
or opacity dimensions aren't found in the point input, zeroes will be written for
these attributes.
```

Currently, the SPZ writer expects a particular set of named dimensions for each point, 
following the conventions used in 3DGS {ref}`PLY <readers.ply>` files. Dimension names
are as follows:
- **X, Y, Z**
- **f_dc_0, f_dc_1, f_dc_2**: Red, Green and Blue colors (zeroth order spherical harmonics).
- **opacity**: scalar representation of Opacity/Alpha. 
- **scale_0, scale_1, scale_2**: X/Y/Z scale transform applied to each gaussian when rendering.
- **rot_0, rot_1, rot_2, rot_3**: W (real component)/X/Y/Z normalized rotation quaternion.
- Optional: **Spherical Harmonics** -- 0, 9, 24 or 45 dimensions labeled `f_dc_*`, with SH
coefficients as the fastest-changing axis and color as the slower-changing axis.

## Example

```json
[
    {
        "type":"readers.ply",
        "filename":"inputfile.ply"
    },
    {
        "type":"writers.spz",
        "antialiased":true,
        "filename":"outputfile.spz"
    }
]
```

## Options

filename

: File to write. \[Required\]

antialiased

: Whether to mark the output file as containing antialiased data.
  \[Default: false\]

input_orientation

: Orientation of incoming points. SPZ data is saved in RUB coordinate system by default, but points from other formats
  may use coordinates in a different orientation (like RDF for PLY format). \[Default: RUB\]

```{include} writer_opts.md
```

[spz]: https://github.com/nianticlabs/spz
[gaussian splat]: https://en.wikipedia.org/wiki/Gaussian_splatting#3D_Gaussian_splatting
