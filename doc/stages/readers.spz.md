(readers.spz)=

# readers.spz

The **SPZ reader** reads points in the [spz] format, designed for
storing compressed [3D gaussian splat] data. The format stores compressed
points, with their associated scale, rotation, color and spherical harmonics.

```{eval-rst}
.. embed::
```

Reading an SPZ file will output the following dimensions:
- **X, Y, Z**
- **f_dc_0, f_dc_1, f_dc_2**: Red, Green and Blue represented as 0-1 decimal values.
- **opacity**: Opacity/Alpha represented as a 0-1 decimal.
- **scale_0, scale_1, scale_2**: X/Y/Z scale transform applied to each gaussian.
- **rot_0, rot_1, rot_2, rot_3**: W (real component)/X/Y/Z normalized rotation quaternion.
- Optional: **Spherical Harmonics** -- 0, 9, 24 or 45 dimensions labeled `f_dc_*`, with SH
coefficients as the fastest-changing axis and color as the slower-changing axis.

## Example

```json
[
    {
        "type":"readers.spz",
        "filename":"inputfile.spz"
    },
    {
        "type":"writers.ply",
        "filename":"outputfile.ply"
    }
]
```

## Options

filename

: File to write. \[Required\]

```{include} reader_opts.md
```

[spz]: https://github.com/nianticlabs/spz
[3D gaussian splat]: https://en.wikipedia.org/wiki/Gaussian_splatting#3D_Gaussian_splatting
