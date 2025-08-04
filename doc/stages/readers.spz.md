(readers.spz)=

# readers.spz

The **SPZ reader** reads points in the [spz] format, designed for
storing compressed [3D gaussian splat] data. The format stores compressed
points, with their associated scale, rotation, color and spherical harmonics.

```{eval-rst}
.. plugin::
```

Reading an SPZ file will output the following dimensions:
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
        "type":"readers.spz",
        "filename":"inputfile.spz"
    },
    {
        "type":"writers.ply",
        "filename":"outputfile.ply"
    }
]
```

## Writing Splat Data to a Point Cloud

When carrying over 3DGS data to a standard point cloud, the splat's `f_dc_*` and `opacity` values can be
mapped to PDAL's corresponding RGB/Alpha dimensions using {ref}`filters.assign`. For `opacity`, a [sigmoid function]
needs to be applied to convert the input into a 0-1 value, which is multiplied by 255 to get our
standard 8-bit opacity. 

Color in 3D Gaussian splats is encoded as the normalized spherical harmonics value at Y(0,0) (AKA the "DC component"); 
to convert these values to 0-1, we need to multiply it by the SH0 constant `1/sqrt(4pi)`, AKA `0.2821` 
and add 0.5 to get rid of negative values. To translate that to 0-255, we multiply both of the values previously mentioned by 255.

However, depending on the input splat data, using the SH0 constant can result in some values that are out of range of 
the 8-bit color dimension; to account for this, we adjusted the constant down a bit to `0.2`. This can give colors 
that are a little desaturated, because it constrains the range of values that can be assigned; feel free to change the 
constant to your liking. This function was interpreted from Niantic's [spz library], but it should work for all 
gaussian splat data in PDAL.

```json
{
    "type":"filters.assign",
    "value": [
        "Alpha = 255 * (1 / (1 + exp(-opacity)))",
        "Red = (0.2 * 255) * f_dc_0 + 127.5",
        "Green = (0.2 * 255) * f_dc_1 + 127.5",
        "Blue = (0.2 * 255) * f_dc_2 + 127.5"
    ]
}
```

## Options

filename

: File to read. \[Required\]

```{include} reader_opts.md
```

[spz]: https://github.com/nianticlabs/spz
[3D gaussian splat]: https://en.wikipedia.org/wiki/Gaussian_splatting#3D_Gaussian_splatting
[sigmoid function]: https://en.wikipedia.org/wiki/Sigmoid_function
[spz library]: https://github.com/nianticlabs/spz/blob/bf305418722bb0663a3074f3828699df44b8c1d2/src/cc/load-spz.cc#L268
