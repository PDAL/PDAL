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

## Writing Splat data to a Point Cloud

When carrying over 3DGS data to a standard point cloud, the splat's `f_dc_*` and `opacity` values can be
mapped to PDAL's corresponding RGB/Alpha dimensions using {ref}`filters.assign`. For `opacity`, a [sigmoid function]
needs to be applied to convert the input into a 0-1 value, which is multiplied by 255 to get our
standard 8-bit opacity. 3D Gaussian splats also encode color as a scale factor; these conversions were interpreted
from Niantic's [spz] library, but they should work for all splat data in PDAL.

```json
    {
        "type":"filters.assign",
        "value": [
            "Alpha = 255 * (1 / (1 + exp(-opacity)))",
        	"Red = 12.75 * (3 * f_dc_0 + 10)",
       		"Green = 12.75 * (3 * f_dc_1 + 10)",
        	"Blue = 12.75 * (3 * f_dc_2 + 10)"
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
