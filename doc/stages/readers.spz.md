(readers.spz)=

# readers.spz

The **SPZ reader** reads points in the [spz] format, designed for
storing compressed [3D gaussian splat] data. The format stores compressed
points, with their associated scale, rotation, color and spherical harmonics.

```{eval-rst}
.. embed::
```

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
