(writers.spz)=

# writers.spz

The **SPZ writer** writes files in the [spz] format, designed for
storing compressed [gaussian splat] data. The format stores compressed
points, with their associated scale, rotation, color and spherical harmonics.

```{note}
The SPZ writer expects valid gaussian splat data. If scale, rotation, color, 
or opacity dimensions aren't found in the point input, zeroes will be written for
these attributes.
```

```{eval-rst}
.. embed::
```

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

```{include} writer_opts.md
```

[spz]: https://github.com/nianticlabs/spz
[gaussian splat]: https://en.wikipedia.org/wiki/Gaussian_splatting#3D_Gaussian_splatting
