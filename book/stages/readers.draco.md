(readers-draco)=

# readers.draco

[Draco] is a library for compressing and decompressing 3D geometric meshes and
point clouds and was designed and built for compression efficiency and speed.
The code supports compressing points, connectivity information, texture coordinates,
color information, normals, and any other generic attributes associated with geometry.

## Example

```json
[
    {
        "type": "readers.draco",
        "filename": "color.las"
    }
]
```

## Options

filename

: Input file name. \[Required\]

```{eval-rst}
.. include:: reader_opts.rst
```

[draco]: https://github.com/google/draco
