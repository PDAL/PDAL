(readers.ply)=

# readers.ply

The **ply reader** reads points and vertices from the [polygon file format], a
common file format for storing three dimensional models.  The ply reader
can read ASCII and binary ply files.

```{eval-rst}
.. embed::
```

```{eval-rst}
.. streamable::
```

## Example

```json
[
    {
        "type":"readers.ply",
        "filename":"inputfile.ply"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

filename

: ply file to read \[Required\]

```{include} reader_opts.md
```

[polygon file format]: http://paulbourke.net/dataformats/ply/
