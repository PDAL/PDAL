(readers.terrasolid)=

# readers.terrasolid

The **Terrasolid Reader** loads points from [Terrasolid] files (.bin).
It supports both Terrasolid format 1 and format 2.

## Example

```json
[
    {
        "type":"readers.terrasolid",
        "filename":"autzen.bin"
    },
    {
        "type":"writers.las",
        "filename":"output.las"
    }
]
```

## Options

filename

: Input file name \[Required\]

```{include} reader_opts.md
```

[terrasolid]: https://terrasolid.com/products/terrascan/
