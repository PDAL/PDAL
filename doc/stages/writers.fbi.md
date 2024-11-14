(writers.fbi)=

# writers.fbi

The **fbi writer** writes the `FastBinary file format`. FastBinary is the
internal format for [TerraScan](https://terrasolid.com/products/terrascan/).
This driver allows to write FBI files in version 1 of the FBI specification.

```{note}
Support for all point attributes in LAS 1.2 format so data can be converted between LAS 1.2
and Fast Binary formats without any loss of point attribute information.
```

Point attributes are stored as attribute streams instead of point records. This makes it
possible for reading software to read only those attributes it is interested in.

```{eval-rst}
.. embed::
```

## Example

```json
[
    {
        "type":"readers.las",
        "filename":"inputfile.las"
    },
    {
        "type":"writers.fbi",
        "filename":"outputfile.fbi"
    }
]
```

## Options

filename

: FBI file to write \[Required\]
