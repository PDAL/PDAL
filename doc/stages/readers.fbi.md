(readers.fbi)=

# readers.fbi

The **FBI Reader** supports reading from `FastBinary format` files. FastBinary
is the internal format for TerraScan. This driver allows to read FBI files in
version 1 of the FBI specification.

```{note}
Support for all point attributes in LAS 1.2 format so data can be converted between LAS 1.2
and Fast Binary formats without any loss of point attribute information.
```

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
        "type":"readers.fbi",
        "filename":"inputfile.fbi"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

filename

: FBI file to read \[Required\]
