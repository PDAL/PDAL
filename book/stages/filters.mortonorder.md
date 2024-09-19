(filters.mortonorder)=

# filters.mortonorder

Sorts the XY data using [Morton ordering].

It's also possible to compute a reverse Morton code by reading the binary
representation from the end to the beginning. This way, points are sorted
with a good dispersement. For example, by successively selecting N
representative points within tiles:

```{figure} filters.mortonorder.img1.png
:alt: Reverse Morton indexing
:scale: 100 %
```

```{seealso}
See [LOPoCS] and [pgmorton] for some use case examples of the
Reverse Morton algorithm.
```

```{eval-rst}
.. embed::
```

## Example

```json
[
    "uncompressed.las",
    {
        "type":"filters.mortonorder",
        "reverse":"false"
    },
    {
        "type":"writers.las",
        "filename":"compressed.laz",
        "compression":"true"
    }
]
```

## Options

```{include} filter_opts.md
```

[lopocs]: https://github.com/Oslandia/lopocs
[morton ordering]: http://en.wikipedia.org/wiki/Z-order_curve
[pgmorton]: https://github.com/Oslandia/pgmorton
