(filters.sort)=

# filters.sort

The sort filter orders a point view based on the values of a {ref}`dimensions`. The
sorting can be done in increasing (ascending) or decreasing (descending) order.

```{eval-rst}
.. embed::
```

## Example

```json
[
    "unsorted.las",
    {
        "type":"filters.sort",
        "dimension":"X",
        "order":"ASC"
    },
    "sorted.las"
]
```

```{note}
See {ref}`filters.label_duplicates` for an example of using multiple {ref}`filters.sort` sequentially. See {ref}`filters.groupby` for an example using a single {ref}`filters.sort` with multiple dimensions.
```

## Options

dimensions

: A list of 1 or more dimensions by which to sort points.

    When multiple dimensions are specified points are sorted [lexicographically](https://en.wikipedia.org/wiki/Lexicographic_order), where the first dimension listed is most significant and the last listed is least significant. This follows how multi-column sort works in most spreadsheet and data analysis programs.  \[Required\]

    ```{note}
    In PDAL 2.11 the significance of the dimension list changed from least-to-most to most-to-least.
    ```

order

: The order in which to sort, ASC or DESC. This applies to all dimensions. \[Default: "ASC"\]

algorithm

: Use a NORMAL or a [STABLE](https://en.wikipedia.org/wiki/Sorting_algorithm#Stability) sorting algorithm. This option will be respected only when sorting by a single dimension. \[Default: "NORMAL"\]

```{include} filter_opts.md
```
