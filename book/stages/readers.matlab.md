(readers-matlab)=

# readers.matlab

The **Matlab Reader** supports readers Matlab `.mat` files. Data
must be in a [Matlab struct], with field names that correspond to
{ref}`dimension <dimensions>` names. No ability to provide a name map is yet
provided.

Additionally, each array in the struct should ideally have the
same number of points. The reader takes its number of points
from the first array in the struct. If the array has fewer
elements than the first array in the struct, the point's field
beyond that number is set to zero.

:::{note}
The Matlab reader requires the Mat-File API from MathWorks, and it must be
explicitly enabled at compile time with the `BUILD_PLUGIN_MATLAB=ON`
variable
:::

```{eval-rst}
.. plugin::
```

```{eval-rst}
.. streamable::
```

## Example

```json
[
    {
        "type":"readers.matlab",
        "struct":"PDAL",
        "filename":"autzen.mat"
    },
    {
        "type":"writers.las",
        "filename":"output.las"
    }
]
```

## Options

filename

: Input file name. \[Required\]

```{eval-rst}
.. include:: reader_opts.rst
```

struct

: Array structure name to read. \[Default: 'PDAL'\]

[matlab struct]: https://www.mathworks.com/help/matlab/ref/struct.html
