(filters.python)=

# filters.python

The **Python Filter** allows [Python] software to be embedded in a
{ref}`pipeline` that allows modification of PDAL points through a [NumPy]
array.  Additionally, some global {ref}`metadata` is also
available that Python functions can interact with.

The function must have two [NumPy] arrays as arguments, `ins` and `outs`.
The `ins` array represents the points before the `filters.python`
filter and the `outs` array represents the points after filtering.

````{warning}
Make sure [NumPy] is installed in your [Python] environment.

```shell
$ python3 -c "import numpy; print(numpy.__version__)"
1.18.1
```
````

```{warning}
Each array contains all the {ref}`dimensions` of the incoming `ins`
point schema.  Each array in the `outs` list matches the [NumPy]
array of the same type as provided as `ins` for shape and type.
```

```{eval-rst}
.. plugin::
```

```python
import numpy as np

def multiply_z(ins,outs):
    Z = ins['Z']
    Z = Z * 10.0
    outs['Z'] = Z
    return True
```

1. The function must always return `True` upon success. If the function
   returned `False`, an error would be thrown and the {ref}`pipeline` exited.

2. If you want write a dimension that might not be available, you can specify
   it with the [add_dimension] option:

   ```
   "add_dimension": "NewDimensionOne"
   ```

   To create more than one dimension, this option also accepts an array:

   ```
   "add_dimension": [ "NewDimensionOne", "NewDimensionTwo", "NewDimensionThree" ]
   ```

   You can also specify the {ref}`type <types>` of the dimension using an `=`.

   ```
   "add_dimension": "NewDimensionOne=uint8"
   ```

## Modification Example

```json
[
    "file-input.las",
    {
        "type":"filters.smrf"
    },
    {
        "type":"filters.python",
        "script":"multiply_z.py",
        "function":"multiply_z",
        "module":"anything"
    },
    {
        "type":"writers.las",
        "filename":"file-filtered.las"
    }
]
```

The JSON pipeline file referenced the external `multiply_z.py` [Python] script,
which scales the `Z` coordinate by a factor of 10.

```python
import numpy as np

def multiply_z(ins,outs):
    Z = ins['Z']
    Z = Z * 10.0
    outs['Z'] = Z
    return True
```

## Predicates

Points can be retained/removed from the stream by setting true/false values
into a special "Mask" dimension in the output point array.

The example above sets the "mask" to true for points that are in
classifications 1 or 2 and to false otherwise, causing points that are not
classified 1 or 2 to be dropped from the point stream.

```python
import numpy as np

def filter(ins,outs):
   cls = ins['Classification']

   keep_classes = [1, 2]

   # Use the first test for our base array.
   keep = np.equal(cls, keep_classes[0])

   # For 1:n, test each predicate and join back
   # to our existing predicate array
   for k in range(1, len(keep_classes)):
       t = np.equal(cls, keep_classes[k])
       keep = keep + t

   outs['Mask'] = keep
   return True
```

```{note}
{ref}`filters.range` is a specialized filter that implements the exact
functionality described in this Python operation. It is likely to be much
faster than Python, but not as flexible. {ref}`filters.python` is the tool
you can use for prototyping point stream processing operations.
```

```{seealso}
If you want to read a {ref}`pipeline` of operations into a numpy
array, the [PDAL Python extension](https://pypi.python.org/pypi/PDAL)
is available.
```

### Example pipeline

```json
[
    "file-input.las",
    {
        "type":"filters.smrf"
    },
    {
        "type":"filters.python",
        "script":"filter_pdal.py",
        "function":"filter",
        "module":"anything"
    },
    {
        "type":"writers.las",
        "filename":"file-filtered.las"
    }
]
```

## Module Globals

Three global variables are added to the Python module as it is run to allow
you to get {ref}`dimensions`, {ref}`metadata`, and coordinate system
information.
Additionally, the `metadata` object can be set by the function
to modify metadata
for the in-scope {ref}`filters.python` {cpp:class}`pdal::Stage`.

```python
def myfunc(ins,outs):
    print('schema: ', schema)
    print('srs: ', spatialreference)
    print('metadata: ', metadata)
    outs = ins
    return True
```

### Setting stage metadata

```{note}
The name of the output metadata variable has changed from `metadata` to `out_metadata`.
```

Stage metadata can be created by using the `out_metadata` dictionary **global** variable.
The `name` key must be set. The type of the `value` can usually be inferred, but
can be set to one of `integer`, `nonNegativeInteger`, `double`, `bounds`,
`boolean`, `spatialreference`, `uuid` or `string`.

Children may be set using the `children` key whose value is a list of dictionaries.

```python
def myfunc(ins,outs):
  global out_metadata
  out_metadata = {'name': 'root', 'value': 'a string', 'type': 'string', 'description': 'a description', 'children': [{'name': 'somekey', 'value': 52, 'type': 'integer', 'description': 'a filter description', 'children': []}, {'name': 'readers.faux', 'value': 'another string', 'type': 'string', 'description': 'a reader description', 'children': []}]}
  return True
```

### Passing Python objects

An JSON-formatted option can be passed to the filter representing a
Python dictionary containing objects you want to use in your function.
This feature is useful in situations where you
wish to call {ref}`pipeline_command` with substitutions.

If we needed to be able to provide the Z scaling factor of [Example Pipeline]
with a
Python argument, we can place that in a dictionary and pass that to the filter
as a separate argument. This feature allows us to be able easily reuse the same
basic Python function while substituting values as necessary.

```json
[
    "input.las",
    {
        "type":"filters.python",
        "module":"anything",
        "function":"filter",
        "script":"arguments.py",
        "pdalargs":"{\"factor\":0.3048,\"an_argument\":42, \"another\": \"a string\"}"
    },
    "output.las"
]
```

With that option set, you can now fetch the [pdalargs] dictionary in your
Python script and use it:

```python
import numpy as np

def multiply_z(ins,outs):
    Z = ins['Z']
    Z = Z * float(pdalargs['factor'])
    outs['Z'] = Z
    return True
```

### Standard output and error

A `redirector` module is available for scripts to output to PDAL's log stream
explicitly. The module handles redirecting `sys.stderr` and
`sys.stdout` for you
transparently, but it can be used directly by scripts. See the PDAL source
code for more details.

## Options

script

: When reading a function from a separate [Python] file, the file name to read
  from.

source

: The literal [Python] code to execute, when the script option is
  not being used.

module

: The Python module that is holding the function to run. \[Required\]

function

: The function to call. \[Required\]

add_dimension

: A dimension name or an array of dimension names to add to the pipeline that do not already exist.

pdalargs

: A JSON dictionary of items you wish to pass into the modules globals as the
  `pdalargs` object.

```{include} filter_opts.md
```

[numpy]: http://www.numpy.org/
[python]: http://python.org/
