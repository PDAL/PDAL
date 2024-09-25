(ground_command)=

# ground

````{warning}
As of PDAL v2.6.0, the `ground` command is marked as DEPRECATED. It will
be removed from the default install in PDAL v2.7 and removed completely in
PDAL v2.8.

The basic pipeline detailed in the kernel is given below in JSON.

```
[
    "input.laz",
    {
        "type": "filters.assign",
        "value": "Classification=0"
    },
    {
        "type": "filters.outlier"
    },
    {
        "type": "filters.smrf",
        "window": 18.0,
        "threshold": 0.5,
        "slope": 0.15,
        "cell": 1.0,
        "cut": 0.0,
        "scalar": 1.25,
        "returns": "last, only"
    },
    {
        "type": "filters.expression",
        "expression": "Classification==2"
    },
    "output.laz"
]
```

Written programmatically in Python, as shown below, resetting of
Classification labels, denoising, and extraction of ground returns
only can all be conditionally included.

```
pipeline = pdal.Reader("input.laz").pipeline()
if reset:
    pipeline |= pdal.Filter.assign(value="Classification=0")
if denoise:
    pipeline |= pdal.Filter.outlier()
pipeline |= pdal.Filter.smrf(window=18.0,
                             threshold=0.5,
                             slope=0.15,
                             cell=1.0,
                             cut=0.0,
                             scalar=1.25,
                             returns="last, only")
if extract:
    pipeline |= pdal.Filter.expression(expression="Classification==2")
pipeline |= pdal.Writer("output.laz")
pipeline.execute()
```
````

The `ground` command is used to segment the input point cloud into ground
versus non-ground returns using {ref}`filters.smrf` and {ref}`filters.outlier`.

```
$ pdal ground [options] <input> <output>
```

```
--input, -i         Input filename
--output, -o        Output filename
--max_window_size   Max window size
--slope             Slope
--max_distance      Max distance
--initial_distance  Initial distance
--cell_size         Cell size
--extract           Extract ground returns?
--reset             Reset classifications prior to segmenting?
--denoise           Apply statistical outlier removal prior to segmenting?
--returns           Include last returns?
--scalar            Elevation scalar?
--threshold         Elevation threshold?
--cut               Cut net size?
--ignore            A range query to ignore when processing
```
