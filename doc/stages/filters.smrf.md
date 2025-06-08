---
jupytext:
  formats: md:myst
  text_representation:
    extension: .md
    format_name: myst
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

(filters.smrf)=

# filters.smrf

The **Simple Morphological Filter (SMRF)** classifies ground points based
on the approach outlined in {cite:p}`pingel2013improved`.

```{eval-rst}
.. embed::
```

## Example #1

The sample pipeline below uses the SMRF filter to segment ground and non-ground
returns, using default options, and writing only the ground returns to the
output file.

```{code-cell}
:tags: [remove-cell]

import os
import sys

conda_env_path = os.environ.get('CONDA_PREFIX', sys.prefix)
proj_data = os.path.join(os.path.join(conda_env_path, 'share'), 'proj')
os.environ["PROJ_DATA"] = proj_data
```

```{code-cell}
json = """
[
    {
        "bounds": "([-10425171.940, -10423171.940], [5164494.710, 5166494.710])",
        "filename": "https://s3-us-west-2.amazonaws.com/usgs-lidar-public/IA_FullState/ept.json",
        "type": "readers.ept"
    },
    {
        "type":"filters.smrf"
    },
    {
        "type":"filters.expression",
        "expression":"Classification == 2"
    },
    "output.laz"
]
"""

import pdal
pipeline = pdal.Pipeline(json)
count = pipeline.execute()
print(f"Output contains {count} points")
```

## Example #2

A more complete example, specifying some options. These match the
optimized parameters for Sample 1 given in Table 3 of {cite:p}`pingel2013improved`.

```{code-cell}
json = """
[
    {
        "bounds": "([-10425171.940, -10423171.940], [5164494.710, 5166494.710])",
        "filename": "https://s3-us-west-2.amazonaws.com/usgs-lidar-public/IA_FullState/ept.json",
        "type": "readers.ept"
    },
    {
        "type":"filters.smrf",
        "scalar":1.2,
        "slope":0.2,
        "threshold":0.45,
        "window":16.0
    },
    {
        "type":"filters.expression",
        "expression":"Classification == 2"
    },
    "output.laz"
]
"""

import pdal
pipeline = pdal.Pipeline(json)
count = pipeline.execute()
print(f"Output contains {count} points")
```

## Options

cell

: Cell size. \[Default: 1.0\]

classbits

: Selectively ignore points marked as "synthetic", "keypoint", or "withheld".
  \[Default: empty string, use all points\]

cut

: Cut net size (`cut=0` skips the net cutting step). \[Default: 0.0\]

dir

: Optional output directory for debugging intermediate rasters.

ignore

: A {ref}`range <ranges>` of values of a dimension to ignore.

returns

: Return types to include in output.  Valid values are "first", "last",
  "intermediate" and "only". \[Default: "last, only"\]

scalar

: Elevation scalar. \[Default: **1.25**\]

slope

: Slope (rise over run). \[Default: **0.15**\]

threshold

: Elevation threshold. \[Default: **0.5**\]

window

: Max window size. \[Default: **18.0**\]

```{include} filter_opts.md
```
