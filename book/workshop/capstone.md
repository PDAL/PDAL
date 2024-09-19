(capstone)=

# Final Project

```{eval-rst}
.. include:: ./includes/substitutions.rst
```

```{index} capstone, project
```

The final project brings together a number of PDAL processing workflow
operations into a single effort It builds upon the exercises to enable
you to use the capabilities of PDAL in a coherent processing strategy, and it
will give you ideas about how to orchestrate PDAL in the context of larger data
processing scenarios.

Given the following pipeline for fetching the data, complete the rest of the tasks:

```json
{
    "pipeline": [
        {
            "type": "readers.ept",
            "filename":"https://s3-us-west-2.amazonaws.com/usgs-lidar-public/MA_CentralEastern_1_2021/ept.json",
            "bounds":"([-7911859.4, -7911077.0],[5213787.7, 5214543.3],[-40, 400])"
        },
        {
            "type": "filters.expression",
            "expression": "Classification < 20"
        },
        {
            "type": "writers.las",
            "compression": "true",
            "minor_version": "4",
            "dataformat_id": "0",
            "filename":"public-garden.laz"
        },
        {
            "type": "writers.copc",
            "filename": "public-garden.copc.laz",
            "forward": "all"
        }
    ]
}
```

- Read data from an EPT resource using {ref}`readers.ept` (See {ref}`workshop-entwine`)

```{note}
The particular data we are pulling has some high classification values due to how it was processed.
These aren't useful to us, and we can use {ref}`filters.expression` in the pipeline to only write
points with a classification value under 20.
```

- Thin it to 1.0 meter spacing using {ref}`filters.sample` (See {ref}`workshop-thinning`)
- Filter out noise using {ref}`filters.outlier` (See {ref}`workshop-denoising`)
- Classify ground points using {ref}`filters.smrf` (See {ref}`workshop-ground`)
- Compute height above ground using {ref}`filters.hag_nn`
- Generate a digital terrain model (DTM) using {ref}`writers.gdal` (See {ref}`workshop-dtm`)
- Find the average vegetative height model using {ref}`writers.gdal`

```{note}
You should review specific exercises for specifics on how to
achieve each task.
```
