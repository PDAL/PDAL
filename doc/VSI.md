(VSI)=

# VSI

PDAL can read a file over a network, fetching only the portion that is needed for processing.

Here is an example.

```
$ pdal info https://github.com/PDAL/data/raw/refs/heads/main/autzen/autzen-classified.copc.laz

```

This makes use of the [GDAL Virtual File Systems](https://gdal.org/en/stable/user/virtual_file_systems.html).
