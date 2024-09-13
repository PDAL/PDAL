(readers-pcd)=

# readers.pcd

The **PCD Reader** supports reading from [Point Cloud Data (PCD)] formatted
files, which are used by the [Point Cloud Library (PCL)].

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
        "type":"readers.pcd",
        "filename":"inputfile.pcd"
    },
    {
        "type":"writers.text",
        "filename":"outputfile.txt"
    }
]
```

## Options

filename

: PCD file to read \[Required\]

```{eval-rst}
.. include:: reader_opts.rst
```

[point cloud data (pcd)]: https://pcl-tutorials.readthedocs.io/en/latest/pcd_file_format.html
[point cloud library (pcl)]: http://pointclouds.org
