(compression)=

# Compression

## Exercise

This exercise uses PDAL to compress {{ ASPRSLAS }} data into {{ LASzip }}.

1. Issue the following command in your `Conda Shell`.

   > ```console
   > $ pdal translate ./exercises/translation/interesting.las \
   > ./exercises/translation/interesting.laz
   > ```
   >
   > The translate command converts files based on their extensions.
   >
   > ```doscon
   > > pdal translate ./exercises/translation/interesting.las ^
   > ./exercises/translation/interesting.laz
   > ```
   >
   > ```{note}
   > LAS is a very fluffy binary format. Because of the way the data are
   > stored, there is ample redundant information, and {{ LASzip }} is an open
   > source solution for compressing this information. Note that we are
   > actually inflating the data here. Its laz from the workshop and we are
   > converting it to las.
   > ```

2. Verify that the laz data is compressed over the las:

   > ```console
   > $ ls -alh  ./exercises/translation/interesting.la*
   > ...
   > ```
   >
   > ```doscon
   > > dir ./exercises/translation/interesting.la*
   > ...
   > ```

Output:

> ```console
> -rw-r--r--@ 1 ogi  staff    36K Aug  8  2019 ./exercises/translation/interesting.las
> -rwxr-xr-x@ 1 ogi  staff    18K May  9 11:30 ./exercises/translation/interesting.laz
> ```

```{seealso}
{ref}`las_tutorial` contains many pointers about settings for {{ ASPRSLAS }}
data and how to achieve specific data behaviors with PDAL.
```

## Notes

1. Typical {{ LASzip }} compression is 5:1 to 8:1, depending on the type of
   {{ LiDAR }}. It is a compression format specifically for the {{ ASPRSLAS }}
   model, however, and will not be as efficient for other types of
   point cloud data.
2. You can open and view LAZ data in web browsers using <http://plas.io> but we will be using QGIS as examples.

[csv]: https://en.wikipedia.org/wiki/Comma-separated_values
[json]: https://en.wikipedia.org/wiki/JSON
[laszip]: http://laszip.org
[xml]: https://en.wikipedia.org/wiki/XML
