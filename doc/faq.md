(faq)=

# FAQ

```{index} pronounce
```

- Why do I get the error `Couldn't create ... stage of type ...`?

  In almost all cases this error occurs because you're trying to run a stage
  that is built as a plugin and the plugin (a shared library file or DLL)
  can't be found by pdal.  You can verify whether the plugin can
  be found by running `pdal --drivers`

  If you've built pdal yourself, make sure you've requested to build the
  plugin in question (set BUILD_PLUGIN_TILEDB=ON, for example,
  in CMakeCache.txt).

  If you've successfully built the plugin, a
  shared object called

  ```
  libpdal_plugin_<plugin type>_<plugin name>.<shared library extension>
  ```

  should have been created that's installed in a location where pdal
  can find it.  pdal will search
  the following paths for plugins: `.`, `./lib`, `../lib`, `./bin`,
  `../bin`.

  You can also override the default search path by setting the environment
  variable `PDAL_DRIVER_PATH` to a list of directories that pdal should search
  for plugins.

  {{ nbsp }}

- I'm missing the python filter/numpy reader. Where is it?

  If you're building PDAL from source, you'll find the python filter and numpy
  reader in a separate repository, <https://github.com/PDAL/python-plugins>. If you're
  using a package, Python support may be in a separate package, often called "python-pdal".

- Why do I get the error `Unable to convert scaled value ...`

  This error usually occurs when writing LAS files, but can occur with other
  formats.

  Simply, the output format you've chosen doesn't support values as large
  (or small) as those that you're trying to write.  For example. if the
  output format specifies 32-bit signed integers, attempting to write a
  value larger than 2,147,483,647 will cause this error, as 2,147,483,647
  is the largest 32-bit signed integer.

  The LAS format always stores X, Y and Z values as 32-bit integers.
  You can specify a scale factor to be applied to those values in order
  to change their magnitude, but their precision is limited to 32 bits.
  If the value
  you're attempting to write, when divided by the scale factor you've
  specified, is larger than 2,147,483,647, you will get this error.
  For example, if you attempt to write the value 6 with a scale factor
  of .000000001, you'll get this error, as 6 / .000000001 is 6,000,000,000,
  which is larger than the maximum integer value.

  {{ nbsp }}

- Why am I using 100GB of memory when trying to process a 10GB LAZ file?

  If you're performing an operation that is using
  {ref}`standard mode <processing_modes>`, PDAL will read all points into
  memory at once.  Compressed files, like LAZ, can decompress to much larger
  sizes before PDAL can process the data. Furthermore, some operations
  (notably {ref}`DEM creation<writers.gdal>`) can use large amounts of
  additional memory during processing before the output can be written.
  Depending on the operation, PDAL will attempt operate in
  {ref}`stream mode <processing_modes>` to limit memory consumption when possible.
  If you want to limit the dimensions loaded, you may be able to use the `dims` option
  that is available with some PDAL commands.

  {{ nbsp }}

- How do you pronounce PDAL?

  The proper spelling of the project name is PDAL, in uppercase. It is
  pronounced to rhyme with "GDAL".

  % it is properly pronounced like the dog though :) -- hobu

  {{ nbsp }}

- What is PDAL?

  PDAL is not a workflow engine for processing point cloud data.
  PDAL is a library *for making* point cloud processing workflow engines.

  {{ nbsp }}

- What is PDAL's relationship to PCL?

  PDAL is PCL's data translation cousin. PDAL is focused on providing a
  declarative pipeline syntax for orchestrating translation operations.
  PDAL also supports reading and writing PCL PCD files using {ref}`readers.pcd`
  and {ref}`writers.pcd`.

  ```{seealso}
  {ref}`about_pcl` describes PDAL and PCL's relationship.
  ```

  {{ nbsp }}

- What is PDAL's relationship to libLAS?

  The idea behind libLAS was limited to LIDAR data and basic
  manipulation. libLAS was also trying to be partially compatible
  with LASlib and LAStools. PDAL, on the other hand, aims to be
  a ultimate library and a set of tools for manipulating and processing
  point clouds and is easily extensible by its users. Howard Butler
  talked more about this history in a [GeoHipster interview] in
  2018\.

  {{ nbsp }}

- Are there any command line tools in PDAL similar to LAStools?

  Yes. The {ref}`pdal <apps>` command provides a wide range of features which go
  far beyond basic LIDAR data processing. Additionally, PDAL is licensed
  under an open source license (this applies to the whole library and
  all command line tools).

  ```{seealso}
  {ref}`apps` describes application operations you can
  achieve with PDAL.
  ```

  {{ nbsp }}

- Is there any compatibility with libLAS's LAS Utility Applications or LAStools?

  No. The the command line interface was developed from scratch with
  focus on usability and readability. You will find that the `pdal`
  command has several well-organized subcommands such as `info`
  or `translate` (see {ref}`apps`).

  {{ nbsp }}

- I get GeoTIFF errors. What can I do about them?

  ```
  (readers.las Error) Geotiff directory contains key 0 with short entry
  and more than one value.
  ```

  If {ref}`readers.las` is emitting error messages about GeoTIFF, this means
  the keys that were written into your file were incorrect or at least not
  readable by [libgeotiff]. Rewrite the file using PDAL to fix the issue:

  ```
  pdal translate badfile.las goodfile.las --writers.las.forward=all
  ```

[geohipster interview]: http://geohipster.com/2018/03/05/howard-butler-like-good-song-open-source-software-chance-immortal/
[libgeotiff]: https://github.com/OSGeo/libgeotiff
