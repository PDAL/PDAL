.. _development_usecases:

================================
Use Cases
================================

This document lists some use cases that we would like PDAL to be able to
satisfy.  We are speaking here about PDAL at the API level, and not the 
command line apps that come with it (although in many cases the apps will
directly address these use cases).

Note we have the cases explicitly numbered, so we can refer back to them 
from elsewhere.

Many of these cases these use cases are composable (such as 1a and 3a).

This list is intended to be representative of workflows we want to target, 
but is by no means intended to be an exhaustive list.


The Use Cases
=============

1. straight format conversion

   a. convert an ASCII file to LAS

   b. compress LAS to lossless LAZ
   
2. mosaicking and demosaicking

   a. user has a set of N tiles and wants to merge them into 1 file

   b. user has 1 large file and wants to chop up into N files, based on
      a given bbox size or a given target file size (or point count)
   
3. basic filtering/processing operations

   a. crop a file to a given spatial extent

   b. remove points with a given classification

   c. decimate the file (discard 9 of every 10 points)

   d. ...
   
4. info and metadata

   a. display (or dump to XML) the properties of the point cloud (extents,
      number of points, etc)

   b. display/dump the spatial reference system of the file

   c. display/dump any metadata in the file, such as creation date, weather
      conditions, etc

   d. collect and display statistics about the file, such as the elevation
      distribution, point density, etc

5. projection

   a. assign an SRS to a file that does not have one (or overwrite the
      existing one)

   b. reproject the points in a file to a new SRS

6. indexing and caching

   a. construct a spatial index for a point cloud, such that points within a
      relatively small given region can quickly be accessed

   b. merge (mosaic) two files that are both indexed

   c. given a file, allow for repeated access to a point during a single run
      without additional I/O

7. DEM construction, rasterization, colorization

   a. given an XYZ point cloud, produce a regular XY grid of the Z elevation
      points

   b. similar to 7a, but colorize each point based on a given color ramp and
      the elevation height Z

8. 3D visualization

   a. basic, static display of a point cloud

   b. same as (a), but with colorization of Z, simple filtering, etc

   c. use indexing and caching (6a and 6c) to provide useful interactive 
      behaviour

   d. given a DEM and a raster, display the raster draped over the surface

9. Fixup, Cleanup

   a. given a file with an incorrect header (such as incorrect extents),
      create a file with the header corrected by otherwise identical
