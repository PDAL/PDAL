.. _filters:

Filters
=======

Filters operate on data as inline operations. They can remove, modify,
reorganize, and add points to the data stream as it goes by. Some filters can
only operate on dimensions they understand (consider :ref:`filters.reprojection`
doing geographic reprojection on XYZ coordinates), while others do not
interrogate the point data at all and simply reorganize or split data.

Create
------

PDAL filters commonly create new dimensions (e.g., ``HeightAboveGround``) or
alter existing ones (e.g., ``Classification``). These filters will not
invalidate an existing KD-tree.

.. note::

  We treat those filters that alter XYZ coordinates separately.

.. note::

  When creating new dimensions, be mindful of the writer you are using and
  whether or not the custom dimension can be written to disk if that is the
  desired behavior.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.approximatecoplanar
   filters.assign
   filters.cluster
   filters.colorinterp
   filters.colorization
   filters.covariancefeatures
   filters.csf
   filters.dbscan
   filters.dem
   filters.eigenvalues
   filters.estimaterank
   filters.elm
   filters.ferry
   filters.hag_delaunay
   filters.hag_dem
   filters.hag_nn
   filters.info
   filters.lof
   filters.miniball
   filters.neighborclassifier
   filters.nndistance
   filters.normal
   filters.outlier
   filters.overlay
   filters.planefit
   filters.pmf
   filters.radialdensity
   filters.reciprocity
   filters.skewnessbalancing
   filters.smrf

:ref:`filters.approximatecoplanar`
    Estimate pointwise planarity, based on k-nearest neighbors. Returns a new
    dimension ``Coplanar`` where a value of 1 indicates that a point is part of
    a coplanar neighborhood (0 otherwise).

:ref:`filters.assign`
    Assign values for a dimension range to a specified value.

:ref:`filters.cluster`
    Extract and label clusters using Euclidean distance metric. Returns a new
    dimension ``ClusterID`` that indicates the cluster that a point belongs
    to. Points not belonging to a cluster are given a cluster ID of 0.

:ref:`filters.dbscan`
    Perform Density-Based Spatial Clustering of Applications with Noise
    (DBSCAN) [Ester1996]_.

:ref:`filters.colorinterp`
    Assign RGB colors based on a dimension and a ramp

:ref:`filters.colorization`
    Fetch and assign RGB color information from a GDAL-readable datasource.

:ref:`filters.covariancefeatures`
    Filter that calculates local features based on the covariance matrix of a
    point's neighborhood.

:ref:`filters.csf`
    Label ground/non-ground returns using [Zhang2016]_.

:ref:`filters.eigenvalues`
    Compute pointwise eigenvalues, based on k-nearest neighbors.

:ref:`filters.estimaterank`
    Compute pointwise rank, based on k-nearest neighbors.

:ref:`filters.elm`
    Marks low points as noise.

:ref:`filters.ferry`
    Copy data from one dimension to another.

:ref:`filters.hag`
    Compute pointwise height above ground estimate. Requires points to be
    classified as ground/non-ground prior to estimating.

:ref:`filters.hag_delaunay`
    Compute pointwise height above ground using triangulation. Requires points to
    classified as ground/non-ground prior to estimating.

:ref:`filters.hag_dem`
    Compute pointwise height above GDAL-readable DEM raster.

:ref:`filters.lof`
    Compute pointwise Local Outlier Factor (along with K-Distance and Local
    Reachability Distance).

:ref:`filters.miniball`
    Compute a criterion for point neighbors based on the miniball algorithm.

:ref:`filters.neighborclassifier`
    Update pointwise classification using k-nearest neighbor consensus voting.

:ref:`filters.nndistance`
    Compute a distance metric based on nearest neighbors.

:ref:`filters.normal`
    Compute pointwise normal and curvature, based on k-nearest neighbors.

:ref:`filters.outlier`
    Label noise points using either a statistical or radius outlier detection.

:ref:`filters.overlay`
    Assign values to a dimension based on the extent of an OGR-readable data
    source or an OGR SQL query.

:ref:`filters.planefit`
    Compute a deviation of a point from a manifold approximating its neighbors.

:ref:`filters.pmf`
    Label ground/non-ground returns using [Zhang2003]_.

:ref:`filters.radialdensity`
    Compute pointwise density of points within a given radius.

:ref:`filters.reciprocity`
    Compute the percentage of points that are considered uni-directional
    neighbors of a point.

:ref:`filters.skewnessbalancing`
    Label ground/non-ground returns using [Bartels2010]_.

:ref:`filters.smrf`
    Label ground/non-ground returns using [Pingel2013]_.

Order
-----

There are currently three PDAL filters that can be used to reorder points. These
filters will invalidate an existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.mortonorder
   filters.randomize
   filters.sort

:ref:`filters.mortonorder`
    Sort XY data using Morton ordering (aka Z-order/Z-curve).

:ref:`filters.randomize`
    Randomize points in a view.

:ref:`filters.sort`
    Sort data based on a given dimension.

Move
----

PDAL filters that move XYZ coordinates will invalidate an existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.cpd
   filters.icp
   filters.projpipeline
   filters.reprojection
   filters.transformation

:ref:`filters.cpd`
    Compute and apply transformation between two point clouds using the
    Coherent Point Drift algorithm.

:ref:`filters.icp`
    Compute and apply transformation between two point clouds using the
    Iterative Closest Point algorithm.

:ref:`filters.projpipeline`
    Apply coordinates operation on point triplets, based on PROJ pipeline string,
    WKT2 coordinates operations or URN definitions.

:ref:`filters.reprojection`
    Reproject data using GDAL from one coordinate system to another.

:ref:`filters.transformation`
    Transform each point using a 4x4 transformation matrix.

Cull
----

Some PDAL filters will cull points, returning a point cloud that is smaller than
the input. These filters will invalidate an existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.crop
   filters.decimation
   filters.farthestpointsampling
   filters.head
   filters.iqr
   filters.locate
   filters.mad
   filters.mongo
   filters.range
   filters.sample
   filters.tail
   filters.voxelcenternearestneighbor
   filters.voxelcentroidnearestneighbor
   filters.voxeldownsize

:ref:`filters.crop`
    Filter points inside or outside a bounding box or a polygon

:ref:`filters.decimation`
    Keep every Nth point.

:ref:`filters.dem`
    Remove points that are in a raster cell but have a value far from the
    value of the raster.

:ref:`filters.farthestpointsampling`
    The Farthest Point Sampling Filter adds points from the input to the output
    PointView one at a time by selecting the point from the input cloud that is
    farthest from any point currently in the output.

:ref:`filters.head`
    Return N points from beginning of the point cloud.

:ref:`filters.iqr`
    Cull points falling outside the computed Interquartile Range for a given
    dimension.

:ref:`filters.locate`
    Return a single point with min/max value in the named dimension.

:ref:`filters.mad`
    Cull points falling outside the computed Median Absolute Deviation for a
    given dimension.

:ref:`filters.mongo`
    Cull points using MongoDB-style expression syntax.

:ref:`filters.range`
    Pass only points given a dimension/range.

:ref:`filters.sample`
    Perform Poisson sampling and return only a subset of the input points.

:ref:`filters.tail`
    Return N points from end of the point cloud.

:ref:`filters.voxelcenternearestneighbor`
    Return the point within each voxel that is nearest the voxel center.

:ref:`filters.voxelcentroidnearestneighbor`
    Return the point within each voxel that is nearest the voxel centroid.

:ref:`filters.voxeldownsize`
    Retain either first point detected in each voxel or center of a populated
    voxel, depending on mode argument.

New
---

PDAL filters can be used to split the incoming point cloud into subsets. These
filters will invalidate an existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.chipper
   filters.divider
   filters.groupby
   filters.returns
   filters.separatescanline
   filters.splitter

:ref:`filters.chipper`
    Organize points into spatially contiguous, squarish, and non-overlapping
    chips.

:ref:`filters.divider`
    Divide points into approximately equal sized groups based on a simple
    scheme.

:ref:`filters.groupby`
    Split data categorically by dimension.

:ref:`filters.returns`
    Split data by return order (e.g., 'first', 'last', 'intermediate', 'only').

:ref:`filters.separatescanline`
    Split data based on scan lines.

:ref:`filters.splitter`
    Split data based on a X/Y box length.

Join
----

Multiple point clouds can be joined to form a single point cloud. These filters
will invalidate an existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.merge

:ref:`filters.merge`
    Merge data from two different readers into a single stream.

Metadata
--------

PDAL filters can be used to create new metadata. These filters will not
invalidate an existing KD-tree.

.. note::

  :ref:`filters.cpd` and :ref:`filters.icp` can optionally create metadata as
  well, inserting the computed transformation matrix.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.hexbin
   filters.stats

:ref:`filters.hexbin`
    Tessellate XY domain and determine point density and/or point boundary.

:ref:`filters.info`
    Generate metadata about the point set, including a point count and
    spatial reference information.

:ref:`filters.stats`
    Compute statistics about each dimension (mean, min, max, etc.).


Mesh
----

Meshes can be computed from point clouds. These filters will invalidate an
existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.delaunay
   filters.greedyprojection
   filters.gridprojection
   filters.movingleastsquares
   filters.poisson

:ref:`filters.delaunay`
    Create mesh using Delaunay triangulation.

:ref:`filters.greedyprojection`
    Create mesh using the Greedy Projection Triangulation approach.

:ref:`filters.gridprojection`
    Create mesh using the Grid Projection approach [Li2010]_.

:ref:`filters.movingleastsquares`
    Data smoothing and normal estimation using the approach of [Alexa2003]_.

:ref:`filters.poisson`
    Create mesh using the Poisson surface reconstruction algorithm
    [Kazhdan2006]_.

Languages
---------

PDAL has two filters than can be used to pass point clouds to other languages.
These filters will invalidate an existing KD-tree.

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.matlab
   filters.python
   filters.julia

:ref:`filters.matlab`
    Embed MATLAB software in a pipeline.

:ref:`filters.python`
    Embed Python software in a pipeline.

:ref:`filters.julia`
    Embed Julia software in a pipeline.

Other
-----

.. toctree::
   :maxdepth: 1
   :glob:
   :hidden:

   filters.streamcallback
   filters.voxelgrid

:ref:`filters.streamcallback`
    Provide a hook for a simple point-by-point callback.

:ref:`filters.voxelgrid`
    Create a new point cloud composed of voxel centroids computed from the
    input point cloud. All incoming dimension data (e.g., intensity, RGB) will
    be lost.
