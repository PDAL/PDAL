.. _java:

********************************************************************
Java
********************************************************************

.. index:: Java, JNI, Scala, Bindings


PDAL provides `Java bindings to use PDAL on JVM <https://github.com/PDAL/java>`_. It is released independently from PDAL itself as of PDAL 1.7.
Native binaries are prebuilt for Linux and MacOS and delivered in a jar, so there is no need
in building PDAL with a special flag or building JNI binaries manually.

The project consists of the following modules:

* ``pdal-native`` - with packed OS specific libraries to link PDAL to JNI proxy classes. Dependency contains bindings for ``x86_64-darwin`` and ``x86_64-linux``, other versions are not supported yet.
* ``pdal`` - with the core bindings functionality.
* ``pdal-scala`` - a Scala API package that simplifies PDAL Pipeline construction.

Versions
--------------------------------------------------------------------------------

PDAL JNI major version usually follows PDAL versioning i.e. ``pdal-java 1.8.x`` was
built and tested against ``PDAL 1.8.x`` and ``pdal-java 2.1.x`` against ``PDAL 2.x.x``.

Using PDAL Java bindings
--------------------------------------------------------------------------------

.. index:: Bindings, Java, Scala

PDAL provides `JNI bindings <https://docs.oracle.com/javase/8/docs/technotes/guides/jni/index.html>`_
that gives users access to executing
:ref:`pipeline <pipeline>` instantiations and capturing the results
in ``Java`` interfaces.
This mode of operation is useful if you are looking to have PDAL simply act as
your data format and processing handler.

Users are expected to construct their own PDAL :ref:`pipeline <pipeline>`,
execute it, and retrieve points into Java memory:

.. code-block:: scala

    import io.pdal._

    val json =
    """
      |{
      |  "pipeline":[
      |    {
      |      "filename":"1.2-with-color.las",
      |      "spatialreference":"EPSG:2993"
      |    },
      |    {
      |      "type": "filters.reprojection",
      |      "out_srs": "EPSG:3857"
      |    },
      |    {
      |       "type": "filters.delaunay"
      |    }
      |  ]
      |}
    """.stripMargin

    val pipeline = Pipeline(json)
    pipeline.validate() // check if our JSON and options were good
    pipeline.setLogLevel(8) // make it really noisy
    pipeline.execute() // execute the pipeline
    val metadata: String       = pipeline.getMetadata() // retrieve metadata
    val pvs: PointViewIterator = pipeline.getPointViews() // iterator over PointViews
    val pv: PointView          = pvs.next() // let's take the first PointView

    // load all points into JVM memory
    // PointCloud provides operations on PDAL points that
    // are loaded in this case into JVM memory as a single Array[Byte]
    val pointCloud: PointCloud = pv.getPointCloud()
    val x: Double = pointCloud.getDouble(0, DimType.X) // get a point with PointId = 0 and only a single dimensions

    // in some cases it is not neccesary to load everything into JVM memory
    // so it is possible to get only required points directly from the PointView
    val y: Double = pv.getDouble(0, DimType.Y)

    // it is also possible to get access to the triangular mesh generated via PDAL
    val mesh: TriangularMesh       = pv.getTriangularMesh()
    // the output is an Array of Triangles
    // Each Triangle contains PointIds from the PDAL point table
    val triangles: Array[Triangle] = mesh.asArray

    pv.close()
    pipeline.close()

Using PDAL Scala
--------------------------------------------------------------------------------

PDAL Scala project introduces a DSL to simplify PDAL Pipeline construction (this is the same pipeline from the section above):

.. code-block:: scala

    import io.pdal._
    import io.pdal.pipeline._

    val expression =
      ReadLas("1.2-with-color.las", spatialreference = Some("EPSG:2993")) ~
      FilterReprojection("EPSG:3857") ~
      FilterDelaunay()

    val pipeline = expression.toPipeline
    pipeline.validate() // check if our JSON and options were good
    pipeline.setLogLevel(8) // make it really noisy
    pipeline.execute() // execute the pipeline
    val metadata: String       = pipeline.getMetadata() // retrieve metadata
    val pvs: PointViewIterator = pipeline.getPointViews() // iterator over PointViews
    val pv: PointView          = pvs.next() // let's take the first PointView

    // load all points into JVM memory
    // PointCloud provides operations on PDAL points that
    // are loaded in this case into JVM memory as a single Array[Byte]
    val pointCloud: PointCloud = pv.getPointCloud()
    val x: Double = pointCloud.getDouble(0, DimType.X) // get a point with PointId = 0 and only a single dimensions

    // in some cases it is not neccesary to load everything into JVM memory
    // so it is possible to get only required points directly from the PointView
    val y: Double = pv.getDouble(0, DimType.Y)

    // it is also possible to get access to the triangular mesh generated via PDAL
    val mesh: TriangularMesh       = pv.getTriangularMesh()
    // the output is an Array of Triangles
    // Each Triangle contains PointIds from the PDAL point table
    val triangles: Array[Triangle] = mesh.asArray

    pv.close()
    pipeline.close()

It covers PDAL 2.0.x, but to use any custom DSL that is not covered by the
current Scala API you can use ``RawExpr`` type to build a ``Pipeline Expression``:

.. code-block:: scala

    import io.pdal._
    import io.pdal.pipeline._
    import io.circe.syntax._

    val pipelineWithRawExpr =
      ReadLas("1.2-with-color.las") ~
      RawExpr(Map("type" -> "filters.crop").asJson) ~
      WriteLas("1.2-with-color-out.las")

Installation
................................................................................

.. index:: Install, Java, Scala

PDAL Java artifacts are cross published for ``Scala 2.13``, ``2.12`` and ``2.11``.
However, if it is not required, a separate artifact that has no Scala specific
artifact postfix is published as well.

.. code-block:: scala

    // pdal is published to maven central, but you can use following repos in addition
    resolvers ++= Seq(
      Resolver.sonatypeRepo("releases"),
      Resolver.sonatypeRepo("snapshots") // for snaphots
    )

    libraryDependencies ++= Seq(
      "io.pdal" %% "pdal" % "x.x.x",        // core library
      "io.pdal" %  "pdal-native" % "x.x.x", // jni binaries
      "io.pdal" %% "pdal-scala" % "x.x.x"   // if scala core library (if required)
    )

The latest version is: |Maven Central|

.. |Maven Central| image:: https://maven-badges.herokuapp.com/maven-central/io.pdal/pdal/badge.png
   :target: https://search.maven.org/search?q=g:io.pdal

There is also an `example SBT PDAL Demo project <https://github.com/PDAL/java/tree/master/examples/pdal-jni>`_ in the
bindings repository, that can be used for a quick start.

Compilation
................................................................................

.. index:: Compile, Java, Scala

Development purposes (including binaries) compilation:
  1. Install PDAL (using brew / package managers (unix) / build from sources / etc)
  2. Build native libs ``./sbt native/nativeCompile`` (optionally, binaries would be built during tests run)
  3. Run ``./sbt core/test`` to run PDAL tests

Only Java development purposes compilation:
  1. Provide ``$LD_LIBRARY_PATH`` or ``$DYLD_LIBRARY_PATH``
  2. If you don't want to provide global variable you can pass ``-Djava.library.path=<path>`` into sbt:
    ``./sbt -Djava.library.path=<path>``
  3. Set ``PDAL_DEPEND_ON_NATIVE=false`` (to disable ``native`` project build)
  4. Run ``PDAL_DEPEND_ON_NATIVE=false ./sbt``

If you would like to use your own bindings binary, it is necessary to set ``java.library.path``:

.. code-block:: scala

    // Mac OS X example with manual JNI installation
    // cp -f native/target/resource_managed/main/native/x86_64-darwin/libpdaljni.2.1.dylib /usr/local/lib/libpdaljni.2.1.dylib
    // place built binary into /usr/local/lib, and pass java.library.path to your JVM
    javaOptions += "-Djava.library.path=/usr/local/lib"


You can use ``pdal-native`` dep in case you don't have installed JNI bindings and to avoid steps described above.
Dependency contains bindings for ``x86_64-darwin`` and ``x86_64-linux``, other versions are not supported yet.
