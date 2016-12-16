# PDAL Java bindings

[![Join the chat at https://gitter.im/PDAL/PDAL](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PDAL/PDAL?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Java bindings to use PDAL on JVM.

## How to compile

1. Install PDAL (using brew / package managers (unix) / build from sources / etc)
2. Build native libs `./sbt native/nativeCompile` (optionally, binaries would be built during tests run)
3. Run `./sbt core/test` to run PDAL tests

If you want to run project and to use already built bindings:

1. Provide `$LD_LIBRARY_PATH` or `$DYLD_LIBRARY_PATH`
2. If you don't want to provide global variable you can pass `-Djava.library.path=<path>` into sbt:
  `./sbt -Djava.library.path=<path>`
3. Set `PDAL_DEPEND_ON_NATIVE=false`: `PDAL_DEPEND_ON_NATIVE=false ./sbt`, or set it as global.

Finally your possible command to launch sbt can be the following:

```bash
PDAL_DEPEND_ON_NATIVE=false ./sbt -Djava.library.path=<path>
```

### Possible issues and solutions

1. If you don't have PDAL installed global you can change [this](./java/native/src/CMakeLists.txt#L25) line to:

  ```cmake
  set(CMAKE_CXX_FLAGS "$ENV{PDAL_LD_FLAGS} $ENV{PDAL_CXX_FLAGS} -std=c++11")
  ```
  In this case sbt launch would be the following:

  ```bash
  PDAL_LD_FLAGS=`pdal-config --libs` PDAL_CXX_FLAGS=`pdal-config --includes` ./sbt
  ```

2. You can have bad dynamic linking issue (somehow spoiled environment), the quick workaround would be to replace [this](./java/native/src/CMakeLists.txt#L25) line to:

  ```cmake
  set(CMAKE_CXX_FLAGS "-L<path to dynamic libs> -std=c++11")
  ```

## Using with SBT

```scala
// pdal is bulished to maven central, but you can use following repos in addition
resolvers ++= Seq(
  Resolver.sonatypeRepo("releases"),
  Resolver.sonatypeRepo("snapshots") // for snaphots
)

libraryDependencies ++= Seq(
  "io.pdal" %% "pdal" % "1.4.0"
)
```

It's required to have native JNI binary into your app classpath (if you decided to build jni with this sbt project):

```scala
// Mac OS X example with manual jni installation 
// It's strongly recommended to use WITH_PDAL_JNI flag to build the whole PDAL
// cp -f native/target/resource_managed/main/native/x86_64-darwin/libpdaljni.1.4.dylib /usr/local/lib/libpdaljni.1.4.dylib
// place built binary into /usr/local/lib, and pass java.library.path to your JVM
javaOptions += "-Djava.library.path=/usr/local/lib"
```