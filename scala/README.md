# PDAL Scala

[![Join the chat at https://gitter.im/PDAL/PDAL](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PDAL/PDAL?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Scala library to work with PDAL JNI Bindings.

## Using PDAL Scala with SBT

```scala
// pdal is published to maven central, but you can use following repos in addition
resolvers ++= Seq(
  Resolver.sonatypeRepo("releases"),
  Resolver.sonatypeRepo("snapshots") // for snaphots
)

libraryDependencies ++= Seq(
  "io.pdal" %% "pdal-scala" % "1.4.0"
)
```
