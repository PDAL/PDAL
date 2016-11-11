package com.azavea.pdal

object Main {
  def getJson(resource: String): String = {
    val source = scala.io.Source.fromFile(resource)
    val json = source.getLines.mkString(" ")
    source.close()
    json
  }

  def main(args: Array[String]): Unit = {
    val string = getJson("/Users/daunnc/subversions/git/github/pomadchin/pdal-jni/data/las.json")
    val p = Pipeline(string)
    p.setLogLevel(9)
    println(s"p.ptr: ${p.ptr}")
    val result = p.test
    println(s"java: $result")
    p.execute
    println(s"p.getLogLevel: ${p.getLogLevel}")
    println(s"p.getMetadata: ${p.getMetadata}")
    println(s"p.getSchema: ${p.getSchema}")
    println(s"p.getLog: ${p.getLog}")
    p.dispose()
    println(s"p.ptr: ${p.ptr}")
  }

}
