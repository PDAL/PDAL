package com.azavea.pdal

object Main {
  val iter: Iterator[Int] = Set(2).toIterator

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
    val pvi = p.pointViews()
    println(s"pvi.ptr: ${pvi.ptr}")
    println(s"pvi.hasNext: ${pvi.hasNext}")
    val pv1 = pvi.next()
    println(s"pv1.ptr: ${pv1.ptr}")
    println(s"pvi.hasNext: ${pvi.hasNext}")
    println(s"pvi.ptr: ${pvi.ptr}")
    //pvi.next()
    //println(s"pvi.hasNext: ${pvi.hasNext}")
    //println(s"pv1.ptr: ${pv1.ptr}")
    //println(s"pv.ptr: ${pvi.ptr}")
    //println(s"p.getLogLevel: ${p.getLogLevel}")
    //println(s"p.getMetadata: ${p.getMetadata}")
    //println(s"p.getSchema: ${p.getSchema}")
    //println(s"p.getLog: ${p.getLog}")
    p.dispose()
    println(s"p.ptr: ${p.ptr}")
  }

}
