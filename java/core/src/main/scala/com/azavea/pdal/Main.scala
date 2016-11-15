package com.azavea.pdal

import collection.JavaConverters._

/** Test object should be removed from this PR and replaced with tests */

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
    p.execute
    val pvi = p.pointViews()
    println(s"pvi.ptr: ${pvi.ptr}")
    println(s"pvi.hasNext: ${pvi.hasNext}")
    val pv1 = pvi.next()
    println(s"pv1.ptr: ${pv1.ptr}")
    println(s"pvi.hasNext: ${pvi.hasNext}")
    println(s"pvi.ptr: ${pvi.ptr}")
    println(s"pv1.ptr: ${pv1.ptr}")
    println(s"pv1.size: ${pv1.size}")
    println(s"pv1.getPackedPoints.size: ${pv1.getPackedPoints.length}")
    val layout = pv1.layout
    val list = layout.dimTypes()
    val list2 = pv1.getPackedPoint(layout.dimTypes(), 0)
    println(s"list2.length: ${list2.length}")
    val list3 = pv1.getPackedPoint(layout.dimTypes().take(1), 0)
    println(s"list3.length: ${list3.length}")
    println(list.toList)
    println(layout.findDimType("X"))
    println(layout.dimSize(layout.findDimType("X")))
    println(layout.pointSize())
    //pvi.next()
    //println(s"pvi.hasNext: ${pvi.hasNext}")
    //println(s"pv1.ptr: ${pv1.ptr}")
    //println(s"pv.ptr: ${pvi.ptr}")
    //println(s"p.getLogLevel: ${p.getLogLevel}")
    //println(s"p.getMetadata: ${p.getMetadata}")
    //println(s"p.getSchema: ${p.getSchema}")
    //println(s"p.getLog: ${p.getLog}")
    layout.dispose()
    pv1.dispose()
    pvi.dispose()
    p.dispose()
    println(s"layout.ptr: ${layout.ptr}")
    println(s"pv1.ptr: ${pv1.ptr}")
    println(s"pvi.ptr: ${pvi.ptr}")
    println(s"p.ptr: ${p.ptr}")
  }

}
