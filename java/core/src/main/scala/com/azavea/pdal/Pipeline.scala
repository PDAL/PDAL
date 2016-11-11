package com.azavea.pdal

import ch.jodersky.jni.nativeLoader

@nativeLoader("pdaljni0")
class Pipeline(val json: String) extends Native {
  // System.load("/abs_path_to/libpdaljni0.dylib")

  @native def initialise(): Unit
  @native def execute(): Unit
  @native def pointViews(): PointViewIterator
  /** Create an iterator of point views, with each point
    * only containing the data contained in the supplied layout
    */
  @native def pointViews(layout: PointLayout): PointViewIterator
  @native def test(): Int
  @native def dispose(): Unit
  @native def getMetadata(): String
  @native def getSchema(): String
  @native def validate(): Boolean
  @native def setLogLevel(i: Int): Unit
  @native def getLogLevel: Int
}

object Pipeline {
  def apply(json: String): Pipeline = { val p = new Pipeline(json); p.initialise(); p }
}
