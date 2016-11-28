package io.pdal

import ch.jodersky.jni.nativeLoader

class Pipeline(val json: String) extends Native {
  Pipeline // reference the object so that the nativeLoader will load the JNI native libraries

  @native def initialise(): Unit
  @native def execute(): Unit
  @native def pointViews(): PointViewIterator
  @native def dispose(): Unit
  @native def getMetadata(): String
  @native def getSchema(): String
  @native def validate(): Boolean
  @native def setLogLevel(i: Int): Unit
  @native def getLogLevel: Int
  @native def getLog: String
}

@nativeLoader("pdaljni.1.4")
object Pipeline {
  def apply(json: String): Pipeline = { val p = new Pipeline(json); p.initialise(); p }
}
