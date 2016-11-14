package com.azavea.pdal

class PointLayout extends Native {
  @native def dimTypes(): java.util.ArrayList[String]
  @native def dispose(): Unit
}
