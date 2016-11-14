package com.azavea.pdal

import ch.jodersky.jni.nativeLoader

@nativeLoader("pdaljni0")
class PointView extends Native {
  // This grabs the points into some
  // class that contains members which represent
  // the data contained in the layout.
  @native def points[T]: Iterable[T]

  @native def dispose(): Unit
}
