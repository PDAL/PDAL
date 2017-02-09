package io.pdal.pipeline

import io.pdal.PointCloud
import com.vividsolutions.jts.geom.Coordinate

object Implicits extends Implicits

trait Implicits extends Serializable {
  implicit class withPointCloudMethods(pointCloud: PointCloud) {
    def getCoordinate(i: Int) =
      new Coordinate(pointCloud.getX(i), pointCloud.getY(i), pointCloud.getZ(i))
  }
}
