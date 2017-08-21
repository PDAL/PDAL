/*
 * Copyright 2017 Azavea
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
