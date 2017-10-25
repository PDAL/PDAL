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

sealed trait ReaderType extends ExprType { val `type` = "readers" }

object ReaderTypes {
  case object bpf extends ReaderType
  case object buffer extends ReaderType
  case object faux extends ReaderType
  case object gdal extends ReaderType
  case object geowave extends ReaderType
  case object greyhound extends ReaderType
  case object ilvis2 extends ReaderType
  case object las extends ReaderType
  case object matlab extends ReaderType
  case object mbio extends ReaderType
  case object mrsid extends ReaderType
  case object nitf extends ReaderType
  case object oci extends ReaderType
  case object optech extends ReaderType
  case object osg extends ReaderType
  case object pcd extends ReaderType
  case object pgpointcloud extends ReaderType
  case object ply extends ReaderType
  case object pts extends ReaderType
  case object qfit extends ReaderType
  case object rxp extends ReaderType
  case object sbet extends ReaderType
  case object sqlite extends ReaderType
  case object text extends ReaderType
  case object tindex extends ReaderType
  case object terrasolid extends ReaderType
  case object icebridge extends ReaderType

  lazy val all = List(
    bpf, buffer, faux, gdal, geowave, greyhound, ilvis2, las, matlab, mbio, mrsid, nitf,
    oci, optech, osg, pcd, pgpointcloud, ply, pts, qfit, rxp, sbet, sqlite, text,
    tindex, terrasolid, icebridge
  )

  def fromName(name: String): ReaderType =
    all.find(_.name == name).getOrElse(throw new Exception(s"ReaderType $name is not supported."))
}