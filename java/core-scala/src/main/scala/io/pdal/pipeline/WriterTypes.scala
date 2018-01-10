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

sealed trait WriterType extends ExprType { val `type` = "writers" }

object WriterTypes {
  case object bpf extends WriterType
  case object gdal extends WriterType
  case object geowave extends WriterType
  case object las extends WriterType
  case object matlab extends WriterType
  case object nitf extends WriterType
  case object `null` extends WriterType
  case object oci extends WriterType
  case object optech extends WriterType
  case object pcd extends WriterType
  case object pgpointcloud extends WriterType
  case object pclvisualizer extends WriterType
  case object ply extends WriterType
  case object rialto extends WriterType
  case object sbet extends WriterType
  case object sqlite extends WriterType
  case object text extends WriterType

  lazy val all = List(
    bpf, gdal, geowave, las, matlab, nitf, oci, optech,
    pcd, pgpointcloud, pclvisualizer, ply, rialto, sbet, sqlite, text
  )

  def fromName(name: String): WriterType =
    all.find(_.name == name).getOrElse(throw new Exception(s"WriterType $name is not supported."))
}