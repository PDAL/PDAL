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

import scala.util.Try

trait ExprType {
  val `type`: String
  lazy val name = s"${`type`}.${this.getClass.getName.split("\\$").last}"

  override def toString = name
}

object ExprType {
  def fromName(name: String): ExprType =
    Try(FilterTypes.fromName(name))
      .getOrElse(Try(ReaderTypes.fromName(name))
        .getOrElse(Try(WriterTypes.fromName(name))
          .getOrElse(throw new Exception(s"ExprType $name is not supported."))))
}
