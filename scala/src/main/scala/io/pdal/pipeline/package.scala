/*
 * Copyright 2016 Azavea
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

package geotrellis.pointcloud

import io.circe.Json
import io.circe.generic.extras.auto._
import io.circe.syntax._

/**
  * There is no implicit PipelineExprToString function to avoid
  * implicit casts in places where PipelineConstructor should be used.
  */

package object pipeline extends json.Implicits {
  implicit def pipelineExprToConstructor[T <: PipelineExpr](expr: T): PipelineConstructor = PipelineConstructor(expr :: Nil)
  implicit def pipelineExprToJson(expr: PipelineExpr): Json = expr.asJson
  implicit def pipelineConstructorToJson(expr: PipelineConstructor): Json = expr.asJson
  implicit def pipelineConstructorToString(expr: PipelineConstructor): String = expr.asJson.noSpaces
}
