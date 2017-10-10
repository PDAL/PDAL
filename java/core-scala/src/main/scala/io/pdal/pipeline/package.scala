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

package io.pdal

import io.circe.Json
import io.circe.syntax._

/**
  * There is no implicit PipelineExprToString function to avoid
  * implicit casts in places where PipelineConstructor should be used.
  */

package object pipeline extends json.Implicits with Implicits with Serializable {
  type PipelineConstructor = List[PipelineExpr]

  implicit class withPipelineConstructor(list: PipelineConstructor) {
    def ~(e: PipelineExpr): PipelineConstructor = list :+ e
    def ~(e: Option[PipelineExpr]): PipelineConstructor = e.fold(list)(el => list :+ el)
    def map[B](f: PipelineExpr => B): List[B] = list.map(f)
    def toPipeline = Pipeline(list.asJson.noSpaces)
  }

  implicit def pipelineExprToConstructor[T <: PipelineExpr](expr: T): PipelineConstructor = expr :: Nil
  implicit def pipelineExprToJson(expr: PipelineExpr): Json = expr.asJson
  implicit def pipelineConstructorToJson(expr: PipelineConstructor): Json = expr.asJson
  implicit def pipelineConstructorToString(expr: PipelineConstructor): String = expr.asJson.noSpaces
}
