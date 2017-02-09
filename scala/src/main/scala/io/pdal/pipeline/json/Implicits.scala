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

package io.pdal.pipeline.json

import io.pdal.pipeline._

import io.circe.{Decoder, Encoder, Json}
import io.circe.generic.extras._
import io.circe.generic.extras.auto._
import io.circe.syntax._

object Implicits extends Implicits

trait Implicits extends Serializable {
  implicit val customConfig: Configuration =
    Configuration.default.withSnakeCaseKeys.withDiscriminator("class_type")

  implicit def exprTypeEncoder[T <: ExprType]: Encoder[T] = Encoder.instance { _.toString.asJson }
  implicit val rawExprEncoder: Encoder[RawExpr] = Encoder.instance { _.json }
  implicit val pipelineConstructorEncoder: Encoder[PipelineConstructor] = Encoder.instance { constructor =>
    Json.obj(
      "pipeline" -> constructor
        .map(
          _.asJsonObject
            .remove("class_type") // remove type
            .filter { case (_, value) => !value.isNull } // cleanup options
        ).asJson
    )
  }

  implicit val rawExprDecoder: Decoder[RawExpr] = Decoder.instance { _.as[Json].right.map(RawExpr) }
  implicit val pipelineConstructorDecoder: Decoder[PipelineConstructor] = Decoder.instance {
    _.downField("pipeline").as[PipelineConstructor]
  }
}
