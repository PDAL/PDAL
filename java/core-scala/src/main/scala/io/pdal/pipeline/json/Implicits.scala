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

import io.circe.{Decoder, Encoder, Json, Printer}
import io.circe.generic.extras._
import io.circe.syntax._
import cats.syntax.either._

object Implicits extends Implicits

trait Implicits extends AutoDerivation with Serializable {
  implicit val customConfig: Configuration =
    Configuration.default.withSnakeCaseKeys.withDiscriminator("class_type")

  val pipelinePrettyPrinter: Printer = Printer.spaces2.copy(dropNullKeys = true)

  implicit def exprTypeEncoder[T <: ExprType]: Encoder[T] = Encoder.instance { _.toString.asJson }
  implicit val pipelineConstructorEncoder: Encoder[PipelineConstructor] = Encoder.instance { constructor =>
    Json.obj(
      "pipeline" -> constructor
        .flatMap {
          _.flatMap {
            case RawExpr(json) => json.asObject
            case expr => expr.asJson.asObject
          }.map {
            _.remove("class_type") // remove type
             .filter { case (_, value) => !value.isNull } // cleanup options
          }
        }.asJson
    )
  }

  implicit val rawExprDecoder: Decoder[RawExpr] = Decoder.decodeJson.emap { json =>
    Either.catchNonFatal(RawExpr(json)).leftMap(_ => "RawExpr")
  }
  implicit val pipelineConstructorDecoder: Decoder[PipelineConstructor] = Decoder.instance {
    _.downField("pipeline").as[PipelineConstructor]
  }
}
