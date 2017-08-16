/******************************************************************************
  * Copyright (c) 2017, hobu Inc.  (info@hobu.co)
  *
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following
  * conditions are met:
  *
  *     * Redistributions of source code must retain the above copyright
  *       notice, this list of conditions and the following disclaimer.
  *     * Redistributions in binary form must reproduce the above copyright
  *       notice, this list of conditions and the following disclaimer in
  *       the documentation and/or other materials provided
  *       with the distribution.
  *     * Neither the name of Hobu, Inc. nor the names of its
  *       contributors may be used to endorse or promote products derived
  *       from this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
  * OF SUCH DAMAGE.
  ****************************************************************************/

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
