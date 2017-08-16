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
