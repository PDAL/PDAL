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

import io.circe._
import io.circe.syntax._
import io.circe.parser._

import org.scalatest._

class PipelineExpressionsSpec extends FunSpec with Matchers with BeforeAndAfterAll {
  describe("Pipeline Expressions spec") {
    it("should print a correct json, using DSL") {
      val expected =
        """
          |{
          |  "pipeline" : [
          |    {
          |      "filename" : "/path/to/las",
          |      "type" : "readers.las"
          |    },
          |    {
          |      "type" : "filters.crop"
          |    },
          |    {
          |      "filename" : "/path/to/new/las",
          |      "type" : "writers.las"
          |    }
          |  ]
          |}
        """.stripMargin


      val pc: PipelineConstructor = LasRead("/path/to/las") ~ CropFilter() ~ LasWrite("/path/to/new/las")
      val pipelineJson: Json = LasRead("/path/to/las") ~ CropFilter() ~ LasWrite("/path/to/new/las")

      parse(expected) match {
        case Right(r) => pipelineJson shouldBe r
        case Left(e) => throw e
      }
    }

    it("should print a correct json, using RAW JSON") {
      val expected =
        """
          |{
          |  "pipeline" : [
          |    {
          |      "filename" : "/path/to/las",
          |      "type" : "readers.las"
          |    },
          |    {
          |      "type" : "filters.crop"
          |    },
          |    {
          |      "filename" : "/path/to/new/las",
          |      "type" : "writers.las"
          |    }
          |  ]
          |}
        """.stripMargin

      val pipelineJson: Json = LasRead("/path/to/las") ~ RawExpr(Map("type" -> "filters.crop").asJson) ~ LasWrite("/path/to/new/las")

      parse(expected) match {
        case Right(r) => pipelineJson shouldBe r
        case Left(e) => throw e
      }
    }
  }
}
