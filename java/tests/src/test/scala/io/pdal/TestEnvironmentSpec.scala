package io.pdal

import org.scalatest._

trait TestEnvironmentSpec extends FunSpec with Matchers with BeforeAndAfterAll {
  def getJson(resource: String): String = {
    val stream = getClass.getResourceAsStream(resource)
    val lines = scala.io.Source.fromInputStream(stream).getLines
    val json = lines.mkString(" ")
    stream.close()
    json
  }

  val json = getJson("/las.json")
  val badJson =
    """
      |{
      |  "pipeline": [
      |    "nofile.las",
      |    {
      |        "type": "filters.sort",
      |        "dimension": "X"
      |    }
      |  ]
      |}
     """.stripMargin

  val proj4String = "+proj=lcc +lat_1=43 +lat_2=45.5 +lat_0=41.75 +lon_0=-120.5 +x_0=400000 +y_0=0 +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs"

  val pipeline: Pipeline = Pipeline(json)

  override def afterAll() = {
    pipeline.dispose()
  }
}
