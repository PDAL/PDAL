import scala.util.Properties

object Environment {
  def either(environmentVariable: String, default: String): String =
    Properties.envOrElse(environmentVariable, default)

  lazy val versionSuffix  = either("PDAL_VERSION_SUFFIX", "-SNAPSHOT")
}