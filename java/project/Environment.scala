import sbt.ClasspathDependency

import scala.util.Properties

object Environment {
  def either(environmentVariable: String, default: String): String =
    Properties.envOrElse(environmentVariable, default)

  lazy val versionSuffix = either("PDAL_VERSION_SUFFIX", "-SNAPSHOT")
  lazy val pdalDependOnNative = either("PDAL_DEPEND_ON_NATIVE", "true")
  def dependOnNative(native: ClasspathDependency) = if(pdalDependOnNative == "true") Seq(native) else Seq.empty
}