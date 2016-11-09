scalaVersion := "2.11.8"

val geotrellisVersion = "1.0.0-cd1ca27"

libraryDependencies ++= Seq(
  "com.azavea.geotrellis" %% "geotrellis-spark" % geotrellisVersion,
  "com.azavea.geotrellis" %% "geotrellis-accumulo" % geotrellisVersion,
  "com.azavea.geotrellis" %% "geotrellis-geomesa" % geotrellisVersion,
  "org.apache.spark" %% "spark-core" % "2.0.0" % "provided",
  "org.scalatest"    %%  "scalatest" % "3.0.0" % "test"
)

assemblyMergeStrategy in assembly := {
  case "reference.conf" | "application.conf"            => MergeStrategy.concat
  case "META-INF/MANIFEST.MF" | "META-INF\\MANIFEST.MF" => MergeStrategy.discard
  case "META-INF/ECLIPSEF.RSA" | "META-INF/ECLIPSEF.SF" => MergeStrategy.discard
  case _ => MergeStrategy.first
}
