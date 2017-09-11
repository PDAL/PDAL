import sbt._

object Dependencies {
  val circeCore = "io.circe" %% "circe-core" % Version.circe
  val circeGeneric = "io.circe" %% "circe-generic" % Version.circe
  val circeGenericExtras = "io.circe" %% "circe-generic-extras" % Version.circe
  val circeParser = "io.circe" %% "circe-parser" % Version.circe
  val jtsCore = "com.vividsolutions" % "jts-core" % Version.jtsCore
  val scalaTest = "org.scalatest" %% "scalatest" % Version.scalaTest
}
