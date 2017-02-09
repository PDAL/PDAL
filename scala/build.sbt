name := "pdal-scala"

version := s"1.4.0${Environment.versionSuffix}"
scalaVersion := "2.11.8"
crossScalaVersions := Seq("2.12.1", "2.11.8")
organization := "io.pdal"
description := "PDAL Scala library"
licenses := Seq("BSD" -> url("https://github.com/PDAL/PDAL/blob/master/LICENSE.txt"))
homepage := Some(url("http://www.pdal.io"))
publishMavenStyle := true
pomIncludeRepository := { _ => false }
scalacOptions ++= Seq(
  "-deprecation",
  "-unchecked",
  "-language:implicitConversions",
  "-language:reflectiveCalls",
  "-language:higherKinds",
  "-language:postfixOps",
  "-language:existentials",
  "-feature")
test in assembly := {}
shellPrompt := { s => Project.extract(s).currentProject.id + " > " }
commands ++= Seq(
  Commands.processJavastyleCommand("publish"),
  Commands.processJavastyleCommand("publishSigned")
)
publishArtifact in Test := false
publishTo := {
  val nexus = "https://oss.sonatype.org/"
  if (isSnapshot.value)
    Some("snapshots" at nexus + "content/repositories/snapshots")
  else
    Some("releases" at nexus + "service/local/staging/deploy/maven2")
}
pomExtra := (
  <scm>
    <url>git@github.com:PDAL/PDAL.git</url>
    <connection>scm:git:git@github.com:PDAL/PDAL.git</connection>
  </scm>
  <developers>
    <developer>
      <id>pomadchin</id>
      <name>Grigory Pomadchin</name>
      <url>http://github.com/pomadchin/</url>
    </developer>
  </developers>
)

javaOptions += s"-Djava.library.path=${Environment.ldLibraryPath}"

libraryDependencies ++= Seq(
  "io.circe" %% "circe-core" % Environment.circeVersion,
  "io.circe" %% "circe-generic" % Environment.circeVersion,
  "io.circe" %% "circe-generic-extras" % Environment.circeVersion,
  "io.circe" %% "circe-literal" % Environment.circeVersion,
  "io.circe" %% "circe-parser" % Environment.circeVersion,  
  "io.pdal"  %% "pdal" % "1.4.0",
  "com.vividsolutions" % "jts-core" % "1.14.0",
  "org.scalatest" %% "scalatest" % "3.0.0" % "test"
)
