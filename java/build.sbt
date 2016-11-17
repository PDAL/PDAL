name := "pdal-jni"

lazy val commonSettings = Seq(
  version := "1.4.0-SNAPSHOT",
  scalaVersion := "2.11.8",
  crossScalaVersions := Seq("2.12.0", "2.11.8"),
  organization := "io.pdal",
  description := "PDAL JNI bindings",
  licenses := Seq("BSD" -> url("https://github.com/PDAL/PDAL/blob/master/LICENSE.txt")),
  homepage := Some(url("http://www.pdal.io")),
  publishMavenStyle := true,
  bintrayRepository := "maven",
  bintrayOrganization := None,
  scalacOptions ++= Seq(
    "-deprecation",
    "-unchecked",
    "-language:implicitConversions",
    "-language:reflectiveCalls",
    "-language:higherKinds",
    "-language:postfixOps",
    "-language:existentials",
    "-feature"),
  test in assembly := {},
  shellPrompt := { s => Project.extract(s).currentProject.id + " > " },
  commands += Command.command("publish-javastyle")((state: State) => {
    val extracted = Project extract state
    import extracted._
    val publishState = Command.process("publish", append(Seq(crossPaths := false), state))
    append(Seq(crossPaths := true), publishState)
  })
)

lazy val root = (project in file(".")).aggregate(core, native)

lazy val core = (project in file("core")).
  settings(commonSettings: _*).
  settings(name := "pdal").
  settings(target in javah := (sourceDirectory in nativeCompile in native).value / "include")

lazy val native = (project in file("native")).
  settings(sourceDirectory in nativeCompile := sourceDirectory.value).
  enablePlugins(JniNative)

lazy val tests = (project in file("tests")).
  settings(commonSettings: _*).
  settings(name := "pdal-tests").
  settings(libraryDependencies += "org.scalatest" %% "scalatest" % "3.0.0" % "test").
  dependsOn(core, native % Runtime)
