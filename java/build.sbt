name := "pdal-jni"

lazy val commonSettings = Seq(
  version := "1.7.0" + Environment.versionSuffix,
  scalaVersion := "2.11.11",
  crossScalaVersions := Seq("2.12.4", "2.11.11"),
  organization := "io.pdal",
  description := "PDAL JNI bindings",
  licenses := Seq("BSD" -> url("https://github.com/PDAL/PDAL/blob/master/LICENSE.txt")),
  homepage := Some(url("http://www.pdal.io")),
  publishMavenStyle := true,
  pomIncludeRepository := { _ => false },
  scalacOptions ++= Seq(
    "-deprecation",
    "-unchecked",
    "-language:implicitConversions",
    "-language:reflectiveCalls",
    "-language:higherKinds",
    "-language:postfixOps",
    "-language:existentials",
    "-feature"
  ),
  test in assembly := {},
  shellPrompt := { s => Project.extract(s).currentProject.id + " > " },
  commands ++= Seq(
    Commands.processJavastyleCommand("publish"),
    Commands.processJavastyleCommand("publishSigned")
  ),
  publishArtifact in Test := false,
  publishTo := {
    val nexus = "https://oss.sonatype.org/"
    if (isSnapshot.value)
      Some("snapshots" at nexus + "content/repositories/snapshots")
    else
      Some("releases"  at nexus + "service/local/staging/deploy/maven2")
  },
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
)

lazy val root = (project in file("."))
  .settings(commonSettings: _*)
  .aggregate(`core-scala`, core, native)

lazy val `core-scala` = project
  .settings(commonSettings: _*)
  .settings(name := "pdal-scala")
  .settings(target in javah := (sourceDirectory in nativeCompile in native).value / "include")
  .settings(libraryDependencies ++= Seq(
    Dependencies.circeCore,
    Dependencies.circeGeneric,
    Dependencies.circeGenericExtras,
    Dependencies.circeParser,
    Dependencies.jtsCore,
    Dependencies.scalaTest % Test
  ))
  .settings(headerLicense := Some(HeaderLicense.ALv2("2017", "Azavea")))
  .settings(licenses := Seq("Apache-2.0" -> url("https://www.apache.org/licenses/LICENSE-2.0.html")))
  .dependsOn(core)

lazy val core = project
  .settings(commonSettings: _*)
  .settings(name := "pdal")
  .settings(target in javah := (sourceDirectory in nativeCompile in native).value / "include")
  .settings(libraryDependencies += Dependencies.scalaTest % Test)
  .dependsOn(Environment.dependOnNative(native % Runtime): _*)

lazy val native = project
  .settings(sourceDirectory in nativeCompile := sourceDirectory.value)
  .enablePlugins(JniNative)
