name := "pdal-jni"

lazy val commonSettings = Seq(
  version := "1.4.0" + Environment.versionSuffix,
  scalaVersion := "2.11.8",
  crossScalaVersions := Seq("2.12.1", "2.11.8"),
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
    "-feature"),
  test in assembly := {},
  shellPrompt := { s => Project.extract(s).currentProject.id + " > " },
  commands += Command.command("publish-javastyle")((state: State) => {
    val extracted = Project extract state
    import extracted._
    val publishState = Command.process("publish", append(Seq(crossPaths := false), state))
    append(Seq(crossPaths := true), publishState)
  }),
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

lazy val root = (project in file(".")).aggregate(core, native)

lazy val core = (project in file("core")).
  settings(commonSettings: _*).
  settings(name := "pdal").
  settings(target in javah := (sourceDirectory in nativeCompile in native).value / "include").
  settings(libraryDependencies += "org.scalatest" %% "scalatest" % "3.0.0" % "test").
  dependsOn(Environment.dependOnNative(native % Runtime):_*)

lazy val native = (project in file("native")).
  settings(sourceDirectory in nativeCompile := sourceDirectory.value).
  enablePlugins(JniNative)
