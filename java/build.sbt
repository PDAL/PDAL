name := "pdal-jni"
version := "0.1.0-SNAPSHOT"
scalaVersion := "2.11.8"
organization := "com.azavea"
scalacOptions ++= Seq(
  "-deprecation",
  "-unchecked",
  "-Yinline-warnings",
  "-language:implicitConversions",
  "-language:reflectiveCalls",
  "-language:higherKinds",
  "-language:postfixOps",
  "-language:existentials",
  "-feature")

lazy val root = (project in file(".")).aggregate(core, native)

lazy val core = (project in file("core")).
  settings(libraryDependencies += "org.scalatest" %% "scalatest" % "3.0.0" % "test").
  settings(target in javah := (sourceDirectory in nativeCompile in native).value / "include").
  dependsOn(native % Runtime)

lazy val native = (project in file("native")).
  settings(sourceDirectory in nativeCompile := sourceDirectory.value).
  enablePlugins(JniNative)
