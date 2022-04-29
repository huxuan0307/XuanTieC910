// See README.md for license details.

ThisBuild / scalaVersion     := "2.13.8"
ThisBuild / version          := "0.1.0"
ThisBuild / organization     := "com.github.huxuan0307"

val chiselVersion = "3.5.1"

lazy val root = (project in file("."))
  .settings(
    name := "XuanTieC910",
    libraryDependencies ++= Seq(
      "edu.berkeley.cs" %% "chisel3" % chiselVersion,
      "edu.berkeley.cs" %% "chiseltest" % "0.5.1" % "test"
    ),
    scalacOptions ++= Seq(
      "-language:reflectiveCalls",
      "-deprecation",
      "-feature",
      "-Xcheckinit",
      "-P:chiselplugin:genBundleElements",
    ),
    addCompilerPlugin("edu.berkeley.cs" % "chisel3-plugin" % chiselVersion cross CrossVersion.full),
    addCompilerPlugin("org.scalamacros" % "paradise" % "2.1.1" cross CrossVersion.full)
  )
  .dependsOn(difftest)
lazy val difftest = project in file("./ThirdParty/difftest")