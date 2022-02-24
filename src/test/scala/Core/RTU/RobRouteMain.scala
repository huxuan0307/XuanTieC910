package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RobRouteMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RobRoute)
    )
  )
}