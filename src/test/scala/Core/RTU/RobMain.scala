package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RobMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Rob)
    )
  )
}