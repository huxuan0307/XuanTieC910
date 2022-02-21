package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RobExceptMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RobExcept)
    )
  )
}
