package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RobRetireMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RobRetire)
    )
  )
}