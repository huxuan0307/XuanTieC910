package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RtuTopMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RtuTop)
    )
  )
}