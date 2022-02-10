package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RobEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RobEntry)
    )
  )
}
