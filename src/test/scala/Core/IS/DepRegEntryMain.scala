package Core.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object DepRegEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new DepRegEntry)
    )
  )
}
