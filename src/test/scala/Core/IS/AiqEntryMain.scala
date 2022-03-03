package Core.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object AiqEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new AiqEntry)
    )
  )
}
