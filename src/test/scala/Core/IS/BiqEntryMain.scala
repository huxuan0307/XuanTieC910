package Core.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object BiqEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new BiqEntry)
    )
  )
}
