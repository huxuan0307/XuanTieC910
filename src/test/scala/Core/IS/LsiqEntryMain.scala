package Core.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LsiqEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LsiqEntry)
    )
  )
}
