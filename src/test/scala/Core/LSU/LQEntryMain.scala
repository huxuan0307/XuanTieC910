package Core.LSU

import Core.LSU.Lq.LQEntry
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LQEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LQEntry)
    )
  )
}