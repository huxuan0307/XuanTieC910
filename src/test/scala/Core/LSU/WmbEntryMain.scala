package Core.LSU

import Core.LSU.Wmb.WmbEntry
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object WmbEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new WmbEntry)
    )
  )
}