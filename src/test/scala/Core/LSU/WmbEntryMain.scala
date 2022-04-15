package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object WmbEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new WmbEntry)
    )
  )
}