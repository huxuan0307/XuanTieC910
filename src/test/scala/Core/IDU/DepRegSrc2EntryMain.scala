package Core.IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object DepRegSrc2EntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new DepRegSrc2Entry)
    )
  )
}
