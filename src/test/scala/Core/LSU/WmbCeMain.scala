package Core.LSU

import Core.LSU.Wmb.WmbCe
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object WmbCeMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new WmbCe)
    )
  )
}

