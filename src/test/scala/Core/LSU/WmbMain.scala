package Core.LSU

import Core.LSU.Wmb.Wmb
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object WmbMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Wmb)
    )
  )
}