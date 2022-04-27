package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object IdFifo8Main extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IdFifo8)
    )
  )
}