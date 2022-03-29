package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadDAMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadDA)
    )
  )
}