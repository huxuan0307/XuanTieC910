package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LQMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LQ)
    )
  )
}