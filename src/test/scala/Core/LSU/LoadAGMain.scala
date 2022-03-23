package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadAGMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadAG)
    )
  )
}