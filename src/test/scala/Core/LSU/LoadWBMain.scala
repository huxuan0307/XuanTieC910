package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadWBMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadWB)
    )
  )
}