package Core.LSU

import Core.LSU.LoadExStage.LoadWB
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadWBMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadWB)
    )
  )
}