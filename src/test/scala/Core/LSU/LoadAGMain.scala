package Core.LSU

import Core.LSU.LoadExStage.LoadAG
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadAGMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadAG)
    )
  )
}