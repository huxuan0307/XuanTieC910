package Core.LSU

import Core.LSU.LoadExStage.LoadDA
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadDAMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadDA)
    )
  )
}