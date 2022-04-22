package Core.LSU

import Core.LSU.LoadExStage.LoadDC
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadDCMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadDC)
    )
  )
}