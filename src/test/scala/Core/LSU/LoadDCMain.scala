package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LoadDCMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LoadDC)
    )
  )
}