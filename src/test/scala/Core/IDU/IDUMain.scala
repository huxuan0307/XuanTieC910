package Core.IDU

import Core.LSU.LoadExStage.LoadDC
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object IDUMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IDU)
    )
  )
}
