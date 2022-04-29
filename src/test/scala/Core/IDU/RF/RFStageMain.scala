package Core.IDU.RF

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RFStageMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RFStage)
    )
  )
}
