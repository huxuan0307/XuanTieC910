package Core.IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object ISStageMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new ISStage)
    )
  )
}