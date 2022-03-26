package Core.IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object IRStageMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IRStage)
    )
  )
}
