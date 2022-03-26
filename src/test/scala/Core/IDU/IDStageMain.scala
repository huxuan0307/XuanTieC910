package Core.IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object IDStageMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IDStage)
    )
  )
}
