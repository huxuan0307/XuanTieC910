package Core.IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object IDUMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IDU)
    )
  )
}
