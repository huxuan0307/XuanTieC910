package Core.IDU.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object BiqMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Biq)
    )
  )
}
