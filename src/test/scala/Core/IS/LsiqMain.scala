package Core.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LsiqMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Lsiq)
    )
  )
}
