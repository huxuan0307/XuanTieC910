package Core.RS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object ArithInstQueueMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new ArithInstQueue)
    )
  )
}
