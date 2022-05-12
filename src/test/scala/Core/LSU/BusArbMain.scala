package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object BusArbMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new BusArb)
    )
  )
}