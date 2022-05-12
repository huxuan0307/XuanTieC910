package Core.LSU

import Core.LSU.DCache.DCacheArb
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object DCacheArbMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new DCacheArb)
    )
  )
}