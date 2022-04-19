package Core.IFU_IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object ct_coreMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new ct_core)
    )
  )
}