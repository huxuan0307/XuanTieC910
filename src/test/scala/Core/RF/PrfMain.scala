package Core.RF

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object PrfMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Prf)
    )
  )
}
