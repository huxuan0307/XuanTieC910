package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RetireMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Retire)
    )
  )
}
