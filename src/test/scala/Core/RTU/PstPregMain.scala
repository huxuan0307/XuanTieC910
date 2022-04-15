package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object PstPregMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new PstPreg)
    )
  )
}
