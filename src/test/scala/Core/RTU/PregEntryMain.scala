package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object PregEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new PregEntry)
    )
  )
}
