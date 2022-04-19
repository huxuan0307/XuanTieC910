package Core.RTU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object PstPregEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new PstPregEntry)
    )
  )
}
