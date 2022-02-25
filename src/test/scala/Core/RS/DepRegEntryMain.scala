package Core.RS

import Core.RTU.PregEntry
import Core.RTU.PregEntryMain.args
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object DepRegEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new DepRegEntry)
    )
  )
}
