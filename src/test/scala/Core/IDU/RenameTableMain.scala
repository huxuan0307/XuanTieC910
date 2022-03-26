package Core.IDU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RenameTableMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RenameTable)
    )
  )
}
