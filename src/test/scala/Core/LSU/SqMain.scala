package Core.LSU

import Core.LSU.Sq
import Core.LSU.Sq.Sq
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object SqMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Sq)
    )
  )
}

