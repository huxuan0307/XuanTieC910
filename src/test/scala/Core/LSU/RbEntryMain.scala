package Core.LSU


import Core.LSU.Rb.RbEntry
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RbEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new RbEntry)
    )
  )
}
