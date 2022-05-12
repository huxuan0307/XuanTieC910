package Core.LSU


import Core.LSU.Rb.Rb
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object RbMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Rb)
    )
  )
}