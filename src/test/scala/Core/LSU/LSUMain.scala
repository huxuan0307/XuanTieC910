package Core.LSU


import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object LSUMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new LSU)
    )
  )
}