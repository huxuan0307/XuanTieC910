package Core.IDU.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object SdiqMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new Sdiq)
    )
  )
}
