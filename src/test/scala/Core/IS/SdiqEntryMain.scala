package Core.IS

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}

object SdiqEntryMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new SdiqEntry)
    )
  )
}
