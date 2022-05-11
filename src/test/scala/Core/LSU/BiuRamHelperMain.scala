package Core.LSU

import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}


object BiuRamHelperMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new BiuRamHelper)
    )
  )
}
