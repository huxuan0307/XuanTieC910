package Core.IFU
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}
object IFUMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IFU)
    )
  )
}