package Core.IFU
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}
object PCfifoMain extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new PCfifo)
    )
  )
}