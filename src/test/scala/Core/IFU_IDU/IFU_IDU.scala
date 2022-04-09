package Core.IFU_IDU
import Core.IFU.IFU
import chisel3.stage.{ChiselGeneratorAnnotation, ChiselStage}
object IFU_IDU extends App {
  (new ChiselStage).execute(
    args,
    Seq(
      ChiselGeneratorAnnotation(() => new IDU_test)
    )
  )
}