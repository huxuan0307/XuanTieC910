package Core.IFU_IDU
import Core.IDU.IDStage
import Core.IFU.IFU
import Core.Config
import chisel3._
class IDU_test extends Module with Config{
  val ifu = Module(new IFU)
  val idu = Module(new IDStage)
  ifu.io.instData <> idu.io.in.fromIFU.instData
  ifu.io.instVld  <> idu.io.in.fromIFU.instVld
}
