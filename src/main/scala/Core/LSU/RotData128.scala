package Core.LSU

import Core.Config.XLEN
import Core.LsuConfig
import chisel3._
import chisel3.util._

class RotData128 extends Module with LsuConfig {
  val io = IO(new Bundle() {
    val dataIn     = Input(UInt((XLEN*2).W))
    val rotSel     = Input(UInt(ROT_SEL_WIDTH_8.W))
    val dataSettle = Output(UInt((XLEN*2).W))
  })
  val data = io.dataIn(XLEN-1,0) | io.dataIn(2*XLEN-1,XLEN)
  val data_rot = Seq.fill(8)(Wire(UInt(XLEN.W)))
  data_rot(0) := data(63,0)
  data_rot(1) := Cat(data(7,0) ,data(63,8))
  data_rot(2) := Cat(data(15,0) ,data(63,16))
  data_rot(3) := Cat(data(23,0) ,data(63,24))
  data_rot(4) := Cat(data(31,0) ,data(63,32))
  data_rot(5) := Cat(data(39,0) ,data(63,40))
  data_rot(6) := Cat(data(47,0) ,data(63,48))
  data_rot(7) := Cat(data(55,0) ,data(63,56))

  io.dataSettle  := Cat(0.U(XLEN.W),
    MuxLookup(io.rotSel, 0.U(XLEN.W), Seq(
    "b0000_0001".U -> data_rot(0),
    "b0000_0010".U -> data_rot(1),
    "b0000_0100".U -> data_rot(2),
    "b0000_1000".U -> data_rot(3),
    "b0001_0000".U -> data_rot(4),
    "b0010_0000".U -> data_rot(5),
    "b0100_0000".U -> data_rot(6),
    "b1000_0000".U -> data_rot(7)
  ))
  )

}
