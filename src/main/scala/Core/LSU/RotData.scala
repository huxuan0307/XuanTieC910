package Core.LSU
import chisel3._
import chisel3.util._

class RotData extends Module {
  val io = IO(new Bundle{
    val data_in = Input(UInt(128.W))
    val rot_sel = Input(UInt(8.W))
    val data_settle_out = Output(UInt(128.W))
  })

  val data = io.data_in(63,0) | io.data_in(127,64)

  val data_rot = Wire(Vec(8, UInt(64.W)))
  data_rot(0) := data
  for(i <- 1 until 8){
    data_rot(i) := Cat(data(i*8-1,i*8-8), data(63,i*8))
  }

  val data_settle = MuxLookup(io.rot_sel, 0.U(64.W), Seq(
    0x01.U -> data_rot(0),
    0x02.U -> data_rot(1),
    0x04.U -> data_rot(2),
    0x08.U -> data_rot(3),
    0x10.U -> data_rot(4),
    0x20.U -> data_rot(5),
    0x40.U -> data_rot(6),
    0x80.U -> data_rot(7)
  ))

  io.data_settle_out := Cat(0.U(64.W), data_settle)
}
