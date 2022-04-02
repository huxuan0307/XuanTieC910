package Core.IFU
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._
import Core.utils._

class IPDecodeOutput extends CoreBundle {
  val auipc   = Output(Vec(8+1,Bool()))
  val branch  = Output(Vec(8+1,Bool()))
  val chgflw  = Output(Vec(8+1,Bool()))
  val con_br  = Output(Vec(8+1,Bool()))
  val dst_vld = Output(Vec(8+1,Bool()))
  val ind_br  = Output(Vec(8+1,Bool()))
  val jal     = Output(Vec(8+1,Bool()))
  val jalr    = Output(Vec(8+1,Bool()))
  val call    = Output(Vec(8+1,Bool()))
  val ret     = Output(Vec(8+1,Bool()))
  val offset  = Output(Vec(8+1,UInt(21.W)))
}

class IPDecodeIO extends CoreBundle {
  val half_inst = Input(Vec(8+1,UInt(16.W)))
  val icache_br = Input(Vec(8+1,Bool()))

  val decode_info = new IPDecodeOutput
}

class IPDecode extends Module with Config {
  val io = IO(new IPDecodeIO)

  val inst = Wire(Vec(8+1,UInt(32.W)))
  for(i <- 0 until 8){
    inst(i) := Cat(io.half_inst(i+1),io.half_inst(i))
  }
  inst(8) := Cat(0.U(16.W),io.half_inst(8))

  val ifu_decode = Seq.fill(8+1)(Module(new IFUDecode))
  for(i <- 0 until 8+1){
    ifu_decode(i).io.inst := inst(i)
    ifu_decode(i).io.icache_br := io.icache_br(i)
  }

  for(i <- 0 until 8+1){
    io.decode_info.auipc(i)   := ifu_decode(i).io.is_auipc
    io.decode_info.branch(i)  := ifu_decode(i).io.is_branch
    io.decode_info.chgflw(i)  := ifu_decode(i).io.is_chgflw
    io.decode_info.con_br(i)  := ifu_decode(i).io.is_con_br
    io.decode_info.dst_vld(i) := ifu_decode(i).io.dst_valid
    io.decode_info.ind_br(i)  := ifu_decode(i).io.is_ind_br
    io.decode_info.jal(i)     := ifu_decode(i).io.is_jal
    io.decode_info.jalr(i)    := ifu_decode(i).io.is_jalr
    io.decode_info.call(i)    := ifu_decode(i).io.is_call
    io.decode_info.ret(i)     := ifu_decode(i).io.is_return
    io.decode_info.offset(i)  := ifu_decode(i).io.offset
  }
  // pc mask   &&  changeflow mask

}
