package Core.IFU

import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

object BRtype {
  def B = "b00".U  // branch
  def R = "b01".U  // jalr
  def N = "b10".U  // err instr
  def J = "b11".U  // jal

  def apply() = UInt(2.W)
}


class inst_info extends  CoreBundle {
  val data         = UInt(16.W)
  val inst_start   = Bool()
  val is_br        = Bool()
  val br_type      = UInt(2.W)
  val br_dir       = Bool()    //front 0, back 1
  val br_taken     = Bool()
  val pc           = UInt(VAddrBits.W)
}

class Loop_inst extends CoreBundle {
  val data     = UInt(32.W)
  val pc       = UInt(VAddrBits.W)
  val is_br    = Bool()
  val br_type  = UInt(2.W)
  val br_taken = Bool()
}

class LoopBufferIO extends CoreBundle {
  val inst_in          = Vec(8 + 1, Flipped(Valid(new inst_info)))
  val taken_back_br_pc = Flipped(Valid(UInt(VAddrBits.W)))
  val inst_out         = Vec(3,Decoupled(new Loop_inst))
  val flush            = Input(Bool())
  val loop_out         = Input(Bool())
  val ibuf_empty       = Input(Bool())
  val lbuf_cache       = Output(Bool())
  val lbuf_act         = Output(Bool())
}

class LoopBuffer extends Module with Config {
  val io = IO(new LoopBufferIO)

  val valid = RegInit(VecInit(Seq.fill(16)(false.B)))
  val data  = RegInit(VecInit(Seq.fill(16)(0.U.asTypeOf(new inst_info))))

  val loop_br_pc = RegInit(0.U(VAddrBits.W))
  val fill_ptr   = RegInit(0.U(5.W))
  val inst_location = RegInit(VecInit(Seq.fill(16)(0.U(4.W))))
  val inst_cnt   = RegInit(0.U(5.W))

  val inst_pos_ptr    = RegInit(0.U(4.W))

  val s_idle :: s_fill :: s_cache :: s_activate :: Nil = Enum(4)
  val state = RegInit(s_idle)

  //s_idle loop check
  when(io.taken_back_br_pc.valid && state === s_idle){
    loop_br_pc := io.taken_back_br_pc
  }

  //s_fill
  val fill_inv_check = WireInit(false.B)
  val front_br_cnt   = RegInit(false.B)

  when(state === s_fill) {
    fill_ptr := fill_ptr + PopCount(io.inst_in.map(_.valid))

    for (i <- 0 until 8 + 1) {
      val inst_info_ = io.inst_in(i).bits
      val front_br = inst_info_.br_type === BRtype.B && !inst_info_.br_dir && front_br_cnt
      when(io.inst_in(i).valid && inst_info_.inst_start) {
        when(inst_info_.is_br && (inst_info_.br_type === BRtype.R || front_br)) {
          fill_inv_check := true.B
        }
        when(inst_info_.is_br && inst_info_.br_type === BRtype.B && !inst_info_.br_dir){
          front_br_cnt := true.B
        }
      }
    }

    when(io.inst_in(0).valid) {
      for (i <- 0 until 8 + 1) {
        valid(fill_ptr + i.U) := io.inst_in(i).valid
        data(fill_ptr + i.U) := io.inst_in(i).bits
      }
    }.elsewhen(!io.inst_in(0).valid) {
      for (i <- 0 until 8) {
        valid(fill_ptr + i.U) := io.inst_in(i + 1).valid
        data(fill_ptr + i.U) := io.inst_in(i + 1).bits
      }
    }
    val inst_in_valid = io.inst_in.map(inst => inst.valid && inst.bits.inst_start)
    inst_cnt := inst_cnt + PopCount(inst_in_valid)
    for (i <- 0 until 8 + 1) {
      when(inst_in_valid(i)) {
        val inst_pos = inst_cnt + PopCount(inst_in_valid.take(i + 1)) - 1.U
        inst_location(inst_pos) := Mux(io.inst_in(0).valid, fill_ptr + i.U, fill_ptr + i.U - 1.U)
      }
    }
  }
  //s_cache
  io.lbuf_cache := state === s_cache

  //s_activate
  io.lbuf_act := state === s_activate

  when(state === s_activate) {
    val loop_inst_num = WireInit(Vec(3, UInt(4.W)))
    when(inst_cnt > 2.U) {
      val inst_remain_cnt = inst_cnt - inst_pos_ptr
      for (i <- 0 until 3) {
        loop_inst_num(i) := Mux(inst_remain_cnt > i.U, inst_location(inst_pos_ptr + i.U), inst_location(i.U - inst_remain_cnt))
      }
      when(io.inst_out(0).ready){
        inst_pos_ptr := Mux(inst_remain_cnt > 3.U, inst_pos_ptr + 3.U, 3.U - inst_remain_cnt)
      }
    }.elsewhen(inst_cnt === 2.U) {
      loop_inst_num(0) := inst_location(inst_pos_ptr)
      loop_inst_num(1) := Mux(inst_pos_ptr === 0.U, inst_location(1), inst_location(0))
      loop_inst_num(2) := inst_location(inst_pos_ptr)
      when(io.inst_out(0).ready) {
        inst_pos_ptr := Mux(inst_pos_ptr === 0.U, 1.U, 0.U)
      }
    }

    for(i <- 0 until 3){
      io.inst_out(i).valid := true.B
      io.inst_out(i).bits.pc := data(loop_inst_num(i)).pc
      io.inst_out(i).bits.is_br := data(loop_inst_num(i)).is_br
      io.inst_out(i).bits.br_type := data(loop_inst_num(i)).br_type
      io.inst_out(i).bits.br_taken := data(loop_inst_num(i)).br_taken
      io.inst_out(i).bits.data := Cat(Mux(loop_inst_num(i)===15.U, 0.U(16.W), data(loop_inst_num(i)+1.U).data),data(loop_inst_num(i)).data)
    }
  }


  switch(state) {
    is(s_idle){
      when(io.taken_back_br_pc.valid && loop_br_pc =/= io.taken_back_br_pc.bits && !io.flush){
        state    := s_fill
        fill_ptr := 0.U
        front_br_cnt := 0.U
        valid := VecInit(Seq.fill(16)(false.B))
        inst_location := VecInit(Seq.fill(16)(0.U(4.W)))
        inst_cnt := 0.U
      }.elsewhen(io.taken_back_br_pc.valid && loop_br_pc === io.taken_back_br_pc.bits && !io.flush){
        state    := s_cache
      }
    }

    is(s_fill){
      when(fill_ptr + PopCount(io.inst_in.map(_.valid)) > 16.U || fill_inv_check || io.flush){
        state := s_idle
        loop_br_pc := 0.U
        valid := VecInit(Seq.fill(16)(false.B))
      }.elsewhen(io.taken_back_br_pc.valid && loop_br_pc === io.taken_back_br_pc.bits){
        state := s_cache
      }
    }

    is(s_cache){
      when(io.flush){
        state := s_idle
        loop_br_pc := 0.U
        valid := VecInit(Seq.fill(16)(false.B))
      }.elsewhen(io.ibuf_empty){
        state := s_activate
        inst_pos_ptr := 0.U
      }
    }

    is(s_activate){
      when(io.flush && !io.loop_out){
        state := s_idle
        loop_br_pc := 0.U
        valid := VecInit(Seq.fill(16)(false.B))
      }.elsewhen(io.flush && io.loop_out){
        state := s_idle
      }
    }
  }

}
