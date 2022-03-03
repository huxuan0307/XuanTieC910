package Core.IU.Bju

import Core.AddrConfig.PcWidth
import chisel3._
import chisel3.util._
import Core.IU.IduRfPipe2
import Utils.LookupTree

object BRUOpType {
  def jal  = "b1011000".U
  def jalr = "b1011010".U
  def beq  = "b0010000".U
  def bne  = "b0010001".U
  def blt  = "b0010100".U
  def bge  = "b0010101".U
  def bltu = "b0010110".U
  def bgeu = "b0010111".U
  def isJmp(func: UInt): Bool = func(6)
  def isBr(func: UInt): Bool = func(6,4) === "b001".U
}
class CmpltSignal extends Bundle{
  val sel = Bool()
  val iid = UInt(7.W)
  val abnormal = Bool()
  val jmpMispred = Bool()
  val bhtMispred = Bool()
}
class IduCancelSignal extends Bundle{
  val cancel = Output(Bool())
}

class IfuInterfaceSignal extends Bundle{
  val chgflwVld      = Bool()
  val chgflwPc      = UInt(PcWidth.W)
  val curPc          = UInt(PcWidth.W)
  val bhtPred        = Bool()
  val chkIdx         = UInt(25.W)
  val bhtCheckVld    = Bool()
  val bhtCondbrTaken = Bool()
}
class RtuReadSignal extends Bundle{
  // length
  val bhtPred    = Bool()
  val bhtMispred = Bool()
  val jmp        = Bool()
  val pRet       = Bool()
  val pCall      = Bool()
  val condBr     = Bool()
  val pc         = UInt(PcWidth.W)
}
class BjuOut extends Bundle{
  val toCbus = Output(new CmpltSignal)
  val toIdu  = Output(new IduCancelSignal)
  val toIfu = Output(new IfuInterfaceSignal)
  val toRtu = Output(new RtuReadSignal)
  val allowPid = Vec(2, Output(UInt(5.W)))
}
class BjuIn extends Bundle{
  val flush = Input(Bool())
  val rfPipe2  = Flipped(ValidIO(new IduRfPipe2))
  val ifuForward = Flipped(DecoupledIO(Vec(2,new BhtPredDataForward)))
}
class BjuIO extends Bundle{
  val in  = new BjuIn
  val out = Output(new BjuOut)
}
class Bju extends Module{
  val io = IO(new BjuIO())
  val flush = io.in.flush
  val pc_fifo = Module(new PcFifo)
  // forward store to pcfifo
  io.in.ifuForward.ready := pc_fifo.io.iduWrite.valid // valid to out - pcfifo is full ? bht can not in : bht can in
  when(io.in.ifuForward.fire){
    for(i<- 0 to 1) {
      pc_fifo.io.iduWrite.bits.writePcfifo(i).pc          := Mux(io.in.ifuForward.bits(i).jalr, io.in.ifuForward.bits(i).tar_pc, io.in.ifuForward.bits(i).cur_pc) // BHT data go in through, to pc_fifo
      pc_fifo.io.iduWrite.bits.writePcfifo(i).chk_idx     := io.in.ifuForward.bits(i).pred_store.chk_idx
      pc_fifo.io.iduWrite.bits.writePcfifo(i).bht_pred    := io.in.ifuForward.bits(i).pred_store.bht_pred
      pc_fifo.io.iduWrite.bits.writePcfifo(i).jmp_mispred := io.in.ifuForward.bits(i).pred_store.jmp_mispred
    }
  }
  io.out.allowPid := pc_fifo.io.iduWrite.bits.alloPid
  //==========================================================
  //                  EX1 Execution Logic
  //==========================================================
  // EX1 pipeline regs : 1. idu input data, 2. pcfifo read data
  pc_fifo.io.bjuRw.pid := io.in.rfPipe2.bits.pid
  val ex1_pipe_en = true.B // in XT910 is gatesel, should add this pipe enable?
  val ex1_pipe_rf = RegEnable(io.in.rfPipe2.bits, ex1_pipe_en)
  val ex1_pipe_pcfifo_read = RegEnable(pc_fifo.io.bjuRw.readPcfifo,ex1_pipe_en)

  val (src1, src2, op) = (ex1_pipe_rf.src0, ex1_pipe_rf.src1, ex1_pipe_rf.func)
  //----------------------------------------------------------
  //               Branch direction determination
  //----------------------------------------------------------
  //    | branch inst  |          taken condition
  // 1  |     BEQ      |           src0 == src1
  // 2  |     BNE      |           src0 != src1
  // 3  |     BLT      | (src0 < src1) || src0[63] && !src1[63]
  // 4  |     BLTU     |           src0 < src1
  // 5  |     BGE      |!(src0 < src1) &&! (src0[63] && !src1[63])
  // 6  |     BGEU     |          !(src0 < src1)
  //----------------------------------------------------------
  val bj_taken = LookupTree(op, List(
    BRUOpType.jal   ->  (true.B),
    BRUOpType.jalr  ->  (true.B),
    BRUOpType.beq   ->  (src1 === src2),
    BRUOpType.bne   ->  (src1 =/= src2),
    BRUOpType.blt   ->  (src1.asSInt < src2.asSInt),
    BRUOpType.bge   ->  (src1.asSInt >= src2.asSInt),
    BRUOpType.bltu  ->  (src1 < src2),
    BRUOpType.bgeu  ->  (src1 >= src2)
  ))

  val is_jmp = BRUOpType.isJmp(op)
  val is_br  = BRUOpType.isBr(op)

  //----------------------------------------------------------
  //                BJU jump address calculation
  //---------------------------------------------------------

  val jump_pc = Mux(op === BRUOpType.jalr,
    Cat(ex1_pipe_rf.src0(63,1), 0.U(1.W)) + ex1_pipe_rf.offset, ex1_pipe_pcfifo_read.pc + ex1_pipe_rf.offset)
  val brunch_pc = Mux(is_br&&bj_taken, ex1_pipe_pcfifo_read.pc + ex1_pipe_rf.offset ,ex1_pipe_pcfifo_read.pc + 4.U) // otherwise PC+4 TODO pc + pclengh
  val bju_jump_pc = Mux(is_jmp&&bj_taken,jump_pc,brunch_pc)

  //----------------------------------------------------------
  //                      BHT Check
  //---------------------------------------------------------- @732 XOR

  val bju_bht_mispred = bj_taken ^ ex1_pipe_pcfifo_read.bht_pred

  //----------------------------------------------------------
  //                Indirect Jump / RAS Check
  //---------------------------------------------------------- @742

  val bju_jmp_mispred = bju_jump_pc =/= ex1_pipe_pcfifo_read.pc // TODO  @ 745 rts?


  //----------------------------------------------------------
  //                Change Flow (cancel) signal                 @change flow signal @ct_iu_bju.v @752
  //----------------------------------------------------------
  val bju_mispred = bju_bht_mispred || bju_jmp_mispred
  val bju_cf = bju_mispred && ex1_pipe_en // && rtu_iu_flush
  val bju_condbr_vld = is_br // && bju_older_inst_vld TODO from ROB
  val bju_chgflw_vld = bju_mispred //&& !rtu_iu_flush_chgflw_mask  && bju_older_inst_vld

  //==========================================================
  //                     EX1 Cancel Logic
  //==========================================================





  // EX2 - branch inst predict check
  //----------------------------------------------------------
  //               Pipe2 EX2 Instruction Valid
  //----------------------------------------------------------
  val ex2_pipe_en = RegInit(false.B)
  val ex2_pipe_chgflw_vld = RegInit(false.B)
  val ex2_pipe_conbr_vld = RegInit(false.B)
  when(flush){
    ex2_pipe_en          := false.B
    ex2_pipe_chgflw_vld := false.B
    ex2_pipe_conbr_vld  := false.B
  }.otherwise{
    ex2_pipe_en          := ex1_pipe_en
    ex2_pipe_chgflw_vld  := bju_chgflw_vld
    ex2_pipe_conbr_vld   := bju_condbr_vld
  }
  //----------------------------------------------------------
  //               Pipe2 EX2 Instruction Data
  //----------------------------------------------------------
  val ex2_pipe_pcfifo_read = RegEnable(ex1_pipe_pcfifo_read,ex2_pipe_en)
  val ex2_pipe_abnormal    = RegEnable(bju_bht_mispred || bju_jmp_mispred,ex2_pipe_en)
  val ex2_pipe_bht_mispred = RegEnable(bju_bht_mispred,ex2_pipe_en)
  val ex2_pipe_jmp_mispred = RegEnable(bju_jmp_mispred,ex2_pipe_en)
  val ex2_pipe_tar_pc      = RegEnable(bju_jump_pc,ex2_pipe_en)
  val ex2_pipe_is_br       = RegEnable(is_br,ex2_pipe_en)
  val ex2_pipe_is_jmp      = RegEnable(is_jmp,ex2_pipe_en)
  //----------------------------------------------------------
  //                 Write Result to PCFIFO
  //----------------------------------------------------------
  val bju_write = 0.U.asTypeOf(new PredCheckRes)
  bju_write.bht_mispred := ex2_pipe_bht_mispred
  bju_write.jmp         := ex2_pipe_is_jmp
  bju_write.pcall       := DontCare
  bju_write.pret        := DontCare
  bju_write.condbr      := ex2_pipe_is_br
  bju_write.pc          := ex2_pipe_tar_pc
  val ex2_pipe_pcfifo_write = RegEnable(bju_write,ex2_pipe_en)
  pc_fifo.io.iduWrite.bits.writePcfifo := ex2_pipe_pcfifo_write
  //----------------------------------------------------------
  //               Interface to Complete Bus
  //----------------------------------------------------------
  io.out.toCbus.sel         := ex2_pipe_en
  // TODO io.out.toCbus.iid         := ex2_pipe_iid
  io.out.toCbus.abnormal    := ex2_pipe_abnormal
  io.out.toCbus.jmpMispred  := ex2_pipe_jmp_mispred
  io.out.toCbus.bhtMispred  := ex2_pipe_bht_mispred
  //----------------------------------------------------------
  //                  Interface to IDU
  //----------------------------------------------------------
  io.out.toIdu.cancel       := ex2_pipe_chgflw_vld
  //----------------------------------------------------------
  //                  Interface to IFU
  //----------------------------------------------------------
  io.out.toIfu.chgflwVld      := ex2_pipe_chgflw_vld
  io.out.toIfu.chgflwPc       := ex2_pipe_tar_pc
  io.out.toIfu.curPc          := ex2_pipe_pcfifo_read.pc
  io.out.toIfu.bhtPred        := ex2_pipe_pcfifo_read.bht_pred
  io.out.toIfu.chkIdx         := ex2_pipe_pcfifo_read.chk_idx
  io.out.toIfu.bhtCheckVld    := ex2_pipe_conbr_vld
  io.out.toIfu.bhtCondbrTaken := ex2_pipe_is_br
  // EX3 - branch inst result write back
  val ex3_pipe_en = RegInit(false.B)
  when(flush){
    ex3_pipe_en          := false.B
  }.otherwise{
    ex3_pipe_en          := ex2_pipe_en
  }
  val rob_read_en = ex3_pipe_en

  io.out.toRtu.bhtPred     := pc_fifo.io.robRead.ifu_pred.bht_pred
  io.out.toRtu.bhtMispred  := pc_fifo.io.robRead.bht_check.bht_mispred
  io.out.toRtu.jmp         := pc_fifo.io.robRead.bht_check.jmp
  io.out.toRtu.pRet        := pc_fifo.io.robRead.bht_check.pret
  io.out.toRtu.pCall       := pc_fifo.io.robRead.bht_check.pcall
  io.out.toRtu.condBr      := pc_fifo.io.robRead.bht_check.condbr
  io.out.toRtu.pc          := pc_fifo.io.robRead.bht_check.pc


}