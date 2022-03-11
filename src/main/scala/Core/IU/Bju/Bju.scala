package Core.IU.Bju

import Core.AddrConfig.PcWidth
import chisel3._
import chisel3.util._
import Core.IU.{IduRfPipe2, unitSel}
import Core.IUConfig.PcFifoAddr
import Core.IntConfig.InstructionIdWidth
import Core.ROBConfig.NumRobReadEntry
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
class bjuCmpltSignal extends Bundle{
  val abnormal = Bool()
  val bhtMispred = Bool()
  val jmpMispred = Bool()
  val iid = UInt(InstructionIdWidth.W)
  val sel = Bool()
}
class iduFeedbackSignal extends Bundle{
  val misPredStall = (Bool())
  val cancel   = (Bool())
  val allowPid = Vec(2, (UInt(PcFifoAddr.W)))
}
class ifuChangeFlow extends Bundle{
  val bhtCheckVld    = Bool()
  val bhtCondbrTaken = Bool()
  val bhtPred        = Bool()
  val chgflwPc       = UInt(PcWidth.W)
  val chgflwVld      = Bool()
  val chkIdx         = UInt(25.W)
  val curPc          = UInt(PcWidth.W)
  val misPredStall   = Bool()
  val pcFifoFull     = Bool()
 // TODO with trait hasVecExtends
}
class rtuReadSignal extends Bundle{
  // length
  val bhtPred    = Bool()
  val bhtMispred = Bool()
  val jmp        = Bool()
  val pRet       = Bool()
  val pCall      = Bool()
  val condBr     = Bool()
  val pc         = UInt(PcWidth.W)
}
class isuAllowFifo extends Bundle{
 val instNum = UInt(3.W) // temp define 3, indicate how many inst can store in PCfifo
 val instVld = Bool()
}
class rtuControl extends Bundle{
  val flushChgflwMask = Bool()
  val flushFe = Bool() // TODO whats this?
  val robReadPcfifovld = Vec(NumRobReadEntry,Bool())
  val robReadPcfifovldGateEn = Bool()
  val flush = Bool()
}


class BjuIn extends Bundle{
  val rtuIn      = Input(new rtuControl)
  val isIn       = Input(new isuAllowFifo)
  val rfPipe2    = Input(new IduRfPipe2)
  val sel        = Input(new unitSel)
  val ifuForward = Input(Vec(2,new ifuDataForward))
  val specialPid = Input(UInt(PcFifoAddr.W))
}
class BjuOut extends Bundle{
  val toCbus    = Output(new bjuCmpltSignal)
  val toIdu     = Output(new iduFeedbackSignal)
  val toIfu     = Output(new ifuChangeFlow)
  val toRtu     = Output(new rtuReadSignal)
  val specialPc = Output(UInt(PcWidth.W))
}
class BjuIO extends Bundle{
  val in  = new BjuIn
  val out = new BjuOut
}
class Bju extends Module{
  val io = IO(new BjuIO())
  val flush = io.in.rtuIn.flush
  val pc_fifo = Module(new PcFifo)
  val idu_mispred_stall = RegInit(false.B)
  val ifu_mispred_stall = RegInit(false.B)
  val bju_chgflw_vld = Wire(Bool())
  //==========================================================
  //                   PCFIFO Create Entry
  //==========================================================
  //----------------------------------------------------------
  //                    IFU Create Data
  //----------------------------------------------------------
  for(i<- 0 to 1) { // IFU data go in through, to pcFifo
    when(io.in.ifuForward(i).en){
      pc_fifo.io.iduWrite.writePcfifo(i).bits.pc         := Mux(io.in.ifuForward(i).jalr && io.in.ifuForward(i).dstVld, io.in.ifuForward(i).tarPc, io.in.ifuForward(i).curPc)
      pc_fifo.io.iduWrite.writePcfifo(i).bits.chkIdx     := io.in.ifuForward(i).predStore.chkIdx
      pc_fifo.io.iduWrite.writePcfifo(i).bits.bhtPred    := io.in.ifuForward(i).predStore.bhtPred
      pc_fifo.io.iduWrite.writePcfifo(i).bits.jmpMispred := io.in.ifuForward(i).predStore.jmpMispred
    }.otherwise{
      pc_fifo.io.iduWrite.writePcfifo(i).bits := DontCare
    }
  }
  //----------------------------------------------------------
  //                    IFU Create Valid
  //----------------------------------------------------------
  pc_fifo.io.iduWrite.writePcfifo(0).valid := io.in.ifuForward(0).en && (!io.in.rtuIn.flushFe) && io.in.isIn.instVld && (io.in.isIn.instNum >= 1.U )
  pc_fifo.io.iduWrite.writePcfifo(1).valid := io.in.ifuForward(1).en && (!io.in.rtuIn.flushFe) && io.in.isIn.instVld && (io.in.isIn.instNum === 2.U)
  //----------------------------------------------------------
  //                 output to IDU IS dispatch
  //----------------------------------------------------------
  io.out.toIfu.pcFifoFull := pc_fifo.io.iduWrite.isFull
  io.out.toIdu.allowPid   := pc_fifo.io.iduWrite.alloPid
  //==========================================================
  //                  EX1 Execution Logic
  //==========================================================
  // EX1 pipeline regs : 1. idu input data, 2. pcfifo read data
  // rf send pid to read pcfifo, get the first out instruction's ifu predict data
  pc_fifo.io.bjuRw.pid.read := io.in.rfPipe2.pid
  val bju_rf_iid_oldest    = (!idu_mispred_stall) && (!bju_chgflw_vld ) // TODO add compare iid???????
  val ex1_pipe_en          = io.in.sel.gateSel // in XT910 is gatesel, should add this pipe enable?
  val ex1_pipe_inst_vld    = RegInit(false.B)
  ex1_pipe_inst_vld        := Mux(flush, false.B, io.in.sel.sel)
  val ex1_pipe_rf          = RegEnable(io.in.rfPipe2, ex1_pipe_en)
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
  val bju_bht_mispred = bj_taken ^ ex1_pipe_pcfifo_read.bhtPred
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
  bju_chgflw_vld := bju_mispred && (!io.in.rtuIn.flushChgflwMask) //&& !rtu_iu_flush_chgflw_mask  && bju_older_inst_vld

  //==========================================================
  //                     EX1 Cancel Logic
  //==========================================================
  // TODO
  //----------------------------------------------------------
  //              Misprediction Stall and Mask
  //----------------------------------------------------------

  when(flush){
    idu_mispred_stall := false.B
  }.elsewhen(bju_chgflw_vld){
    idu_mispred_stall := true.B
  }.otherwise {
    idu_mispred_stall := idu_mispred_stall
  }
  io.out.toIdu.misPredStall := idu_mispred_stall

  when(flush){
    ifu_mispred_stall := false.B
  }.elsewhen(io.in.rtuIn.flushFe){
    ifu_mispred_stall := false.B
  }.elsewhen(bju_chgflw_vld){
    ifu_mispred_stall := true.B
  }.otherwise {
    ifu_mispred_stall := ifu_mispred_stall
  }
  io.out.toIfu.misPredStall := ifu_mispred_stall

  // EX2 - branch inst predict check
  //----------------------------------------------------------
  //               Pipe2 EX2 Instruction Valid
  //----------------------------------------------------------
  val ex2_pipe_inst_vld = RegInit(false.B)
  val ex2_pipe_chgflw_vld = RegInit(false.B)
  val ex2_pipe_conbr_vld = RegInit(false.B)
  when(flush){
    ex2_pipe_inst_vld   := false.B
    ex2_pipe_chgflw_vld := false.B
    ex2_pipe_conbr_vld  := false.B
  }.otherwise{
    ex2_pipe_inst_vld    := ex1_pipe_inst_vld
    ex2_pipe_chgflw_vld  := bju_chgflw_vld
    ex2_pipe_conbr_vld   := bju_condbr_vld
  }
  //----------------------------------------------------------
  //               Pipe2 EX2 Instruction Data
  //----------------------------------------------------------
  val ex2_pipe_pcfifo_read = RegEnable(ex1_pipe_pcfifo_read,ex2_pipe_inst_vld)
  val ex2_pipe_abnormal    = RegEnable(bju_bht_mispred || bju_jmp_mispred,ex2_pipe_inst_vld)
  val ex2_pipe_bht_mispred = RegEnable(bju_bht_mispred,ex2_pipe_inst_vld)
  val ex2_pipe_jmp_mispred = RegEnable(bju_jmp_mispred,ex2_pipe_inst_vld)
  val ex2_pipe_tar_pc      = RegEnable(bju_jump_pc,ex2_pipe_inst_vld)
  val ex2_pipe_is_br       = RegEnable(is_br,ex2_pipe_inst_vld)
  val ex2_pipe_is_jmp      = RegEnable(is_jmp,ex2_pipe_inst_vld)
  val ex2_pipe_iid          = RegEnable(ex1_pipe_rf.iid,ex2_pipe_inst_vld)
  val ex2_pipe_pid          = RegEnable(ex1_pipe_rf.pid,ex2_pipe_inst_vld)
  //----------------------------------------------------------
  //                 Write Result to PCFIFO
  //----------------------------------------------------------
  pc_fifo.io.bjuRw.pid.write := ex2_pipe_pid
  val bju_write = 0.U.asTypeOf(new PredCheckRes)
  bju_write.bhtMispred  := ex2_pipe_bht_mispred
  bju_write.jmp         := ex2_pipe_is_jmp
  bju_write.pcall       := DontCare
  bju_write.pret        := DontCare
  bju_write.condbr      := ex2_pipe_is_br
  bju_write.pc          := ex2_pipe_tar_pc
  pc_fifo.io.bjuRw.writePcfifo.valid := ex2_pipe_inst_vld
  pc_fifo.io.bjuRw.writePcfifo.bits := bju_write
  //----------------------------------------------------------
  //               Interface to Complete Bus
  //----------------------------------------------------------
  io.out.toCbus.sel         := ex2_pipe_inst_vld
  io.out.toCbus.iid         := ex2_pipe_iid
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
  io.out.toIfu.bhtPred        := ex2_pipe_pcfifo_read.bhtPred
  io.out.toIfu.chkIdx         := ex2_pipe_pcfifo_read.chkIdx
  io.out.toIfu.bhtCheckVld    := ex2_pipe_conbr_vld
  io.out.toIfu.bhtCondbrTaken := ex2_pipe_is_br
  // EX3 - branch inst result write back
  val ex3_pipe_en = RegInit(false.B)
  when(flush){
    ex3_pipe_en          := false.B
  }.otherwise{
    ex3_pipe_en          := ex2_pipe_inst_vld
  }
  val rob_read_en = ex3_pipe_en
  pc_fifo.io.robRead.dealloPid := rob_read_en
  io.out.toRtu.bhtPred     := pc_fifo.io.robRead.ifu_pred.bhtPred
  io.out.toRtu.bhtMispred  := pc_fifo.io.robRead.bht_check.bhtMispred
  io.out.toRtu.jmp         := pc_fifo.io.robRead.bht_check.jmp
  io.out.toRtu.pRet        := pc_fifo.io.robRead.bht_check.pret
  io.out.toRtu.pCall       := pc_fifo.io.robRead.bht_check.pcall
  io.out.toRtu.condBr      := pc_fifo.io.robRead.bht_check.condbr
  io.out.toRtu.pc          := pc_fifo.io.robRead.bht_check.pc
  // special pc
  pc_fifo.io.bjuRw.specialPid := io.in.specialPid
  io.out.specialPc := pc_fifo.io.bjuRw.specialPc
}