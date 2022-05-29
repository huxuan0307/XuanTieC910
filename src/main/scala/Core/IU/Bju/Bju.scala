package Core.IU.Bju

import Core.AddrConfig.PcWidth
import chisel3._
import chisel3.util._
import Core.IU.{IduRfPipe2, unitSel}
import Core.IUConfig.PcFifoAddr
import Core.IntConfig.InstructionIdWidth
import Core.ROBConfig.NumRobReadEntry
import Utils.{LookupTree, SignExt}
import Core.IDU.Opcode.BjuOpcode
import Core.RTU.{CompareIid, PcFifoData}

class BjuCmpltSignal extends Bundle{
  val abnormal = Bool()
  val bhtMispred = Bool()
  val jmpMispred = Bool()
  val iid = UInt(InstructionIdWidth.W)
  val sel = Bool()
}
class IduFeedbackSignal extends Bundle{
  val misPredStall = (Bool())
  val alloPid = Vec(4, (UInt(PcFifoAddr.W))) //
  val yyXxCancel = Output(Bool())
}
class IfuChangeFlow extends Bundle{
  val bhtCheckVld    = Bool()
  val bhtCondbrTaken = Bool()
  val bhtPred        = Bool()
//  val chgflwPc       = TODO: because 63bits = 24 + 39 = msb + pc, msb is MMU ref// (UInt(PcWidth.W))
  val chgflwVld      = Bool()
  val chkIdx         = UInt(25.W)
  val curPc          = UInt(PcWidth.W)
  val misPredStall   = Bool()
  val pcFifoFull     = Bool()
 // TODO with trait hasVecExtends
}
//class RtuReadSignal extends Bundle{
//  // length
//  val bhtPred    = Bool()
//  val bhtMispred = Bool()
//  val jmp        = Bool()
//  val pRet       = Bool()
//  val pCall      = Bool()
//  val condBr     = Bool()
//  val pc         = UInt(PcWidth.W)
//}
class IsuAlloFifo extends Bundle{
 val instNum = UInt(3.W) // temp define 3, indicate how many inst can store in PCfifo
 val instVld = Bool()
}
class RtuControl extends Bundle{
  val flushChgflwMask = Bool()
  val robReadPcfifovld = Vec(NumRobReadEntry,Bool())
  val robReadPcfifovldGateEn = Bool()
  val rtuFlush = Input(new Bundle() {
    val fe = (Bool())
    val flush = Bool()
  })
}


class BjuIn extends Bundle{
  val rtuIn      = Input(new RtuControl)
  val isIn       = Input(new IsuAlloFifo)
  val rfPipe2    = Input(new IduRfPipe2)
  val sel        = Input(new unitSel)
  val ifuForward = Input(Vec(2,new ifuDataForward))
  val specialPid = Input(UInt(PcFifoAddr.W)) // pipe0 pid
//  val mmuEN = Input(Bool())
}
class BjuOut extends Bundle{
  val toCbus    = Output(new BjuCmpltSignal)
  val toIdu     = Output(new IduFeedbackSignal)
  val toIfu     = Output(new IfuChangeFlow)
  val rtuPopData  = Output(Vec(NumRobReadEntry,new PcFifoData))
  val specialPc = Output(UInt(PcWidth.W))

}
class BjuIO extends Bundle{
  val in  = new BjuIn
  val out = new BjuOut
}
class Bju extends Module{
  val io = IO(new BjuIO())
  val flush = io.in.rtuIn.rtuFlush.flush
  //  PcFifo inst
  val pc_fifo = Module(new PcFifo)
  //----------------------------------------------------------
  //       Pred Store stage: PcFifo predict store from IDU
  //----------------------------------------------------------
  pc_fifo.io.iduWrite.writePcfifo := io.in.ifuForward // IFU data go in through, to pcFifo
  pc_fifo.io.iduWrite.isIn        := io.in.isIn
  pc_fifo.io.bjuRw.pid.rfPipe2 := io.in.rfPipe2.pid //rf send pid to read pcfifo

  // output to IDU IS dispatch
  io.out.toIfu.pcFifoFull := pc_fifo.io.iduWrite.isFull
  io.out.toIdu.alloPid    := pc_fifo.io.iduWrite.alloPid
  // rtu in
  pc_fifo.io.rtuRw.rtuFlush      := io.in.rtuIn.rtuFlush
  pc_fifo.io.rtuRw.readPcfifovld := io.in.rtuIn.robReadPcfifovld
  //----------------------------------------------------------
  //                 EX1 stage: execute bj take check
  //----------------------------------------------------------
  // EX1 pipeline regs : 1. idu input data, 2. pcfifo read data
  val bju_rf_iid_oldest  = Wire(Bool())
  val ex1_pipe_rf          = RegInit(0.U.asTypeOf(io.in.rfPipe2))
  val ex1_pipe_pcfifo_read = RegInit(0.U.asTypeOf(pc_fifo.io.bjuRw.readPcfifo)) // equal to IfuPredStore bundle
  val ex1_pipe2_iid_oldest = RegInit(false.B)
  when(io.in.sel.gateSel){
    ex1_pipe_rf          := io.in.rfPipe2
    ex1_pipe_pcfifo_read := pc_fifo.io.bjuRw.readPcfifo
    ex1_pipe2_iid_oldest := bju_rf_iid_oldest
  }
  val ex1_pipe_inst_vld    = RegInit(false.B)
  ex1_pipe_inst_vld        := Mux(flush, false.B, io.in.sel.sel)
  //----------------------------------------------------------
  //                   RF IID Age compare
  //during the mispred changeflow and retire, BJU exe inst may
  //older or newer than the previous mispred inst.
  //BJU will save and compare age by IID: new mispred could be generated
  //only when it is older
  //timing optimization: move ex1 iid age compare to rf stage
  val ex_iid_compare = Module(new CompareIid)
  ex_iid_compare.io.iid0 := io.in.rfPipe2.iid
  ex_iid_compare.io.iid1 := ex1_pipe_rf.iid
  val rfOlderEx1 = ex_iid_compare.io.older

  val mispred_iid = RegInit(0.U(InstructionIdWidth.W))
  val mispred_iid_compare = Module(new CompareIid)
  mispred_iid_compare.io.iid0 := io.in.rfPipe2.iid
  mispred_iid_compare.io.iid1 := ex1_pipe_rf.iid
  val mispredOlderEx1 = mispred_iid_compare.io.older

  val idu_mispred_stall = RegInit(false.B)
  val ifu_mispred_stall = RegInit(false.B)
  val bju_chgflw_vld = Wire(Bool())
  //==========================================================
  //                  EX1 Execution Logic
  //==========================================================
   bju_rf_iid_oldest    := (!idu_mispred_stall|| rfOlderEx1) && (!bju_chgflw_vld || mispredOlderEx1) //TODO what this
  val (src1, src2, op) = (ex1_pipe_rf.src0, ex1_pipe_rf.src1, ex1_pipe_rf.opcode)
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
  val is_jmp = BjuOpcode.isJmp(op)
  val is_br  = BjuOpcode.isBr(op)
  val bj_taken = LookupTree(op, List(
    BjuOpcode.JAL   ->  (true.B),
    BjuOpcode.JALR  ->  (true.B),
    BjuOpcode.BEQ   ->  (src1 === src2),
    BjuOpcode.BNE   ->  (src1 =/= src2),
    BjuOpcode.BLT   ->  (src1.asSInt < src2.asSInt),
    BjuOpcode.BGE   ->  (src1.asSInt >= src2.asSInt),
    BjuOpcode.BLTU  ->  (src1 < src2),
    BjuOpcode.BGEU  ->  (src1 >= src2)
  ))
  //----------------------------------------------------------
  //                BJU jump address calculation
  //---------------------------------------------------------
  val jump_pc = ex1_pipe_rf.src0(PcWidth+1,0) +  SignExt(ex1_pipe_rf.offset,(PcWidth+1))
  val branch_pc = Mux(is_br&&bj_taken, ex1_pipe_pcfifo_read.pc(PcWidth,1) + SignExt(ex1_pipe_rf.offset(20,1),(PcWidth)) ,
    ex1_pipe_pcfifo_read.pc(PcWidth,1)  + SignExt(Cat(ex1_pipe_rf.length,!ex1_pipe_rf.length),PcWidth)) // otherwise PC+4 TODO pc + pclengh
  val bju_tar_pc = Mux(is_jmp&&bj_taken,jump_pc(39,1),branch_pc)
  // todo MMU ref
  //----------------------------------------------------------
  //                      BHT Check
  //---------------------------------------------------------- @732 XOR
  val bju_bht_mispred = is_br && (bj_taken ^ ex1_pipe_pcfifo_read.bhtPred) // todo: && page fault
  //----------------------------------------------------------
  //                Indirect Jump / RAS Check
  //---------------------------------------------------------- @742
  //pcfifo create pc of jalr is predict pc minus offset
  //so only need compare to src0
  val bju_tarpc_cmp_fail = ex1_pipe_pcfifo_read.pc =/= ex1_pipe_rf.src0(PcWidth+1,0)
  val bju_jmp_mispred = (is_jmp&&bju_tarpc_cmp_fail) ||  (is_jmp && !ex1_pipe_rf.rts && ex1_pipe_pcfifo_read.jmpMispred)   // todo: && page fault
  //----------------------------------------------------------
  //                Change Flow (cancel) signal                 @change flow signal @ct_iu_bju.v @752
  //----------------------------------------------------------
  // to ensure the mispred is the newest by compare iid, becasue may stall or rob
  val bju_older_inst_vld    = ex1_pipe_inst_vld && ex1_pipe2_iid_oldest
  val bju_bht_mispred_vld   = bju_older_inst_vld && bju_bht_mispred
  val bju_jmp_mispred_vld   = bju_older_inst_vld && bju_jmp_mispred
  //mask iu change flow on wrong path during flush state machine
  //when mispred iu will mask wrong path change flow by itself
  val bju_mispred = bju_bht_mispred || bju_jmp_mispred
  val bju_condbr_vld = is_br && bju_older_inst_vld
  bju_chgflw_vld := bju_mispred && !io.in.rtuIn.flushChgflwMask && bju_older_inst_vld
  //==========================================================
  //                     EX1 Cancel Logic
  //==========================================================
  //----------------------------------------------------------
  //              Misprediction Stall and Mask
  //----------------------------------------------------------
  //stall ID stage until all frontend and backend pipeline empty
  when(flush){
    idu_mispred_stall := false.B
    mispred_iid := mispred_iid
  }.elsewhen(bju_chgflw_vld){
    idu_mispred_stall := true.B
    mispred_iid := ex1_pipe_rf.iid
  }
  io.out.toIdu.misPredStall := idu_mispred_stall

  //stop IFU fetching RAS/JMP instruction before mispred inst retire
  when(flush){
    ifu_mispred_stall := false.B
    //since no RAS/JMP inst retire after flush fe, IFU could fetch
    //new RAS/JMP instruction after flush fe, but IDU should not
    //still stall new inst pipedown at this time
  }.elsewhen(io.in.rtuIn.rtuFlush.fe){
    ifu_mispred_stall := false.B
  }.elsewhen(bju_chgflw_vld){
    ifu_mispred_stall := true.B
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
  val ex2_pipe_pcfifo_read = RegEnable(ex1_pipe_pcfifo_read,0.U.asTypeOf(new IfuPredStore),ex1_pipe_inst_vld)
  val ex2_pipe_abnormal    = RegEnable(bju_bht_mispred || bju_jmp_mispred,false.B,ex1_pipe_inst_vld)
  val ex2_pipe_bht_mispred = RegEnable(bju_bht_mispred,false.B,ex1_pipe_inst_vld)
  val ex2_pipe_jmp_mispred = RegEnable(bju_jmp_mispred,false.B,ex1_pipe_inst_vld)
  val ex2_pipe_tar_pc      = RegEnable(bju_tar_pc,0.U(PcWidth.W),ex1_pipe_inst_vld)
  val ex2_pipe_is_br       = RegEnable(is_br,false.B,ex1_pipe_inst_vld)
  val ex2_pipe_is_jmp      = RegEnable(is_jmp,false.B,ex1_pipe_inst_vld)
  val ex2_pipe_iid         = RegEnable(ex1_pipe_rf.iid,0.U(InstructionIdWidth),ex1_pipe_inst_vld) // rtu ptr
  val ex2_pipe_pid         = RegEnable(ex1_pipe_rf.pid,0.U(PcFifoAddr.W),ex1_pipe_inst_vld) // pid pc fifo ptr
  val ex2_pipe2_pcall      = RegEnable(ex1_pipe_rf.pCall, false.B, ex1_pipe_inst_vld)
  val ex2_pipe2_pret       = RegEnable(ex1_pipe_rf.rts,   false.B,   ex1_pipe_inst_vld)
  val ex2_pipe2_length     = RegEnable(ex1_pipe_rf.length, false.B, ex1_pipe_inst_vld)
  val ex2_pipe2_bht_pred   = RegEnable(ex1_pipe_pcfifo_read.bhtPred,false.B,ex1_pipe_inst_vld)
  //----------------------------------------------------------
  //                 Write Result to PCFIFO
  //----------------------------------------------------------
  //write result to PCFIFO at EX2 stage and bypass in pcfifo
  pc_fifo.io.bjuRw.pid.ex2 := ex2_pipe_pid
  val bju_write = 0.U.asTypeOf(new PredCheckRes)
  bju_write.bhtMispred  := ex2_pipe_bht_mispred
  bju_write.jmp         := ex2_pipe_is_jmp
  bju_write.pcall       := ex2_pipe2_pcall
  bju_write.pret        := ex2_pipe2_pret
  bju_write.condbr      := ex2_pipe_is_br
  bju_write.pc          := Cat(ex2_pipe_tar_pc,1.U)
  bju_write.instVld     := ex2_pipe_inst_vld
  bju_write.length      := ex2_pipe2_length
  bju_write.bhtPred     := ex2_pipe2_bht_pred
  pc_fifo.io.bjuRw.ex2WritePcfifo := bju_write
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
  io.out.toIdu.yyXxCancel    := ex2_pipe_chgflw_vld
  pc_fifo.io.bjuRw.iuYyXxCancel := ex2_pipe_chgflw_vld
    //----------------------------------------------------------
  //                  Interface to IFU
  //----------------------------------------------------------
  io.out.toIfu.chgflwVld      := ex2_pipe_chgflw_vld
//  io.out.toIfu.chgflwPc       := ex2_pipe_tar_pc
  io.out.toIfu.curPc          := ex2_pipe_pcfifo_read.pc
  io.out.toIfu.bhtPred        := ex2_pipe_pcfifo_read.bhtPred
  io.out.toIfu.chkIdx         := ex2_pipe_pcfifo_read.chkIdx
  io.out.toIfu.bhtCheckVld    := ex2_pipe_conbr_vld
  io.out.toIfu.bhtCondbrTaken := ex2_pipe_is_br

  //==========================================================
  //              EX3 PCFIFO Bypass Pipeline
  //==========================================================
  // EX3 - branch inst result write back
  val ex3_pipe_inst_vld = RegInit(false.B)
  when(flush){
    ex3_pipe_inst_vld          := false.B
  }.otherwise{
    ex3_pipe_inst_vld          := ex2_pipe_inst_vld
  }
  val ex3_pipe_iid          = RegEnable(ex2_pipe_iid,0.U(InstructionIdWidth.W) , ex2_pipe_inst_vld)
  val ex3_pipe_bht_mispred  = RegEnable(ex2_pipe_bht_mispred,false.B , ex2_pipe_inst_vld)
  val ex3_pipe_tar_pc       = RegEnable(ex2_pipe_tar_pc,0.U(PcWidth.W) , ex2_pipe_inst_vld)
  val ex3_pipe_bht_pred     = RegEnable(ex2_pipe2_bht_pred,false.B , ex2_pipe_inst_vld)
  val ex3_pipe_pid          = RegEnable(ex2_pipe_pid,0.U(PcFifoAddr.W) , ex2_pipe_inst_vld)
  val ex3_pipe_conbr_vld    = RegEnable(ex2_pipe_conbr_vld,false.B , ex2_pipe_inst_vld)
  val ex3_pipe_jmp          = RegEnable(ex2_pipe_is_jmp,false.B , ex2_pipe_inst_vld)
  val ex3_pipe_pret         = RegEnable(ex2_pipe2_pret,false.B , ex2_pipe_inst_vld)
  val ex3_pipe_pcall        = RegEnable(ex2_pipe2_pcall ,false.B, ex2_pipe_inst_vld)
  val ex3_pipe_length       = RegEnable(ex2_pipe2_length,false.B , ex2_pipe_inst_vld)
  pc_fifo.io.bjuRw.pid.ex3 := ex3_pipe_pid
  pc_fifo.io.bjuRw.ex3Bypass.instVld     := ex3_pipe_inst_vld
  pc_fifo.io.bjuRw.ex3Bypass.iid         := ex3_pipe_iid
  pc_fifo.io.bjuRw.ex3Bypass.bht_mispred := ex3_pipe_bht_mispred
  pc_fifo.io.bjuRw.ex3Bypass.pc          := ex3_pipe_tar_pc
  pc_fifo.io.bjuRw.ex3Bypass.bht_pred    := ex3_pipe_bht_pred
  pc_fifo.io.bjuRw.ex3Bypass.pid         := ex3_pipe_pid
  pc_fifo.io.bjuRw.ex3Bypass.conbr       := ex3_pipe_conbr_vld
  pc_fifo.io.bjuRw.ex3Bypass.jmp         := ex3_pipe_jmp
  pc_fifo.io.bjuRw.ex3Bypass.pret        := ex3_pipe_pret
  pc_fifo.io.bjuRw.ex3Bypass.pcall       := ex3_pipe_pcall
  pc_fifo.io.bjuRw.ex3Bypass.length      := ex3_pipe_length


  io.out.rtuPopData   := pc_fifo.io.rtuRw.rtuReadData

  // special pc
  pc_fifo.io.bjuRw.specialPid := io.in.specialPid
  io.out.specialPc := pc_fifo.io.bjuRw.specialPc
}