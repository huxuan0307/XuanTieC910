package Core.IU

import Core.AddrConfig.PcWidth
import chisel3._
import chisel3.util._
import Utils.LookupTree

object SpecialOpType {
  def NOP           = "b00000".U
  def ECALL         = "b00010".U
  def EBREAK        = "b00011".U
  def AUIPC         = "b00100".U
  def PSEUDO_AUIPC  = "b00101".U
  def VSETVLI       = "b00110".U
}

class SpecialOut extends Bundle {
  val abnormal = Bool()
  val bkpt = Bool()
  val exptVec = UInt(5.W)
  val exptVld = Bool()
  val flush = Bool()
  val highHwExpt = Bool()
  val iid = UInt(7.W)
  val immuExpt = Bool()
  val instVld = Bool()
  val mtval = UInt(32.W)
  val data = UInt(64.W)
  val dataVld = Bool()
  val preg = UInt(7.W)
}

class SpecialIO extends Bundle{
  val in = Input(new IduRfPipe0)
  val bjuSpecialPc = Input(UInt(PcWidth.W))
  val flush = Input(Bool())
  val bju_special_pc = UInt(PcWidth.W)
  val cp0_yy_priv_mode = UInt(2.W)
  val out = Output(new SpecialOut)
}

class Special extends Module{
  val io = IO(new SpecialIO)
  // Special unit is aim to process: 1. exception 2. pseudo auipc - pc + imm(un-shifted) 3. vec-inst - vsetvl (Dontcare)
  //----------------------------------------------------------
  //              Pipe2 EX1 Instruction valid
  //----------------------------------------------------------
  val flush = io.flush
  val special_ex1_inst_vld = RegInit(false.B)
  when(flush){
    special_ex1_inst_vld := false.B
  }.otherwise {
    special_ex1_inst_vld := io.in.sel
  }
  //----------------------------------------------------------
  //               Pipe2 EX1 Instruction Data
  //----------------------------------------------------------
  val pipe1_en = WireInit(true.B) // TODO add gate_sel
  val ex1_pipe = RegEnable(io.in, pipe1_en)
  val ex1_pipe_pc = io.bjuSpecialPc
  //==========================================================
  //                  Instruction Selection
  //==========================================================
  // 1. exception vec process
  val special_ex1_ecall_expt_vec = LookupTree(io.cp0_yy_priv_mode, List(
    "b00".U  -> "b01000".U,
    "b01".U  -> "b01001".U,
    "b11".U  -> "b01011".U,
  ))
  // 2. auipc
  //==========================================================
  //                     AUIPC result
  //==========================================================
  val special_ex1_offset = ex1_pipe.specialImm << 12.U // auipc , left shift 12bits for jalr
  val special_auipc_rslt = ex1_pipe_pc + special_ex1_offset
  // 3. vector inst
  //==========================================================
  //               RF stage Complete Bus signals
  //==========================================================
  io.out.instVld         := special_ex1_inst_vld
  io.out.abnormal         := special_ex1_inst_vld && (io.in.opcode === SpecialOpType.NOP)  || (io.in.opcode === SpecialOpType.ECALL)  || (io.in.opcode === SpecialOpType.EBREAK)
  io.out.bkpt             := (io.in.opcode === SpecialOpType.EBREAK)
  io.out.iid              := ex1_pipe.iid
  //----------------------------------------------------------
  //                     Exception
  //----------------------------------------------------------
  io.out.exptVec := LookupTree(io.in.opcode, List(
    SpecialOpType.NOP           -> io.in.exptVec,
    SpecialOpType.ECALL         -> special_ex1_ecall_expt_vec,
    SpecialOpType.EBREAK        -> "b00011".U,
  ))
  io.out.exptVld := special_ex1_inst_vld && (io.in.opcode === SpecialOpType.NOP)  || (io.in.opcode === SpecialOpType.ECALL)  || (io.in.opcode === SpecialOpType.EBREAK)
  //==========================================================
  //                    Result Bus signals
  //==========================================================
  io.out.dataVld := special_ex1_inst_vld
  io.out.preg := ex1_pipe.dstPreg
  io.out.data := special_auipc_rslt
}
