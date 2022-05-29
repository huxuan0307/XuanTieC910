package Core.IU

import Core.FuncOpType.width
import Core.IUConfig
import Core.IntConfig.NumPhysicRegsBits
import Utils.{LookupTree, SignExt}
import chisel3._
import chisel3.util._
import Core.IDU.Opcode.AluOpcode

class AluRegData extends Bundle with IUConfig {
  val data    = UInt(XLEN.W)
  val dataVld = Bool()
  val fwdData = UInt(XLEN.W)
  val fwdVld  = Bool()
  val preg    = UInt(NumPhysicRegsBits.W)
}

class AluIO extends Bundle{
  val in         = Input(new CtrlSignalHasDestIO)
  val flush      = Input(Bool())
  val toRbus     = Output(new AluRegData)
  val sel        = Input(new unitSel)
}

class Alu extends Module with IUConfig{
  val io = IO(new AluIO)
  //----------------------------------------------------------
  //              Pipe2 EX1 Instruction valid
  //----------------------------------------------------------
  val alu_ex1_inst_vld = RegInit(false.B)
  val alu_ex1_fwd_vld  = RegInit(false.B)
  val flush   = io.flush
  val fwd_vld = io.in.aluShort && io.sel.sel && io.in.dstVld
  when(flush){
    alu_ex1_inst_vld := false.B
    alu_ex1_fwd_vld  := false.B
  }.otherwise{
    alu_ex1_inst_vld := io.sel.sel
    alu_ex1_fwd_vld  := fwd_vld
  }
  //----------------------------------------------------------
  //               Pipe2 EX1 Instruction Data
  //----------------------------------------------------------
  val pipe1_en = io.sel.gateSel // TODO add gate_sel
  val ex1_pipe = RegInit(0.U.asTypeOf(io.in))
  val res = RegInit(0.U(XLEN.W))
  val src1 = WireInit(0.U(XLEN.W))
  val src2 = WireInit(0.U(XLEN.W))
  val op = WireInit(0.U(width))
  val opReg = RegInit(0.U(width))

  //----------------------------------------------------------
  //                    add && shift TODO misc
  //----------------------------------------------------------
  op := io.in.opcode
  src1 := io.in.src0
  src2 := io.in.src1
  val shamt = Mux(AluOpcode.isWordOp(op), src2(4, 0), src2(5, 0))
  when(pipe1_en) {
    ex1_pipe := io.in
    opReg := io.in.opcode
    res := LookupTree(op, List(
      AluOpcode.ADD   -> (src1 + src2),
      AluOpcode.SLL   -> (src1 << shamt),
      AluOpcode.SLT   -> Cat(0.U((XLEN - 1).W), src1.asSInt < src2.asSInt),
      AluOpcode.SLTU  -> (src1 < src2),
      AluOpcode.XOR   -> (src1 ^ src2),
      AluOpcode.SRL   -> (src1 >> shamt),
      AluOpcode.OR    -> (src1 | src2),
      AluOpcode.AND   -> (src1 & src2),
      AluOpcode.SUB   -> (src1 - src2),
      AluOpcode.SRA   -> (src1.asSInt >> shamt).asUInt,
      AluOpcode.ADDW  -> (src1 + src2),
      AluOpcode.SUBW  -> (src1 - src2),
      AluOpcode.SLLW  -> (src1 << shamt),
      AluOpcode.SRLW  -> (src1(31, 0) >> shamt),
      AluOpcode.SRAW  -> (src1(31, 0).asSInt >> shamt).asUInt,
      AluOpcode.LUI   -> src2
    ))
  }
  //==========================================================
  //                  Complete Bus signals
  //==========================================================
  // deal alu complete bus signal in cbus : ATTENSION  alu sel is en, means alu complete

  //==========================================================
  //                   Result Bus signals
  //==========================================================
  //----------------------------------------------------------
  //                      Result Bus
  //----------------------------------------------------------
  io.toRbus.dataVld  := ex1_pipe.dstVld && alu_ex1_inst_vld
  io.toRbus.fwdVld   := alu_ex1_fwd_vld
  io.toRbus.fwdData  := Mux(AluOpcode.isWordOp(opReg), SignExt(res(31,0), 64), res) //////ex1 pipe Reg
  io.toRbus.preg     := ex1_pipe.dstPreg
  io.toRbus.data     := Mux(AluOpcode.isWordOp(opReg), SignExt(res(31,0), 64), res) //////todo: check isWordOp
}

