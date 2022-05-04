package Core.IU

import Core.ExceptionConfig.{ExceptionVecWidth, MtvalWidth}
import Core.{FuncOpType, FuncType, IUConfig}
import Core.IU.Bju.{Bju, IduFeedbackSignal, IfuChangeFlow, IsuAllowFifo, RtuControl, RtuReadSignal, ifuDataForward}
import Core.IU.Du.Du
import Core.IUConfig.{IuPipeNum, PcFifoAddr, PcOffsetWidth}
import Core.IntConfig.{InstructionIdWidth, NumPhysicRegsBits}
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._
trait hasVecExtends extends Bundle {
  val vl    = UInt(VlmaxBits.W)
  val vlmul = UInt(VlmulBits.W)
  val vsew  = UInt(VsewBits.W)
}
class unitSel extends Bundle {
  val sel     = Bool()
  val gateSel = Bool()
}
//class cfpuIn extends Bundle {
//  vfpu_iu_ex2_pipe6_mfr_data
//  vfpu_iu_ex2_pipe6_mfvr_data_vld
//  vfpu_iu_ex2_pipe6_mfvr_preg
//  vfpu_iu_ex2_pipe7_mfvr_data
//  vfpu_iu_ex2_pipe7_mfvr_data_vld
//  vfpu_iu_ex2_pipe7_mfvr_preg
//}


class Cp0ToRbus extends Bundle with IUConfig{
  val rsltData      = UInt(XLEN.W)
  val rsltPreg      = UInt(NumPhysicRegsBits.W)
  val rsltVld       = Bool()
}
class Cp0ToCbus extends Bundle with IUConfig{
    val abnormal      = Bool()
    val efpc          = Bool()
    val efpcVld       = Bool()
    val exptVec       = UInt(ExceptionVecWidth.W)
    val exptVld       = Bool()
    val flush         = Bool()
    val iid           = UInt(InstructionIdWidth.W)
    val instVld       = Bool()
    val mtval         = UInt(MtvalWidth.W)
}
class  cp0ToIu extends Bundle with IUConfig{
//  val divDisable    = Bool()
//  val divDisableClr = Bool()
  val toCbus    = new Cp0ToCbus
  val toRbus    = new Cp0ToRbus
//  val icgEn         = Bool()
//  val cp0ClkEn      = Bool()
  val cp0PrivMode   = UInt(MPPWidth.W)
  //  val vill =
  //  val vl =
  //  val vsetvli_pre_decd_disable =
  //  val vstart =
}
class isIn extends Bundle with IUConfig{
  val gateClkIssue = Bool()
  val issue = Bool()
  val isuToFifo = new IsuAllowFifo
}
class CtrlSignalIO extends Bundle with IUConfig{
  val func = FuncOpType.uwidth
  val iid  = UInt(InstructionIdWidth.W) // ROB PTR
  val src0 = UInt(XLEN.W)
  val src1 = UInt(XLEN.W)
}
class CtrlSignalHasDestIO extends CtrlSignalIO{
  val aluShort  = Bool()
  val dstPreg   = UInt(NumPhysicRegsBits.W)
  val dstVld    = Bool()
  val src1NoImm = UInt(XLEN.W)
  val src2      = UInt(XLEN.W)
  val imm       = UInt(7.W)
}
class IduRfPipe0 extends CtrlSignalHasDestIO{
  val opcode     = FuncOpType.uwidth
  val specialImm = UInt(20.W)
  val exptVec    = UInt(ExceptionVecWidth.W)
  val exptVld    = Bool()
  val highHwExpt = Bool()
}
class IduRfPipe1 extends CtrlSignalHasDestIO{
  val multFunc    = UInt(8.W)
  val mlaSrc2Preg = UInt(NumPhysicRegsBits.W)
  val mlaSrc2Vld  = Bool()
  val mulSel      = Input(Bool())
}
class IduRfPipe2 extends CtrlSignalIO{
  val length = Bool()
  val offset = UInt(PcOffsetWidth.W)
  val pCall  = Bool()
  val pid    = UInt(PcFifoAddr.W)
  val rts    = Bool()
}
class IuRslt extends Bundle {
  val cbusRslt = new CbusOut
  val rbusRslt = Output(Vec(IuPipeNum-1, new RbusOut))
}
class IntegerUnitIO extends Bundle{
  val cp0In      = Input(new cp0ToIu)
  // to alu0, special, div
  val alu0Sel    = Input(new unitSel)
  val pipe0      = Input(new IduRfPipe0)
  val divSel      = Input(new unitSel)
  val specialSel = Input(new unitSel)
  // to alu1&mult
  val alu1Sel    = Input(new unitSel)
  val pipe1      = Input(new IduRfPipe1)
  val mulSel     = Input(new unitSel)
  // to BJU
  val isIn       = Input(new isIn)
  val rtuIn      = Input(new RtuControl)
  val pipe2      = Input(new IduRfPipe2)
  val bjuSel     = Input(new unitSel)
  val ifuForward = Input(Vec(2,new ifuDataForward))
  val specialPid = Input(UInt(PcFifoAddr.W))

  // out ifu
  val bjuToIfu      = Output(new IfuChangeFlow)
  val bjuToIdu      = Output(new IduFeedbackSignal)
  val bjuToRtu      = Output(new RtuReadSignal)

  // out rtu
  val iuToRtu       = Output(new IuRslt)
}

class IntegeUnit extends Module{
  val io = IO(new IntegerUnitIO)
  //==========================================================
  //                Pipeline 0 - alu0, special, div
  //==========================================================
  val alu0 = Module(new Alu)
  val special = Module(new Special)
  val du = Module(new Du)
 // alu0.io.in       := io.pipe0
  alu0.io.in.func      := io.pipe0.func
  alu0.io.in.iid       := io.pipe0.iid
  alu0.io.in.src0      := io.pipe0.src0
  alu0.io.in.src1      := io.pipe0.src1
  alu0.io.in.aluShort  := io.pipe0.aluShort
  alu0.io.in.dstPreg   := io.pipe0.dstPreg
  alu0.io.in.dstVld    := io.pipe0.dstVld
  alu0.io.in.src1NoImm := io.pipe0.src1NoImm
  alu0.io.in.src2      := io.pipe0.src2
  alu0.io.in.imm       := io.pipe0.imm

  alu0.io.sel      := io.alu0Sel
  alu0.io.flush    := io.rtuIn.flush
  special.io.in    := io.pipe0
  special.io.sel   := io.specialSel
  special.io.flush := io.rtuIn.flush
  special.io.cp0PrivMode := io.cp0In.cp0PrivMode
  du.io.in         := io.pipe0
  du.io.sel        := io.divSel
  du.io.flush      := io.rtuIn.flush
  //==========================================================
  //                Pipeline 1 - alu1, mult
  //==========================================================
  val alu1 = Module(new Alu)
  val mu   = Module(new Mu)
 // alu1.io.in       := io.pipe1

  alu1.io.in.func      := io.pipe1.func
  alu1.io.in.iid       := io.pipe1.iid
  alu1.io.in.src0      := io.pipe1.src0
  alu1.io.in.src1      := io.pipe1.src1
  alu1.io.in.aluShort  := io.pipe1.aluShort
  alu1.io.in.dstPreg   := io.pipe1.dstPreg
  alu1.io.in.dstVld    := io.pipe1.dstVld
  alu1.io.in.src1NoImm := io.pipe1.src1NoImm
  alu1.io.in.src2      := io.pipe1.src2
  alu1.io.in.imm       := io.pipe1.imm


  alu1.io.sel      := io.alu1Sel
  alu1.io.flush    := io.rtuIn.flush
  mu.io.in         := io.pipe1
  mu.io.sel        := io.mulSel
  mu.io.flush      := io.rtuIn.flush
  //==========================================================
  //                Pipeline 2 - bju
  //==========================================================
  val bju = Module(new Bju)
  bju.io.in.rtuIn      := io.rtuIn
  bju.io.in.isIn       := io.isIn.isuToFifo
  bju.io.in.rfPipe2    := io.pipe2
  bju.io.in.sel        := io.bjuSel
  bju.io.in.ifuForward := io.ifuForward
  bju.io.in.specialPid := io.specialPid
  io.bjuToIfu          := bju.io.out.toIfu
  io.bjuToIdu          := bju.io.out.toIdu
  io.bjuToRtu          := bju.io.out.toRtu
  // special pc io should be below Module 'bju' & 'special', because 'Suspicious forward reference'
  special.io.bjuSpecialPc := bju.io.out.specialPc
  //==========================================================
  //                Cbus - Iu pipes complete signal process
  //==========================================================
  val cbus = Module(new Cbus)
  cbus.io.flush        := io.rtuIn.flush
  cbus.io.normIn.divSel   := io.divSel.sel
  cbus.io.normIn.pipe0Sel := io.alu0Sel.sel
  cbus.io.normIn.pipe0Iid := io.pipe0.iid
  cbus.io.normIn.multSel  := io.pipe1.mulSel
  cbus.io.normIn.pipe1Sel := io.alu1Sel.sel
  cbus.io.normIn.pipe1Iid := io.pipe1.iid
  cbus.io.bjuIn           := bju.io.out.toCbus
  cbus.io.cp0In           := io.cp0In.toCbus
  cbus.io.specialIn       := special.io.out.toCbus
  io.iuToRtu.cbusRslt     := cbus.io.out
  //==========================================================
  //                Rbus - Iu pipes data process
  //==========================================================
  val rbus = Module(new Rbus)
  rbus.io.flush        := io.rtuIn.flush
  rbus.io.in.aluIn(0)  := alu0.io.toRbus
  rbus.io.in.aluIn(1)  := alu1.io.toRbus
  rbus.io.in.muIn      := mu.io.out
  rbus.io.in.duIn      := du.io.out
  rbus.io.in.specialIn := special.io.out.toRbus
  rbus.io.in.cp0In     := io.cp0In.toRbus
  io.iuToRtu.rbusRslt  := rbus.io.out
}









