package Core.IU
import Core.ExceptionConfig.{ExceptionVecWidth, MtvalWidth}
import Core.{FuncOpType, FuncType, IUConfig}
import Core.IU.Bju.{Bju,iduFeedbackSignal, ifuChangeFlow, ifuDataForward, isuAllowFifo, rtuControl, rtuReadSignal}
import Core.IU.Du.Du
import Core.IUConfig.{PcFifoAddr, PcOffsetWidth}
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
class  cp0In extends Bundle with IUConfig{
//  val divDisable    = Bool()
//  val divDisableClr = Bool()
//  val abnormal      = Bool()
//  val efpc          = Bool()
//  val efpcVld       = Bool()
//  val exptVec       = UInt(ExceptionVecWidth.W)
//  val exptVld       = Bool()
//  val flush         = Bool()
//  val iid           = UInt(InstructionIdWidth.W)
//  val instVld       = Bool()
//  val mtval         = UInt(MtvalWidth.W)
//  val rsltData      = UInt(XLEN.W)
//  val rsltPreg      = UInt(NumPhysicRegsBits.W)
//  val rsltVld       = Bool()
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
  val isuToFifo = new isuAllowFifo
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
  val sel       = Bool()
}
class IduRfPipe0 extends CtrlSignalHasDestIO{
  val opcode     = FuncOpType.uwidth
  val specialImm = UInt(20.W)
  val exptVec    = UInt(ExceptionVecWidth.W)
  val exptVld    = Bool()
  val highHwExpt = Bool()
  val divSel     = Input(Bool())
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
class IntegerUnitIO extends Bundle{
  val cp0In      = Input(new cp0In)
  // to alu0, special, div
  val pipe0      = Flipped(ValidIO(new IduRfPipe0))
  val divSel     = Input(new unitSel)
  val specialSel = Input(new unitSel)
  // to alu1&mult
  val pipe1      = Input(new IduRfPipe1)
  val mulSel     = Input(new unitSel)
  // to BJU
  val isIn       = Input(new isIn)
  val rtuIn      = Input(new rtuControl)
  val pipe2      = Input(new IduRfPipe2)
  val bjuSel     = Input(new unitSel)
  val ifuForward = Input(Vec(2,new ifuDataForward))
  val specialPid = Input(UInt(PcFifoAddr.W))

  // out ifu
  val bjuToIfu      = Output(new ifuChangeFlow)
  val bjuToIdu      = Output(new iduFeedbackSignal)
  val bjuToRtu      = Output(new rtuReadSignal)
}

class IntegeUnit extends Module{
  val io = IO(new IntegerUnitIO)
  //==========================================================
  //                Pipeline 0 - alu0, special, div
  //==========================================================
  val alu0 = Module(new Alu)
  val special = Module(new Special)
  val du = Module(new Du)
  alu0.io.in       := io.pipe0
  special.io.in    := io.pipe0
  special.io.sel   := io.specialSel
  special.io.flush := io.rtuIn.flush
  special.io.cp0PrivMode := io.cp0In.cp0PrivMode
  du.io.in       := io.pipe0
  //==========================================================
  //                Pipeline 1 - alu1, mult
  //==========================================================
  val alu1 = Module(new Alu)
  val mu   = Module(new Mu)
  alu1.io.in := io.pipe1
  mu.io.in   := io.pipe1
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
  cbus.io.normIn.divSel   := io.pipe0.bits.divSel
  cbus.io.normIn.pipe0Sel := io.pipe0.bits.sel
  cbus.io.normIn.pipe0Iid := io.pipe0.bits.iid
  cbus.io.normIn.multSel  := io.pipe1.mulSel
  cbus.io.normIn.pipe1Sel := io.pipe1.sel
  cbus.io.normIn.pipe1Iid := io.pipe1.iid
  cbus.io.bjuIn           := bju.io.out.toCbus

  //==========================================================
  //                Rbus - Iu pipes data process
  //==========================================================
  val rbus = Module(new Rbus)
  rbus.io.in.aluIn(0) := alu0.io.out
  rbus.io.in.aluIn(1) := alu1.io.out
  rbus.io.in.duIn     := du.io.out
  rbus.io.in.muIn     := mu.io.out

}









