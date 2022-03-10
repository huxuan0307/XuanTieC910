package Core.IU
import Core.{FuncOpType, FuncType, IUConfig}
import Core.IU.Bju.{ Bju, ifuDataForward}
import Core.IU.Du.Du
import Core.IUConfig.{PcFifoAddr, PcOffsetWidth}
import Core.IntConfig.InstructionIdWidth
import chisel3._
import chisel3.util._



trait hasVecExtends extends Bundle {
  val vl    = UInt(8.W)
  val vlmul = UInt(2.W)
  val vsew  = UInt(3.W)
}
class unitSel extends Bundle {
  val sel = Bool()
  val gateSel = Bool()
}
class CtrlSignalIO extends Bundle with IUConfig{
  val func = FuncOpType.uwidth
  val iid = UInt(InstructionIdWidth.W) // ROB PTR
  val src0 = UInt(XLEN.W)
  val src1 = UInt(XLEN.W)
}
class CtrlSignalHasDestIO extends CtrlSignalIO{
  val aluShort = Bool()
  val dstPreg = UInt(7.W)
  val dstVld = Bool()
  val src1NoImm = UInt(64.W)
  val src2 = UInt(64.W)
  val imm = UInt(7.W)
  val sel = Bool()
}
class IduRfPipe0 extends CtrlSignalHasDestIO{
  val opcode = FuncOpType.uwidth
  val specialImm = UInt(20.W)
  val exptVec = UInt(5.W)
  val exptVld = Bool()
  val highHwExpt = Bool()
  val divSel = Input(Bool())
}
class IduRfPipe1 extends CtrlSignalHasDestIO{
  val multFunc = UInt(8.W)
  val mlaSrc2Preg = UInt(7.W)
  val mlaSrc2Vld = Bool()
  val mulSel = Input(Bool())
}
class IduRfPipe2 extends CtrlSignalIO{
  val length = Bool()
  val offset = UInt(PcOffsetWidth.W)
  val pCall  = Bool()
  val pid = UInt(PcFifoAddr.W)
  val rts    = Bool()
}

class IntegerUnitIO extends Bundle{
  // to alu0, special, div
  val pipe0 = Flipped(ValidIO(new IduRfPipe0))
  // to alu1&mult
  val pipe1 = Input(new IduRfPipe1)
  // to BJU
  val pipe2 = Input(new IduRfPipe2)
  val ifuForward = Flipped(DecoupledIO(Vec(2,new ifuDataForward)))
  val specialPid = UInt(PcFifoAddr.W)
}

class IntegeUnit extends Module{
  val io = IO(new IntegerUnitIO)
  //==========================================================
  //                Pipeline 0 - alu0, special, div
  //==========================================================
  val alu0 = Module(new Alu)
  val special = Module(new Special)
  val du = Module(new Du)
  alu0.io.in    := io.pipe0
  special.io.in := io.pipe0
  du.io.in      := io.pipe0
  //==========================================================
  //                Pipeline 1 - alu1, mult
  //==========================================================
  val alu1 = Module(new Alu)
  val mu = Module(new Mu)
  alu1.io.in := io.pipe1
  mu.io.in   := io.pipe1
  //==========================================================
  //                Pipeline 2 - bju
  //==========================================================
  val bju = Module(new Bju)
  bju.io.in.ifuForward := io.ifuForward
  bju.io.in.rfPipe2 := io.pipe2
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
  cbus.io.bjuIn            := bju.io.out.toCbus

  //==========================================================
  //                Rbus - Iu pipes data process
  //==========================================================
  val rbus = Module(new Rbus)
  rbus.io.in.aluIn(0) := alu0.io.out
  rbus.io.in.aluIn(1) := alu1.io.out
  rbus.io.in.duIn     := du.io.out
  rbus.io.in.muIn     := mu.io.out

}









