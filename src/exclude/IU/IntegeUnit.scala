package Core.IU
import Core.FuncOpType
import Core.IU.Bju.{BhtPredDataForward, Bju}
import Core.IU.Du.Du
import chisel3._
import chisel3.util._

class CtrlSignalIO extends Bundle{
  val src0 = UInt(64.W)
  val src1 = UInt(64.W)
  val func = FuncOpType.uwidth
  val iid = UInt(5.W) // ROB PTR
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
  val opcode = UInt(32.W)
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
  val pid = UInt(5.W)
  val specialPid = UInt(5.W)
  val length = Bool()
  val offset = UInt(21.W)
  val pcall = Bool()
  val rts = Bool()
}

class IntegerUnitIO extends Bundle{
  // to alu0, special, div
  val idu_iu_rf_pipe0 = Flipped(ValidIO(new IduRfPipe0))
  // to alu1&mult
  val idu_iu_rf_pipe1 = Input(new IduRfPipe1)
  // to BJU
  val idu_iu_rf_pipe2 = Input(new IduRfPipe2)
  val ifuForward = Flipped(DecoupledIO(Vec(2,new BhtPredDataForward)))
}

class IntegeUnit extends Module{
  val io = IO(new IntegerUnitIO)
  //==========================================================
  //                Pipeline 0 - alu0, special, div
  //==========================================================
  val alu0 = Module(new Alu)
  val special = Module(new Special)
  val du = Module(new Du)
  alu0.io.in    := io.idu_iu_rf_pipe0
  special.io.in := io.idu_iu_rf_pipe0
  du.io.in      := io.idu_iu_rf_pipe0
  //==========================================================
  //                Pipeline 1 - alu1, mult
  //==========================================================
  val alu1 = Module(new Alu)
  val mu = Module(new Mu)
  alu1.io.in := io.idu_iu_rf_pipe1
  mu.io.in  := io.idu_iu_rf_pipe1
  //==========================================================
  //                Pipeline 2 - bju
  //==========================================================
  val bju = Module(new Bju)
  bju.io.in.ifuForward := io.ifuForward
  bju.io.in.rfPipe2 := io.idu_iu_rf_pipe2
  // special pc io should be below Module 'bju' & 'special', because 'Suspicious forward reference'
  special.io.bjuSpecialPc := bju.io.out.specialPc
  //==========================================================
  //                Cbus - Iu pipes complete signal process
  //==========================================================
  val cbus = Module(new Cbus)
  cbus.io.normIn.divSel   := io.idu_iu_rf_pipe0.bits.divSel
  cbus.io.normIn.pipe0Sel := io.idu_iu_rf_pipe0.bits.sel
  cbus.io.normIn.pipe0Iid := io.idu_iu_rf_pipe0.bits.iid
  cbus.io.normIn.multSel  := io.idu_iu_rf_pipe1.mulSel
  cbus.io.normIn.pipe1Sel := io.idu_iu_rf_pipe1.sel
  cbus.io.normIn.pipe1Iid := io.idu_iu_rf_pipe1.iid
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









