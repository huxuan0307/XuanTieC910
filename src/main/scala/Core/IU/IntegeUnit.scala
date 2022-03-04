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
  val alu_short = Bool()
  val dst_preg = UInt(7.W)
  val dst_vreg = UInt(7.W)
  val dstv_vld = Bool()
  val src1_no_imm = UInt(64.W)
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
}
class IduRfPipe1 extends CtrlSignalHasDestIO{
  val mult_func = UInt(8.W)
  val mla_src2_preg = UInt(7.W)
  val mla_src2_vld = Bool()
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
  val mul = Module(new Mu)
  alu1.io.in := io.idu_iu_rf_pipe1
  mul.io.in  := io.idu_iu_rf_pipe1
  //==========================================================
  //                Pipeline 2 - bju
  //==========================================================
  val bju = Module(new Bju)
  bju.io.in.ifuForward := io.ifuForward
  bju.io.in.rfPipe2 := io.idu_iu_rf_pipe2

  // special pc io should be below Module 'bju' & 'special', because 'Suspicious forward reference'
  special.io.bjuSpecialPc := bju.io.out.specialPc
}









