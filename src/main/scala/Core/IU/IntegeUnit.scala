package Core.IU
import Core.FuncOpType
import Core.IU.Bju.{BhtPredDataForward, Bju}
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

}
class IduRfPipe0 extends CtrlSignalHasDestIO{
  val opcode = UInt(32.W)
  val pid = UInt(5.W)
  val special_imm = UInt(20.W)
  val expt_vec = UInt(5.W)
  val expt_vld = Bool()
}
class IduRfPipe1 extends CtrlSignalHasDestIO{
  val mult_func = UInt(8.W)
  val mla_src2_preg = UInt(7.W)
  val mla_src2_vld = Bool()
}
class IduRfPipe2 extends CtrlSignalIO{
  val pid = UInt(5.W)
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
  // pipeline 2 - bju
  val bju = Module(new Bju)
  bju.io.in.ifuForward := io.ifuForward
  bju.io.in.rfPipe2 := io.idu_iu_rf_pipe2


}









