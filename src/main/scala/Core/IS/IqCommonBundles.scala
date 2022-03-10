package Core.IS

import Core.IntConfig._
import chisel3._
import chisel3.util._

abstract class IqEntryInput(NumEntry: Int, NumSrc: Int, NumSrcX: Int) extends Bundle with DepRegEntryConfig {
  val fromCp0 = new IqEntryFromCp0
  /**
   * Include load fwd, alu0 reg fwd, alu1 reg fwd
   * alu0: x_alu0_reg_fwd_vld
   * alu1: x_alu1_reg_fwd_vld
   * load: lsu_idu_dc_pipe3_load_fwd_inst_vld_dupx
   */
  val fwdValid : Vec[FwdValidBundle] = Vec(NumSrcX, new FwdValidBundle)
  /**
   * Include alu0, alu1, mul, div, load, vfpu0, vfpu1 <br>
   * alu0 : ctrl_xx_rf_pipe0_preg_lch_vld_dupx <br>
   *        dp_xx_rf_pipe0_dst_preg_dupx <br>
   * alu1 : ctrl_xx_rf_pipe1_preg_lch_vld_dupx <br>
   *        dp_xx_rf_pipe1_dst_preg_dupx <br>
   * mult : iu_idu_ex2_pipe1_mult_inst_vld_dupx <br>
   *        iu_idu_ex2_pipe1_preg_dupx <br>
   * div  : iu_idu_div_inst_vld <br>
   *        iu_idu_div_preg_dupx <br>
   * load : lsu_idu_dc_pipe3_load_inst_vld_dupx <br>
   *        lsu_idu_dc_pipe3_preg_dupx <br>
   * vfpu0: vfpu_idu_ex1_pipe6_mfvr_inst_vld_dupx <br>
   *        vfpu_idu_ex1_pipe6_preg_dupx <br>
   * vfpu1: vfpu_idu_ex1_pipe7_mfvr_inst_vld_dupx <br>
   *        vfpu_idu_ex1_pipe7_preg_dupx <br>
   */
  val fuDstPreg : Vec[ValidIO[UInt]] = Vec(NumFuHasDstReg, ValidIO(UInt(NumPhysicRegsBits.W)))

  /**
   * Include pipe0,1,3 wb <br>
   * pipe0wb : iu_idu_ex2_pipe0_wb_preg_vld_dupx
   * pipe1wb : iu_idu_ex2_pipe1_wb_preg_vld_dupx
   * pipe3wb : lsu_idu_wb_pipe3_wb_preg_vld_dupx
   */
  val wbPreg : Vec[ValidIO[UInt]] = Vec(WbNum, ValidIO(UInt(NumPhysicRegsBits.W)))

  /**
   * LSU reg Bypass
   * lsu_idu_ag_pipe3_load_inst_vld
   * lsu_idu_ag_pipe3_preg_dupx
   */
  val loadPreg = ValidIO(UInt(NumPhysicRegsBits.W))

  val popValid  : Bool = Bool()
  val readyClr  : Vec[Bool] = Vec(NumSrc, Bool())
  val fromRtu   : InstQueFromRtu = new InstQueFromRtu
  val fromPad   : InstQueFromPad = new InstQueFromPad
  val issueEn   : Bool = Bool()
  val popCurEntry   : Bool = Bool()
  val popOtherEntry : Vec[Bool] = Vec(NumEntry - 1, Bool())
}

abstract class IqEntryOutput(NumEntry: Int) extends Bundle with DepRegEntryConfig {
  val ageVec  : Vec[Bool] = Vec(this.NumEntry - 1, Bool())
  val ready   : Bool = Bool()
  val valid   : Bool = Bool()
  val validWithoutFreeze : Bool = Bool()
}

trait IqHasVectorInputBundle {
  /**
   * lsu_idu_dc_pipe3_vload_fwd_inst_vld
   */
  val vFwdValid : Bool = Bool()
  def NumVfuHasDstReg = 7

  /**
   * vfpu0_ex3_data_ready: vfpu_idu_ex1_pipe6_data_vld_dupx
   * vfpu0_ex4_data_ready: vfpu_idu_ex2_pipe6_data_vld_dupx
   * vfpu0_ex5_data_ready: vfpu_idu_ex3_pipe6_data_vld_dupx
   * vfpu1_ex3_data_ready: vfpu_idu_ex1_pipe7_data_vld_dupx
   * vfpu1_ex4_data_ready: vfpu_idu_ex2_pipe7_data_vld_dupx
   * vfpu1_ex5_data_ready: vfpu_idu_ex3_pipe7_data_vld_dupx
   * load_data_ready: lsu_idu_dc_pipe3_vload_inst_vld_dupx
   */
  val vFuDstPreg : Vec[ValidIO[UInt]] = Vec(NumVfuHasDstReg, ValidIO(UInt(NumPhysicRegsBits.W)))

  def VWbNum = 3
  /**
   * Include pipe0,1,3 wb <br>
   * pipe3wb : lsu_idu_wb_pipe3_wb_vreg_vld_dupx
   * pipe6wb : vfpu_idu_ex5_pipe6_wb_vreg_vld_dupx
   * pipe7wb : vfpu_idu_ex5_pipe7_wb_vreg_vld_dupx
   */
  val vWbPreg : Vec[ValidIO[UInt]] = Vec(VWbNum, ValidIO(UInt(NumPhysicRegsBits.W)))

  /**
   * LSU vreg Bypass
   * lsu_idu_ag_pipe3_vload_inst_vld
   * lsu_idu_ag_pipe3_vreg_dupx
   */
  val vLoadPreg = ValidIO(UInt(NumPhysicRegsBits.W))
}