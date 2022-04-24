package Core.IDU

import Core.Config
import Core.IDU.IS._
import Core.IntConfig.XLEN
import Core.IDU.RF._
import chisel3._
import chisel3.util._

class IDUInput extends Bundle{
  val fromCp0 = new Bundle{
    val icgEn = Bool()
    val yyClkEn  = Bool()
    /**
     * val ISfromCp0 = new Bundle {
     * val icgEn = Bool()
     * val yyClkEn = Bool()
     * }
    val dlb_disable = Bool()
    val iq_bypass_disable = Bool()
    val rob_fold_disable = Bool()
    val src2_fwd_disable = Bool()
    val srcv2_fwd_disable = Bool()
     */
    val cskyee = Bool()
    val frm    = UInt(3.W)
    val fs     = UInt(2.W)
    val vill   = Bool()
    val vs     = UInt(2.W)
    val vstart = UInt(7.W)
    val zeroDelayMoveDisable = Bool()
    val yyHyper = Bool()//yy_hyper
  }
  val IRfromCp0Sub = new Bundle {
    val dlbDisable = Bool()
    val robFoldDisable = Bool()
  }
  val aiq0fromCp0sub = new Bundle {
    val iqBypassDisable : Bool = Bool()
  }
  val IDfromFence = new Bundle{
    val idStall = Bool()
    val instVld = Vec(3, Bool())

    val instData = Vec(3, new IRData)
  }
  val IDfromIFUIB = new Bundle{
    val instVld = Vec(3,Bool())
    val pipedownGateclk = Bool()

    val instData = Vec(3, new IDData)
  }
  val IDfromHad = new Bundle{
    val debugIdInstEn = Bool()
    /**
     * val wbbr_data = UInt(64.W)
     * val wbbr_vld = Bool()
     */
  }
  val RFfromHad = new RFStageFromHadBundle
  val fromHpcp = new Bundle{
    val cntEn = Bool()
  }//////same to ID/IR/RF

  val fromIU = new Bundle{
    val mispred_stall = Bool() //////not in ID
    val yyxxCancel = Bool()
  }
  val RTfromIU = new Bundle{
    val div_inst_vld                 = Bool()
    val div_preg_dupx                = UInt(7.W)
    val ex2_pipe0_wb_preg_dupx       = UInt(7.W)
    val ex2_pipe0_wb_preg_vld_dupx   = Bool()
    val ex2_pipe1_mult_inst_vld_dupx = Bool()
    val ex2_pipe1_preg_dupx          = UInt(7.W)
    val ex2_pipe1_wb_preg_dupx       = UInt(7.W)
    val ex2_pipe1_wb_preg_vld_dupx   = Bool()
  }
  val aiq0fromIU = new Bundle {
    val div = new Bundle {
      val busy : Bool = Bool()
    }
  }
  val ISfromIUsub = new Bundle{
    val pcfifo_dis_inst_pid = Vec(4, UInt(5.W))
  }
  //  val IDfromRTU = new Bundle{
  //    val flushFe = Bool()
  //  }
  val fromRTU = new Bundle{
    val flush_fe = Bool()
    val flush_is = Bool()
    val flush_stall = Bool()
    val yy_xx_flush = Bool()
    val srt_en = Bool()

    val ereg_vld = Vec(4, Bool())
    val freg_vld = Vec(4, Bool())
    val preg_vld = Vec(4, Bool())
    val vreg_vld = Vec(4, Bool())
    val ereg = Vec(4, UInt(5.W))
    val freg = Vec(4, UInt(6.W))
    val preg = Vec(4, UInt(7.W))
    val vreg = Vec(4, UInt(6.W))
  }
  val ISfromRTUsub = new Bundle {
    val rob_full = Bool()

    val retire_int_vld = Bool()

    val rob_inst_idd = Vec(4, UInt(7.W))
  }
  val RTfromRTUsub = new Bundle{
    val rt_recover_preg = Vec(32, UInt(7.W))
  }
  val fromPad = new Bundle{
    val yyIcgScanEn = Bool()
  }//////same to ID/IR/IS/RF

  val RTfromVFPU = new Bundle{
    val ex1_pipe_mfvr_inst_vld_dupx = Vec(2, Bool())//pipe6 pipe7
    val ex1_pipe_preg_dupx          = Vec(2, UInt(7.W))
  }
  val ifu_xx_sync_reset = Bool()


  val fromLSU = new Bundle {
    /**
    val RTfromLSU = new Bundle{
      val ag_pipe3_load_inst_vld      = Bool()
      val ag_pipe3_preg_dupx          = UInt(7.W)
      val dc_pipe3_load_inst_vld_dupx = Bool()
      val dc_pipe3_preg_dupx          = UInt(7.W)
      val wb_pipe3_wb_preg_dupx       = UInt(7.W)
      val wb_pipe3_wb_preg_vld_dupx   = Bool()
    }
    */
    val ISfromLSU = new Bundle {
      val ag_pipe3_load_inst_vld = Bool()
      val ag_pipe3_preg_dupx = UInt(7.W)
      val ag_pipe3_vload_inst_vld = Bool()
      val ag_pipe3_vreg_dupx = UInt(7.W)
      val dc_pipe3_load_fwd_inst_vld_dupx = Bool()
      val dc_pipe3_load_inst_vld_dupx = Bool()
      val dc_pipe3_preg_dupx = UInt(7.W)
      val dc_pipe3_vload_fwd_inst_vld = Bool()
      val dc_pipe3_vload_inst_vld_dupx = Bool()
      val dc_pipe3_vreg_dupx = UInt(7.W)
      val vmb_create0_entry = UInt(8.W)
      val vmb_create1_entry = UInt(8.W)
      val wb_pipe3_wb_preg_dupx = UInt(7.W)
      val wb_pipe3_wb_preg_vld_dupx = Bool()
      val wb_pipe3_wb_vreg_dupx = UInt(7.W)
      val wb_pipe3_wb_vreg_vld_dupx = Bool()
    }
    val aiq0fromLSU = new Bundle {
      val load = new Bundle {
        val dcFwdInstValid : Bool = Bool()
      }
    }
  }
  val ISfromVFPU = Vec(2, new VFPU2IS)
}

class IDUOutput extends Bundle{
  val IDtoHad = new Bundle{
    val instVld = Vec(3,Bool())
    val pipeStall = Bool()

    val instInfo = Vec(3, new DebugInfo)
    /**
     * val pipeline_empty = Bool()
     * val wb_data = UInt(64.W)
     * val wb_vld = Bool()
     */
  }
  val IStoHad = new Bundle {
    val iq_empty = Bool()
  }
  val IDtoHpcp = new Bundle {
    val backendStall = Bool()
  }
  val IRtoHpcp = new Bundle {
    val inst_vld = Vec(4, new Bool())
    val inst_type = Vec(4, new hpcp_type)
  }
  val RFtoHpcp = new RFStageToHpcpBundle
  val IDtoIFU = new Bundle{
    val bypassStall = Bool()
    val stall = Bool()
  }
  val IStoIU = new Bundle{
    val pcfifo_inst_num = UInt(3.W)
    val pcfifo_inst_vld = Bool()
    /**
     * val div_gateclk_issue = Bool()
     * val div_issue = Bool()
     */
  }
  val RFtoIU = new RFStageToExuGateClkBundle

  /**
  val RFtoIU_pipe = new RFtoIU_pipeBundle
   */
  val IStoLSU = new Bundle {
    val vmb_create = Vec(2, new Bundle{
      val dp_en = Bool()
      val en = Bool()
      val gateclk_en = Bool()
      val dst_ready = Bool()
      val sdiq_entry = UInt(12.W)
      val split_num = UInt(7.W)
      val unit_stride = Bool()
      val vamo = Bool()
      val vl = UInt(8.W)
      val vreg = UInt(6.W)
      val vsew = UInt(2.W)
    })
  }
  val IRtoRTU = new Bundle{
    val ereg_alloc_vld = Vec(4, Bool())
    val ereg_alloc_gateclk_vld = Bool()

    val freg_alloc_vld = Vec(4, Bool())
    val freg_alloc_gateclk_vld = Bool()

    val preg_alloc_vld = Vec(4, Bool())
    val preg_alloc_gateclk_vld = Bool()

    val vreg_alloc_vld = Vec(4, Bool())
    val vreg_alloc_gateclk_vld = Bool()
  }
  val IStoRTU = new Bundle{
    val pst_dis = Vec(4, new Bundle{
      val ereg_vld = Bool()
      val freg_vld = Bool()
      val preg_vld = Bool()
      val vreg_vld = Bool()

      val dst_reg  = UInt(5.W)
      val dstv_reg = UInt(5.W)
      val ereg     = UInt(5.W)
      val ereg_iid = UInt(7.W)
      val preg     = UInt(7.W)
      val preg_iid = UInt(7.W)
      val rel_ereg = UInt(5.W)
      val rel_preg = UInt(7.W)
      val rel_vreg = UInt(6.W)
      val vreg     = UInt(6.W)
      val vreg_iid = UInt(7.W)
    })
    val rob_create = Vec(4, new Bundle{
      val dp_en = Bool()
      val en = Bool()
      val gateclk_en = Bool()
      val data = new ROBData
    })
  }
  /**
  val tovfpu = new tovfpuBundle
   */

}

class IDUIO extends Bundle{
  val in  = Input(new IDUInput)
  val out = Output(new IDUOutput)
}

class IDU extends Module with Config {
  val io = IO(new IDUIO)

  //aiq0 aiq1 biq lsiq sdiq viq0 viq1 vmb
  val iq_cnt_info = Vec(8, Wire(new IQCntInfo))
  iq_cnt_info(0).left_1_updt := aiq0.io.out.ctrl.oneLeftUpdate
  iq_cnt_info(0).full := aiq0.io.out.ctrl.full
  iq_cnt_info(0).empty := aiq0.io.out.ctrl.empty
  iq_cnt_info(0).full_updt := aiq0.io.out.ctrl.fullUpdate
  iq_cnt_info(0).full_updt_clk_en := aiq0.io.out.ctrl.fullUpdateClkEn
  iq_cnt_info(1) := DontCare
  iq_cnt_info(2).left_1_updt := biq.io.out.ctrl.oneLeftUpdate
  iq_cnt_info(2).full := biq.io.out.ctrl.full
  iq_cnt_info(2).empty := biq.io.out.ctrl.empty
  iq_cnt_info(2).full_updt := biq.io.out.ctrl.fullUpdate
  iq_cnt_info(2).full_updt_clk_en := biq.io.out.ctrl.fullUpdateClkEn
  iq_cnt_info(3).left_1_updt := lsiq.io.out.ctrl.oneLeftUpdate
  iq_cnt_info(3).full := lsiq.io.out.ctrl.full
  iq_cnt_info(3).empty := lsiq.io.out.ctrl.empty
  iq_cnt_info(3).full_updt := lsiq.io.out.ctrl.fullUpdate
  iq_cnt_info(3).full_updt_clk_en := lsiq.io.out.ctrl.fullUpdateClkEn
  iq_cnt_info(4) := DontCare
  iq_cnt_info(5) := DontCare
  iq_cnt_info(6) := DontCare
  iq_cnt_info(7) := DontCare

  val iq_create_entry = Wire((new ISStageInput).iq_create_entry)
  iq_create_entry.aiq0_aiq := aiq0.io.out.entryEnqOHVec //////todo: check it
  iq_create_entry.aiq1_aiq := DontCare
  iq_create_entry.biq_aiq := biq.io.out.entryEnqOHVec
  iq_create_entry.lsiq_aiq := lsiq.io.out.entryEnqOHVec
  iq_create_entry.sdiq_aiq := DontCare
  iq_create_entry.sdiq_dp := DontCare
  iq_create_entry.viq0_viq := DontCare
  iq_create_entry.viq1_viq := DontCare


  //==========================================================
  //                       ID Stage
  //==========================================================
  // &Instance("ct_idu_id_ctrl", "x_ct_idu_id_ctrl"); @32
  val idstage = Module(new IDStage)

  // &Instance("ct_idu_id_dp", "x_ct_idu_id_dp"); @33
  idstage.io.in.fromIR.irStall := irstage.io.out.ir_stall
  idstage.io.in.fromIR.irStageStall := irstage.io.out.ir_stage_stall
  idstage.io.in.fromCp0 := io.in.fromCp0
  idstage.io.in.fromPad := io.in.fromPad
  idstage.io.in.fromIFU := io.in.IDfromIFUIB
  idstage.io.in.fromIU := io.in.fromIU.yyxxCancel
  idstage.io.in.fromFence := io.in.IDfromFence
  idstage.io.in.fromHad := io.in.IDfromHad
  idstage.io.in.fromHpcp := io.in.fromHpcp
  idstage.io.in.fromRTU.flushFe := io.in.fromRTU.flush_fe

  //////todo: id_fence
  // &Instance("ct_idu_id_fence", "x_ct_idu_id_fence"); @34

  //==========================================================
  //                       IR Stage
  //==========================================================
  // &Instance("ct_idu_ir_ctrl", "x_ct_idu_ir_ctrl"); @39
  val irstage = Module(new IRStage)
  irstage.io.in.rt_resp := rt.io.out
  irstage.io.in.frt_resp := DontCare
  irstage.io.in.fromIU := io.in.fromIU
  irstage.io.in.fromPad := io.in.fromPad
  irstage.io.in.fromCp0.icgEn := io.in.fromCp0.icgEn
  irstage.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  irstage.io.in.fromCp0.dlbDisable := io.in.IRfromCp0Sub.dlbDisable
  irstage.io.in.fromCp0.robFoldDisable := io.in.IRfromCp0Sub.robFoldDisable
  irstage.io.in.aiq_entry_cnt_updt := DontCare //////todo: check it
  irstage.io.in.id_pipedown := idstage.io.out.pipedown
  irstage.io.in.fromHpcp := io.in.fromHpcp
  irstage.io.in.fromIS := isstage.io.out.toIR
  irstage.io.in.fromRTU := io.in.fromRTU
  irstage.io.in.viq_entry_cnt_updt := DontCare
  irstage.io.in.vrt_resp := DontCare
  // &Instance("ct_idu_ir_dp", "x_ct_idu_ir_dp"); @40

  // &ConnRule(s/_dupx/_dup0/); @41
  // &Instance("ct_idu_ir_rt", "x_ct_idu_ir_rt"); @42
  val rt = Module(new RenameTable)
  rt.io.in.rt_req := irstage.io.out.rt_req
  rt.io.in.fromRTU.flush_fe := io.in.fromRTU.flush_fe
  rt.io.in.fromRTU.flush_is := io.in.fromRTU.flush_is
  rt.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  rt.io.in.fromRTU.rt_recover_preg := io.in.RTfromRTUsub.rt_recover_preg
  rt.io.in.fromIU := io.in.RTfromIU
  rt.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  rt.io.in.fromCp0.IcgEn := io.in.fromCp0.icgEn
  rt.io.in.fromRf := DontCare //////todo: find out
  rt.io.in.fromLSU.wb_pipe3_wb_preg_dupx := io.in.fromLSU.ISfromLSU.wb_pipe3_wb_preg_dupx
  rt.io.in.fromLSU.wb_pipe3_wb_preg_vld_dupx := io.in.fromLSU.ISfromLSU.wb_pipe3_wb_preg_vld_dupx
  rt.io.in.fromLSU.ag_pipe3_preg_dupx := io.in.fromLSU.ISfromLSU.ag_pipe3_preg_dupx
  rt.io.in.fromLSU.ag_pipe3_load_inst_vld := io.in.fromLSU.ISfromLSU.ag_pipe3_load_inst_vld
  rt.io.in.fromLSU.dc_pipe3_preg_dupx := io.in.fromLSU.ISfromLSU.dc_pipe3_preg_dupx
  rt.io.in.fromLSU.dc_pipe3_load_inst_vld_dupx := io.in.fromLSU.ISfromLSU.dc_pipe3_load_inst_vld_dupx
  rt.io.in.fromVFPU := io.in.RTfromVFPU
  rt.io.in.ifu_xx_sync_reset := io.in.ifu_xx_sync_reset
  rt.io.in.ir_stall := irstage.io.out.ir_stall
  rt.io.in.pad_yy_icg_scan_en := io.in.fromPad.yyIcgScanEn


  //==========================================================
  //                       IS Stage
  //==========================================================
  // &Instance("ct_idu_is_ctrl", "x_ct_idu_is_ctrl"); @51
  val isstage = Module(new ISStage)
  // &ConnRule(s/_dupx/_dup1/); @52
  // &Instance("ct_idu_is_dp", "x_ct_idu_is_dp"); @53
  isstage.io.in.fromLSU := io.in.fromLSU.ISfromLSU
  isstage.io.in.fromPad := io.in.fromPad
  isstage.io.in.fromIU.yyxxCancel := io.in.fromIU.yyxxCancel
  isstage.io.in.fromIU.div_inst_vld := io.in.RTfromIU.div_inst_vld
  isstage.io.in.fromIU.div_preg_dupx := io.in.RTfromIU.div_preg_dupx
  isstage.io.in.fromIU.pcfifo_dis_inst_pid := io.in.ISfromIUsub.pcfifo_dis_inst_pid
  isstage.io.in.fromIU.ex2_pipe0_wb_preg_dupx := io.in.RTfromIU.ex2_pipe0_wb_preg_dupx
  isstage.io.in.fromIU.ex2_pipe1_wb_preg_dupx := io.in.RTfromIU.ex2_pipe1_wb_preg_dupx
  isstage.io.in.fromIU.ex2_pipe1_preg_dupx := io.in.RTfromIU.ex2_pipe1_preg_dupx
  isstage.io.in.fromIU.ex2_pipe0_wb_preg_vld_dupx := io.in.RTfromIU.ex2_pipe0_wb_preg_vld_dupx
  isstage.io.in.fromIU.ex2_pipe1_wb_preg_vld_dupx := io.in.RTfromIU.ex2_pipe1_wb_preg_vld_dupx
  isstage.io.in.fromIU.ex2_pipe1_mult_inst_vld_dupx := io.in.RTfromIU.ex2_pipe1_mult_inst_vld_dupx
  isstage.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  isstage.io.in.fromCp0.icgEn := io.in.fromCp0.icgEn
  isstage.io.in.fromRTU.flush_fe := io.in.fromRTU.flush_fe
  isstage.io.in.fromRTU.flush_is := io.in.fromRTU.flush_is
  isstage.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  isstage.io.in.fromRTU.flush_stall := io.in.fromRTU.flush_stall
  isstage.io.in.fromRTU.rob_full := io.in.ISfromRTUsub.rob_full
  isstage.io.in.fromRTU.retire_int_vld := io.in.ISfromRTUsub.retire_int_vld
  isstage.io.in.fromRTU.rob_inst_idd := io.in.ISfromRTUsub.rob_inst_idd
  isstage.io.in.fromRf := DontCare //////todo: find out
  isstage.io.in.fromVFPU := io.in.ISfromVFPU
  isstage.io.in.iq_cnt_info := iq_cnt_info
  isstage.io.in.iq_create_entry := iq_create_entry
  isstage.io.in.ir_pipedown := irstage.io.out.ir_pipedown
  isstage.io.in.ir_type_stall_inst2_vld := irstage.io.out.ir_type_stall_inst2_vld
  isstage.io.in.ir_type_stall_inst3_vld := irstage.io.out.ir_type_stall_inst3_vld
  isstage.io.in.lsiq_dp_create_bypass_oldest := lsiq.io.out.data.createBypassOldest
  isstage.io.in.lsiq_dp_no_spec_store_vld := lsiq.io.out.data.noSpecStoreValid
  isstage.io.in.pre_dispatch := irstage.io.out.pre_dispatch

  // &ConnRule(s/_dupx/_dup2/); @54
  // &Instance("ct_idu_is_aiq0", "x_ct_idu_is_aiq0"); @55
  val aiq0 = Module(new ArithInstQueue)
  aiq0.io.in.ctrl.createVec(0).createEn(0) := isstage.io.out.iqCreateEn(0)(0).en //////todo: check it
  aiq0.io.in.ctrl.createVec(0).createEn(1) := isstage.io.out.iqCreateEn(0)(1).en
  aiq0.io.in.ctrl.createVec(0).createDpEn(0) := isstage.io.out.iqCreateEn(0)(0).dp_en
  aiq0.io.in.ctrl.createVec(0).createDpEn(1) := isstage.io.out.iqCreateEn(0)(1).dp_en
  aiq0.io.in.ctrl.createVec(0).createGateClkEn(0) := isstage.io.out.iqCreateEn(0)(0).gateclk_en
  aiq0.io.in.ctrl.createVec(0).createGateClkEn(1) := isstage.io.out.iqCreateEn(0)(1).gateclk_en
  aiq0.io.in.ctrl.createVec(0).createSel := isstage.io.out.iqCreateEn(0)(0).sel //////todo: is something wrong???
  aiq0.io.in.ctrl.createVec(0).stall := rf.io.ctrl.out.toIq(0).stall
  aiq0.io.in.ctrl.createVec(0).rfLaunchFailValid := rf.io.ctrl.out.toIq(0).launchFailValid
  aiq0.io.in.ctrl.createVec(0).rfAluRegFwdValid := DontCare //////todo: complete rf
  aiq0.io.in.ctrl.createVec(0).rfPopDlbValid := rf.io.ctrl.out.toIq(0).popDlbValid
  aiq0.io.in.ctrl.createVec(0).rfPopValid := rf.io.ctrl.out.toIq(0).popValid
  aiq0.io.in.ctrl.xxRfPipe0PregLaunchValidDupx := DontCare//////todo: complete rf
  aiq0.io.in.ctrl.xxRfPipe1PregLaunchValidDupx := DontCare//////todo: complete rf
  aiq0.io.in.data.createData := isstage.io.out.toAiq0.create_data
  aiq0.io.in.data.bypassData := isstage.io.out.toAiq0.bypass_data
  aiq0.io.in.data.createDiv := isstage.io.out.toAiq0.create_div
  aiq0.io.in.data.srcReadyForBypass := isstage.io.out.toAiq0.src_rdy_for_bypass
  aiq0.io.in.data.rfReadyClr := rf.io.data.out.toAiq0.readyClr
  aiq0.io.in.data.rfLaunchEntry := rf.io.data.out.toAiq0.launchEntryOH
  aiq0.io.in.data.dispatchInstSrcPregVec := isstage.io.out.toAiq.inst_src_preg
  aiq0.io.in.data.sdiqCreateSrcSelVec := isstage.io.out.toAiq.sdiq_create_src_sel
  aiq0.io.in.fromLsu := io.in.fromLSU.aiq0fromLSU
  aiq0.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  aiq0.io.in.fromCp0.icgEn := io.in.fromCp0.icgEn
  aiq0.io.in.fromCp0.iqBypassDisable := io.in.aiq0fromCp0sub.iqBypassDisable
  aiq0.io.in.fromIu := io.in.aiq0fromIU
  aiq0.io.in.aiqEntryCreateVec//////

  // &ConnRule(s/_dupx/_dup3/); @56
  // &Instance("ct_idu_is_aiq1", "x_ct_idu_is_aiq1"); @57
  //todo: add aiq1

  // &ConnRule(s/_dupx/_dup4/); @58
  // &Instance("ct_idu_is_biq", "x_ct_idu_is_biq"); @59
  val biq = Module(new Biq)

  // &ConnRule(s/_dupx/_dup1/); @60
  // &Instance("ct_idu_is_lsiq", "x_ct_idu_is_lsiq"); @61
  val lsiq = Module(new Lsiq)

  // &ConnRule(s/_dupx/_dup1/); @62
  // &Instance("ct_idu_is_sdiq", "x_ct_idu_is_sdiq"); @63
  //todo: add sdiq

  // &ConnRule(s/_dupx/_dup2/); @66
  // &Instance("ct_idu_is_viq0_dummy", "x_ct_idu_is_viq0"); @67
  // &ConnRule(s/_dupx/_dup3/); @68
  // &Instance("ct_idu_is_viq1_dummy", "x_ct_idu_is_viq1"); @69
  // &ConnRule(s/_dupx/_dup2/); @71
  // &Instance("ct_idu_is_viq0", "x_ct_idu_is_viq0"); @72
  //todo: add viq0, viq1


  //==========================================================
  //                       RF Stage
  //==========================================================
  // &Instance("ct_idu_rf_ctrl", "x_ct_idu_rf_ctrl"); @80
  val rf = Module(new RFStage)
  // &Instance("ct_idu_rf_dp", "x_ct_idu_rf_dp"); @81

  // &Instance("ct_idu_rf_fwd", "x_ct_idu_rf_fwd"); @82
  //todo: add rf_fwd

}
