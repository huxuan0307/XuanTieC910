package Core.IDU

import Core.Config

import Core.IDU.IS.AiqConfig.{NumAiqCreatePort, NumSrcArith}
import Core.IDU.RF.PrfConfig.NumPregReadPort

import Core.IntConfig.{NumPhysicRegsBits, XLEN}
import IS._
import RF._
import chisel3._
import chisel3.util._

class IDUInput extends Bundle with AiqConfig with DepRegEntryConfig{
  val fromCp0 = new Bundle{
    val icgEn = Bool()
    val yyClkEn  = Bool()
    /**
     * val ISfromCp0 = new Bundle {
     * val icgEn = Bool()
     * val yyClkEn = Bool()
     * }
    val dlb_disable = Bool()
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
  val IQfromCp0sub = new Bundle {
    val iq_bypass_disable = Bool()
  }
  val IRfromCp0Sub = new Bundle {
    val dlbDisable = Bool()
    val robFoldDisable = Bool()
  }
  val RFfromCp0sub = new Bundle {
    val lsuFenceIBroadDis = Bool()
    val lsuFenceRwBroadDis = Bool()
    val lsuTlbBroadDis = Bool()
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
  val RFfromIU = new Bundle{
    val stall = new Bundle {
      val divWb   : Bool = Bool()
      val mulEx1  : Bool = Bool()
    }
  }
  val aiq0fromIU = new Bundle {
    val div = new Bundle {
      val busy : Bool = Bool()
    }
  }
  val biqfromIU = new Bundle {
    val div = new Bundle {
      val instValid : Bool = Bool()
      val preg : UInt = UInt(NumPhysicRegsBits.W)
    }
  }

  val IQfromIUsub = new Bundle {
    val wbPreg : Vec[ValidIO[UInt]] = Vec(WbNum, ValidIO(UInt(NumPhysicRegsBits.W)))
  }
  val ISfromIUsub = new Bundle{
    val pcfifo_dis_inst_pid = Vec(4, UInt(5.W))
  }
  val PRFfromIU = new Bundle{
    val ex2_pipe0_wb_preg_data = UInt(XLEN.W)
    //todo: add //val ex2_pipe0_wb_preg_expand = UInt(NumPhysicRegs.W)
    val ex2_pipe0_wb_preg_vld = Bool()
    val ex2_pipe1_wb_preg_data = UInt(XLEN.W)
    val ex2_pipe0_wb_preg = UInt(NumPhysicRegsBits.W)
    val ex2_pipe1_wb_preg_vld = Bool()
    val ex2_pipe1_wb_preg = UInt(NumPhysicRegsBits.W)
    val lsu_wb_pipe3_wb_preg_data = UInt(XLEN.W)
    val lsu_wb_pipe3_wb_preg_vld = Bool()
    val lsu_wb_pipe3_wb_preg = UInt(NumPhysicRegsBits.W)
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
  val PRFfromRTUsub = new Bundle {
    val yyXxDebugOn : Bool = Bool()
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
  val RFfromVFPU = new Bundle {
    val stall = new Bundle {
      val vdivWb = Bool()
    }
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
    val IQfromLSU = new Bundle {
      val load = new Bundle {
        val dcFwdInstValid : Bool = Bool()
      }
    }
    val aiq0fromLSUsub = new Bundle {
      val loadPreg = ValidIO(UInt(NumPhysicRegsBits.W))
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
  val PRFtoHad = new PrfToHadBundle
  val IDtoHpcp = new Bundle {
    val backendStall = Bool()
  }
  val IRtoHpcp = new Bundle {
    val inst_vld = Vec(4, new Bool())
    val inst_type = Vec(4, new hpcp_type)
  }
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

  //////val RFtoCp0 = new RFStageToCp0Bundle
  //////val RFtoHpcp = new RFStageToHpcpBundle
  val RFCtrl = new RFStageCtrlOutput
  val RFData = (new RFStageDataOutput).toIu
  /**
  val RFtoIU = new RFStageToExuGateClkBundle
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
  //////todo: complete io
}

class IDU extends Module with Config {
  val io = IO(new IDUIO)
  val idstage = Module(new IDStage)
  val irstage = Module(new IRStage)
  val rt = Module(new RenameTable)
  val isstage = Module(new ISStage)
  val aiq0 = Module(new ArithInstQueue)
  val biq = Module(new Biq)
  val lsiq = Module(new Lsiq)
  val rfstage = Module(new RFStage)
  val prf = Module(new Prf)
  //////todo: add 1ereg, 2 vreg, 2freg

  io.out := DontCare
  //aiq0 aiq1 biq lsiq sdiq viq0 viq1 vmb
  //val iq_cnt_info = WireInit(VecInit(Seq.fill(8)(VecInit(Seq.fill(5)(false.B)))))
  val iq_cnt_info = WireInit(VecInit(Seq.fill(8)(0.U.asTypeOf(new IQCntInfo))))
  iq_cnt_info(0).left_1_updt := aiq0.io.out.ctrl.oneLeftUpdate
  iq_cnt_info(0).full := aiq0.io.out.ctrl.full
  iq_cnt_info(0).empty := aiq0.io.out.ctrl.empty
  iq_cnt_info(0).full_updt := aiq0.io.out.ctrl.fullUpdate
  iq_cnt_info(0).full_updt_clk_en := aiq0.io.out.ctrl.fullUpdateClkEn
  //  iq_cnt_info(0)(0) := aiq0.io.out.ctrl.oneLeftUpdate
  //  iq_cnt_info(0)(1) := aiq0.io.out.ctrl.full
  //  iq_cnt_info(0)(2) := aiq0.io.out.ctrl.empty
  //  iq_cnt_info(0)(3) := aiq0.io.out.ctrl.fullUpdate
  //  iq_cnt_info(0)(4) := aiq0.io.out.ctrl.fullUpdateClkEn
  //  iq_cnt_info(0) := Seq(aiq0.io.out.ctrl.oneLeftUpdate,aiq0.io.out.ctrl.full,aiq0.io.out.ctrl.empty,aiq0.io.out.ctrl.fullUpdate,aiq0.io.out.ctrl.fullUpdateClkEn)
  iq_cnt_info(1) := DontCare
  iq_cnt_info(2).left_1_updt := biq.io.out.ctrl.oneLeftUpdate
  iq_cnt_info(2).full := biq.io.out.ctrl.full
  iq_cnt_info(2).empty:= biq.io.out.ctrl.empty
  iq_cnt_info(2).full_updt := biq.io.out.ctrl.fullUpdate
  iq_cnt_info(2).full_updt_clk_en := biq.io.out.ctrl.fullUpdateClkEn
  iq_cnt_info(3).left_1_updt := lsiq.io.out.ctrl.oneLeftUpdate
  iq_cnt_info(3).full := lsiq.io.out.ctrl.full
  iq_cnt_info(3).empty := lsiq.io.out.ctrl.empty
  iq_cnt_info(3).full_updt := lsiq.io.out.ctrl.fullUpdate
  iq_cnt_info(3).full_updt_clk_en := lsiq.io.out.ctrl.fullUpdateClkEn
  //  iq_cnt_info(2)(0) := biq.io.out.ctrl.oneLeftUpdate
  //  iq_cnt_info(2)(1) := biq.io.out.ctrl.full
  //  iq_cnt_info(2)(2) := biq.io.out.ctrl.empty
  //  iq_cnt_info(2)(3) := biq.io.out.ctrl.fullUpdate
  //  iq_cnt_info(2)(4) := biq.io.out.ctrl.fullUpdateClkEn
  //  iq_cnt_info(3)(0) := lsiq.io.out.ctrl.oneLeftUpdate
  //  iq_cnt_info(3)(1) := lsiq.io.out.ctrl.full
  //  iq_cnt_info(3)(2) := lsiq.io.out.ctrl.empty
  //  iq_cnt_info(3)(3) := lsiq.io.out.ctrl.fullUpdate
  //  iq_cnt_info(3)(4) := lsiq.io.out.ctrl.fullUpdateClkEn
  iq_cnt_info(4) := DontCare
  iq_cnt_info(5) := DontCare
  iq_cnt_info(6) := DontCare
  iq_cnt_info(7) := DontCare

  val iq_create_entry = WireInit(0.U.asTypeOf((new ISStageInput).iq_create_entry))
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



  // &Instance("ct_idu_id_dp", "x_ct_idu_id_dp"); @33
  idstage.io.in.fromIR.irStall := irstage.io.out.ir_stall
  idstage.io.in.fromIR.irStageStall := irstage.io.out.ir_stage_stall
  idstage.io.in.fromCp0 := io.in.fromCp0
  idstage.io.in.fromPad := io.in.fromPad
  idstage.io.in.fromIFU := io.in.IDfromIFUIB
  idstage.io.in.fromIU.yyxxCancel := io.in.fromIU.yyxxCancel
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

  rt.io.in.rt_req := irstage.io.out.rt_req
  rt.io.in.fromRTU.flush_fe := io.in.fromRTU.flush_fe
  rt.io.in.fromRTU.flush_is := io.in.fromRTU.flush_is
  rt.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  rt.io.in.fromRTU.rt_recover_preg := io.in.RTfromRTUsub.rt_recover_preg
  rt.io.in.fromIU := io.in.RTfromIU
  rt.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  rt.io.in.fromCp0.IcgEn := io.in.fromCp0.icgEn
  rt.io.in.fromRf := DontCare //////todo: find out, mainly about dup signals, can be ignore?
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
  for(i <- 0 to 7) {
    //    isstage.io.in.iq_cnt_info(i).left_1_updt := iq_cnt_info(i)(0)
    //    isstage.io.in.iq_cnt_info(i).full := iq_cnt_info(i)(1)
    //    isstage.io.in.iq_cnt_info(i).empty := iq_cnt_info(i)(2)
    //    isstage.io.in.iq_cnt_info(i).full_updt := iq_cnt_info(i)(3)
    //    isstage.io.in.iq_cnt_info(i).full_updt_clk_en := iq_cnt_info(i)(4)
    isstage.io.in.iq_cnt_info(i).left_1_updt := iq_cnt_info(i).left_1_updt
    isstage.io.in.iq_cnt_info(i).full := iq_cnt_info(i).full
    isstage.io.in.iq_cnt_info(i).empty := iq_cnt_info(i).empty
    isstage.io.in.iq_cnt_info(i).full_updt := iq_cnt_info(i).full_updt
    isstage.io.in.iq_cnt_info(i).full_updt_clk_en := iq_cnt_info(i).full_updt_clk_en
  }
  isstage.io.in.iq_create_entry := iq_create_entry
  isstage.io.in.ir_pipedown := irstage.io.out.ir_pipedown
  isstage.io.in.ir_type_stall_inst2_vld := irstage.io.out.ir_type_stall_inst2_vld
  isstage.io.in.ir_type_stall_inst3_vld := irstage.io.out.ir_type_stall_inst3_vld
  isstage.io.in.lsiq_dp_create_bypass_oldest := lsiq.io.out.data.createBypassOldest
  isstage.io.in.lsiq_dp_no_spec_store_vld := lsiq.io.out.data.noSpecStoreValid
  isstage.io.in.pre_dispatch := irstage.io.out.pre_dispatch

  // &ConnRule(s/_dupx/_dup2/); @54
  // &Instance("ct_idu_is_aiq0", "x_ct_idu_is_aiq0"); @55

  aiq0.io.in.ctrl.createVec.zipWithIndex.foreach {
    case (cbundle, i) =>
      for(j<- 0 to NumAiqCreatePort-1){
        cbundle.createEn(j) := isstage.io.out.iqCreateEn(i)(j).en
        cbundle.createDpEn(j) := isstage.io.out.iqCreateEn(i)(j).dp_en
        cbundle.createGateClkEn(j) := isstage.io.out.iqCreateEn(i)(j).gateclk_en
      }
      cbundle.createSel := isstage.io.out.iqCreateEn(i)(0).sel //////todo: is something wrong???
      cbundle.stall := rfstage.io.ctrl.out.toIq(i).stall
      cbundle.rfLaunchFailValid := rfstage.io.ctrl.out.toIq(i).launchFailValid
      cbundle.rfAluRegFwdValid := DontCare //////todo: complete rf
      cbundle.rfPopDlbValid := rfstage.io.ctrl.out.toIq(i).popDlbValid
      cbundle.rfPopValid := rfstage.io.ctrl.out.toIq(i).popValid
  }
  //  aiq0.io.in.ctrl.createVec(0).createEn(0) := isstage.io.out.iqCreateEn(0)(0).en //////todo: check it
  //  aiq0.io.in.ctrl.createVec(0).createEn(1) := isstage.io.out.iqCreateEn(0)(1).en
  //  aiq0.io.in.ctrl.createVec(0).createDpEn(0) := isstage.io.out.iqCreateEn(0)(0).dp_en
  //  aiq0.io.in.ctrl.createVec(0).createDpEn(1) := isstage.io.out.iqCreateEn(0)(1).dp_en
  //  aiq0.io.in.ctrl.createVec(0).createGateClkEn(0) := isstage.io.out.iqCreateEn(0)(0).gateclk_en
  //  aiq0.io.in.ctrl.createVec(0).createGateClkEn(1) := isstage.io.out.iqCreateEn(0)(1).gateclk_en
  //  aiq0.io.in.ctrl.createVec(0).createSel := isstage.io.out.iqCreateEn(0)(0).sel //////todo: is something wrong???
  //  aiq0.io.in.ctrl.createVec(0).stall := rfstage.io.ctrl.out.toIq(0).stall
  //  aiq0.io.in.ctrl.createVec(0).rfLaunchFailValid := rfstage.io.ctrl.out.toIq(0).launchFailValid
  //  aiq0.io.in.ctrl.createVec(0).rfAluRegFwdValid := DontCare //////todo: complete rf
  //  aiq0.io.in.ctrl.createVec(0).rfPopDlbValid := rfstage.io.ctrl.out.toIq(0).popDlbValid
  //  aiq0.io.in.ctrl.createVec(0).rfPopValid := rfstage.io.ctrl.out.toIq(0).popValid
  //  aiq0.io.in.ctrl.createVec(1) := DontCare
  //  aiq0.io.in.ctrl.createVec(2) := DontCare
  //  aiq0.io.in.ctrl.createVec(3) := DontCare
  //  aiq0.io.in.ctrl.createVec(4) := DontCare
  aiq0.io.in.ctrl.xxRfPipe0PregLaunchValidDupx := DontCare//////todo: complete rf
  aiq0.io.in.ctrl.xxRfPipe1PregLaunchValidDupx := DontCare//////todo: complete rf
  aiq0.io.in.data.createData.zipWithIndex.foreach {
    case (cbundle, i) =>
      cbundle.div := isstage.io.out.toAiq0.create_data(i).DIV
      cbundle.iid := isstage.io.out.toAiq0.create_data(i).IID
      cbundle.aluShort := isstage.io.out.toAiq0.create_data(i).ALU_SHORT
      cbundle.dstPreg := isstage.io.out.toAiq0.create_data(i).DST_PREG
      cbundle.dstValid := isstage.io.out.toAiq0.create_data(i).DST_VLD
      cbundle.dstVreg := isstage.io.out.toAiq0.create_data(i).DST_VREG
      cbundle.dstVValid := isstage.io.out.toAiq0.create_data(i).DSTV_VLD
      cbundle.exceptVec.valid := isstage.io.out.toAiq0.create_data(i).EXPT_VLD
      cbundle.exceptVec.bits := isstage.io.out.toAiq0.create_data(i).EXPT_VEC
      cbundle.highHwExcept := isstage.io.out.toAiq0.create_data(i).HIGH_HW_EXPT
      cbundle.launchPreg := isstage.io.out.toAiq0.create_data(i).LCH_PREG
      cbundle.mtvr := isstage.io.out.toAiq0.create_data(i).MTVR
      cbundle.pcFifo := isstage.io.out.toAiq0.create_data(i).PCFIFO
      cbundle.pid := isstage.io.out.toAiq0.create_data(i).PID
      cbundle.special := isstage.io.out.toAiq0.create_data(i).SPECIAL
      cbundle.srcValid := isstage.io.out.toAiq0.create_data(i).src_vld
      for (j <- 0 to NumSrcArith-1) {
        cbundle.srcVec(j).preg := isstage.io.out.toAiq0.create_data(i).src_info(j).src_data.preg
        cbundle.srcVec(j).wb := isstage.io.out.toAiq0.create_data(i).src_info(j).src_data.wb
        cbundle.srcVec(j).ready := isstage.io.out.toAiq0.create_data(i).src_info(j).src_data.rdy
        cbundle.srcVec(j).lsuMatch := isstage.io.out.toAiq0.create_data(i).src_info(j).lsu_match
      }
      cbundle.vl := isstage.io.out.toAiq0.create_data(i).VL
      cbundle.vlmul := isstage.io.out.toAiq0.create_data(i).VLMUL
      cbundle.vsew := isstage.io.out.toAiq0.create_data(i).VSEW
      cbundle.inst := isstage.io.out.toAiq0.create_data(i).OPCODE //todo: check it
      //todo: find isstage.io.out.toAiq0.create_data(i).LCH_RDY_AIQ0... to where
  } //:= isstage.io.out.toAiq0.create_data //////todo: make it same
  //todo: find isstage.io.out.toAiq0.bypass_data.LCH_RDY_AIQ0... to where
  aiq0.io.in.data.bypassData.inst := isstage.io.out.toAiq0.bypass_data.OPCODE//isstage.io.out.toAiq0.bypass_data //////todo: make it same
  aiq0.io.in.data.bypassData.div := isstage.io.out.toAiq0.bypass_data.DIV
  aiq0.io.in.data.bypassData.iid := isstage.io.out.toAiq0.bypass_data.IID
  aiq0.io.in.data.bypassData.aluShort := isstage.io.out.toAiq0.bypass_data.ALU_SHORT
  aiq0.io.in.data.bypassData.dstPreg := isstage.io.out.toAiq0.bypass_data.DST_PREG
  aiq0.io.in.data.bypassData.dstValid := isstage.io.out.toAiq0.bypass_data.DST_VLD
  aiq0.io.in.data.bypassData.dstVreg := isstage.io.out.toAiq0.bypass_data.DST_VREG
  aiq0.io.in.data.bypassData.dstVValid := isstage.io.out.toAiq0.bypass_data.DSTV_VLD
  aiq0.io.in.data.bypassData.exceptVec.valid := isstage.io.out.toAiq0.bypass_data.EXPT_VLD
  aiq0.io.in.data.bypassData.exceptVec.bits := isstage.io.out.toAiq0.bypass_data.EXPT_VEC
  aiq0.io.in.data.bypassData.highHwExcept := isstage.io.out.toAiq0.bypass_data.HIGH_HW_EXPT
  aiq0.io.in.data.bypassData.launchPreg := isstage.io.out.toAiq0.bypass_data.LCH_PREG
  aiq0.io.in.data.bypassData.mtvr := isstage.io.out.toAiq0.bypass_data.MTVR
  aiq0.io.in.data.bypassData.pcFifo := isstage.io.out.toAiq0.bypass_data.PCFIFO
  aiq0.io.in.data.bypassData.pid := isstage.io.out.toAiq0.bypass_data.PID
  aiq0.io.in.data.bypassData.special := isstage.io.out.toAiq0.bypass_data.SPECIAL
  aiq0.io.in.data.bypassData.srcValid := isstage.io.out.toAiq0.bypass_data.src_vld
  for (j <- 0 to NumSrcArith-1) {
    aiq0.io.in.data.bypassData.srcVec(j).preg := isstage.io.out.toAiq0.bypass_data.src_info(j).src_data.preg
    aiq0.io.in.data.bypassData.srcVec(j).wb := isstage.io.out.toAiq0.bypass_data.src_info(j).src_data.wb
    aiq0.io.in.data.bypassData.srcVec(j).ready := isstage.io.out.toAiq0.bypass_data.src_info(j).src_data.rdy
    aiq0.io.in.data.bypassData.srcVec(j).lsuMatch := isstage.io.out.toAiq0.bypass_data.src_info(j).lsu_match
  }
  aiq0.io.in.data.bypassData.vl := isstage.io.out.toAiq0.bypass_data.VL
  aiq0.io.in.data.bypassData.vlmul := isstage.io.out.toAiq0.bypass_data.VLMUL
  aiq0.io.in.data.bypassData.vsew := isstage.io.out.toAiq0.bypass_data.VSEW
  aiq0.io.in.data.createDiv := isstage.io.out.toAiq0.create_div
  aiq0.io.in.data.srcReadyForBypass := isstage.io.out.toAiq0.src_rdy_for_bypass
  aiq0.io.in.data.rfReadyClr := rfstage.io.data.out.toAiq0.readyClr.asTypeOf(aiq0.io.in.data.rfReadyClr)
  aiq0.io.in.data.rfLaunchEntry := rfstage.io.data.out.toAiq0.launchEntryOH.asTypeOf(aiq0.io.in.data.rfLaunchEntry)
  aiq0.io.in.data.dispatchInstSrcPregVec := isstage.io.out.toAiq.inst_src_preg
  aiq0.io.in.data.sdiqCreateSrcSelVec := isstage.io.out.toAiq.sdiq_create_src_sel
  aiq0.io.in.fromLsu := io.in.fromLSU.IQfromLSU
  aiq0.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  aiq0.io.in.fromCp0.icgEn := io.in.fromCp0.icgEn
  aiq0.io.in.fromCp0.iqBypassDisable := io.in.IQfromCp0sub.iq_bypass_disable
  aiq0.io.in.fromIu := io.in.aiq0fromIU
  aiq0.io.in.aiqEntryCreateVec := DontCare //////todo: add aiq1
  aiq0.io.in.biqEntryCreateVec := biq.io.out.entryEnqOHVec
  aiq0.io.in.fromRtu.flushFe := io.in.fromRTU.flush_fe
  aiq0.io.in.fromRtu.flushIs := io.in.fromRTU.flush_is
  aiq0.io.in.fromRtu.yyXXFlush := io.in.fromRTU.yy_xx_flush
  aiq0.io.in.fuResultDstPreg := DontCare ////todo: complete rf: rf.io.data.out.dp_xx_rf_pipe0_dst_preg_dup0
  aiq0.io.in.wbPreg := io.in.IQfromIUsub.wbPreg//iu_idu_ex2_pipe0_wb_preg_vld_dupx
  aiq0.io.in.loadPreg := io.in.fromLSU.aiq0fromLSUsub.loadPreg

  // &ConnRule(s/_dupx/_dup3/); @56
  // &Instance("ct_idu_is_aiq1", "x_ct_idu_is_aiq1"); @57
  //todo: add aiq1

  // &ConnRule(s/_dupx/_dup4/); @58
  // &Instance("ct_idu_is_biq", "x_ct_idu_is_biq"); @59

  biq.io.in.fuResultDstPreg := DontCare
  biq.io.in.wbPreg := io.in.IQfromIUsub.wbPreg
  biq.io.in.loadPreg := io.in.fromLSU.aiq0fromLSUsub.loadPreg
  biq.io.in.fromRtu.flushFe := io.in.fromRTU.flush_fe
  biq.io.in.fromRtu.flushIs := io.in.fromRTU.flush_is
  biq.io.in.fromRtu.yyXXFlush := io.in.fromRTU.yy_xx_flush
  biq.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  biq.io.in.fromCp0.icgEn := io.in.fromCp0.icgEn
  biq.io.in.fromCp0.iqBypassDisable := io.in.IQfromCp0sub.iq_bypass_disable
  biq.io.in.fromIu.div := io.in.biqfromIU.div
  biq.io.in.fromLsu := io.in.fromLSU.IQfromLSU
  biq.io.in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn
  biq.io.in.ctrl.gateClkEn(0) := isstage.io.out.iqCreateEn(2)(0).gateclk_en
  biq.io.in.ctrl.gateClkEn(1) := isstage.io.out.iqCreateEn(2)(1).gateclk_en
  biq.io.in.ctrl.createDpEn(0) := isstage.io.out.iqCreateEn(2)(0).dp_en
  biq.io.in.ctrl.createDpEn(1) := isstage.io.out.iqCreateEn(2)(1).dp_en
  biq.io.in.ctrl.createEn(0) := isstage.io.out.iqCreateEn(2)(0).en
  biq.io.in.ctrl.createEn(1) := isstage.io.out.iqCreateEn(2)(1).en
  biq.io.in.ctrl.rfPopValid := rfstage.io.ctrl.out.toIq(2).popValid
  biq.io.in.ctrl.rfAluRegFwdValid := DontCare//////todo: add rfstage.io.ctrl.out.alu_reg_fwd_vld
  biq.io.in.ctrl.rfLaunchFailValid := rfstage.io.ctrl.out.toIq(2).launchFailValid
  biq.io.in.data.createData := isstage.io.out.toBiq.create_data.asTypeOf(biq.io.in.data.createData)
  biq.io.in.data.bypassData := isstage.io.out.toBiq.bypass_data.asTypeOf(biq.io.in.data.bypassData)
  biq.io.in.data.rfReadyClr := rfstage.io.data.out.toBiq.readyClr.asTypeOf(biq.io.in.data.rfReadyClr)
  biq.io.in.data.rfLaunchEntry := rfstage.io.data.out.toBiq.launchEntryOH.asTypeOf(biq.io.in.data.rfLaunchEntry) //////todo: check it
  biq.io.in.data.srcReadyForBypass := isstage.io.out.toBiq.src_rdy_for_bypass

  // &ConnRule(s/_dupx/_dup1/); @60
  // &Instance("ct_idu_is_lsiq", "x_ct_idu_is_lsiq"); @61

  lsiq.io.in := DontCare ////todo: connect lsiq

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

  rfstage.io.ctrl.in.fromRtu.flush.fe := io.in.fromRTU.flush_fe
  rfstage.io.ctrl.in.fromRtu.flush.is := io.in.fromRTU.flush_is
  rfstage.io.ctrl.in.fromRtu.yyXxFlush := io.in.fromRTU.yy_xx_flush
  rfstage.io.ctrl.in.fromIu.stall := io.in.RFfromIU.stall
  rfstage.io.ctrl.in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn
  rfstage.io.ctrl.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  rfstage.io.ctrl.in.fromCp0.iduIcgEn := io.in.fromCp0.icgEn //////todo: check it
  rfstage.io.ctrl.in.fromCp0.lsuTlbBroadDis := io.in.RFfromCp0sub.lsuTlbBroadDis
  rfstage.io.ctrl.in.fromCp0.lsuFenceIBroadDis := io.in.RFfromCp0sub.lsuFenceIBroadDis
  rfstage.io.ctrl.in.fromCp0.lsuFenceRwBroadDis := io.in.RFfromCp0sub.lsuFenceRwBroadDis
  rfstage.io.ctrl.in.issueEnVec(0).gateClkIssueEn := aiq0.io.out.xxGateClkIssueEn
  rfstage.io.ctrl.in.issueEnVec(0).issueEn := aiq0.io.out.xxIssueEn
  rfstage.io.ctrl.in.issueEnVec(1) := DontCare //////todo: add aiq1
  rfstage.io.ctrl.in.issueEnVec(2).gateClkIssueEn := biq.io.out.xxGateClkIssueEn
  rfstage.io.ctrl.in.issueEnVec(2).issueEn := biq.io.out.xxIssueEn
  rfstage.io.ctrl.in.issueEnVec(3).gateClkIssueEn := lsiq.io.out.gateClkIssueEn
  rfstage.io.ctrl.in.issueEnVec(3).issueEn := lsiq.io.out.issueEnVec(0) //////todo: is rfstage.io.ctrl.in.issueEnVec(3) wrong?
  rfstage.io.ctrl.in.issueEnVec(4) := DontCare //////todo: add lsiq
  rfstage.io.ctrl.in.issueEnVec(5) := DontCare //////todo: add sdiq
  rfstage.io.ctrl.in.issueEnVec(6) := DontCare //////todo: add viq0
  rfstage.io.ctrl.in.issueEnVec(7) := DontCare //////todo: add viq1
  rfstage.io.ctrl.in.fromVfpu := io.in.RFfromVFPU
  rfstage.io.data.in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn //////todo: check it
  rfstage.io.data.in.fromHad.iduWbBrValid := io.in.RFfromHad.iduWbBrValid
  rfstage.io.data.in.fromHad.iduWbBrData := io.in.RFfromHad.iduWbBrData
  rfstage.io.data.in.aiq0.issueEn := aiq0.io.out.xxIssueEn //////todo: check it
  rfstage.io.data.in.aiq0.issueGateClkEn := aiq0.io.out.xxGateClkIssueEn //////todo: check it
  rfstage.io.data.in.aiq0.issueReadData := aiq0.io.out.data.issueData
  rfstage.io.data.in.aiq0.issueEntryOH := aiq0.io.out.data.issueEntryVec.asUInt//.asTypeOf(rfstage.io.data.in.aiq0.issueEntryOH)
  rfstage.io.data.in.aiq1 := DontCare
  rfstage.io.data.in.biq.issueEn := biq.io.out.xxIssueEn
  rfstage.io.data.in.biq.issueGateClkEn := biq.io.out.xxGateClkIssueEn
  rfstage.io.data.in.biq.issueReadData := biq.io.out.data.issueReadData
  rfstage.io.data.in.biq.issueEntryOH := biq.io.out.data.issueEntryVec.asTypeOf(rfstage.io.data.in.biq.issueEntryOH)
  rfstage.io.data.in.lsiq0.issueEn := lsiq.io.out.issueEnVec(0) //////todo: check it
  rfstage.io.data.in.lsiq0.issueGateClkEn := lsiq.io.out.gateClkIssueEn
  rfstage.io.data.in.lsiq0.issueReadData := lsiq.io.out.data.issueReadData(0) //////todo: check it
  rfstage.io.data.in.lsiq0.issueEntryOH := lsiq.io.out.data.issueEntryVec(0).asTypeOf(rfstage.io.data.in.lsiq0.issueEntryOH) //////todo: check it
  rfstage.io.data.in.lsiq1.issueEn := lsiq.io.out.issueEnVec(1)
  rfstage.io.data.in.lsiq1.issueGateClkEn := lsiq.io.out.gateClkIssueEn
  rfstage.io.data.in.lsiq1.issueReadData := lsiq.io.out.data.issueReadData(1)
  rfstage.io.data.in.lsiq1.issueEntryOH := lsiq.io.out.data.issueEntryVec(1).asTypeOf(rfstage.io.data.in.lsiq1.issueEntryOH)
  rfstage.io.data.in.sdiq := DontCare //////todo: add sdiq
  rfstage.io.data.in.vfiq0 := DontCare
  rfstage.io.data.in.vfiq1 := DontCare
  rfstage.io.data.r <> prf.io.r//DontCare
  rfstage.io.ctrl.in.fromHpcp.iduCntEn := io.in.fromHpcp.cntEn
  // &Instance("ct_idu_rf_dp", "x_ct_idu_rf_dp"); @81

  // &Instance("ct_idu_rf_fwd", "x_ct_idu_rf_fwd"); @82
  //todo: add rf_fwd

  // &Instance("ct_idu_rf_prf_pregfile", "x_ct_idu_rf_prf_pregfile"); @83
  prf.io.in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
  prf.io.in.fromCp0.iduIcgEn := io.in.fromCp0.icgEn
  prf.io.in.fromPad.yyIcgScanEn := io.in.fromPad.yyIcgScanEn
  prf.io.in.fromRtu.yyXxDebugOn := io.in.PRFfromRTUsub.yyXxDebugOn
  io.out.PRFtoHad := prf.io.out.toHad
  prf.io.w(0).data := io.in.PRFfromIU.ex2_pipe0_wb_preg_data
  prf.io.w(0).en := io.in.PRFfromIU.ex2_pipe0_wb_preg_vld
  prf.io.w(0).preg := io.in.PRFfromIU.ex2_pipe0_wb_preg//from IU
  prf.io.w(1).data := io.in.PRFfromIU.ex2_pipe1_wb_preg_data
  prf.io.w(1).en := io.in.PRFfromIU.ex2_pipe1_wb_preg_vld
  prf.io.w(1).preg := io.in.PRFfromIU.ex2_pipe1_wb_preg
  prf.io.w(2).data := io.in.PRFfromIU.lsu_wb_pipe3_wb_preg_data
  prf.io.w(2).en := io.in.PRFfromIU.lsu_wb_pipe3_wb_preg_vld
  prf.io.w(2).preg := io.in.PRFfromIU.lsu_wb_pipe3_wb_preg
  //  for(i <- 0 to NumPregReadPort) {
  //    prf.io.r(i).preg := rfstage.io.data.r(i).preg
  //  }
  //  prf.io.r.zipWithIndex.foreach {
  //    case (rbundle, i) =>
  //      if (i > 2)
  //        rbundle.preg := DontCare
  //  }

  io.out.IDtoIFU := idstage.io.out.toIFU
  io.out.IDtoHad := idstage.io.out.toHad
  io.out.IDtoHpcp := idstage.io.out.toHpcp
  io.out.IStoIU := isstage.io.out.toIU
  io.out.IStoLSU := isstage.io.out.toLSU
  io.out.IStoHad := isstage.io.out.toHad
  io.out.IStoRTU := isstage.io.out.toRTU
  io.out.IRtoRTU := irstage.io.out.toRTU
  io.out.IRtoHpcp := irstage.io.out.toHpcp
  //////io.out.RFtoCp0 := rfstage.io.ctrl.out.toCp0
  //////io.out.RFtoHpcp := rfstage.io.ctrl.out.toHpcp
  io.out.RFCtrl := rfstage.io.ctrl.out
  io.out.RFData := rfstage.io.data.out.toIu
}
