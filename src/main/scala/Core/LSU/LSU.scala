package Core.LSU

import Core.IUConfig.MPPWidth
import Core.LSU.CacheRelated._
import Core.LSU.LoadExStage._
import Core.LSU.StoreExStage._
import Core.LSU.DCache._
import Core.LSU.Lfb._
import Core.LSU.Lq._
import Core.LSU.Rb._
import Core.LSU.Sq._
import Core.LSU.Vb._
import Core.LSU.Wmb._
import Core.LSU._
import Core.LsuConfig
import Core.LsuConfig.{CACHE_DIST_SELECT, LSIQ_ENTRY}
import Core.ROBConfig.{IidWidth, NumCommitEntry}
import chisel3._
import chisel3.util._
import firrtl.backends.experimental.smt.BVNot

class LSUInput extends Bundle {
  val fromCp0 = new Bundle{
    val lsu_cb_aclr_dis = Bool()
    val lsu_da_fwd_dis = Bool()
    val lsu_dcache_en = Bool()
    val lsu_icg_en = Bool()
    val lsu_mm = Bool()
    val yy_clk_en = Bool()
    val amr   = Bool()
    val amr2  = Bool()
    val lsu_wr_burst_dis = Bool()
    val lsu_no_op_req = Bool()
    val lsu_tvm         = Bool()
    val lsu_ucme        = Bool()
    val lsu_wa          = Bool()
    val yy_priv_mode    = UInt(MPPWidth.W)
    val yy_virtual_mode = Bool()
    val lsu_corr_dis = Bool()
    val lsu_l2_pref_en = Bool()
    val lsu_nsfe = Bool()
    val yy_dcache_pref_en = Bool()
    val lsu_dcache_clr = Bool()
    val lsu_dcache_inv       = Bool()
    val lsu_dcache_read_index = UInt(17.W)
    val lsu_dcache_read_ld_tag = Bool()
    val lsu_dcache_read_req   = Bool()
    val lsu_dcache_read_st_tag = Bool()
    val lsu_dcache_read_way   = Bool()
    val lsu_dcache_pref_dist = UInt(CACHE_DIST_SELECT.W)
    val lsu_l2_pref_dist     = UInt(CACHE_DIST_SELECT.W)
  }

  val fromRTU = new Bundle{
    val yy_xx_commit = Vec(3, Bool())
    val yy_xx_commit_iid = Vec(3, UInt(7.W))
    val yy_xx_flush = Bool()
    val lsu_async_flush = Bool()
    val commitIidUpdata = Vec(NumCommitEntry, UInt(IidWidth.W))
  }
  val fromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val fromMMU = new Bundle{
    val ld_ag = new Bundle{
      val buf0 = Bool()
      val ca0 = Bool()
      val pa0 = UInt(28.W)
      val pa0_vld = Bool()
      val page_fault0 = Bool()
      val sec0 = Bool()
      val sh0 = Bool()
      val so0 = Bool()
      val stall0 = Bool()
    }
    val st_ag = new MmuToStAg
    val data_req_size = Bool()
    val mmu_en = Bool()
    val tlb_busy = Bool()
    val mmu_lsu_access_fault0 = Bool()
    val mmu_lsu_access_fault1 = Bool()
    val mmu_lsu_tlb_wakeup = UInt(LSIQ_ENTRY.W)
  }
  val ld_ag = new Bundle{
    val rf_pipe3 = new Pipe3In
  }
  val st_ag = new Bundle{
    val rf_pipe4     = new RfPipe4
  }
  val sd_ex1 = new Bundle{
    val rf_pipe5 = new RfPipe5ToStEx1
  }
  val ld_dc = new Bundle{
    val fromHad = new Bundle{
      val yy_xx_bkpta_base = UInt(40.W)
      val yy_xx_bkpta_mask = UInt(8.W)
      val yy_xx_bkpta_rc   = Bool()
      val yy_xx_bkptb_base = UInt(40.W)
      val yy_xx_bkptb_mask = UInt(8.W)
      val yy_xx_bkptb_rc   = Bool()
    }
  }

  val rb = new Bundle{
    val fromBiu = new Bundle{
      val b_id   = UInt(5.W)
      val b_vld  = Bool()
      val r_data = UInt(128.W)
      val r_id   = UInt(5.W)
      val r_resp = UInt(4.W)
      val r_vld  = Bool()
      val r_last = Bool()
    }
  }
  val wmb = new Bundle{
    val fromBiu = new Bundle{
      val b_resp = UInt(2.W)
    }
  }
  val ld_wb = new Bundle{
    val fromHad = new Bundle{
      val bus_trace_en = Bool()
      val dbg_en = Bool()
    }
  }

  val ctrl = new Bundle{
    val rfPipeIn = new IduRfSelToCtrl
    val idu_lsu_vmb_create_gateclk_enVec = Vec(2, Bool())
  }
  val bus_arb = new Bundle{
    val fromBiu = new Bundle{
      val ar_ready    = Bool()
      val aw_vb_grnt  = Bool()
      val aw_wmb_grnt = Bool()
      val w_vb_grnt   = Bool()
      val w_wmb_grnt  = Bool()
    }
  }
}

class LSUOutput extends Bundle with LsuConfig{
  val ctrl = new Bundle{
    val toIDU = new CtrltoIdu
  }
  val ld_ag = new Bundle{
    val toHpcp = new Bundle{
      val cross_4k_stall = Bool()
      val other_stall = Bool()
    }
    val toIDU = new Bundle{
      val pipe3_load_inst_vld = Bool()
      val pipe3_preg_dup = Vec(5, UInt(7.W))
      val pipe3_vload_inst_vld = Bool()
      val pipe3_vreg_dup = Vec(4, UInt(7.W))
      val ag_wait_old = Vec(LSIQ_ENTRY, Bool())
      val ag_wait_old_gateclk_en = Bool()
    }
    val toMMU = new Bundle{
      val abort0 = Bool()
      val id0 = UInt(7.W)
      val st_inst0 = Bool()
      val va0 = UInt(64.W)
      val va0_vld = Bool()
    }
  }
  val st_ag = new Bundle{
    val toIdu       = new StAgToIdu
    val toMmu       = new StAgToMmu
    val rfVld       = Bool() // all done
  }

  val st_ex1 = new Bundle{
    val toIdu = new StExtoIdu
  }

  //todo: before loaddc
  val ld_dc = new Bundle{
    val toIDU = new Bundle{
      val pipe3_load_fwd_inst_vld_dup = Vec(5, Bool())
      val pipe3_load_inst_vld_dup = Vec(5, Bool())
      val pipe3_preg_dup = Vec(5, UInt(7.W))
      val pipe3_vload_fwd_inst_vld = Bool()
      val pipe3_vload_inst_vld_dup = Vec(4, Bool())
      val pipe3_vreg_dup = Vec(4, UInt(7.W))
    }
    val lsu_mmu_vabuf0 = UInt(28.W)
  }
  val st_dc = new Bundle{
    val toIdu  = new StDcToIdu
    val toMmu  = new StDcToMmu
  }
  val lq = new LQOutput
  val sq = new Bundle{
    val toIDU = new Bundle{
      val SqNotFull = Bool()
    }
    val toRTU = new Bundle{
      val AllCommitDataVld = Bool()
    }

  }
  val ld_da = new Bundle{
    val toIDU = new Bundle{
      val da_pipe3_fwd_preg = UInt(7.W)
      val da_pipe3_fwd_preg_data = UInt(64.W)
      val da_pipe3_fwd_preg_vld = Bool()
      val da_pipe3_fwd_vreg = UInt(7.W)
      val da_pipe3_fwd_vreg_fr_data = UInt(64.W)
      val da_pipe3_fwd_vreg_vld = Bool()
      val da_pipe3_fwd_vreg_vr0_data = UInt(64.W)
      val da_pipe3_fwd_vreg_vr1_data = UInt(64.W)
      val ld_da_wait_old = UInt(12.W)
      val ld_da_wait_old_gateclk_en = Bool()
    }
    val toRTU = new Bundle{
      val da_pipe3_split_spec_fail_iid = UInt(7.W)
      val da_pipe3_split_spec_fail_vld = Bool()
    }
  }
  val st_da = new Bundle{
    val toRTU = new StDaToRtu
  }
  val rb = new Bundle{
    val toIDU = new Bundle{
      val lsu_idu_no_fence = Bool()
      val lsu_idu_rb_not_full = Bool()
    }
    val toRTU = new Bundle{
      val lsu_rtu_all_commit_ld_data_vld = Bool()
    }
  }
  val ld_wb = new Bundle{
    val toHad = (new LoadWBOutput).toHad
    val toIDU = (new LoadWBOutput).toIDU
    val toRTU = (new LoadWBOutput).toRTU
  }
  val st_wb = new Bundle{
    val toRTU = new StWbToRtu
  }
  val bus_arb = new Bundle{
    val toBiu = new Bundle{
      val ar = new biu_ar
      val lsu_biu_ar_req_gate = Bool()

      val lsu_biu_aw_req_gate = Bool()
      val st_aw = new biu_aw
      val lsu_biu_aw_st_unique = Bool()
      val vict_aw = new biu_aw
      val lsu_biu_aw_vict_unique = Bool()

      val st_w = new biu_w
      val vict_w = new biu_w
    }
  }
}

class LSUIO extends Bundle with LsuConfig{
  val in  = Input(new LSUInput)
  val out = Output(new LSUOutput)
}

class LSU extends Module {
  val io = IO(new LSUIO)
  //rotData has been used in LoadDA

  //==========================================================
  //                    AG/EX1 Stage
  //==========================================================
  // &Instance("ct_lsu_ld_ag","x_ct_lsu_ld_ag"); @85
  val loadag = Module(new LoadAG)

  //&Instance("ct_lsu_cmit_monitor","x_ct_lsu_cmit_monitor");
  // &Instance("ct_lsu_st_ag","x_ct_lsu_st_ag"); @87
  val storeag = Module(new StoreAg)

  // &Instance("ct_lsu_sd_ex1","x_ct_lsu_sd_ex1"); @88
  val storeex1 = Module(new StoreEx1)

  // &Instance("ct_lsu_mcic","x_ct_lsu_mcic"); @90
  //val mcic = Module(new ) todo: complete mcic

  // &Instance("ct_lsu_dcache_arb","x_ct_lsu_dcache_arb"); @91
  val dcachearb = Module(new DCacheArb)

  //==========================================================
  //                       DC Stage
  //==========================================================
  // &Instance("ct_lsu_dcache_top","x_ct_lsu_dcache_top"); @95
  val dcachetop = Module(new DCacheTop)

  // &Instance("ct_lsu_ld_dc","x_ct_lsu_ld_dc"); @96
  val loaddc = Module(new LoadDC)

  // &Instance("ct_lsu_st_dc","x_ct_lsu_st_dc"); @97
  val storedc = Module(new StoreDc)

  // &Instance("ct_lsu_lq","x_ct_lsu_lq"); @99
  val lq = Module(new LQ)

  // &Instance("ct_lsu_sq","x_ct_lsu_sq"); @100
  val sq = Module(new Sq)

  //==========================================================
  //                       DA Stage
  //==========================================================
  // &Instance("ct_lsu_ld_da","x_ct_lsu_ld_da"); @104
  val loadda = Module(new LoadDA)

  // &Instance("ct_lsu_st_da","x_ct_lsu_st_da"); @105
  val storeda = Module(new StoreDa)

  // &Instance("ct_lsu_rb","x_ct_lsu_rb"); @107
  val rb = Module(new Rb)

  // &Instance("ct_lsu_wmb","x_ct_lsu_wmb"); @108
  val wmb = Module(new Wmb)

  // &Instance("ct_lsu_wmb_ce","x_ct_lsu_wmb_ce"); @109
  val wmbce = Module(new WmbCe)


  //==========================================================
  //                       WB Stage
  //==========================================================
  // &Instance("ct_lsu_ld_wb","x_ct_lsu_ld_wb"); @114
  val loadwb = Module(new LoadWB)

  // &Instance("ct_lsu_st_wb","x_ct_lsu_st_wb"); @115
  val storewb = Module(new StoreWb)


  //==========================================================
  //                Linefill/Victim buffer
  //==========================================================
  // &Instance("ct_lsu_lfb","x_ct_lsu_lfb"); @120
  val lfb = Module(new Lfb)

  // &Instance("ct_lsu_vb","x_ct_lsu_vb"); @121
  val vb = Module(new Vb)

  // &Instance("ct_lsu_vb_sdb_data","x_ct_lsu_vb_sdb_data"); @122
  //val sdbdata = Module(new ) todo: complete sdbdata

  //==========================================================
  //                      Snoop
  //==========================================================
  // &Instance("ct_lsu_snoop_req_arbiter","x_ct_lsu_snoop_req_arbiter"); @127
  //val snoop_req_arbiter todo: complete snoop_req_arbiter

  // &Instance("ct_lsu_snoop_resp","x_ct_lsu_snoop_resp"); @128
  //val snoop_resp todo: complete snoop_resp

  // &Instance("ct_lsu_snoop_ctcq","x_ct_lsu_snoop_ctcq"); @129
  //val snoop_ctcq todo: complete snoop_ctcq

  // &Instance("ct_lsu_snoop_snq","x_ct_lsu_snoop_snq"); @131
  //val snoop_snq todo: complete snoop_snq

  // &Instance("ct_lsu_snoop_dummy","x_ct_lsu_snoop_dummy"); @133

  //==========================================================
  //                    Other modules
  //==========================================================
  // &Instance("ct_lsu_lm","x_ct_lsu_lm"); @139
  //val lm = Module(new ) todo: complete lsu_lm

  // &Instance("ct_lsu_amr","x_ct_lsu_amr"); @140
  val amr = Module(new Amr)

  // &Instance("ct_lsu_icc","x_ct_lsu_icc"); @141
  val icc = Module(new Icc)

  // &Instance("ct_lsu_ctrl","x_ct_lsu_ctrl"); @142
  val ctrl = Module(new Ctrl)

  // &Instance("ct_lsu_bus_arb","x_ct_lsu_bus_arb"); @143
  val busarb = Module(new BusArb)

  // &Instance("ct_lsu_pfu","x_ct_lsu_pfu"); @144
  //val pfu = todo: complete pfu

  // &Instance("ct_lsu_cache_buffer","x_ct_lsu_cache_buffer"); @145
  //val cache_buffer todo: not need? check it

  // &Instance("ct_lsu_spec_fail_predict","x_ct_lsu_spec_fail_predict"); @146
  //val spec_fail_predict todo: complete

  //==========================================================
  //                    VECTOR
  //==========================================================
  // &Instance("ct_lsu_vmb","x_ct_lsu_vmb"); @152


  //for ecc info
  // &Instance("ct_lsu_ecc_info","x_ct_lsu_ecc_info"); @160
  //&Instance("ct_lsu_snq_assert","x_ct_lsu_snq_assert");


  //dcache icc assertion


  //==========================================================
  //                    AG/EX1 Stage
  //==========================================================
  // &Instance("ct_lsu_ld_ag","x_ct_lsu_ld_ag"); @85
  loadag.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  loadag.io.in.fromRTU.yy_xx_commit := io.in.fromRTU.yy_xx_commit
  loadag.io.in.fromRTU.yy_xx_commit_iid := io.in.fromRTU.yy_xx_commit_iid
  loadag.io.in.fromPad := io.in.fromPad
  loadag.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  loadag.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  loadag.io.in.fromCp0.lsu_dcache_en := io.in.fromCp0.lsu_dcache_en
  loadag.io.in.fromCp0.lsu_da_fwd_dis := io.in.fromCp0.lsu_da_fwd_dis
  loadag.io.in.fromCp0.lsu_cb_aclr_dis := io.in.fromCp0.lsu_cb_aclr_dis
  loadag.io.in.fromCp0.lsu_mm := io.in.fromCp0.lsu_mm
  loadag.io.in.fromMMU := io.in.fromMMU.ld_ag
  loadag.io.in.fromDcacheArb.ag_ld_sel := dcachearb.io.out.dcache_arb_ag_ld_sel
  loadag.io.in.fromDcacheArb.ld_ag_addr := dcachearb.io.out.dcache_arb_ld_ag_addr
  loadag.io.in.fromDcacheArb.ld_ag_borrow_addr_vld := dcachearb.io.out.dcache_arb_ld_ag_borrow_addr_vld
  loadag.io.in.ctrl_ld_clk := ctrl.io.out.ctrlClk.ldClk
  loadag.io.in.st_ag_iid := storeag.io.out.toDc.iid
  loadag.io.in.pipe3 := io.in.ld_ag.rf_pipe3
  io.out.ld_ag.toMMU := loadag.io.out.toMMU
  io.out.ld_ag.toIDU := loadag.io.out.toIDU
  io.out.ld_ag.toHpcp := loadag.io.out.toHpcp

  //&Instance("ct_lsu_cmit_monitor","x_ct_lsu_cmit_monitor");
  // &Instance("ct_lsu_st_ag","x_ct_lsu_st_ag"); @87
  storeag.io.in.rfIn := io.in.st_ag.rf_pipe4
  storeag.io.in.cp0In.tvm := io.in.fromCp0.lsu_tvm
  storeag.io.in.cp0In.ucme := io.in.fromCp0.lsu_ucme
  storeag.io.in.cp0In.clkEn := io.in.fromCp0.yy_clk_en
  storeag.io.in.cp0In.icgEn := io.in.fromCp0.lsu_icg_en
  storeag.io.in.cp0In.dcacheEn := io.in.fromCp0.lsu_dcache_en
  storeag.io.in.cp0In.mm := io.in.fromCp0.lsu_mm
  storeag.io.in.cp0In.wa := io.in.fromCp0.lsu_wa
  storeag.io.in.cp0In.privMode := io.in.fromCp0.yy_priv_mode
  storeag.io.in.cp0In.virtualMode := io.in.fromCp0.yy_virtual_mode
  storeag.io.in.rtuIn.iid := io.in.fromRTU.yy_xx_commit_iid
  storeag.io.in.rtuIn.commit := io.in.fromRTU.yy_xx_commit
  storeag.io.in.rtuIn.flush := io.in.fromRTU.yy_xx_flush
  storeag.io.in.mmuIn := io.in.fromMMU.st_ag
  storeag.io.in.dcacheIn.sel := dcachearb.io.out.dcache_arb_ag_st_sel
  storeag.io.in.dcacheIn.addr := dcachearb.io.out.dcache_arb_st_ag_addr
  storeag.io.in.dcacheIn.borrowAddrVld := dcachearb.io.out.dcache_arb_st_ag_borrow_addr_vld
  storeag.io.in.lmIn := 0.U.asTypeOf(storeag.io.in.lmIn) //////todo: complete lm
  io.out.st_ag.toMmu := storeag.io.out.toMmu
  io.out.st_ag.toIdu := storeag.io.out.toIdu
  io.out.st_ag.rfVld := storeag.io.out.rfVld

  // &Instance("ct_lsu_sd_ex1","x_ct_lsu_sd_ex1"); @88
  storeex1.io.in.cp0In.clkEn := io.in.fromCp0.yy_clk_en
  storeex1.io.in.cp0In.lsuIcgEn := io.in.fromCp0.lsu_icg_en
  storeex1.io.in.iduIn := io.in.sd_ex1.rf_pipe5
  storeex1.io.in.rtuFlush := io.in.fromRTU.yy_xx_flush

  io.out.st_ex1.toIdu := storeex1.io.out.toIdu

  // &Instance("ct_lsu_mcic","x_ct_lsu_mcic"); @90
  //mcic //todo: complete mcic

  // &Instance("ct_lsu_dcache_arb","x_ct_lsu_dcache_arb"); @91
  dcachearb.io.in := DontCare
  dcachearb.io.in.cp0_lsu_icg_en := io.in.fromCp0.lsu_icg_en
  dcachearb.io.in.pad_yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  dcachearb.io.in.vb_rcl_sm_data_id := vb.io.out.toSdb.rclSmDataId
  dcachearb.io.in.fromIcc.data_way := icc.io.out.toArb.dataWay
  dcachearb.io.in.fromIcc.way := icc.io.out.toArb.way
  dcachearb.io.in.fromIcc.ld_req := icc.io.out.toArb.ld.req
  dcachearb.io.in.fromIcc.ld_borrow_req := icc.io.out.toArb.ld.borrowReq
  dcachearb.io.in.fromIcc.ld_tag_req := icc.io.out.toArb.ld.tagReq
  dcachearb.io.in.fromIcc.ld_tag_read := icc.io.out.toArb.ld.tagRead
  dcachearb.io.in.fromIcc.ld_data_gateclk_en := icc.io.out.toArb.ld.dataGateclkEn
  dcachearb.io.in.fromIcc.ld_data_high_idx := icc.io.out.toArb.ld.dataHighIdx
  dcachearb.io.in.fromIcc.ld_data_low_idx := icc.io.out.toArb.ld.dataLowIdx
  dcachearb.io.in.fromIcc.ld_data_req := icc.io.out.toArb.ld.dataReq
  dcachearb.io.in.fromIcc.ld_tag_gateclk_en := icc.io.out.toArb.ld.tagGateclkEn
  dcachearb.io.in.fromIcc.ld_tag_idx := icc.io.out.toArb.ld.tagIdx
  dcachearb.io.in.fromIcc.st_borrow_req := icc.io.out.toArb.st.borrowReq
  dcachearb.io.in.fromIcc.st_dirty_din := icc.io.out.toArb.st.dirtyDin
  dcachearb.io.in.fromIcc.st_dirty_gateclk_en := icc.io.out.toArb.st.dirtyGateclkEn
  dcachearb.io.in.fromIcc.st_dirty_gwen := icc.io.out.toArb.st.dirtyGwen
  dcachearb.io.in.fromIcc.st_dirty_idx := icc.io.out.toArb.st.dirtyIdx
  dcachearb.io.in.fromIcc.st_dirty_req := icc.io.out.toArb.st.dirtyReq
  dcachearb.io.in.fromIcc.st_dirty_wen := icc.io.out.toArb.st.dirtyWen
  dcachearb.io.in.fromIcc.st_req := icc.io.out.toArb.st.req
  dcachearb.io.in.fromIcc.st_tag_gateclk_en := icc.io.out.toArb.st.tagGateclkEn
  dcachearb.io.in.fromIcc.st_tag_idx := icc.io.out.toArb.st.tagIdx
  dcachearb.io.in.fromIcc.st_tag_req := icc.io.out.toArb.st.tagReq
  dcachearb.io.in.fromIcc.way := icc.io.out.toArb.way
  dcachearb.io.in.fromLfb.ld_data_gateclk_en := lfb.io.out.toArb.ld_data_gateclk_en
  dcachearb.io.in.fromLfb.ld_data_high_din := lfb.io.out.toArb.ld_data_high_din
  dcachearb.io.in.fromLfb.ld_data_idx := lfb.io.out.toArb.ld_data_idx
  dcachearb.io.in.fromLfb.ld_data_low_din := lfb.io.out.toArb.ld_data_low_din
  dcachearb.io.in.fromLfb.ld_req := lfb.io.out.toArb.ld_req
  dcachearb.io.in.fromLfb.ld_tag_din := lfb.io.out.toArb.ld_tag_din
  dcachearb.io.in.fromLfb.ld_tag_gateclk_en := lfb.io.out.toArb.ld_tag_gateclk_en
  dcachearb.io.in.fromLfb.ld_tag_idx := lfb.io.out.toArb.ld_tag_idx
  dcachearb.io.in.fromLfb.ld_tag_req := lfb.io.out.toArb.ld_tag_req
  dcachearb.io.in.fromLfb.ld_tag_wen := lfb.io.out.toArb.ld_tag_wen
  dcachearb.io.in.fromLfb.serial_req := lfb.io.out.toArb.serial_req
  dcachearb.io.in.fromLfb.st_dirty_din := lfb.io.out.toArb.st_dirty_din
  dcachearb.io.in.fromLfb.st_dirty_gateclk_en := lfb.io.out.toArb.st_dirty_gateclk_en
  dcachearb.io.in.fromLfb.st_dirty_idx := lfb.io.out.toArb.st_dirty_idx
  dcachearb.io.in.fromLfb.st_dirty_req := lfb.io.out.toArb.st_dirty_req
  dcachearb.io.in.fromLfb.st_dirty_wen := lfb.io.out.toArb.st_dirty_wen
  dcachearb.io.in.fromLfb.st_req := lfb.io.out.toArb.st_req
  dcachearb.io.in.fromLfb.st_tag_din := lfb.io.out.toArb.st_tag_din
  dcachearb.io.in.fromLfb.st_tag_gateclk_en := lfb.io.out.toArb.st_tag_gateclk_en
  dcachearb.io.in.fromLfb.st_tag_idx := lfb.io.out.toArb.st_tag_idx
  dcachearb.io.in.fromLfb.st_tag_req := lfb.io.out.toArb.st_tag_req
  dcachearb.io.in.fromLfb.st_tag_wen := lfb.io.out.toArb.st_tag_wen
  dcachearb.io.in.fromVb.borrow_addr := vb.io.out.toDcacheArb.borrow_addr
  dcachearb.io.in.fromVb.data_way := vb.io.out.toDcacheArb.data_way
  dcachearb.io.in.fromVb.dcache_replace := vb.io.out.toDcacheArb.dcache_replace
  dcachearb.io.in.fromVb.ld_borrow_req := vb.io.out.toDcacheArb.ld_borrow_req
  dcachearb.io.in.fromVb.ld_borrow_req_gate := vb.io.out.toDcacheArb.ld_borrow_req_gate
  dcachearb.io.in.fromVb.ld_data_gateclk_en := vb.io.out.toDcacheArb.ld_data_gateclk_en
  dcachearb.io.in.fromVb.ld_data_idx := vb.io.out.toDcacheArb.ld_data_idx
  dcachearb.io.in.fromVb.ld_req := vb.io.out.toDcacheArb.ld_req
  dcachearb.io.in.fromVb.ld_tag_gateclk_en := vb.io.out.toDcacheArb.ld_tag_gateclk_en
  dcachearb.io.in.fromVb.ld_tag_idx := vb.io.out.toDcacheArb.ld_tag_idx
  dcachearb.io.in.fromVb.ld_tag_req := vb.io.out.toDcacheArb.ld_tag_req
  dcachearb.io.in.fromVb.ld_tag_wen := vb.io.out.toDcacheArb.ld_tag_wen
  dcachearb.io.in.fromVb.serial_req := vb.io.out.toDcacheArb.serial_req
  dcachearb.io.in.fromVb.set_way_mode := vb.io.out.toDcacheArb.set_way_mode
  dcachearb.io.in.fromVb.st_borrow_req := vb.io.out.toDcacheArb.st_borrow_req
  dcachearb.io.in.fromVb.st_dirty_din := vb.io.out.toDcacheArb.st_dirty_din
  dcachearb.io.in.fromVb.st_dirty_gateclk_en := vb.io.out.toDcacheArb.st_dirty_gateclk_en
  dcachearb.io.in.fromVb.st_dirty_gwen := vb.io.out.toDcacheArb.st_dirty_gwen
  dcachearb.io.in.fromVb.st_dirty_idx := vb.io.out.toDcacheArb.st_dirty_idx
  dcachearb.io.in.fromVb.st_dirty_req := vb.io.out.toDcacheArb.st_dirty_req
  dcachearb.io.in.fromVb.st_dirty_wen := vb.io.out.toDcacheArb.st_dirty_wen
  dcachearb.io.in.fromVb.st_req := vb.io.out.toDcacheArb.st_req
  dcachearb.io.in.fromVb.st_tag_gateclk_en := vb.io.out.toDcacheArb.st_tag_gateclk_en
  dcachearb.io.in.fromVb.st_tag_idx := vb.io.out.toDcacheArb.st_tag_idx
  dcachearb.io.in.fromVb.st_tag_req := vb.io.out.toDcacheArb.st_tag_req
  dcachearb.io.in.vb_rcl_sm_data_id := 0.U.asTypeOf(dcachearb.io.in.vb_rcl_sm_data_id)
  //dcachearb.io.in.fromVb.rcl_sm_data_id //todo: have not this signal???
  dcachearb.io.in.fromWmb.data_way := wmb.io.out.toDcacheArb.data_way
  dcachearb.io.in.fromWmb.ld_borrow_req := wmb.io.out.toDcacheArb.ld_borrow_req
  dcachearb.io.in.fromWmb.ld_data_gateclk_en := wmb.io.out.toDcacheArb.ld_data_gateclk_en
  dcachearb.io.in.fromWmb.ld_data_gwen := wmb.io.out.toDcacheArb.ld_data_gwen
  dcachearb.io.in.fromWmb.ld_data_high_din := wmb.io.out.toDcacheArb.ld_data_high_din
  dcachearb.io.in.fromWmb.ld_data_idx := wmb.io.out.toDcacheArb.ld_data_idx
  dcachearb.io.in.fromWmb.ld_data_low_din := wmb.io.out.toDcacheArb.ld_data_low_din
  dcachearb.io.in.fromWmb.ld_data_req := wmb.io.out.toDcacheArb.ld_data_req
  dcachearb.io.in.fromWmb.ld_data_wen := wmb.io.out.toDcacheArb.ld_data_wen
  dcachearb.io.in.fromWmb.ld_req := wmb.io.out.toDcacheArb.ld_req
  dcachearb.io.in.fromWmb.ld_tag_gateclk_en := wmb.io.out.toDcacheArb.ld_tag_gateclk_en
  dcachearb.io.in.fromWmb.ld_tag_idx := wmb.io.out.toDcacheArb.ld_tag_idx
  dcachearb.io.in.fromWmb.ld_tag_req := wmb.io.out.toDcacheArb.ld_tag_req
  dcachearb.io.in.fromWmb.ld_tag_wen := wmb.io.out.toDcacheArb.ld_tag_wen
  dcachearb.io.in.fromWmb.st_dirty_din := wmb.io.out.toDcacheArb.st_dirty_din
  dcachearb.io.in.fromWmb.st_dirty_gateclk_en := wmb.io.out.toDcacheArb.st_dirty_gateclk_en
  dcachearb.io.in.fromWmb.st_dirty_idx := wmb.io.out.toDcacheArb.st_dirty_idx
  dcachearb.io.in.fromWmb.st_dirty_req := wmb.io.out.toDcacheArb.st_dirty_req
  dcachearb.io.in.fromWmb.st_dirty_wen := wmb.io.out.toDcacheArb.st_dirty_wen
  dcachearb.io.in.fromWmb.st_req := wmb.io.out.toDcacheArb.st_req
  //dcachearb.io.in.fromVb := DontCare //////todo: too many Useless signals, DontCare before connect

  dcachearb.io.in.fromVb.st_id := 0.U.asTypeOf(dcachearb.io.in.fromVb.st_id) //////todo: Useless
  dcachearb.io.in.fromVb.ld_data_low_din := 0.U.asTypeOf(dcachearb.io.in.fromVb.ld_data_low_din) //////todo: Useless
  dcachearb.io.in.fromVb.ld_data_high_din := 0.U.asTypeOf(dcachearb.io.in.fromVb.ld_data_high_din) //////todo: Useless
  dcachearb.io.in.fromVb.ld_data_low_idx := 0.U.asTypeOf(dcachearb.io.in.fromVb.ld_data_low_idx) //////todo: Useless
  dcachearb.io.in.fromVb.req_addr := 0.U.asTypeOf(dcachearb.io.in.fromVb.req_addr) //////todo: Useless
  dcachearb.io.in.fromIcc.req_addr := 0.U.asTypeOf(dcachearb.io.in.fromIcc.req_addr) //////todo: Useless
  dcachearb.io.in.fromIcc.ld_tag_wen := 0.U.asTypeOf(dcachearb.io.in.fromIcc.ld_tag_wen) //////todo: Useless? it for snoop_snq
  dcachearb.io.in.fromIcc.dcache_replace := 0.U.asTypeOf(dcachearb.io.in.fromIcc.dcache_replace) //////todo: Useless, it for snoop_snq
  dcachearb.io.in.fromIcc.borrow_addr := 0.U.asTypeOf(dcachearb.io.in.fromIcc.borrow_addr) //////todo: Useless, it for snoop_snq
  dcachearb.io.in.fromIcc.st_id := 0.U.asTypeOf(dcachearb.io.in.fromIcc.st_id) //////todo: Useless, it for snoop_snq
  dcachearb.io.in.snq_dcache_sdb_id := 0.U.asTypeOf(dcachearb.io.in.snq_dcache_sdb_id) //////todo: complete snoop_snq
  dcachearb.io.in.fromMcic := 0.U.asTypeOf( dcachearb.io.in.fromMcic) //////todo: complete mcic
  dcachearb.io.in.fromSnq := 0.U.asTypeOf(dcachearb.io.in.fromSnq) //////todo: complete snq


  //==========================================================
  //                       DC Stage
  //==========================================================
  // &Instance("ct_lsu_dcache_top","x_ct_lsu_dcache_top"); @95
  dcachetop.io.in.cp0_lsu_icg_en := io.in.fromCp0.lsu_icg_en
  dcachetop.io.in.ld_data.gateclk_en := dcachearb.io.out.ld_data.gateclk_en
  dcachetop.io.in.ld_data.gwen_b := dcachearb.io.out.ld_data.gwen_b
  dcachetop.io.in.ld_data.high_din := dcachearb.io.out.ld_data.high_din
  dcachetop.io.in.ld_data.high_idx := dcachearb.io.out.ld_data.high_idx
  dcachetop.io.in.ld_data.low_din := dcachearb.io.out.ld_data.low_din
  dcachetop.io.in.ld_data.low_idx := dcachearb.io.out.ld_data.low_idx
  dcachetop.io.in.ld_data.sel_b := dcachearb.io.out.ld_data.sel_b
  dcachetop.io.in.ld_data.wen_b := dcachearb.io.out.ld_data.wen_b
  dcachetop.io.in.ld_tag.din := dcachearb.io.out.ld_tag.din
  dcachetop.io.in.ld_tag.gateclk_en := dcachearb.io.out.ld_tag.gateclk_en
  dcachetop.io.in.ld_tag.gwen_b := dcachearb.io.out.ld_tag.gwen_b
  dcachetop.io.in.ld_tag.idx := dcachearb.io.out.ld_tag.idx
  dcachetop.io.in.ld_tag.sel_b := dcachearb.io.out.ld_tag.sel_b
  dcachetop.io.in.ld_tag.wen_b := dcachearb.io.out.ld_tag.wen_b
  dcachetop.io.in.st_dirty.din := dcachearb.io.out.st_dirty.din
  dcachetop.io.in.st_dirty.gateclk_en := dcachearb.io.out.st_dirty.gateclk_en
  dcachetop.io.in.st_dirty.gwen_b := dcachearb.io.out.st_dirty.gwen_b
  dcachetop.io.in.st_dirty.idx := dcachearb.io.out.st_dirty.idx
  dcachetop.io.in.st_dirty.sel_b := dcachearb.io.out.st_dirty.sel_b
  dcachetop.io.in.st_dirty.wen_b := dcachearb.io.out.st_dirty.wen_b
  dcachetop.io.in.st_tag.din := dcachearb.io.out.st_tag.din
  dcachetop.io.in.st_tag.gateclk_en := dcachearb.io.out.st_tag.gateclk_en
  dcachetop.io.in.st_tag.gwen_b := dcachearb.io.out.st_tag.gwen_b
  dcachetop.io.in.st_tag.idx := dcachearb.io.out.st_tag.idx
  dcachetop.io.in.st_tag.sel_b := dcachearb.io.out.st_tag.sel_b
  dcachetop.io.in.st_tag.wen_b := dcachearb.io.out.st_tag.wen_b
  dcachetop.io.in.pad_yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en

  // &Instance("ct_lsu_ld_dc","x_ct_lsu_ld_dc"); @96
  loaddc.io.in.cb_ld_dc_addr_hit := 0.U.asTypeOf(loaddc.io.in.cb_ld_dc_addr_hit) //////todo: complete cache_buffer
  loaddc.io.in.fromCp0.lsu_da_fwd_dis := io.in.fromCp0.lsu_da_fwd_dis
  loaddc.io.in.fromCp0.lsu_dcache_en := io.in.fromCp0.lsu_dcache_en
  loaddc.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  loaddc.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  loaddc.io.in.ctrl_ld_clk := ctrl.io.out.ctrlClk.ldClk
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_db := dcachearb.io.out.toLoadDC.borrow_vb
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_icc := dcachearb.io.out.toLoadDC.borrow_icc
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_mmu := dcachearb.io.out.toLoadDC.borrow_mmu
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_sndb := dcachearb.io.out.toLoadDC.borrow_sndb
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_vb := dcachearb.io.out.toLoadDC.borrow_vb
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_vld := dcachearb.io.out.toLoadDC.borrow_vld
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_vld_gate := dcachearb.io.out.toLoadDC.borrow_vld_gate
  loaddc.io.in.fromDcacheArb.ld_dc_settle_way := dcachearb.io.out.toLoadDC.settle_way
  loaddc.io.in.fromDcacheArb.idx := dcachearb.io.out.dcache_idx
  loaddc.io.in.dcache_lsu_ld_tag_dout := dcachetop.io.out.ld_tag_dout.asTypeOf(loaddc.io.in.dcache_lsu_ld_tag_dout)
  loaddc.io.in.fromHad := io.in.ld_dc.fromHad
  loaddc.io.in.icc_dcache_arb_ld_tag_read := icc.io.out.toArb.ld.tagRead
  loaddc.io.in.fromAG := loadag.io.out.toDC
  loaddc.io.in.fromLQ.ld_dc_full := lq.io.out.lq_ld_dc_full
  loaddc.io.in.fromLQ.ld_dc_inst_hit := lq.io.out.lq_ld_dc_inst_hit
  loaddc.io.in.fromLQ.ld_dc_less2 := lq.io.out.lq_ld_dc_less2
  loaddc.io.in.fromLQ.ld_dc_spec_fail := lq.io.out.lq_ld_dc_spec_fail
  loaddc.io.in.lsu_dcache_ld_xx_gwen := dcachearb.io.out.lsu_dcache_ld_xx_gwen
  loaddc.io.in.lsu_has_fence := rb.io.out.lsu_has_fence
  io.out.ld_dc.toIDU := loaddc.io.out.toIDU
  io.out.ld_dc.lsu_mmu_vabuf0 := loaddc.io.out.lsu_mmu_vabuf0
  loaddc.io.in.fromMMU.mmu_en := io.in.fromMMU.mmu_en
  loaddc.io.in.fromMMU.tlb_busy := io.in.fromMMU.tlb_busy
  loaddc.io.in.fromMMU.data_req_size := io.in.fromMMU.data_req_size
  loaddc.io.in.fromPad.yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  loaddc.io.in.fromPFU := 0.U.asTypeOf(loaddc.io.in.fromPFU) //////todo: complete pfu
  loaddc.io.in.rb_fence_ld := rb.io.out.rb_fence_ld
  loaddc.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  loaddc.io.in.fromSQ.ld_dc_addr1_dep_discard := sq.io.out.toLdDc.addr1DepDiscard
  loaddc.io.in.fromSQ.ld_dc_cancel_acc_req := sq.io.out.toLdDc.cancelAccReq
  loaddc.io.in.fromSQ.ld_dc_cancel_ahead_wb := sq.io.out.toLdDc.cancelAheadWb
  loaddc.io.in.fromSQ.ld_dc_fwd_req := sq.io.out.toLdDc.fwdReq
  loaddc.io.in.fromSQ.ld_dc_has_fwd_req := sq.io.out.toLdDc.hasFwdReq
  loaddc.io.in.fromStDC.addr0 := storedc.io.out.toSq.addr0
  loaddc.io.in.fromStDC.bytes_vld := storedc.io.out.toSq.bytesVld
  loaddc.io.in.fromStDC.chk_st_inst_vld := storedc.io.out.toDa.toPwdDa.chkStInstVld
  loaddc.io.in.fromStDC.chk_statomic_inst_vld := storedc.io.out.toDa.toPwdDa.chkStatomicInstVld
  loaddc.io.in.fromStDC.inst_vld := storedc.io.out.toDa.instVld
  loaddc.io.in.fromWmb.fwd_bytes_vld := wmb.io.out.wmb_fwd_bytes_vld
  loaddc.io.in.fromWmb.ld_dc_cancel_acc_req := wmb.io.out.toLoadDC.cancel_acc_req
  loaddc.io.in.fromWmb.ld_dc_discard_req := wmb.io.out.toLoadDC.discard_req
  loaddc.io.in.fromWmb.ld_dc_fwd_req := wmb.io.out.toLoadDC.fwd_req

  // &Instance("ct_lsu_st_dc","x_ct_lsu_st_dc"); @97
  storedc.io.in.cp0In.dcacheEn := io.in.fromCp0.lsu_dcache_en
  storedc.io.in.cp0In.icgEn := io.in.fromCp0.lsu_icg_en
  storedc.io.in.cp0In.l2StPrefEn := io.in.fromCp0.lsu_l2_pref_en
  storedc.io.in.cp0In.clkEn := io.in.fromCp0.yy_clk_en
  //storedc.io.in. todo: ctrl_st_clk
  storedc.io.in.dcacheIn.arbBorrowIcc := dcachearb.io.out.toStoreDC.borrow_icc
  storedc.io.in.dcacheIn.arbBorrowSnq := dcachearb.io.out.toStoreDC.borrow_snq
  storedc.io.in.dcacheIn.arbBorrowSnqId := dcachearb.io.out.toStoreDC.borrow_snq_id
  storedc.io.in.dcacheIn.arbBorrowVld := dcachearb.io.out.toStoreDC.borrow_vld
  storedc.io.in.dcacheIn.arbBorrowVldGate := dcachearb.io.out.toStoreDC.borrow_vld_gate
  storedc.io.in.dcacheIn.arbBcacheReplace := dcachearb.io.out.toStoreDC.dcache_replace
  storedc.io.in.dcacheIn.arbBcacheSw := dcachearb.io.out.toStoreDC.dcache_sw
  storedc.io.in.dcacheIn.dirtyGwen := dcachearb.io.out.dcache_dirty_gwen
  storedc.io.in.dcacheIn.idx := dcachearb.io.out.dcache_idx
  storedc.io.in.dcacheIn.lsuStDirtyDout := dcachetop.io.out.st_dirty_dout
  storedc.io.in.dcacheIn.lsuStTagDout := dcachetop.io.out.st_tag_dout
  //storedc.io.in.dcacheIn.had todo: add from had
  storedc.io.in.lqIn.specFail := lq.io.out.lq_st_dc_spec_fail
  io.out.st_dc.toIdu := storedc.io.out.toIdu
  io.out.st_dc.toMmu := storedc.io.out.toMmu
  storedc.io.in.mmuIn.mmuEn := io.in.fromMMU.mmu_en
  storedc.io.in.mmuIn.tlbBusy := io.in.fromMMU.tlb_busy
  //storedc.io.in.pad_yy_icg_scan_en todo:add signal
  storedc.io.in.rtuIn.flush := io.in.fromRTU.yy_xx_flush
  storedc.io.in.rtuIn.commitIidUpdata := io.in.fromRTU.commitIidUpdata
  storedc.io.in.sdEx1In.sdid := storeex1.io.out.rfEx1Sdid
  storedc.io.in.sqIn.full := sq.io.out.toDc.full //todo: check it
  storedc.io.in.sqIn.instHit := sq.io.out.toDc.instHit
  storedc.io.in.agIn := storeag.io.out.toDc

  // &Instance("ct_lsu_lq","x_ct_lsu_lq"); @99
  lq.io.in.fromCP0.lsu_corr_dis := io.in.fromCp0.lsu_corr_dis
  lq.io.in.fromCP0.lsu_icg_en  := io.in.fromCp0.lsu_icg_en
  lq.io.in.fromCP0.yy_clk_en := io.in.fromCp0.yy_clk_en
  lq.io.in.fromLdDC.addr(0) := loaddc.io.out.toDA.addr0
  lq.io.in.fromLdDC.addr(1) := loaddc.io.out.ld_dc_addr1
  lq.io.in.fromLdDC.bytes_vld(0) := loaddc.io.out.ld_dc_bytes_vld
  lq.io.in.fromLdDC.bytes_vld(1) := loaddc.io.out.ld_dc_bytes_vld1
  lq.io.in.fromLdDC.chk_ld_addr1_vld := loaddc.io.out.toChk.ld_addr1_vld
  lq.io.in.fromLdDC.iid := loaddc.io.out.toDA.iid
  lq.io.in.fromLdDC.inst_chk_vld := loaddc.io.out.ld_dc_inst_chk_vld
  lq.io.in.EntryCreate.dp_vld(1) := loaddc.io.out.toLQ.create1_dp_vld
  lq.io.in.EntryCreate.gateclk_en(1) := loaddc.io.out.toLQ.create1_gateclk_en
  lq.io.in.EntryCreate.vld(1) := loaddc.io.out.toLQ.create1_vld
  lq.io.in.EntryCreate.dp_vld(0) := loaddc.io.out.toLQ.create_dp_vld
  lq.io.in.EntryCreate.gateclk_en(0) := loaddc.io.out.toLQ.create_gateclk_en
  lq.io.in.EntryCreate.vld(0) := loaddc.io.out.toLQ.create_vld
  lq.io.in.fromLdDC.secd := loaddc.io.out.toDA.secd
  io.out.lq := lq.io.out
  lq.io.in.fromPad.yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  lq.io.in.fromRTU := io.in.fromRTU
  lq.io.in.fromStDC.addr0 := storedc.io.out.toSq.addr0
  lq.io.in.fromStDC.bytes_vld := storedc.io.out.toSq.bytesVld
  lq.io.in.fromStDC.chk_st_inst_vld := storedc.io.out.toDa.toPwdDa.chkStInstVld
  lq.io.in.fromStDC.chk_statomic_inst_vld := storedc.io.out.toDa.toPwdDa.chkStatomicInstVld
  lq.io.in.fromStDC.iid  := storedc.io.out.toSq.iid

  // &Instance("ct_lsu_sq","x_ct_lsu_sq"); @100
  sq.io.in.cp0In.lsuIcgEn := io.in.fromCp0.lsu_icg_en
  sq.io.in.cp0In.clkEn := io.in.fromCp0.yy_clk_en
  sq.io.in.cp0In.privMode := io.in.fromCp0.yy_priv_mode
  sq.io.in.dcacheIn.dirtyDin.valid := dcachearb.io.out.dcache_dirty_din(6).asBool
  sq.io.in.dcacheIn.dirtyDin.bits(1) := dcachearb.io.out.dcache_dirty_din(5,3).asTypeOf(sq.io.in.dcacheIn.dirtyDin.bits(1))
  sq.io.in.dcacheIn.dirtyDin.bits(0) := dcachearb.io.out.dcache_dirty_din(2,0).asTypeOf(sq.io.in.dcacheIn.dirtyDin.bits(0))
  sq.io.in.dcacheIn.dirtyGwen := dcachearb.io.out.dcache_dirty_gwen
  sq.io.in.dcacheIn.dirtyWen.valid := dcachearb.io.out.dcache_dirty_wen(6).asBool
  sq.io.in.dcacheIn.dirtyWen.bits(1) := dcachearb.io.out.dcache_dirty_wen(5,3).asTypeOf(sq.io.in.dcacheIn.dirtyWen.bits(1))
  sq.io.in.dcacheIn.dirtyWen.bits(0) := dcachearb.io.out.dcache_dirty_wen(2,0).asTypeOf(sq.io.in.dcacheIn.dirtyWen.bits(0))
  sq.io.in.dcacheIn.idx := dcachearb.io.out.dcache_idx
  sq.io.in.dcacheIn.tagDin(1).valid := dcachearb.io.out.dcache_tag_din(51) //////todo: check it
  sq.io.in.dcacheIn.tagDin(0).valid := dcachearb.io.out.dcache_tag_din(25)
  sq.io.in.dcacheIn.tagDin(1).bits := dcachearb.io.out.dcache_tag_din(50,26)
  sq.io.in.dcacheIn.tagDin(0).bits := dcachearb.io.out.dcache_tag_din(24,0)
  sq.io.in.dcacheIn.tagGwen := dcachearb.io.out.dcache_tag_gwen
  sq.io.in.dcacheIn.tagWen := VecInit(dcachearb.io.out.dcache_tag_wen.asBools)
  //sq.io.in.had //todo: add signals had
  sq.io.in.iccIn.idle := icc.io.out.iccIdle
  sq.io.in.iccIn.sqGrnt := icc.io.out.sqGrnt
  sq.io.in.ldDaIn.lsid := loadda.io.out.ld_da_lsid.asUInt //////todo: may cause bits error, check it
  sq.io.in.ldDaIn.sqDataDiscardVld := loadda.io.out.toSQ.data_discard_vld
  sq.io.in.ldDaIn.sqFwdId := loadda.io.out.toSQ.fwd_id.asUInt
  sq.io.in.ldDaIn.sqFwdMultiVld := loadda.io.out.toSQ.fwd_multi_vld
  sq.io.in.ldDaIn.sqGlobalDiscardVld := loadda.io.out.toSQ.global_discard_vld
  sq.io.in.ldDcIn.addr0 := loaddc.io.out.toDA.addr0
  sq.io.in.ldDcIn.addr1_11to4 := loaddc.io.out.ld_dc_addr1_11to4
  sq.io.in.ldDcIn.bytesVld := loaddc.io.out.ld_dc_bytes_vld
  sq.io.in.ldDcIn.bytesVld1 := loaddc.io.out.ld_dc_bytes_vld1
  sq.io.in.ldDcIn.chkAtomicInstVld := loaddc.io.out.toChk.atomic_inst_vld
  sq.io.in.ldDcIn.chkLdAddr1Vld := loaddc.io.out.toChk.ld_addr1_vld
  sq.io.in.ldDcIn.chkLdBypassVld := loaddc.io.out.toChk.ld_bypass_vld
  sq.io.in.ldDcIn.chkLdInstVld := loaddc.io.out.toChk.ld_inst_vld
  sq.io.in.ldDcIn.iid := loaddc.io.out.toDA.iid
  //lsu_had_sq_not_empty -> ctrl, not exists
  //io.out.tohad todo: complete had
  io.out.sq.toIDU.SqNotFull := sq.io.out.SqNotFull
  io.out.sq.toRTU.AllCommitDataVld := sq.io.out.AllCommitDataVld
  sq.io.in.rbIn.rtuAllCommitLdDataVld := rb.io.out.lsu_rtu_all_commit_ld_data_vld
  //sq.io.in.pad_yy_icg_scan_en todo: add pad
  sq.io.in.rbIn.sqPopHitIdx := rb.io.out.rb_sq_pop_hit_idx
  sq.io.in.rtuIn.flush := io.in.fromRTU.yy_xx_flush
  sq.io.in.rtuIn.asyncFlush := io.in.fromRTU.lsu_async_flush
  sq.io.in.rtuIn.commit := io.in.fromRTU.yy_xx_commit
  sq.io.in.rtuIn.commitIidUpdt := io.in.fromRTU.commitIidUpdata
  sq.io.in.sdEx1In.sd_ex1_data_bypass := storeex1.io.out.sdEx1DataBypass.asTypeOf(sq.io.in.sdEx1In.sd_ex1_data_bypass)
  sq.io.in.sdEx1In.sd_ex1_data := storeex1.io.out.sdEx1Data
  sq.io.in.sdEx1In.toSqEntry.ex1InstVld := storeex1.io.out.sdEx1InstVld
  sq.io.in.sdEx1In.toSqEntry.rfEx1Sdid := storeex1.io.out.rfEx1Sdid
  sq.io.in.sdEx1In.toSqEntry.rfInstVldShort := storeex1.io.out.rfInstVldShort
  sq.io.in.daIn.bkptaData := storeda.io.out.bkptaData
  sq.io.in.daIn.bkptbData := storeda.io.out.bkptbData
  sq.io.in.daIn.iid := storeda.io.out.iid
  sq.io.in.daIn.instVld := storeda.io.out.instVld
  sq.io.in.daIn.stDaIn := storeda.io.out.toSq //////todo: check it, sqDcacheDirty, not snqDcacheDirty, not all need
  sq.io.in.daIn.addr := 0.U.asTypeOf(sq.io.in.daIn.addr) //////todo: check it, not need?
  sq.io.in.dcIn.sqda := storedc.io.out.toDa.toSqDa //////todo: not all need
  sq.io.in.dcIn.sq := storedc.io.out.toSq
  sq.io.in.wmbIn.addr := wmbce.io.out.toWmb.addr
  //////todo: can not find some signal wmbin, bkptab, data128,...
  sq.io.in.wmbIn.dcacheSwInst := wmbce.io.out.toWmb.dcache_sw_inst
  sq.io.in.wmbIn.sqPtr := wmbce.io.out.wmb_ce_sq_ptr.asUInt
  sq.io.in.wmbIn.ceVld := wmbce.io.out.toWmb.vld
  sq.io.in.wmbIn.popGrnt := wmb.io.out.toSQ.pop_grnt
  sq.io.in.wmbIn.popToCeGrnt := wmb.io.out.toSQ.pop_to_ce_grnt

  //==========================================================
  //                       DA Stage
  //==========================================================
  // &Instance("ct_lsu_ld_da","x_ct_lsu_ld_da"); @104
  loadda.io.in.cb_ld_da_data := 0.U.asTypeOf(loadda.io.in.cb_ld_da_data) //////todo: complete cache_buffer
  loadda.io.in.fromCp0.lsu_dcache_en := io.in.fromCp0.lsu_dcache_en
  loadda.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  loadda.io.in.fromCp0.lsu_l2_pref_en := io.in.fromCp0.lsu_l2_pref_en
  loadda.io.in.fromCp0.lsu_nsfe := io.in.fromCp0.lsu_nsfe
  loadda.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  loadda.io.in.fromCp0.yy_dcache_pref_en := io.in.fromCp0.yy_dcache_pref_en
  loadda.io.in.ctrl_ld_clk := ctrl.io.out.ctrlClk.ldClk
  loadda.io.in.dcache_lsu_ld_data_bank_dout := dcachetop.io.out.ld_data_bank_dout
  loadda.io.in.fromDC := loaddc.io.out.toDA
  loadda.io.in.ld_hit_prefetch := lfb.io.out.ldHitPrefetch
  loadda.io.in.lfb_ld_da_hit_idx := lfb.io.out.ldDaHitIdx
  loadda.io.in.lm_ld_da_hit_idx := 0.U.asTypeOf(loadda.io.in.lm_ld_da_hit_idx) //////todo: complete lm
  io.out.ld_da.toIDU := loadda.io.out.toIDU
  io.out.ld_da.toRTU := loadda.io.out.toRTU
  loadda.io.in.lsu_special_clk := ctrl.io.out.lsuSpecialClk
  loadda.io.in.mmu_lsu_access_fault0 := io.in.fromMMU.mmu_lsu_access_fault0
  loadda.io.in.fromPad.yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  loadda.io.in.pfu_biu_req_addr := 0.U.asTypeOf(loadda.io.in.pfu_biu_req_addr) ///////todo: complete pfu
  loadda.io.in.fromRB.full := rb.io.out.toLoadDA.full
  loadda.io.in.fromRB.hit_idx := rb.io.out.toLoadDA.hit_idx
  loadda.io.in.fromRB.merge_fail := rb.io.out.toLoadDA.merge_fail
  loadda.io.in.fromRTU.yy_xx_commit := io.in.fromRTU.yy_xx_commit
  loadda.io.in.fromRTU.yy_xx_commit_iid := io.in.fromRTU.yy_xx_commit_iid
  loadda.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  loadda.io.in.fromSdEx1.data_bypass := storeex1.io.out.sdEx1DataBypass
  loadda.io.in.fromSdEx1.inst_vld := storeex1.io.out.sdEx1InstVld
  loadda.io.in.fromSF := 0.U.asTypeOf(loadda.io.in.fromSF) //////todo: complete spec_fail_predict
  loadda.io.in.fromSQ.fwd_data := sq.io.out.toLdDa.fwdData
  loadda.io.in.fromSQ.fwd_data_pe := sq.io.out.toLdDa.fwdDataPe
  loadda.io.in.fromSQ.data_discard_req := sq.io.out.toLdDc.dataDiscardReq
  loadda.io.in.fromSQ.fwd_bypass_multi := sq.io.out.toLdDc.fwdBypassMulti
  loadda.io.in.fromSQ.fwd_bypass_req := sq.io.out.toLdDc.fwdBypassReq
  loadda.io.in.fromSQ.fwd_id := sq.io.out.toLdDc.fwdId.asBools
  loadda.io.in.fromSQ.fwd_multi := sq.io.out.toLdDc.fwdMulti
  loadda.io.in.fromSQ.fwd_multi_mask := sq.io.out.toLdDc.fwdMultiMask
  loadda.io.in.fromSQ.newest_fwd_data_vld_req := sq.io.out.toLdDc.newestFwdDataVldReq
  loadda.io.in.fromSQ.other_discard_req := sq.io.out.toLdDc.otherDiscardReq
  loadda.io.in.st_da_addr := storeda.io.out.addr
  loadda.io.in.fromWmb.ld_da_fwd_data := wmb.io.out.wmb_ld_da_fwd_data
  loadda.io.in.fromWmb.ld_dc_discard_req := wmb.io.out.toLoadDC.discard_req

  // &Instance("ct_lsu_st_da","x_ct_lsu_st_da"); @105
  storeda.io.in.amrWaCancel := amr.io.out.waCancel
  storeda.io.in.cp0In.lsuDcacheEn := io.in.fromCp0.lsu_dcache_en
  storeda.io.in.cp0In.lsuIcgEn := io.in.fromCp0.lsu_icg_en
  storeda.io.in.cp0In.lsuL2StPrefEn := io.in.fromCp0.lsu_l2_pref_en
  storeda.io.in.cp0In.lsuNsfe := io.in.fromCp0.lsu_nsfe
  storeda.io.in.cp0In.yyClkEn := io.in.fromCp0.yy_clk_en
  storeda.io.in.dcacheIn.dirtyDin.valid := dcachearb.io.out.dcache_dirty_din(6).asBool
  storeda.io.in.dcacheIn.dirtyDin.bits(1) := dcachearb.io.out.dcache_dirty_din(5,3).asTypeOf(storeda.io.in.dcacheIn.dirtyDin.bits(1))
  storeda.io.in.dcacheIn.dirtyDin.bits(0) := dcachearb.io.out.dcache_dirty_din(2,0).asTypeOf(storeda.io.in.dcacheIn.dirtyDin.bits(0))
  storeda.io.in.dcacheIn.dirtyGwen := dcachearb.io.out.dcache_dirty_gwen
  storeda.io.in.dcacheIn.dirtyWen.valid := dcachearb.io.out.dcache_dirty_wen(6)
  storeda.io.in.dcacheIn.dirtyWen.bits(1) := dcachearb.io.out.dcache_dirty_wen(5,3).asTypeOf(storeda.io.in.dcacheIn.dirtyWen.bits(1))
  storeda.io.in.dcacheIn.dirtyWen.bits(0) := dcachearb.io.out.dcache_dirty_wen(2,0).asTypeOf(storeda.io.in.dcacheIn.dirtyWen.bits(0))
  storeda.io.in.dcacheIn.idx := dcachearb.io.out.dcache_idx
  storeda.io.in.dcacheIn.tagDin(1).valid := dcachearb.io.out.dcache_tag_din(51) ////
  storeda.io.in.dcacheIn.tagDin(0).valid := dcachearb.io.out.dcache_tag_din(25)
  storeda.io.in.dcacheIn.tagDin(1).bits := dcachearb.io.out.dcache_tag_din(50,26)
  storeda.io.in.dcacheIn.tagDin(0).bits := dcachearb.io.out.dcache_tag_din(24,0)
  storeda.io.in.dcacheIn.tagGwen := dcachearb.io.out.dcache_tag_gwen
  storeda.io.in.dcacheIn.tagWen := dcachearb.io.out.dcache_tag_wen.asBools
  storeda.io.in.ldDaIn.hitIdx := loadda.io.out.ld_da_st_da_hit_idx
  storeda.io.in.lfbhitIdx := lfb.io.out.stDaHitIdx
  storeda.io.in.lmHitIdx := 0.U.asTypeOf(storeda.io.in.lmHitIdx)//////todo: complete lm
  storeda.io.in.rbIn.lsuHasFence := rb.io.out.lsu_has_fence
  io.out.st_da.toRTU := storeda.io.out.toRtu
  storeda.io.in.mmuAccessFault1 := io.in.fromMMU.mmu_lsu_access_fault1
  storeda.io.in.padYyIcgScanEn := io.in.fromPad.yy_icg_scan_en
  storeda.io.in.pfuBiuReqAddr := 0.U.asTypeOf(storeda.io.in.pfuBiuReqAddr)//////todo: complete pfu
  storeda.io.in.rbIn.full := rb.io.out.rb_st_da_full
  storeda.io.in.rbIn.hitIdx := rb.io.out.rb_st_da_hit_idx
  storeda.io.in.rtuIn.commitIid := io.in.fromRTU.yy_xx_commit_iid
  storeda.io.in.rtuIn.commit := io.in.fromRTU.yy_xx_commit
  storeda.io.in.rtuIn.flush := io.in.fromRTU.yy_xx_flush
  storeda.io.in.dcIn.toPwdDa.addr0 := storedc.io.out.toSq.addr0
  storeda.io.in.dcIn := storedc.io.out.toDa

  // &Instance("ct_lsu_rb","x_ct_lsu_rb"); @107
  rb.io.in.fromBiu := io.in.rb.fromBiu
  rb.io.in.bus_arb_rb_ar_grnt := busarb.io.out.bus_arb_rb_ar_grnt
  rb.io.in.fromCp0.lsu_dcache_en := io.in.fromCp0.lsu_dcache_en
  rb.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  rb.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  rb.io.in.fromCp0.yy_priv_mode := io.in.fromCp0.yy_priv_mode
  rb.io.in.fromLoadDA.addr := loadda.io.out.ld_da_addr
  rb.io.in.fromLoadDA.bkpta_data := loadda.io.out.ld_da_bkpta_data
  rb.io.in.fromLoadDA.bkptb_data := loadda.io.out.ld_da_bkptb_data
  rb.io.in.fromLoadDA.boundary_after_mask := loadda.io.out.toRB.boundary_after_mask
  rb.io.in.fromLoadDA.bytes_vld := loadda.io.out.toRB.bytes_vld
  rb.io.in.fromLoadDA.data_ori := loadda.io.out.toRB.data_ori
  rb.io.in.fromLoadDA.data_rot_sel := loadda.io.out.toRB.data_rot_sel
  rb.io.in.fromLoadDA.dcache_hit := loadda.io.out.ld_da_dcache_hit
  rb.io.in.fromLoadDA.idx := loadda.io.out.ld_da_idx
  rb.io.in.fromLoadDA.iid := loadda.io.out.ld_da_iid
  rb.io.in.fromLoadDA.inst_size := loadda.io.out.toRB.inst_size
  rb.io.in.fromLoadDA.inst_vfls := loadda.io.out.ld_da_inst_vfls
  rb.io.in.fromLoadDA.mcic_borrow_mmu := loadda.io.out.toRB.mcic_borrow_mmu
  rb.io.in.fromLoadDA.old := loadda.io.out.toRB.old
  rb.io.in.fromLoadDA.page_buf := loadda.io.out.toRB.page_buf
  rb.io.in.fromLoadDA.page_ca := loadda.io.out.toRB.page_ca
  rb.io.in.fromLoadDA.page_sec := loadda.io.out.toRB.page_sec
  rb.io.in.fromLoadDA.page_share := loadda.io.out.toRB.page_share
  rb.io.in.fromLoadDA.page_so := loadda.io.out.toRB.page_so
  rb.io.in.fromLoadDA.preg := loadda.io.out.ld_da_preg
  rb.io.in.fromLoadDA.rb_atomic := loadda.io.out.toRB.atomic
  rb.io.in.fromLoadDA.rb_cmit := loadda.io.out.toRB.cmit
  rb.io.in.fromLoadDA.rb_cmplt_success := loadda.io.out.toRB.cmplt_success
  rb.io.in.fromLoadDA.rb_create_dp_vld := loadda.io.out.toRB.create_dp_vld
  rb.io.in.fromLoadDA.rb_create_gateclk_en := loadda.io.out.toRB.create_gateclk_en
  rb.io.in.fromLoadDA.rb_create_judge_vld := loadda.io.out.toRB.create_judge_vld
  rb.io.in.fromLoadDA.rb_create_lfb := loadda.io.out.toRB.create_lfb
  rb.io.in.fromLoadDA.rb_create_vld := loadda.io.out.toRB.create_vld
  rb.io.in.fromLoadDA.rb_data_vld := loadda.io.out.toRB.data_vld
  rb.io.in.fromLoadDA.rb_dest_vld := loadda.io.out.toRB.dest_vld
  rb.io.in.fromLoadDA.rb_discard_grnt := loadda.io.out.toRB.discard_grnt
  rb.io.in.fromLoadDA.rb_ldamo := loadda.io.out.toRB.ldamo
  rb.io.in.fromLoadDA.rb_merge_dp_vld := loadda.io.out.toRB.merge_dp_vld
  rb.io.in.fromLoadDA.rb_merge_expt_vld := loadda.io.out.toRB.merge_expt_vld
  rb.io.in.fromLoadDA.rb_merge_gateclk_en := loadda.io.out.toRB.merge_gateclk_en
  rb.io.in.fromLoadDA.rb_merge_vld := loadda.io.out.toRB.merge_vld
  rb.io.in.fromLoadDA.sign_extend := loadda.io.out.toRB.sign_extend
  rb.io.in.fromLoadDA.vreg := loadda.io.out.ld_da_vreg
  rb.io.in.fromLoadDA.vreg_sign_sel := loadda.io.out.toRB.vreg_sign_sel
  rb.io.in.ld_wb_rb_cmplt_grnt := loadwb.io.out.ld_wb_rb_cmplt_grnt
  rb.io.in.ld_wb_rb_data_grnt := loadwb.io.out.ld_wb_rb_data_grnt
  rb.io.in.fromLfb.addr_full := lfb.io.out.lfbAddrFull
  rb.io.in.fromLfb.rb_biu_req_hit_idx := lfb.io.out.toRb.biuReqHitIdx
  rb.io.in.fromLfb.rb_ca_rready_grnt := lfb.io.out.toRb.caRreadyGrnt
  rb.io.in.fromLfb.rb_create_id := lfb.io.out.toRb.createId
  rb.io.in.fromLfb.rb_nc_rready_grnt := lfb.io.out.toRb.ncRreadyGrnt
  rb.io.in.lm_already_snoop := 0.U.asTypeOf(rb.io.in.lm_already_snoop) //////todo: complete lsu_lm
  io.out.rb.toIDU.lsu_idu_no_fence := rb.io.out.lsu_idu_no_fence
  io.out.rb.toIDU.lsu_idu_rb_not_full := rb.io.out.lsu_idu_rb_not_full
  io.out.rb.toRTU.lsu_rtu_all_commit_ld_data_vld := rb.io.out.lsu_rtu_all_commit_ld_data_vld
  rb.io.in.lsu_special_clk := ctrl.io.out.lsuSpecialClk
  rb.io.in.pad_yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  rb.io.in.pfu_biu_req_addr := 0.U.asTypeOf(rb.io.in.pfu_biu_req_addr) ///////todo: complete pfu
  rb.io.in.fromRTU.lsu_async_flush := io.in.fromRTU.lsu_async_flush
  rb.io.in.fromRTU.yy_xx_commit := io.in.fromRTU.yy_xx_commit
  rb.io.in.fromRTU.yy_xx_commit_iid := io.in.fromRTU.yy_xx_commit_iid
  rb.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  rb.io.in.fromSQ.pop_addr := sq.io.out.toWmb.ce.popAddr
  rb.io.in.fromSQ.pop_page_ca := sq.io.out.toWmb.ce.popPageCa
  rb.io.in.fromSQ.pop_page_so := sq.io.out.toWmb.ce.popPageSo
  rb.io.in.fromStoreDA.addr := storeda.io.out.addr
  rb.io.in.fromStoreDA.dcache_hit := storeda.io.out.toRb.dcacheHit
  rb.io.in.fromStoreDA.fence_inst := storeda.io.out.toRb.fenceInst
  rb.io.in.fromStoreDA.fence_mode := storeda.io.out.toRb.fenceMode
  rb.io.in.fromStoreDA.iid := storeda.io.out.iid
  rb.io.in.fromStoreDA.inst_size := storeda.io.out.toRb.instSize
  rb.io.in.fromStoreDA.old := storeda.io.out.toRb.old
  rb.io.in.fromStoreDA.page_buf := storeda.io.out.toRb.pageBuf
  rb.io.in.fromStoreDA.page_ca := storeda.io.out.toRb.pageCa
  rb.io.in.fromStoreDA.page_sec := storeda.io.out.toRb.pageSec
  rb.io.in.fromStoreDA.page_share := storeda.io.out.toRb.pageShare
  rb.io.in.fromStoreDA.page_so := storeda.io.out.toRb.pageSo
  rb.io.in.fromStoreDA.rb_cmit := storeda.io.out.toRb.cmit
  rb.io.in.fromStoreDA.rb_create_dp_vld := storeda.io.out.toRb.createDpVld
  rb.io.in.fromStoreDA.rb_create_gateclk_en := storeda.io.out.toRb.createGateclkEn
  rb.io.in.fromStoreDA.rb_create_lfb := storeda.io.out.toRb.createLfb
  rb.io.in.fromStoreDA.rb_create_vld := storeda.io.out.toRb.createVld
  rb.io.in.fromStoreDA.sync_fence := storeda.io.out.toSq.syncFence //////todo:not to Sq, check it!
  rb.io.in.fromStoreDA.sync_inst := storeda.io.out.toSq.syncInst //////todo:not to Sq, check it!
  rb.io.in.vb_rb_biu_req_hit_idx := vb.io.out.rbHitIdx
  rb.io.in.fromWmb.ce_addr := wmbce.io.out.toWmb.addr
  rb.io.in.fromWmb.ce_page_ca := wmbce.io.out.toWmb.page_ca
  rb.io.in.fromWmb.ce_page_so := wmbce.io.out.toWmb.page_so
  rb.io.in.fromWmb.has_sync_fence := wmb.io.out.wmb_has_sync_fence
  rb.io.in.fromWmb.rb_biu_req_hit_idx := wmb.io.out.wmb_rb_biu_req_hit_idx
  rb.io.in.fromWmb.rb_so_pending := wmb.io.out.wmb_rb_so_pending
  rb.io.in.fromWmb.sync_fence_biu_req_success := wmb.io.out.wmb_sync_fence_biu_req_success

  // &Instance("ct_lsu_wmb","x_ct_lsu_wmb"); @108
  wmb.io.in.amr_l2_mem_set := amr.io.out.l2MemSet
  wmb.io.in.fromBiu.b_id := io.in.rb.fromBiu.b_id
  wmb.io.in.fromBiu.b_resp := io.in.wmb.fromBiu.b_resp
  wmb.io.in.fromBiu.b_vld := io.in.rb.fromBiu.b_vld
  wmb.io.in.fromBiu.r_id := io.in.rb.fromBiu.r_id
  wmb.io.in.fromBiu.r_vld := io.in.rb.fromBiu.r_vld
  wmb.io.in.fromBusArb.ar_grnt := busarb.io.out.bus_arb_wmb_ar_grnt
  wmb.io.in.fromBusArb.aw_grnt := busarb.io.out.bus_arb_wmb_aw_grnt
  wmb.io.in.fromBusArb.w_grnt := busarb.io.out.bus_arb_wmb_w_grnt
  wmb.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  wmb.io.in.fromCp0.lsu_no_op_req := io.in.fromCp0.lsu_no_op_req
  wmb.io.in.fromCp0.lsu_wr_burst_dis := io.in.fromCp0.lsu_wr_burst_dis
  wmb.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  wmb.io.in.fromDcache.arb_wmb_ld_grnt := dcachearb.io.out.dcache_arb_wmb_ld_grnt
  wmb.io.in.fromDcache.dirty_din.valid := dcachearb.io.out.dcache_dirty_din(6)
  wmb.io.in.fromDcache.dirty_din.bits(1) := dcachearb.io.out.dcache_dirty_din(5,3).asTypeOf(wmb.io.in.fromDcache.dirty_din.bits(1))
  wmb.io.in.fromDcache.dirty_din.bits(0) := dcachearb.io.out.dcache_dirty_din(2,0).asTypeOf(wmb.io.in.fromDcache.dirty_din.bits(0))
  wmb.io.in.fromDcache.dirty_gwen := dcachearb.io.out.dcache_dirty_gwen
  wmb.io.in.fromDcache.dirty_wen.valid := dcachearb.io.out.dcache_dirty_wen(6)
  wmb.io.in.fromDcache.dirty_wen.bits(1) := dcachearb.io.out.dcache_dirty_wen(5,3).asTypeOf(wmb.io.in.fromDcache.dirty_wen.bits(1))
  wmb.io.in.fromDcache.dirty_wen.bits(0) := dcachearb.io.out.dcache_dirty_wen(2,0).asTypeOf(wmb.io.in.fromDcache.dirty_wen.bits(0))
  wmb.io.in.fromDcache.idx := dcachearb.io.out.dcache_idx
  wmb.io.in.fromDcache.snq_st_sel := dcachearb.io.out.dcache_snq_st_sel
  wmb.io.in.fromDcache.tag_din(1).valid := dcachearb.io.out.dcache_tag_din(51) ////
  wmb.io.in.fromDcache.tag_din(0).valid := dcachearb.io.out.dcache_tag_din(25)
  wmb.io.in.fromDcache.tag_din(1).bits := dcachearb.io.out.dcache_tag_din(50,26)
  wmb.io.in.fromDcache.tag_din(0).bits := dcachearb.io.out.dcache_tag_din(24,0)
  wmb.io.in.fromDcache.tag_gwen := dcachearb.io.out.dcache_tag_gwen
  wmb.io.in.fromDcache.tag_wen := dcachearb.io.out.dcache_tag_wen.asBools
  wmb.io.in.fromDcache.vb_snq_gwen := dcachearb.io.out.dcache_vb_snq_gwen
  wmb.io.in.icc_wmb_write_imme := icc.io.out.wmbWriteImme
  wmb.io.in.ld_ag_inst_vld := loadag.io.out.toDC.inst_vld
  wmb.io.in.fromLoadDA.fwd_ecc_stall := loadda.io.out.ld_da_fwd_ecc_stall
  wmb.io.in.fromLoadDA.lsid := loadda.io.out.ld_da_lsid
  wmb.io.in.fromLoadDA.wmb_discard_vld := loadda.io.out.ld_da_wmb_discard_vld
  wmb.io.in.fromLoadDC.addr0 := loaddc.io.out.toDA.addr0
  wmb.io.in.fromLoadDC.addr1_11to4 := loaddc.io.out.ld_dc_addr1_11to4
  wmb.io.in.fromLoadDC.bytes_vld := loaddc.io.out.ld_dc_bytes_vld
  wmb.io.in.fromLoadDC.chk_atomic_inst_vld := loaddc.io.out.toChk.atomic_inst_vld
  wmb.io.in.fromLoadDC.chk_ld_inst_vld := loaddc.io.out.ld_dc_inst_chk_vld
  wmb.io.in.ld_wb_wmb_data_grnt := loadwb.io.out.ld_wb_wmb_data_grnt
  wmb.io.in.fromLfb.read_req_hit_idx := lfb.io.out.toWmb.readReqHitIdx
  wmb.io.in.fromLfb.write_req_hit_idx := lfb.io.out.toWmb.writeReqHitIdx
  wmb.io.in.fromLm := 0.U.asTypeOf(wmb.io.in.fromLm) //////todo: complete lm
  wmb.io.in.fromPad.yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  wmb.io.in.pfu_biu_req_addr := 0.U.asTypeOf(wmb.io.in.pfu_biu_req_addr) ///////todo: complete pfu
  wmb.io.in.fromRB.biu_req_addr := rb.io.out.toBiu.req_addr
  wmb.io.in.fromRB.biu_req_unmask := rb.io.out.toBiu.req_unmask
  wmb.io.in.fromRB.wmb_so_pending := rb.io.out.rb_wmb_so_pending
  wmb.io.in.fromRTU.lsu_async_flush := io.in.fromRTU.lsu_async_flush
  wmb.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  wmb.io.in.fromSnq := 0.U.asTypeOf(wmb.io.in.fromSnq) //////todo: complete Snq
  wmb.io.in.fromSQ.pop_addr := sq.io.out.toWmb.ce.popAddr
  wmb.io.in.fromSQ.pop_priv_mode := sq.io.out.toWmb.ce.popPrivMode
  wmb.io.in.fromSQ.wmb_merge_req := sq.io.out.toWmb.mergeReq
  wmb.io.in.fromSQ.wmb_merge_stall_req := sq.io.out.toWmb.mergeStallReq
  wmb.io.in.fromSQ.wmb_pop_to_ce_dp_req := sq.io.out.toWmb.popToCeDpReq
  wmb.io.in.fromSQ.wmb_pop_to_ce_gateclk_en := sq.io.out.toWmb.popToCeGateclkEn
  wmb.io.in.fromSQ.wmb_pop_to_ce_req := sq.io.out.toWmb.popToCeReq
  wmb.io.in.st_ag_inst_vld := storeag.io.out.toDc.instVld
  wmb.io.in.st_rf_inst_vld := storeag.io.out.rfVld
  wmb.io.in.st_wb_wmb_cmplt_grnt := storewb.io.out.wmbCmpltGrnt
  wmb.io.in.fromVB.create_grnt := vb.io.out.toWmb.create_grnt
  wmb.io.in.fromVB.empty := vb.io.out.toWmb.empty
  wmb.io.in.fromVB.entry_rcl_done := vb.io.out.toWmb.entry_rcl_done.asBools
  wmb.io.in.fromVB.write_req_hit_idx := vb.io.out.toWmb.write_req_hit_idx
  wmb.io.in.fromWmbCe.addr := wmbce.io.out.toWmb.addr
  wmb.io.in.fromWmbCe.atomic := wmbce.io.out.toWmb.atomic
  wmb.io.in.fromWmbCe.bkpta_data := sq.io.out.toWmb.bkptaData
  wmb.io.in.fromWmbCe.bkptb_data := sq.io.out.toWmb.bkptbData
  wmb.io.in.fromWmbCe.bytes_vld := wmbce.io.out.toWmb.bytes_vld
  wmb.io.in.fromWmbCe.bytes_vld_full := wmbce.io.out.toWmb.bytes_vld_full
  wmb.io.in.fromWmbCe.merge_ptr := wmbce.io.out.toWmb.merge_ptr
  wmb.io.in.fromWmbCe.same_dcache_line := wmbce.io.out.toWmb.same_dcache_line
  wmb.io.in.fromWmbCe.create_wmb_data_req := wmbce.io.out.toWmb.create_wmb_data_req
  wmb.io.in.fromWmbCe.create_wmb_dp_req := wmbce.io.out.toWmb.create_wmb_dp_req
  wmb.io.in.fromWmbCe.create_wmb_gateclk_en := wmbce.io.out.toWmb.create_wmb_gateclk_en
  wmb.io.in.fromWmbCe.create_wmb_req := wmbce.io.out.toWmb.create_wmb_req
  wmb.io.in.fromWmbCe.data128 := sq.io.out.toWmb.data128
  wmb.io.in.fromWmbCe.data_vld := wmbce.io.out.toWmb.data_vld
  wmb.io.in.fromWmbCe.dcache_inst := wmbce.io.out.toWmb.addr
  wmb.io.in.fromWmbCe.fence_mode := sq.io.out.toWmb.fenceMode
  wmb.io.in.fromWmbCe.hit_sq_pop_dcache_line := wmbce.io.out.toWmb.hit_sq_pop_dcache_line
  wmb.io.in.fromWmbCe.icc := wmbce.io.out.toWmb.icc
  wmb.io.in.fromWmbCe.iid := sq.io.out.toWmb.iid
  wmb.io.in.fromWmbCe.inst_flush := wmbce.io.out.toWmb.inst_flush
  wmb.io.in.fromWmbCe.inst_mode := wmbce.io.out.toWmb.inst_mode
  wmb.io.in.fromWmbCe.inst_size := wmbce.io.out.toWmb.inst_size
  wmb.io.in.fromWmbCe.inst_type := wmbce.io.out.toWmb.inst_type
  wmb.io.in.fromWmbCe.merge_data_addr_hit := wmbce.io.out.toWmb.merge_data_addr_hit
  wmb.io.in.fromWmbCe.merge_data_stall := wmbce.io.out.toWmb.merge_data_stall
  wmb.io.in.fromWmbCe.merge_en := wmbce.io.out.toWmb.merge_en
  wmb.io.in.fromWmbCe.merge_ptr := wmbce.io.out.toWmb.merge_ptr
  wmb.io.in.fromWmbCe.merge_wmb_req := wmbce.io.out.toWmb.merge_wmb_req
  wmb.io.in.fromWmbCe.merge_wmb_wait_not_vld_req := wmbce.io.out.toWmb.merge_wmb_wait_not_vld_req
  wmb.io.in.fromWmbCe.page_buf := wmbce.io.out.toWmb.page_buf
  wmb.io.in.fromWmbCe.page_ca := wmbce.io.out.toWmb.page_ca
  wmb.io.in.fromWmbCe.page_sec := wmbce.io.out.toWmb.page_sec
  wmb.io.in.fromWmbCe.page_share := wmbce.io.out.toWmb.page_share
  wmb.io.in.fromWmbCe.page_so := wmbce.io.out.toWmb.page_so
  wmb.io.in.fromWmbCe.page_wa := wmbce.io.out.toWmb.page_wa
  wmb.io.in.fromWmbCe.priv_mode := wmbce.io.out.toWmb.priv_mode
  wmb.io.in.fromWmbCe.read_dp_req := wmbce.io.out.toWmb.read_dp_req
  wmb.io.in.fromWmbCe.same_dcache_line := wmbce.io.out.toWmb.same_dcache_line
  wmb.io.in.fromWmbCe.sc_wb_vld := wmbce.io.out.toWmb.sc_wb_vld
  wmb.io.in.fromWmbCe.spec_fail := sq.io.out.toWmb.specFail
  wmb.io.in.fromWmbCe.sync_fence := wmbce.io.out.toWmb.sync_fence
  wmb.io.in.fromSQ.wmb_ce_update_dcache_dirty := sq.io.out.toWmb.updateDcacheMesi.dirty
  wmb.io.in.fromSQ.wmb_ce_update_dcache_share := sq.io.out.toWmb.updateDcacheMesi.share
  wmb.io.in.fromSQ.wmb_ce_update_dcache_valid := sq.io.out.toWmb.updateDcacheMesi.valid
  wmb.io.in.fromSQ.wmb_ce_update_dcache_way := sq.io.out.toWmb.updateDcacheWay
  wmb.io.in.fromWmbCe.vld := wmbce.io.out.toWmb.vld
  wmb.io.in.fromWmbCe.vstart_vld := sq.io.out.toWmb.vstartVld
  wmb.io.in.fromWmbCe.wb_cmplt_success := wmbce.io.out.toWmb.wb_cmplt_success
  wmb.io.in.fromWmbCe.wb_data_success := wmbce.io.out.toWmb.wb_data_success
  wmb.io.in.fromWmbCe.write_biu_dp_req := wmbce.io.out.toWmb.write_biu_dp_req
  wmb.io.in.fromWmbCe.write_imme := wmbce.io.out.toWmb.write_imme

  // &Instance("ct_lsu_wmb_ce","x_ct_lsu_wmb_ce"); @109
  wmbce.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  wmbce.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  wmbce.io.in.lm_sq_sc_fail := 0.U.asTypeOf(wmbce.io.in.lm_sq_sc_fail) //////todo: add lm
  wmbce.io.in.pad_yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  wmbce.io.in.rb_wmb_ce_hit_idx := rb.io.out.rb_wmb_ce_hit_idx
  wmbce.io.in.rtu_lsu_async_flush := io.in.fromRTU.lsu_async_flush
  wmbce.io.in.SQPop.addr := sq.io.out.toWmb.ce.popAddr
  wmbce.io.in.SQPop.atomic := sq.io.out.toWmb.ce.popAtomic
  wmbce.io.in.SQPop.bytes_vld := sq.io.out.toWmb.ce.popBytesVld
  wmbce.io.in.SQPop.icc := sq.io.out.toWmb.ce.popIcc
  wmbce.io.in.SQPop.inst_flush := sq.io.out.toWmb.ce.popInstFlush
  wmbce.io.in.SQPop.inst_mode := sq.io.out.toWmb.ce.popInstMode
  wmbce.io.in.SQPop.inst_size := sq.io.out.toWmb.ce.popInstSize
  wmbce.io.in.SQPop.inst_type := sq.io.out.toWmb.ce.popInstType
  wmbce.io.in.SQPop.page_buf := sq.io.out.toWmb.ce.popPageBuf
  wmbce.io.in.SQPop.page_ca := sq.io.out.toWmb.ce.popPageCa
  wmbce.io.in.SQPop.page_sec := sq.io.out.toWmb.ce.popPageSec
  wmbce.io.in.SQPop.page_share := sq.io.out.toWmb.ce.popPageShare
  wmbce.io.in.SQPop.page_so := sq.io.out.toWmb.ce.popPageSo
  wmbce.io.in.SQPop.page_wa := sq.io.out.toWmb.ce.popPageWa
  wmbce.io.in.SQPop.priv_mode := sq.io.out.toWmb.ce.popPrivMode
  wmbce.io.in.SQPop.sq_ptr := sq.io.out.toWmb.ce.popPtr.asBools
  wmbce.io.in.SQPop.sync_fence := sq.io.out.toWmb.ce.popSyncFence
  wmbce.io.in.SQPop.wo_st := sq.io.out.toWmb.ce.popWoSt
  wmbce.io.in.fromSQ.wmb_ce_dcache_valid := sq.io.out.toWmb.dcacheValid
  wmbce.io.in.fromSQ.wmb_ce_create_hit_rb_idx := sq.io.out.toWmb.createHitRbIdx
  wmbce.io.in.fromSQ.wmb_ce_dcache_share := sq.io.out.toWmb.dcacheShare
  wmbce.io.in.fromWmb.create_dp_vld := wmb.io.out.toWmbCe.create_dp_vld
  wmbce.io.in.fromWmb.pop_vld := wmb.io.out.toWmbCe.pop_vld
  wmbce.io.in.fromWmb.create_vld := wmb.io.out.toWmbCe.create_vld
  wmbce.io.in.fromWmb.create_gateclk_en := wmb.io.out.toWmbCe.create_gateclk_en
  wmbce.io.in.fromWmb.create_merge := wmb.io.out.toWmbCe.create_merge
  wmbce.io.in.fromWmb.create_merge_ptr := wmb.io.out.toWmbCe.create_merge_ptr
  wmbce.io.in.fromWmb.create_same_dcache_line := wmb.io.out.toWmbCe.create_same_dcache_line
  wmbce.io.in.fromWmb.create_stall := wmb.io.out.toWmbCe.create_stall
  wmbce.io.in.fromWmb.entry_vld := wmb.io.out.wmb_entry_vld

  //==========================================================
  //                       WB Stage
  //==========================================================
  // &Instance("ct_lsu_ld_wb","x_ct_lsu_ld_wb"); @114
  loadwb.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  loadwb.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  loadwb.io.in.ctrl_ld_clk := ctrl.io.out.ctrlClk.ldClk
  loadwb.io.in.fromHad := io.in.ld_wb.fromHad
  loadwb.io.in.ld_da_addr := loadda.io.out.ld_da_addr
  loadwb.io.in.ld_da_bkpta_data := loadda.io.out.ld_da_bkpta_data
  loadwb.io.in.ld_da_bkptb_data := loadda.io.out.ld_da_bkptb_data
  loadwb.io.in.ld_da_iid := loadda.io.out.ld_da_iid
  loadwb.io.in.ld_da_inst_vfls := loadda.io.out.ld_da_inst_vfls
  loadwb.io.in.ld_da_inst_vld := loadda.io.out.ld_da_inst_vld
  loadwb.io.in.ld_da_preg := loadda.io.out.ld_da_preg
  loadwb.io.in.fromDA := loadda.io.out.toWB
  loadwb.io.in.ld_da_vreg := loadda.io.out.ld_da_vreg
  io.out.ld_wb.toHad := loadwb.io.out.toHad
  io.out.ld_wb.toIDU := loadwb.io.out.toIDU
  io.out.ld_wb.toRTU := loadwb.io.out.toRTU
  loadwb.io.in.fromPad.yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  loadwb.io.in.fromRB.bkpta_data := rb.io.out.toLoadWB.bkpta_data
  loadwb.io.in.fromRB.bkptb_data := rb.io.out.toLoadWB.bkptb_data
  loadwb.io.in.fromRB.bus_err := rb.io.out.toLoadWB.bus_err
  loadwb.io.in.fromRB.bus_err_addr := rb.io.out.toLoadWB.bus_err_addr
  loadwb.io.in.fromRB.cmplt_req := rb.io.out.toLoadWB.cmplt_req
  loadwb.io.in.fromRB.data := rb.io.out.toLoadWB.data
  loadwb.io.in.fromRB.data_iid := rb.io.out.toLoadWB.data_iid
  loadwb.io.in.fromRB.data_req := rb.io.out.toLoadWB.data_req
  loadwb.io.in.fromRB.expt_gateclk := rb.io.out.toLoadWB.expt_gateclk
  loadwb.io.in.fromRB.expt_vld := rb.io.out.toLoadWB.expt_vld
  loadwb.io.in.fromRB.iid := rb.io.out.toLoadWB.iid
  loadwb.io.in.fromRB.inst_vfls := rb.io.out.toLoadWB.inst_vfls
  loadwb.io.in.fromRB.preg := rb.io.out.toLoadWB.preg
  loadwb.io.in.fromRB.preg_sign_sel := rb.io.out.toLoadWB.preg_sign_sel
  loadwb.io.in.fromRB.vreg := rb.io.out.toLoadWB.vreg
  loadwb.io.in.fromRB.vreg_sign_sel := rb.io.out.toLoadWB.vreg_sign_sel
  loadwb.io.in.fromRTU.yy_xx_flush := io.in.fromRTU.yy_xx_flush
  loadwb.io.in.vmb_ld_wb_data_req := false.B
  loadwb.io.in.fromWmb := wmb.io.out.toLoadWB
  //////todo: check it, vmb

  // &Instance("ct_lsu_st_wb","x_ct_lsu_st_wb"); @115
  storewb.io.in.cp0In.icgEn := io.in.fromCp0.lsu_icg_en
  storewb.io.in.cp0In.clkEn := io.in.fromCp0.yy_clk_en
  storewb.io.in.ctrlStClk := ctrl.io.out.ctrlClk.stClk  //////todo: distinguish stclk and ldclk in future
  io.out.st_wb.toRTU := storewb.io.out.toRtu
  //storewb.io.in.pad todo: add it
  storewb.io.in.rtuFlush := io.in.fromRTU.yy_xx_flush
  storewb.io.in.bkptaData := storeda.io.out.bkptaData
  storewb.io.in.bkptbData := storeda.io.out.bkptbData
  storewb.io.in.iid := storeda.io.out.iid
  storewb.io.in.instVld := storeda.io.out.instVld
  storewb.io.in.stDaIn := storeda.io.out.toSq.wb
  storewb.io.in.wmbIn.bkptaData := wmb.io.out.toStoreWB.bkpta_data
  storewb.io.in.wmbIn.bkptbData := wmb.io.out.toStoreWB.bkptb_data
  storewb.io.in.wmbIn.cmpltReq := wmb.io.out.toStoreWB.cmplt_req
  storewb.io.in.wmbIn.iid := wmb.io.out.toStoreWB.iid
  storewb.io.in.wmbIn.instFlush := wmb.io.out.toStoreWB.inst_flush
  storewb.io.in.wmbIn.specFail := wmb.io.out.toStoreWB.spec_fail

  //==========================================================
  //                Linefill/Victim buffer
  //==========================================================
  // &Instance("ct_lsu_lfb","x_ct_lsu_lfb"); @120
  lfb.io.in.biuIn.rData  := io.in.rb.fromBiu.r_data
  lfb.io.in.biuIn.rLast  := io.in.rb.fromBiu.r_last
  lfb.io.in.biuIn.rVld   := io.in.rb.fromBiu.r_vld
  lfb.io.in.biuIn.rId    := io.in.rb.fromBiu.r_id
  lfb.io.in.biuIn.rResp  := io.in.rb.fromBiu.r_resp
  // todo shared input bundle delete module name prefix
  lfb.io.in.busArbIn.rb_ar_sel  := busarb.io.out.bus_arb_rb_ar_sel
  lfb.io.in.busArbIn.pfu_ar_sel := busarb.io.out.bus_arb_pfu_ar_sel

  lfb.io.in.cp0In.lsuDcacheEn  := io.in.fromCp0.lsu_dcache_en
  lfb.io.in.cp0In.lsuIcgEn     := io.in.fromCp0.lsu_icg_en
  lfb.io.in.cp0In.yyClkEn      := io.in.fromCp0.yy_clk_en
  lfb.io.in.dcache_arb_lfb_ld_grnt := dcachearb.io.out.dcache_arb_lfb_ld_grnt
  lfb.io.in.ldDaIn.idx             := loadda.io.out.ld_da_idx
  lfb.io.in.ldDaIn.discardGrnt     := loadda.io.out.toLfb.discard_grnt
  lfb.io.in.ldDaIn.setWakeupQueue  := loadda.io.out.toLfb.set_wakeup_queue
  lfb.io.in.ldDaIn.wakeupQueueNext.valid := loadda.io.out.toLfb.wakeup_queue_next(12)
  lfb.io.in.ldDaIn.wakeupQueueNext.bits := loadda.io.out.toLfb.wakeup_queue_next(11,0)
  // todo pfu
  lfb.io.in.lmIn := 0.U.asTypeOf(lfb.io.in.lmIn) //////todo: add lm
  lfb.io.in.pfuIn := 0.U.asTypeOf(lfb.io.in.pfuIn) //////todo: add pfu
  lfb.io.in.snqIn := 0.U.asTypeOf(lfb.io.in.snqIn) //////todo: add snq
  lfb.io.in.rbIn.biuReqAddr           := rb.io.out.toLfb.reqAddr
  lfb.io.in.rbIn.addrTto4             := rb.io.out.toLfb.addr_tto4
  lfb.io.in.rbIn.atomic               := rb.io.out.toLfb.atomic
  lfb.io.in.rbIn.boundaryDepdWakeup   := rb.io.out.toLfb.boundary_depd_wakeup
  lfb.io.in.rbIn.createDpVld          := rb.io.out.toLfb.create_dp_vld
  lfb.io.in.rbIn.createGateclkEn      := rb.io.out.toLfb.create_gateclk_en
  lfb.io.in.rbIn.createReq            := rb.io.out.toLfb.create_req
  lfb.io.in.rbIn.createVld            := rb.io.out.toLfb.create_vld
  lfb.io.in.rbIn.depd                 := rb.io.out.toLfb.depd
  lfb.io.in.rbIn.ldamo                := rb.io.out.toLfb.ldamo
  lfb.io.in.rtuFlush := io.in.fromRTU.yy_xx_flush
  lfb.io.in.stDaAddr := storeda.io.out.addr
  lfb.io.in.vbIn := vb.io.out.toLfb
  lfb.io.in.wmbIn.readAddr := wmb.io.out.wmb_read_req_addr
  lfb.io.in.wmbIn.writeAddr := wmb.io.out.wmb_write_req_addr


  // Victim Buffer: decide Dcache line replace
  // &Instance("ct_lsu_vb","x_ct_lsu_vb"); @121
  vb.io.in.biuIn.bId   := io.in.rb.fromBiu. b_id
  vb.io.in.biuIn.bVld  := io.in.rb.fromBiu. b_vld
  vb.io.in.busArbIn.wGrnt := busarb.io.out.bus_arb_vb_w_grnt
  vb.io.in.busArbIn.awGrnt := busarb.io.out.bus_arb_vb_aw_grnt
  vb.io.in.cp0In.lsuIcgEn := io.in.fromCp0.lsu_icg_en
  vb.io.in.cp0In.yyClkEn  := io.in.fromCp0.yy_clk_en
  vb.io.in.dcacheArbIn.ldGrnt := dcachearb.io.out.dcache_arb_vb_ld_grnt
  vb.io.in.dcacheArbIn.stGrnt := dcachearb.io.out.dcache_arb_vb_st_grnt
  vb.io.in.iccIn := icc.io.out.toVb
  vb.io.in.ldDaSnqDataReissue := loadda.io.out.ld_da_vb_snq_data_reissue
  vb.io.in.lfbIn := lfb.io.out.toVb
  vb.io.in.padYyIcgScanEn := io.in.fromPad.yy_icg_scan_en
  // todo lsu_special_clk
  // todo pfu
  // todo snq = vb.io.out.toSnq
  vb.io.in.snqIn := 0.U.asTypeOf(vb.io.in.snqIn)
  vb.io.in.pfuBiuReqAddr := 0.U.asTypeOf(vb.io.in.pfuBiuReqAddr)
  vb.io.in.vbSdbIn := 0.U.asTypeOf(vb.io.in.vbSdbIn) //////todo: add sdb

  vb.io.in.stDaIn.dirty        := storeda.io.out.toVb.dirty
  vb.io.in.stDaIn.miss         := storeda.io.out.toVb.miss
  vb.io.in.stDaIn.replaceDirty := storeda.io.out.toVb.replaceDirty
  vb.io.in.stDaIn.replaceValid := storeda.io.out.toVb.replaceValid
  vb.io.in.stDaIn.replaceWay   := storeda.io.out.toVb.replaceWay
  vb.io.in.stDaIn.way          := storeda.io.out.toVb.way
  vb.io.in.stDaIn.eccErr             :=  storeda.io.out.toSq.vbEccErr
  vb.io.in.stDaIn.eccStall           :=  storeda.io.out.toSq.vbEccStall
  vb.io.in.stDaIn.feedbackDddrTto14  :=  storeda.io.out.toSq.vbFeedbackDddrTto14
  vb.io.in.stDaIn.tagReissue         :=  storeda.io.out.toSq.vbTagReissue
  vb.io.in.stDaIn.hit                :=  storeda.io.out.toRb.dcacheHit

  vb.io.in.wmbIn.toEntry.addrTto6        := wmb.io.out.toVB.addr_tto6
  vb.io.in.wmbIn.toEntry.inv             := wmb.io.out.toVB.inv
  vb.io.in.wmbIn.toEntry.setWayMode      := wmb.io.out.toVB.set_way_mode
  vb.io.in.wmbIn.toEntry.writePtrEncode  := wmb.io.out.wmb_write_ptr_encode
  vb.io.in.wmbIn.toEntry.writeReqAddr    := wmb.io.out.wmb_write_req_addr
  vb.io.in.wmbIn.createDpVld     := wmb.io.out.toVB.create_dp_vld
  vb.io.in.wmbIn.createGateclkEn := wmb.io.out.toVB.create_gateclk_en
  vb.io.in.wmbIn.createReq       := wmb.io.out.toVB.create_req
  vb.io.in.wmbIn.createVld       := wmb.io.out.toVB.create_vld
  vb.io.in.rbBiuReqAddr := rb.io.out.toBiu.req_addr


  // todo sdb data @5486 ct_lsu_vb_sdb_data, is VB DATA ENTRY INST
  // &Instance("ct_lsu_amr","x_ct_lsu_amr"); @140

  amr.io.in.cp0In.amr       := io.in.fromCp0.amr
  amr.io.in.cp0In.amr2      := io.in.fromCp0.amr2
  amr.io.in.cp0In.icgEn     := io.in.fromCp0.lsu_icg_en
  amr.io.in.cp0In.noOpReq   := io.in.fromCp0.lsu_no_op_req
  amr.io.in.cp0In.clkEn     := io.in.fromCp0.yy_clk_en
  amr.io.in.iccIdle         := icc.io.out.iccIdle
  amr.io.in.wmbIn.addr      := wmbce.io.out.toWmb.addr
  amr.io.in.wmbIn.bytesVld  := wmbce.io.out.toWmb.bytes_vld
  amr.io.in.wmbIn.caStInst  := wmbce.io.out.toWmb.ca_st_inst
  amr.io.in.wmbIn.popVld    := wmb.io.out.toWmbCe.pop_vld
  amr.io.in.wmbIn.vld       := wmbce.io.out.toWmb.vld

  // &Instance("ct_lsu_icc","x_ct_lsu_icc"); @141

  icc.io.in.cp0In.dcacheClr        := io.in.fromCp0.lsu_dcache_clr
  icc.io.in.cp0In.dcacheInv        := io.in.fromCp0.lsu_dcache_inv
  icc.io.in.cp0In.dcacheReadIndex        := io.in.fromCp0.lsu_dcache_read_index
  icc.io.in.cp0In.dcacheReadWay        := io.in.fromCp0.lsu_dcache_read_way
  icc.io.in.cp0In.dcacheReadLdTag        := io.in.fromCp0.lsu_dcache_read_ld_tag
  icc.io.in.cp0In.dcacheReadReq        := io.in.fromCp0.lsu_dcache_read_req
  icc.io.in.cp0In.dcacheReadStTag        := io.in.fromCp0.lsu_dcache_read_st_tag
  icc.io.in.cp0In.icgEn  := io.in.fromCp0.lsu_icg_en
  icc.io.in.dcacheArbIccLdGrnt := dcachearb.io.out.dcache_arb_icc_ld_grnt
  icc.io.in.ldDaIn.readData  := loadda.io.out.ld_da_icc_read_data
  icc.io.in.ldDaIn.snqBorrow := loadda.io.out.ld_da_snq_borrow_icc
  icc.io.in.empty.lfb := lfb.io.out.lfbEmpty
  icc.io.in.empty.rb  := rb.io.out.rb_empty
  icc.io.in.empty.snq := 0.U.asTypeOf(icc.io.in.empty.snq)
  icc.io.in.empty.sq  := sq.io.out.toCtrl.empty
  icc.io.in.sqIn := sq.io.out.toIcc
  icc.io.in.empty.vb  := vb.io.out.vbEmpty
  icc.io.in.empty.wmb := wmb.io.out.wmb_empty
  icc.io.in.pfuReady := 0.U.asTypeOf(icc.io.in.pfuReady) //////todo: add pfu
  icc.io.in.vbCreateGrnt := vb.io.out.iccCreateGrnt
  icc.io.in.stDaIn := storeda.io.out.toIcc

  // &Instance("ct_lsu_ctrl","x_ct_lsu_ctrl"); @142
  ctrl.io.in.cp0In.dcachePrefDist := io.in.fromCp0.lsu_dcache_pref_dist
  ctrl.io.in.cp0In.l2PrefDist     := io.in.fromCp0.lsu_l2_pref_dist
  ctrl.io.in.cp0In.icgEn          := io.in.fromCp0.lsu_icg_en
  ctrl.io.in.cp0In.yyClkEn        := io.in.fromCp0.yy_clk_en
  ctrl.io.in.dcacheArbIn.ldDcBorrowGate := dcachearb.io.out.toLoadDC.borrow_vld_gate
  ctrl.io.in.dcacheArbIn.stDcBorrowGate := dcachearb.io.out.toStoreDC.borrow_vld_gate
  ctrl.io.in.rfPipeSel := io.in.ctrl.rfPipeIn
  ctrl.io.in.vmbCreateGateEn := io.in.ctrl.idu_lsu_vmb_create_gateclk_enVec
  ctrl.io.in.ldDaIn.instVld                 := loadda.io.out.toCB.ld_inst_vld
  ctrl.io.in.ldDaIn.borrow_vld              := loadda.io.out.toCtrl.borrow_vld
  ctrl.io.in.ldDaIn.ecc_wakeup              := loadda.io.out.toCtrl.ecc_wakeup
  ctrl.io.in.ldDaIn.idu_already_da          := loadda.io.out.toCtrl.idu_already_da
  ctrl.io.in.ldDaIn.idu_bkpta_data          := loadda.io.out.toCtrl.idu_bkpta_data
  ctrl.io.in.ldDaIn.idu_bkptb_data          := loadda.io.out.toCtrl.idu_bkptb_data
  ctrl.io.in.ldDaIn.idu_boundary_gateclk_en := loadda.io.out.toCtrl.idu_boundary_gateclk_en
  ctrl.io.in.ldDaIn.idu_pop_entry           := loadda.io.out.toCtrl.idu_pop_entry
  ctrl.io.in.ldDaIn.idu_pop_vld             := loadda.io.out.toCtrl.idu_pop_vld
  ctrl.io.in.ldDaIn.idu_rb_full             := loadda.io.out.toCtrl.idu_rb_full
  ctrl.io.in.ldDaIn.idu_secd                := loadda.io.out.toCtrl.idu_secd
  ctrl.io.in.ldDaIn.idu_spec_fail           := loadda.io.out.toCtrl.idu_spec_fail
  ctrl.io.in.ldDaIn.idu_wait_fence          := loadda.io.out.toCtrl.idu_wait_fence
  ctrl.io.in.ldDaIn.rb_full_gateclk_en      := loadda.io.out.toCtrl.rb_full_gateclk_en
  ctrl.io.in.ldDaIn.special_gateclk_en      := loadda.io.out.toCtrl.special_gateclk_en
  ctrl.io.in.ldDaIn.wait_fence_gateclk_en   := loadda.io.out.toCtrl.wait_fence_gateclk_en

  ctrl.io.in.ldDcIn.borrowVld         := loaddc.io.out.toDA.borrow_vld
  ctrl.io.in.ldDcIn.iduLqFull         := loaddc.io.out.ld_dc_idu_lq_full.asUInt
  ctrl.io.in.ldDcIn.iduTlbBusy        := loaddc.io.out.ld_dc_idu_tlb_busy.asUInt
  ctrl.io.in.ldDcIn.immeWakeup        := loaddc.io.out.ld_dc_imme_wakeup.asUInt
  ctrl.io.in.ldDcIn.instVld           := loaddc.io.out.toDA.inst_vld
  ctrl.io.in.ldDcIn.lqFullGateclkEn   := loaddc.io.out.toLQ.full_gateclk_en
  ctrl.io.in.ldDcIn.tlbBusyGateclkEn  := loaddc.io.out.ld_dc_tlb_busy_gateclk_en

  ctrl.io.in.ldWbIn.dataVld := loadwb.io.out.ld_wb_data_vld
  ctrl.io.in.ldWbIn.instVld := loadwb.io.out.ld_wb_inst_vld

  ctrl.io.in.lfbIn.depdWakeUp := lfb.io.out.lfbDepdWakeup
  ctrl.io.in.lfbIn.empty      := lfb.io.out.lfbEmpty
  ctrl.io.in.lfbIn.popDepdFf  := lfb.io.out.popDepdFf

  ctrl.io.in.rbIn.ldWbCmpltReq := rb.io.out.toLoadWB.cmplt_req
  ctrl.io.in.rbIn.empty        := rb.io.out.rb_empty
  ctrl.io.in.rbIn.ldWbDataReq  := rb.io.out.toLoadWB.data_req

  ctrl.io.in.stEx1InstVld := storeex1.io.out.sdEx1InstVld

  ctrl.io.in.stAgIn.instVld           := storeag.io.out.toDc.instVld
  ctrl.io.in.stAgIn.stallOri          := storeag.io.out.toDc.stallOri
  ctrl.io.in.stAgIn.stallRestartEntry := storeag.io.out.toDc.stallRestartEntry

  ctrl.io.in.stDaIn.instVld              := storeda.io.out.instVld
  ctrl.io.in.stDaIn.rbcreateGateclkEn    := storeda.io.out.toRb.createGateclkEn
  ctrl.io.in.stDaIn.waitFenceGateclkEn   := storeda.io.out.toSq.waitFenceGateclkEn
  ctrl.io.in.stDaIn.eccWakeup            := storeda.io.out.toCtrl. eccWakeup
  ctrl.io.in.stDaIn.alreadyDa            := storeda.io.out.toCtrl. alreadyDa
  ctrl.io.in.stDaIn.bkptaData            := storeda.io.out.toCtrl. bkptaData
  ctrl.io.in.stDaIn.bkptbData            := storeda.io.out.toCtrl. bkptbData
  ctrl.io.in.stDaIn.boundaryGateclkEn    := storeda.io.out.toCtrl. boundaryGateclkEn
  ctrl.io.in.stDaIn.popEntry             := storeda.io.out.toCtrl. popEntry
  ctrl.io.in.stDaIn.popVld               := storeda.io.out.toCtrl. popVld
  ctrl.io.in.stDaIn.rbFull               := storeda.io.out.toCtrl. rbFull
  ctrl.io.in.stDaIn.secd                 := storeda.io.out.toCtrl. secd
  ctrl.io.in.stDaIn.specFail             := storeda.io.out.toCtrl. specFail
  ctrl.io.in.stDaIn.waitFence            := storeda.io.out.toCtrl. waitFence
  ctrl.io.in.stDaIn.rbFullGateclkEn      := storeda.io.out.toCtrl. rbFullGateclkEn
  ctrl.io.in.stDaIn.borrowVld            := storeda.io.out.toCtrl. borrowVld

  ctrl.io.in.stDcIn.borrowVld        := storedc.io.out.toDa.toSqDa.borrowVld
  ctrl.io.in.stDcIn.iduSqFull        := storedc.io.out.toDa.iduSqFull
  ctrl.io.in.stDcIn.iduTlbBusy       := storedc.io.out.toDa.iduTlbBusy
  ctrl.io.in.stDcIn.instVld          := storedc.io.out.toDa.instVld
  ctrl.io.in.stDcIn.sqFullGateclkEn  := storedc.io.out.toSq.sqFullGateclkEn
  ctrl.io.in.stDcIn.tlbBusyGateclkEn := storedc.io.out.toCtrl.tlbBusyGateclkEn
  ctrl.io.in.stDcIn.immeWakeup       := storedc.io.out.toCtrl.immeWakeup
  ctrl.io.in.stWbInstVld := storewb.io.out.instVld

  ctrl.io.in.vbEmpty := vb.io.out.vbEmpty

  ctrl.io.in.vmbIn := DontCare // todo

  ctrl.io.in.wmbIn.depdWakeup   := wmb.io.out.wmb_depd_wakeup.asUInt
  ctrl.io.in.wmbIn.empty        := wmb.io.out.wmb_empty
  ctrl.io.in.wmbIn.ldWbDataReq  := wmb.io.out.toLoadWB.data_req
  ctrl.io.in.wmbIn.noOp         := wmb.io.out.wmb_no_op
  ctrl.io.in.wmbIn.stWbCmpltReq := wmb.io.out.toStoreWB.cmplt_req
  ctrl.io.in.wmbIn.writeReqIcc  := wmb.io.out.wmb_write_req_icc

  ctrl.io.in.lsuExWait.ldAg.old := loadag.io.out.toIDU.ag_wait_old.asUInt
  ctrl.io.in.lsuExWait.ldAg.oldGateEn := loadag.io.out.toIDU.ag_wait_old_gateclk_en
  ctrl.io.in.lsuExWait.ldDa.old := loadda.io.out.toIDU.ld_da_wait_old
  ctrl.io.in.lsuExWait.ldDa.oldGateEn := loadda.io.out.toIDU.ld_da_wait_old_gateclk_en
  ctrl.io.in.lsuExWait.stAg.old := storeag.io.out.toIdu.waitOld
  ctrl.io.in.lsuExWait.stAg.oldGateEn := storeag.io.out.toIdu.waitOldGateclkEn
  ctrl.io.in.lsuHasFence := rb.io.out.lsu_has_fence
  ctrl.io.in.sqIn := sq.io.out.toCtrl
  ctrl.io.in.lbfDepdWakeUp := lfb.io.out.lfbDepdWakeup
  ctrl.io.in.iccVbCreateGateEn := icc.io.out.toVb.createGateclkEn  //////gateclken
  ctrl.io.in.ldAgIn.instVld := loadag.io.out.toDC.inst_vld
  ctrl.io.in.ldAgIn.stallOri := loadag.io.out.toDC.stall_ori ////// wrong to in loadag?
  ctrl.io.in.ldAgIn.stallRestartEntry := loadag.io.out.toDC.stall_restart_entry.asUInt
  ctrl.io.in.mmuTlbWakep := io.in.fromMMU.mmu_lsu_tlb_wakeup
  ctrl.io.in.pfuIn := 0.U.asTypeOf(ctrl.io.in.pfuIn) //////todo: add pfu

  io.out.ctrl.toIDU := ctrl.io.out.toIdu

  // &Instance("ct_lsu_bus_arb","x_ct_lsu_bus_arb"); @143

  // todo bus arb
  busarb.io.in.fromBiu := io.in.bus_arb.fromBiu
  busarb.io.in.fromCp0.lsu_icg_en := io.in.fromCp0.lsu_icg_en
  busarb.io.in.fromCp0.yy_clk_en := io.in.fromCp0.yy_clk_en
  io.out.bus_arb.toBiu.ar := busarb.io.out.ar
  io.out.bus_arb.toBiu.st_aw := busarb.io.out.st_aw
  io.out.bus_arb.toBiu.st_w := busarb.io.out.st_w
  io.out.bus_arb.toBiu.lsu_biu_ar_req_gate := busarb.io.out.lsu_biu_ar_req_gate
  io.out.bus_arb.toBiu.lsu_biu_aw_req_gate := busarb.io.out.lsu_biu_aw_req_gate
  io.out.bus_arb.toBiu.lsu_biu_aw_st_unique := busarb.io.out.lsu_biu_aw_st_unique
  io.out.bus_arb.toBiu.lsu_biu_aw_vict_unique := busarb.io.out.lsu_biu_aw_vict_unique
  io.out.bus_arb.toBiu.vict_aw := busarb.io.out.vict_aw
  io.out.bus_arb.toBiu.vict_w := busarb.io.out.vict_w
  busarb.io.in.pad_yy_icg_scan_en := io.in.fromPad.yy_icg_scan_en
  busarb.io.in.pfu_ar := 0.U.asTypeOf(busarb.io.in.pfu_ar) //////todo: add pfu
  busarb.io.in.pfu_biu_ar_req_gateclk_en := 0.U.asTypeOf(busarb.io.in.pfu_biu_ar_req_gateclk_en) //////todo: add pfu
  busarb.io.in.rb_ar.addr := rb.io.out.toBiu.ar_addr
  busarb.io.in.rb_ar.bar := rb.io.out.toBiu.ar_bar
  busarb.io.in.rb_ar.burst := rb.io.out.toBiu.ar_burst
  busarb.io.in.rb_ar.cache := rb.io.out.toBiu.ar_cache
  busarb.io.in.rb_ar.domain := rb.io.out.toBiu.ar_domain
  busarb.io.in.rb_ar.dp_req := rb.io.out.toBiu.ar_dp_req
  busarb.io.in.rb_ar.id := rb.io.out.toBiu.ar_id
  busarb.io.in.rb_ar.len := rb.io.out.toBiu.ar_len
  busarb.io.in.rb_ar.lock := rb.io.out.toBiu.ar_lock
  busarb.io.in.rb_ar.prot := rb.io.out.toBiu.ar_prot
  busarb.io.in.rb_ar.req := rb.io.out.toBiu.ar_req
  busarb.io.in.rb_biu_ar_req_gateclk_en := rb.io.out.toBiu.ar_req_gateclk_en
  busarb.io.in.rb_ar.size := rb.io.out.toBiu.ar_size
  busarb.io.in.rb_ar.snoop := rb.io.out.toBiu.ar_snoop
  busarb.io.in.rb_ar.user := rb.io.out.toBiu.ar_user
  busarb.io.in.vb_aw.addr := vb.io.out.toBiu.aw_addr
  busarb.io.in.vb_aw.bar := vb.io.out.toBiu.aw_bar
  busarb.io.in.vb_aw.burst := vb.io.out.toBiu.aw_burst
  busarb.io.in.vb_aw.cache := vb.io.out.toBiu.aw_cache
  busarb.io.in.vb_aw.domain := vb.io.out.toBiu.aw_domain
  busarb.io.in.vb_aw.dp_req := vb.io.out.toBiu.aw_dp_req
  busarb.io.in.vb_aw.id := vb.io.out.toBiu.aw_id
  busarb.io.in.vb_aw.len := vb.io.out.toBiu.aw_len
  busarb.io.in.vb_aw.lock := vb.io.out.toBiu.aw_lock
  busarb.io.in.vb_aw.prot := vb.io.out.toBiu.aw_prot
  busarb.io.in.vb_aw.req := vb.io.out.toBiu.aw_req
  busarb.io.in.vb_biu_aw_req_gateclk_en := vb.io.out.toBiu.aw_req_gateclk_en
  busarb.io.in.vb_aw.size := vb.io.out.toBiu.aw_size
  busarb.io.in.vb_aw.snoop := vb.io.out.toBiu.aw_snoop
  busarb.io.in.vb_biu_aw_unique := vb.io.out.toBiu.aw_unique
  busarb.io.in.vb_aw.user := vb.io.out.toBiu.aw_user
  busarb.io.in.vb_w.data := vb.io.out.toBiu.w_data
  busarb.io.in.vb_biu_w_id := vb.io.out.toBiu.w_id
  busarb.io.in.vb_w.last := vb.io.out.toBiu.w_last
  busarb.io.in.vb_biu_w_req := vb.io.out.toBiu.w_req
  busarb.io.in.vb_w.strb := vb.io.out.toBiu.w_strb
  busarb.io.in.vb_w.vld := vb.io.out.toBiu.w_vld
  busarb.io.in.vb_w.wns := 0.U.asTypeOf(busarb.io.in.vb_w.wns) //////todo: this signal is not exist in C910
  busarb.io.in.wmb_ar.addr := wmb.io.out.toBiu.ar_addr
  busarb.io.in.wmb_ar.bar := wmb.io.out.toBiu.ar_bar
  busarb.io.in.wmb_ar.burst := wmb.io.out.toBiu.ar_burst
  busarb.io.in.wmb_ar.cache := wmb.io.out.toBiu.ar_cache
  busarb.io.in.wmb_ar.domain := wmb.io.out.toBiu.ar_domain
  busarb.io.in.wmb_ar.dp_req := wmb.io.out.toBiu.ar_dp_req
  busarb.io.in.wmb_ar.id := wmb.io.out.toBiu.ar_id
  busarb.io.in.wmb_ar.len := wmb.io.out.toBiu.ar_len
  busarb.io.in.wmb_ar.lock := wmb.io.out.toBiu.ar_lock
  busarb.io.in.wmb_ar.prot := wmb.io.out.toBiu.ar_prot
  busarb.io.in.wmb_ar.req := wmb.io.out.toBiu.ar_req
  busarb.io.in.wmb_biu_ar_req_gateclk_en := wmb.io.out.toBiu.ar_req_gateclk_en
  busarb.io.in.wmb_ar.size := wmb.io.out.toBiu.ar_size
  busarb.io.in.wmb_ar.snoop := wmb.io.out.toBiu.ar_snoop
  busarb.io.in.wmb_ar.user := wmb.io.out.toBiu.ar_user
  busarb.io.in.wmb_aw.addr := wmb.io.out.toBiu.aw_addr
  busarb.io.in.wmb_aw.bar := wmb.io.out.toBiu.aw_bar
  busarb.io.in.wmb_aw.burst := wmb.io.out.toBiu.aw_burst
  busarb.io.in.wmb_aw.cache := wmb.io.out.toBiu.aw_cache
  busarb.io.in.wmb_aw.domain := wmb.io.out.toBiu.aw_domain
  busarb.io.in.wmb_aw.dp_req := wmb.io.out.toBiu.aw_dp_req
  busarb.io.in.wmb_aw.id := wmb.io.out.toBiu.aw_id
  busarb.io.in.wmb_aw.len := wmb.io.out.toBiu.aw_len
  busarb.io.in.wmb_aw.lock := wmb.io.out.toBiu.aw_lock
  busarb.io.in.wmb_aw.prot := wmb.io.out.toBiu.aw_prot
  busarb.io.in.wmb_aw.req := wmb.io.out.toBiu.aw_req
  busarb.io.in.wmb_biu_aw_req_gateclk_en := wmb.io.out.toBiu.aw_req_gateclk_en
  busarb.io.in.wmb_aw.size := wmb.io.out.toBiu.aw_size
  busarb.io.in.wmb_aw.snoop := wmb.io.out.toBiu.aw_snoop
  busarb.io.in.wmb_aw.user := wmb.io.out.toBiu.aw_user
  busarb.io.in.wmb_w.data := wmb.io.out.toBiu.w_data
  busarb.io.in.wmb_biu_w_id := wmb.io.out.toBiu.w_id
  busarb.io.in.wmb_w.last := wmb.io.out.toBiu.w_last
  busarb.io.in.wmb_biu_w_req := wmb.io.out.toBiu.w_req
  busarb.io.in.wmb_w.strb := wmb.io.out.toBiu.w_strb
  busarb.io.in.wmb_w.vld := wmb.io.out.toBiu.w_vld
  busarb.io.in.wmb_w.wns := wmb.io.out.toBiu.w_wns

}