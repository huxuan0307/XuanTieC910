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
import Core.LsuConfig.LSIQ_ENTRY
import chisel3._
import chisel3.util._

class LSUInput extends Bundle {
  val ld_ag = new Bundle{
    val fromCp0 = new Bundle{
      val lsu_cb_aclr_dis = Bool()
      val lsu_da_fwd_dis = Bool()
      val lsu_dcache_en = Bool()
      val lsu_icg_en = Bool()
      val lsu_mm = Bool()
      val yy_clk_en = Bool()
    }
    val fromMMU = new Bundle{
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
    val fromPad = new Bundle{
      val yy_icg_scan_en = Bool()
    }
    val fromRTU = new Bundle{
      val yy_xx_commit = Vec(3, Bool())
      val yy_xx_commit_iid = Vec(3, UInt(7.W))
      val yy_xx_flush = Bool()
    }
    val fromRF_pipe3 = new Pipe3In
  }
  val st_ag = new Bundle{
    val cp0In    = new Cp0ToStAg
    val rfIn     = new RfPipe4
    val mmuIn    = new MmuToStAg
    val rtuIn    = new RtuToStAg
  }
  val sd_ex1 = new StoreEx1In
  val ld_dc = new Bundle{
    val fromHad = new Bundle{
      val yy_xx_bkpta_base = UInt(40.W)
      val yy_xx_bkpta_mask = UInt(8.W)
      val yy_xx_bkpta_rc   = Bool()
      val yy_xx_bkptb_base = UInt(40.W)
      val yy_xx_bkptb_mask = UInt(8.W)
      val yy_xx_bkptb_rc   = Bool()
    }
    val fromMMU = new Bundle{
      val data_req_size = Bool()
      val mmu_en = Bool()
      val tlb_busy = Bool()
    }
    val fromRTU = new Bundle{
      val yy_xx_flush = Bool()
    }
  }
  val st_dc = new Bundle{
    val fromCp0 = new Bundle{
      val l2StPrefEn  = Bool()
    }
    val mmuIn    = new MmuToStDc
    val rtuIn    = new RtuToStDc
  }
  val lq = new Bundle{
    val fromCp0 = new Bundle{
      val lsu_corr_dis = Bool()
    }
    val fromRTU = new Bundle{
      val yy_xx_commit = Vec(3, Bool())
      val yy_xx_commit_iid = Vec(3, UInt(7.W))
      val yy_xx_flush = Bool()
    }
  }
  val sq = new Bundle{
    val fromCp0 = new Bundle{
      val privMode = UInt(MPPWidth.W)
    }

  }

}

class LSUOutput extends Bundle with LsuConfig{
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
  loadag.io.in.fromRTU := io.in.ld_ag.fromRTU
  loadag.io.in.fromPad := io.in.ld_ag.fromPad
  loadag.io.in.fromCp0 := io.in.ld_ag.fromCp0
  loadag.io.in.fromMMU := io.in.ld_ag.fromMMU
  loadag.io.in.fromDcacheArb.ag_ld_sel := dcachearb.io.out.dcache_arb_ag_ld_sel
  loadag.io.in.fromDcacheArb.ld_ag_addr := dcachearb.io.out.dcache_arb_ld_ag_addr
  loadag.io.in.fromDcacheArb.ld_ag_borrow_addr_vld := dcachearb.io.out.dcache_arb_ld_ag_borrow_addr_vld
  loadag.io.in.ctrl_ld_clk := ctrl.io.out.ctrlClk
  loadag.io.in.st_ag_iid := storeag.io.out.toDc.iid
  loadag.io.in.pipe3 := io.in.ld_ag.fromRF_pipe3

  //&Instance("ct_lsu_cmit_monitor","x_ct_lsu_cmit_monitor");
  // &Instance("ct_lsu_st_ag","x_ct_lsu_st_ag"); @87
  storeag.io.in.rfIn := io.in.st_ag.rfIn
  storeag.io.in.cp0In := io.in.st_ag.cp0In
  storeag.io.in.rtuIn := io.in.st_ag.rtuIn
  storeag.io.in.mmuIn := io.in.st_ag.mmuIn
  storeag.io.in.dcacheIn.sel := dcachearb.io.out.dcache_arb_ag_st_sel
  storeag.io.in.dcacheIn.addr := dcachearb.io.out.dcache_arb_st_ag_addr
  storeag.io.in.dcacheIn.borrowAddrVld := dcachearb.io.out.dcache_arb_st_ag_borrow_addr_vld
  storeag.io.in.lmIn := 0.U.asTypeOf(storeag.io.in.lmIn) //////todo: complete lm

  // &Instance("ct_lsu_sd_ex1","x_ct_lsu_sd_ex1"); @88
  storeex1.io.in := io.in.sd_ex1

  // &Instance("ct_lsu_mcic","x_ct_lsu_mcic"); @90
  //mcic //todo: complete mcic

  // &Instance("ct_lsu_dcache_arb","x_ct_lsu_dcache_arb"); @91
  dcachearb.io.in.cp0_lsu_icg_en := io.in.ld_ag.fromCp0.lsu_icg_en
  dcachearb.io.in.pad_yy_icg_scan_en := io.in.ld_ag.fromPad.yy_icg_scan_en
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
  dcachetop.io.in.cp0_lsu_icg_en := io.in.ld_ag.fromCp0.lsu_icg_en
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
  dcachetop.io.in.pad_yy_icg_scan_en := io.in.ld_ag.fromPad.yy_icg_scan_en

  // &Instance("ct_lsu_ld_dc","x_ct_lsu_ld_dc"); @96
  loaddc.io.in.cb_ld_dc_addr_hit := 0.U.asTypeOf(loaddc.io.in.cb_ld_dc_addr_hit) //////todo: complete cache_buffer
  loaddc.io.in.fromCp0.lsu_da_fwd_dis := io.in.ld_ag.fromCp0.lsu_da_fwd_dis
  loaddc.io.in.fromCp0.lsu_dcache_en := io.in.ld_ag.fromCp0.lsu_dcache_en
  loaddc.io.in.fromCp0.lsu_icg_en := io.in.ld_ag.fromCp0.lsu_icg_en
  loaddc.io.in.fromCp0.yy_clk_en := io.in.ld_ag.fromCp0.yy_clk_en
  loaddc.io.in.ctrl_ld_clk := ctrl.io.out.ctrlClk
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_db := dcachearb.io.out.toLoadDC.borrow_vb
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_icc := dcachearb.io.out.toLoadDC.borrow_icc
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_mmu := dcachearb.io.out.toLoadDC.borrow_mmu
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_sndb := dcachearb.io.out.toLoadDC.borrow_sndb
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_vb := dcachearb.io.out.toLoadDC.borrow_vb
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_vld := dcachearb.io.out.toLoadDC.borrow_vld
  loaddc.io.in.fromDcacheArb.ld_dc_borrow_vld_gate := dcachearb.io.out.toLoadDC.borrow_vld_gate
  loaddc.io.in.fromDcacheArb.ld_dc_settle_way := dcachearb.io.out.toLoadDC.settle_way
  loaddc.io.in.fromDcacheArb.idx := dcachearb.io.out.dcache_idx
  loaddc.io.in.dcache_lsu_ld_tag_dout := dcachetop.io.out.ld_tag_dout
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
  loaddc.io.in.fromMMU := io.in.ld_dc.fromMMU
  loaddc.io.in.fromPad.yy_icg_scan_en := io.in.ld_ag.fromPad.yy_icg_scan_en
  loaddc.io.in.fromPFU := 0.U.asTypeOf(loaddc.io.in.fromPFU) //////todo: complete pfu
  loaddc.io.in.rb_fence_ld := rb.io.out.rb_fence_ld
  loaddc.io.in.fromRTU := io.in.ld_dc.fromRTU
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
  storedc.io.in.cp0In.dcacheEn := io.in.ld_ag.fromCp0.lsu_dcache_en
  storedc.io.in.cp0In.icgEn := io.in.ld_ag.fromCp0.lsu_icg_en
  storedc.io.in.cp0In.l2StPrefEn := io.in.st_dc.fromCp0.l2StPrefEn
  storedc.io.in.cp0In.clkEn := io.in.ld_ag.fromCp0.yy_clk_en
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
  storedc.io.in.mmuIn := io.in.st_dc.mmuIn
  //storedc.io.in.pad_yy_icg_scan_en todo:add signal
  storedc.io.in.rtuIn := io.in.st_dc.rtuIn
  storedc.io.in.sdEx1In.sdid := storeex1.io.out.rfEx1Sdid
  storedc.io.in.sqIn.full := sq.io.out.toDc.full //todo: check it
  storedc.io.in.sqIn.instHit := sq.io.out.toDc.instHit
  storedc.io.in.agIn := storeag.io.out.toDc

  // &Instance("ct_lsu_lq","x_ct_lsu_lq"); @99
  lq.io.in.fromCP0.lsu_corr_dis := io.in.lq.fromCp0.lsu_corr_dis
  lq.io.in.fromCP0.lsu_icg_en  := io.in.ld_ag.fromCp0.lsu_icg_en
  lq.io.in.fromCP0.yy_clk_en := io.in.ld_ag.fromCp0.yy_clk_en
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
  lq.io.in.fromPad.yy_icg_scan_en := io.in.ld_ag.fromPad.yy_icg_scan_en
  lq.io.in.fromRTU := io.in.lq.fromRTU
  lq.io.in.fromStDC.addr0 := storedc.io.out.toSq.addr0
  lq.io.in.fromStDC.bytes_vld := storedc.io.out.toSq.bytesVld
  lq.io.in.fromStDC.chk_st_inst_vld := storedc.io.out.toDa.toPwdDa.chkStInstVld
  lq.io.in.fromStDC.chk_statomic_inst_vld := storedc.io.out.toDa.toPwdDa.chkStatomicInstVld
  lq.io.in.fromStDC.iid  := storedc.io.out.toSq.iid

  // &Instance("ct_lsu_sq","x_ct_lsu_sq"); @100
  sq.io.in.cp0In.lsuIcgEn := io.in.ld_ag.fromCp0.lsu_icg_en
  sq.io.in.cp0In.clkEn := io.in.ld_ag.fromCp0.yy_clk_en
  sq.io.in.cp0In.privMode := io.in.sq.fromCp0.privMode
  sq.io.in.dcacheIn.dirtyDin := dcachearb.io.out.dcache_dirty_din
  sq.io.in.dcacheIn.dirtyGwen := dcachearb.io.out.dcache_dirty_gwen
  sq.io.in.dcacheIn.dirtyWen := dcachearb.io.out.dcache_dirty_wen
  sq.io.in.dcacheIn.idx := dcachearb.io.out.dcache_idx
  sq.io.in.dcacheIn.tagDin := dcachearb.io.out.dcache_tag_din
  sq.io.in.dcacheIn.tagGwen := dcachearb.io.out.dcache_tag_gwen
  sq.io.in.dcacheIn.tagWen := dcachearb.io.out.dcache_tag_wen
  //sq.io.in.had //todo: add signals had
  sq.io.in.iccIn.idle := icc.io.out.iccIdle
  sq.io.in.iccIn.sqGrnt := icc.io.out.sqGrnt
  sq.io.in.ldDaIn.lsid := loadda.io.out.ld_da_lsid
  sq.io.in.ldDaIn.sqDataDiscardVld := loadda.io.out.toSQ.data_discard_vld
  sq.io.in.ldDaIn.sqFwdId := loadda.io.out.toSQ.fwd_id
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
  //io.out.
}