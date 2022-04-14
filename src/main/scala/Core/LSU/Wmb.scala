package Core.LSU
import chisel3._
import chisel3.util._

class FromWmbCe extends Bundle{
  val addr = UInt(40.W)
  val atomic = Bool()
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val bytes_vld = UInt(16.W)
  val bytes_vld_full = Bool()
  val create_wmb_data_req = Bool()
  val create_wmb_dp_req = Bool()
  val create_wmb_gateclk_en = Bool()
  val create_wmb_req = Bool()
  val data128 = UInt(128.W)
  val data_vld = UInt(4.W)
  val dcache_inst = Bool()
  val fence_mode = UInt(4.W)
  val hit_sq_pop_dcache_line = Bool()
  val icc = Bool()
  val iid = UInt(7.W)
  val inst_flush = Bool()
  val inst_mode = UInt(2.W)
  val inst_size = UInt(3.W)
  val inst_type = UInt(2.W)
  val merge_data_addr_hit = Bool()
  val merge_data_stall = Bool()
  val merge_en = Bool()
  val merge_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
  val merge_wmb_req = Bool()
  val merge_wmb_wait_not_vld_req = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val page_wa = Bool()
  val priv_mode = UInt(2.W)
  val read_dp_req = Bool()
  val same_dcache_line = Vec(LSUConfig.WMB_ENTRY, Bool())
  val sc_wb_vld = Bool()
  val spec_fail = Bool()
  val sync_fence = Bool()
  val vld = Bool()
  val vstart_vld = Bool()
  val wb_cmplt_success = Bool()
  val wb_data_success = Bool()
  val write_biu_dp_req = Bool()
  val write_imme = Bool()
}

//from SQ
//val update_dcache_dirty = Bool()
//val update_dcache_share = Bool()
//val update_dcache_valid = Bool()
//val update_dcache_way = Bool()

class WmbInput extends Bundle{
  val amr_l2_mem_set = Bool()
  val fromBiu = new Bundle{
    val b_id = UInt(5.W)
    val b_resp = UInt(2.W)
    val b_vld = Bool()
    val r_id = UInt(5.W)
    val r_vld = Bool()
  }
  val fromBusArb = new Bundle{
    val ar_grnt = Bool()
    val aw_grnt = Bool()
    val w_grnt = Bool()
  }
  val fromCp0 = new Bundle{
    val lsu_icg_en = Bool()
    val lsu_no_op_req = Bool()
    val lsu_wr_burst_dis = Bool()
    val yy_clk_en = Bool()
  }
  val fromDcache = new Bundle{
    val arb_wmb_ld_grnt = Bool()
    val dirty_din = UInt(7.W)
    val dirty_gwen = Bool()
    val dirty_wen = UInt(7.W)
    val idx = UInt(9.W)
    val snq_st_sel = Bool()
    val tag_din = UInt(52.W)
    val tag_gwen = Bool()
    val tag_wen = UInt(2.W)
    val vb_snq_gwen = Bool()
  }
  val icc_wmb_write_imme = Bool()
  val ld_ag_inst_vld = Bool()
  val fromLoadDA = new Bundle{
    val fwd_ecc_stall = Bool()
    val lsid = Vec(LSUConfig.LSIQ_ENTRY, Bool())
    val wmb_discard_vld = Bool()
  }
  val fromLoadDC = new Bundle{
    val addr0 = UInt(40.W)
    val addr1_11to4 = UInt(8.W)
    val bytes_vld = UInt(16.W)
    val chk_atomic_inst_vld = Bool()
    val chk_ld_inst_vld = Bool()
  }
  val ld_wb_wmb_data_grnt = Bool()
  val fromLfb = new Bundle{
    val read_req_hit_idx = Bool()
    val write_req_hit_idx = Bool()
  }
  val fromLm = new Bundle{
    val is_amo_lock = Bool()
    val is_ex_wait_lock = Bool()
    val is_idle = Bool()
  }
  val FromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val pfu_biu_req_addr = UInt(40.W)
  val fromRB = new Bundle{
    val biu_req_addr = UInt(40.W)
    val biu_req_unmask = Bool()
    val wmb_so_pending = Bool()
  }
  val fromRTU = new Bundle{
    val lsu_async_flush = Bool()
    val yy_xx_flush = Bool()
  }
  val fromSnq = new Bundle{
    val can_create_snq_uncheck = Bool()
    val create_addr = UInt(40.W)
    val create_wmb_read_req_hit_idx = Bool()
    val create_wmb_write_req_hit_idx = Bool()
    val wmb_read_req_hit_idx = Bool()
    val wmb_write_req_hit_idx = Bool()
  }
  val fromSQ = new Bundle{
    val pop_addr = UInt(40.W)
    val pop_priv_mode = UInt(2.W)
    val wmb_merge_req = Bool()
    val wmb_merge_stall_req = Bool()
    val wmb_pop_to_ce_dp_req = Bool()
    val wmb_pop_to_ce_gateclk_en = Bool()
    val wmb_pop_to_ce_req = Bool()
    val wmb_ce_update_dcache_dirty = Bool()
    val wmb_ce_update_dcache_share = Bool()
    val wmb_ce_update_dcache_valid = Bool()
    val wmb_ce_update_dcache_way = Bool()
  }
  val st_ag_inst_vld = Bool()
  val st_rf_inst_vld = Bool()
  val st_wb_wmb_cmplt_grnt = Bool()
  val fromVB = new Bundle{
    val create_grnt = Bool()
    val empty = Bool()
    val entry_rcl_done = Vec(LSUConfig.WMB_ENTRY, Bool())
    val write_req_hit_idx = Bool()
  }
  val fromWmbCe = new FromWmbCe
}

class WmbOutput extends Bundle{
  val toHad = new Bundle{
    val ar_pending = Bool()
    val aw_pending = Bool()
    val create_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
    val data_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
    val entry_vld = Vec(LSUConfig.WMB_ENTRY, Bool())
    val read_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
    val w_pending = Bool()
    val write_imme = Bool()
    val write_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
  }
  val toBiu = new Bundle{
    val ar_addr = UInt(40.W)
    val ar_bar = UInt(2.W)
    val ar_burst = UInt(2.W)
    val ar_cache = UInt(4.W)
    val ar_domain = UInt(2.W)
    val ar_dp_req = Bool()
    val ar_id = UInt(5.W)
    val ar_len = UInt(2.W)
    val ar_lock = Bool()
    val ar_prot = UInt(3.W)
    val ar_req = Bool()
    val ar_req_gateclk_en = Bool()
    val ar_size = UInt(3.W)
    val ar_snoop = UInt(4.W)
    val ar_user = UInt(3.W)
    val aw_addr = UInt(40.W)
    val aw_bar = UInt(2.W)
    val aw_burst = UInt(2.W)
    val aw_cache = UInt(4.W)
    val aw_domain = UInt(2.W)
    val aw_dp_req = Bool()
    val aw_id = UInt(5.W)
    val aw_len = UInt(2.W)
    val aw_lock = Bool()
    val aw_prot = UInt(3.W)
    val aw_req = Bool()
    val aw_req_gateclk_en = Bool()
    val aw_size = UInt(3.W)
    val aw_snoop = UInt(3.W)
    val aw_user = Bool()
    val w_data = UInt(128.W)
    val w_id = UInt(5.W)
    val w_last = Bool()
    val w_req = Bool()
    val w_strb = UInt(16.W)
    val w_vld = Bool()
    val w_wns = Bool()
  }
  val toWmbCe = new Bundle{
    val create_dp_vld = Bool()
    val create_gateclk_en = Bool()
    val create_merge = Bool()
    val create_merge_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
    val create_same_dcache_line = Vec(LSUConfig.WMB_ENTRY, Bool())
    val create_stall = Bool()
    val create_vld = Bool()
    val pop_vld = Bool()
  }
  val toDcacheArb = new Bundle{
    val data_way = Bool()
    val ld_borrow_req = Bool()
    val ld_data_gateclk_en = UInt(8.W)
    val ld_data_gwen = UInt(8.W)
    val ld_data_high_din = UInt(128.W)
    val ld_data_idx = UInt(11.W)
    val ld_data_low_din = UInt(128.W)
    val ld_data_req = UInt(8.W)
    val ld_data_wen = UInt(32.W)
    val ld_req = Bool()
    val ld_tag_gateclk_en = Bool()
    val ld_tag_idx = UInt(9.W)
    val ld_tag_req = Bool()
    val ld_tag_wen = UInt(2.W)
    val st_dirty_din = UInt(7.W)
    val st_dirty_gateclk_en = Bool()
    val st_dirty_idx = UInt(9.W)
    val st_dirty_req = Bool()
    val st_dirty_wen = UInt(7.W)
    val st_req = Bool()
  }
  val wmb_depd_wakeup = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val wmb_empty = Bool()
  val wmb_entry_vld = Vec(LSUConfig.WMB_ENTRY, Bool())
  val wmb_fwd_bytes_vld = UInt(16.W)
  val wmb_has_sync_fence = Bool()
  val wmb_ld_da_fwd_data = UInt(128.W)
  val toLoadDC = new Bundle{
    val cancel_acc_req = Bool()
    val discard_req = Bool()
    val fwd_req = Bool()
  }
  val toLoadWB = new Bundle{
    val data = UInt(64.W)
    val data_addr = UInt(40.W)
    val data_iid = UInt(7.W)
    val data_req = Bool()
    val inst_vfls = Bool()
    val preg = UInt(7.W)
    val preg_sign_sel = UInt(4.W)
    val vreg = UInt(6.W)
    val vreg_sign_sel = UInt(2.W)
  }
  val wmb_lm_state_clr = Bool()
  val wmb_no_op = Bool()
  val wmb_pfu_biu_req_hit_idx = Bool()
  val wmb_rb_biu_req_hit_idx = Bool()
  val wmb_rb_so_pending = Bool()
  val wmb_read_req_addr = UInt(40.W)
  val toSnq = new Bundle{
    val depd = Vec(LSUConfig.WMB_ENTRY, Bool())
    val depd_remove = Vec(LSUConfig.WMB_ENTRY, Bool())
  }
  val toSQ = new Bundle{
    val pop_grnt = Bool()
    val pop_to_ce_grn = Bool()
  }
  val toStoreWB = new Bundle{
    val bkpta_data = Bool()
    val bkptb_data = Bool()
    val cmplt_req = Bool()
    val iid = UInt(7.W)
    val inst_flush = Bool()
    val spec_fail = Bool()
  }
  val wmb_sync_fence_biu_req_success = Bool()
  val toVB = new Bundle{
    val addr_tto6 = UInt(34.W)
    val create_dp_vld = Bool()
    val create_gateclk_en = Bool()
    val create_req = Bool()
    val create_vld = Bool()
    val inv = Bool()
    val set_way_mode = Bool()
  }
  val wmb_write_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
  val wmb_write_ptr_encode = UInt(3.W)
  val wmb_write_req_addr = UInt(40.W)
  val wmb_write_req_icc = Bool()
}

class WmbIO extends Bundle{
  val in = Input(new WmbInput)
  val out = Output(new WmbOutput)
}

class Wmb extends Module {
  val io = IO(new WmbIO)




}
