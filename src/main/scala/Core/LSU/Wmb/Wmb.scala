package Core.LSU.Wmb

import Core.LSU.IdFifo8
import Core.LSU.Sq.DcacheDirtyDataEn
import Core.{BIUConfig, DCacheConfig, LsuConfig}
import Utils.ParallelORR
import chisel3._
import chisel3.util._

class FromWmbCe extends Bundle with LsuConfig{
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
  val merge_ptr = Vec(WMB_ENTRY, Bool())
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
  val same_dcache_line = Vec(WMB_ENTRY, Bool())
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

class WmbInput extends Bundle with LsuConfig with DCacheConfig{
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
    val dirty_din = Flipped(ValidIO(Vec(WAYS, new DcacheDirtyDataEn)))
    val dirty_gwen = Bool()
    val dirty_wen = Flipped(ValidIO(Vec(WAYS, new DcacheDirtyDataEn)))
    val idx = UInt(9.W)
    val snq_st_sel = Bool()
    val tag_din = Vec(WAYS,Flipped(ValidIO(UInt(TAG_WIDTH.W))))
    val tag_gwen = Bool()
    val tag_wen = Vec(WAYS,Bool())
    val vb_snq_gwen = Bool()
  }
  val icc_wmb_write_imme = Bool()
  val ld_ag_inst_vld = Bool()
  val fromLoadDA = new Bundle{
    val fwd_ecc_stall = Bool()
    val lsid = Vec(LSIQ_ENTRY, Bool())
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
    val state_is_amo_lock = Bool()
    val state_is_ex_wait_lock = Bool()
    val state_is_idle = Bool()
  }
  val fromPad = new Bundle{
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
    val entry_rcl_done = Vec(WMB_ENTRY, Bool())
    val write_req_hit_idx = Bool()
  }
  val fromWmbCe = new FromWmbCe
}

class WmbOutput extends Bundle with LsuConfig{
  val toHad = new Bundle{
    val ar_pending = Bool()
    val aw_pending = Bool()
    val create_ptr = Vec(WMB_ENTRY, Bool())
    val data_ptr   = Vec(WMB_ENTRY, Bool())
    val entry_vld  = Vec(WMB_ENTRY, Bool())
    val read_ptr   = Vec(WMB_ENTRY, Bool())
    val w_pending = Bool()
    val write_imme = Bool()
    val write_ptr  = Vec(WMB_ENTRY, Bool())
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
    val create_merge_ptr = Vec(WMB_ENTRY, Bool())
    val create_same_dcache_line = Vec(WMB_ENTRY, Bool())
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
  val wmb_depd_wakeup = Vec(LSIQ_ENTRY, Bool())
  val wmb_empty = Bool()
  val wmb_entry_vld = Vec(WMB_ENTRY, Bool())
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
    val depd = Vec(WMB_ENTRY, Bool())
    val depd_remove = Vec(WMB_ENTRY, Bool())
  }
  val toSQ = new Bundle{
    val pop_grnt = Bool()
    val pop_to_ce_grnt = Bool()
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
  val toVB = new Bundle {
    val addr_tto6         = UInt(34.W)
    val create_dp_vld     = Bool()
    val create_gateclk_en = Bool()
    val create_req        = Bool()
    val create_vld        = Bool()
    val inv               = Bool()
    val set_way_mode      = Bool()
  }
  val wmb_write_ptr         = Vec(WMB_ENTRY, Bool())
  val wmb_write_ptr_encode  = UInt(3.W)
  val wmb_write_req_addr    = UInt(40.W)
  val wmb_write_req_icc     = Bool()
}

class WmbIO extends Bundle{
  val in = Input(new WmbInput)
  val out = Output(new WmbOutput)
}

class Wmb extends Module with LsuConfig with BIUConfig{
  val io = IO(new WmbIO)

  val ptr_init = Seq(true.B,false.B,false.B,false.B,false.B,false.B,false.B,false.B)
  //Reg
  val wmb_create_ptr = RegInit(VecInit(ptr_init))
  val wmb_read_ptr = RegInit(VecInit(ptr_init))
  val wmb_write_ptr = RegInit(VecInit(ptr_init))
  val wmb_data_ptr = RegInit(VecInit(ptr_init))
  val wmb_create_ptr_circular = RegInit(false.B)
  val wmb_read_ptr_circular = RegInit(false.B)
  val wmb_write_ptr_circular = RegInit(false.B)
  val wmb_data_ptr_circular = RegInit(false.B)


  val wmb_write_imme_hold = RegInit(false.B)
  val wmb_write_imme = RegInit(false.B)
  val wmb_last_create_addr = RegInit(0.U(PA_WIDTH.W))
  val wmb_read_dp_req = RegInit(false.B)
  val wmb_read_req_addr = RegInit(0.U(PA_WIDTH.W))
  val wmb_write_biu_dp_req = RegInit(false.B)
  val wmb_write_req_atomic     = RegInit(false.B)
  val wmb_write_req_icc        = RegInit(false.B)
  val wmb_write_req_sync_fence = RegInit(false.B)
  val wmb_write_req_page_ca    = RegInit(false.B)
  val wmb_write_req_page_share = RegInit(false.B)
  val wmb_write_req_addr = RegInit(0.U(PA_WIDTH.W))
  val wmb_ctc_secd = RegInit(false.B)

  val wmb_write_dcache_pop_req = RegInit(false.B)
  val wmb_write_dcache_ptr = RegInit(VecInit(Seq.fill(WMB_ENTRY)(false.B)))
  val wmb_write_dcache_addr = RegInit(0.U(PA_WIDTH.W))

  val wmb_wakeup_queue = RegInit(VecInit(Seq.fill(LSIQ_ENTRY)(false.B)))
  val wmb_pop_depd_ff = RegInit(false.B)

  val wmb_fwd_data = RegInit(0.U(128.W))

  //Reg Output
  io.out.wmb_write_ptr := wmb_write_ptr
  io.out.wmb_write_req_addr := wmb_write_req_addr
  io.out.wmb_write_req_icc := wmb_write_req_icc
  io.out.wmb_read_req_addr := wmb_read_req_addr

  //Wire
  val wmb_write_ptr_encode = Wire(UInt(3.W))
  val wmb_empty = Wire(Bool())
  val wmb_read_ptr_unconditional_shift_imme = Wire(Bool())
  val wmb_read_ptr_chk_idx_shift_imme = Wire(Bool())
  val wmb_read_req_unmask = Wire(Bool())
  val wmb_write_ptr_unconditional_shift_imme = Wire(Bool())
  val wmb_write_ptr_chk_idx_shift_imme = Wire(Bool())
  val wmb_write_req = Wire(Bool())
  val wmb_dcache_arb_req_unmask = Wire(Bool())
  val wmb_data_ptr_after_write_shift_imme = Wire(Bool())
  val wmb_data_ptr_with_write_shift_imme = Wire(Bool())
  val wmb_data_req = Wire(Bool())
  val wmb_fwd_data_pe_gateclk_en = Wire(Bool())
  val wmb_write_pop_up_wmb_ce_gateclk_en = Wire(Bool())
  val wmb_dcache_req_next = Wire(Bool())

  val wmb_entry_in       = Wire(Vec(WMB_ENTRY, new WmbEntryIn))
  wmb_entry_in := DontCare
  val wmb_entry_out      = Wire(Vec(WMB_ENTRY, new WmbEntryOutput))
  val wmb_dcache_req_ptr = Wire(Vec(WMB_ENTRY, Bool()))
  val pw_merge_stall = WireInit(false.B)

  val wmb_b_resp_exokay = Wire(Bool())
  val wmb_biu_ar_id    = Wire(UInt(5.W))
  val wmb_biu_aw_id    = Wire(UInt(5.W))
  val wmb_biu_write_en = Wire(Bool())

  val wmb_create_vb_success = Wire(Bool())
  val wmb_dcache_inst_write_req_hit_idx = Wire(Bool())

  val wmb_ce_create_vld = Wire(Bool())
  val wmb_ce_update_same_dcache_line = Wire(Bool())
  val wmb_ce_update_same_dcache_line_ptr = Wire(Vec(WMB_ENTRY, Bool()))
  val wmb_ce_last_addr_plus = Wire(Bool())
  val wmb_ce_last_addr_sub = Wire(Bool())

  val wmb_same_line_resp_ready = Wire(Vec(WMB_ENTRY, Bool()))
  val wmb_wakeup_queue_not_empty = Wire(Bool())

  val wmb_read_ptr_read_req_grnt = Wire(Bool())
  val wmb_read_ptr_shift_imme_grnt = Wire(Bool())
  val wmb_write_biu_dcache_line = Wire(Bool())
  val wmb_write_dcache_success = Wire(Bool())
  val wmb_write_ptr_shift_imme_grnt = Wire(Bool())

  val wmb_create_vld = Wire(Bool())
  val wmb_create_ptr_next1 = Wire(Vec(WMB_ENTRY, Bool()))
  val wmb_read_ptr_shift_vld = Wire(Bool())
  val wmb_read_ptr_next1 = Wire(Vec(WMB_ENTRY, Bool()))
  val wmb_write_ptr_shift_vld = Wire(Bool())
  val wmb_write_ptr_next = Wire(Vec(8, Vec(WMB_ENTRY, Bool())))
  val wmb_data_ptr_shift_vld = Wire(Bool())
  val wmb_data_ptr_next1 = Wire(Vec(WMB_ENTRY, Bool()))

  val wmb_write_req_next_addr_sub = Wire(Vec(4, Bool()))
  val wmb_write_req_next_addr_plus = Wire(Vec(4, Bool()))

  val wmb_write_req_ready_to_dcache_line = Wire(Vec(4, Bool()))

  val wmb_vb_create_vld = Wire(Bool())
  val wmb_read_ptr_met_create = Wire(Bool())
  val wmb_read_req_hit_idx = Wire(Bool())
  val wmb_read_req_ctc_inst = Wire(Bool())
  val wmb_read_req_ctc_end = Wire(Bool())
  val wmb_write_ptr_met_create = Wire(Bool())
  val wmb_write_req_hit_idx = Wire(Bool())
  val wmb_data_ptr_met_create = Wire(Bool())

  val wmb_sq_pop_grnt = Wire(Bool())
  val wmb_write_biu_req_unmask = Wire(Bool())
  val wmb_write_req_page_so = Wire(Bool())

  val wmb_write_imme_hold_set = Wire(Bool())
  val wmb_create_write_imme_set = Wire(Bool())
  val wmb_write_imme_amr_clr = Wire(Bool())
  val wmb_write_imme_clr = Wire(Bool())
  val wmb_write_imme_set = Wire(Bool())
  val wmb_mem_set_write_grnt = Wire(Bool())
  val wmb_read_pop_up_wmb_ce_vld = Wire(Bool())
  val wmb_read_ptr_next1_met_create = Wire(Bool())
  val wmb_write_dcache_req_icc_inst = Wire(Bool())
  val wmb_write_pop_up_wmb_ce_vld = Wire(Bool())

  val wmb_biu_ar_addr = Wire(UInt(PA_WIDTH.W))

  val wmb_write_dcache_ptr_set = Wire(Vec(WMB_ENTRY, Bool()))
  val wmb_write_dcache_addr_set = Wire(UInt(PA_WIDTH.W))

  val wmb_dcache_arb_req = Wire(Bool())
  val wmb_dcache_data_high_sel = Wire(Bool())
  val wmb_biu_nc_req_grnt = Wire(Bool())
  val wmb_biu_so_req_grnt = Wire(Bool())

  val wmb_wakeup_queue_next = Wire(Vec(LSIQ_ENTRY,Bool()))
  val wmb_pop_depd = Wire(Bool())
  val wmb_pop_discard_req = Wire(Bool())
  val wmb_pop_fwd_req = Wire(Bool())
  val wmb_b_nc_id_hit = Wire(Bool())
  val wmb_b_so_id_hit = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //if sq has entry or create sq, then this gateclk is on
  val wmb_clk_en = !wmb_empty || io.in.fromSQ.wmb_pop_to_ce_gateclk_en || io.in.fromWmbCe.vld || wmb_pop_depd_ff ||
    wmb_read_dp_req || wmb_write_biu_dp_req || wmb_write_imme

  //create ptr
  val wmb_create_ptr_clk_en = io.in.fromWmbCe.create_wmb_gateclk_en

  //read ptr
  val wmb_read_ptr_clk_en  = wmb_read_ptr_unconditional_shift_imme || wmb_read_ptr_chk_idx_shift_imme || wmb_read_req_unmask

  //write ptr
  val wmb_write_ptr_clk_en = wmb_write_ptr_unconditional_shift_imme || wmb_write_ptr_chk_idx_shift_imme || wmb_write_req || wmb_dcache_arb_req_unmask

  //data ptr
  val wmb_data_ptr_clk_en  = wmb_data_ptr_after_write_shift_imme || wmb_data_ptr_with_write_shift_imme || wmb_data_req || wmb_dcache_arb_req_unmask

  //wmb fwd ld da pop entry
  val wmb_fwd_data_pe_clk_en = wmb_fwd_data_pe_gateclk_en

  //pop entry signal
  val wmb_write_pop_clk_en = wmb_write_ptr_clk_en || wmb_write_pop_up_wmb_ce_gateclk_en

  val wmb_write_dcache_pop_clk_en = wmb_dcache_req_next || wmb_write_dcache_pop_req

  val wmb_read_pop_clk_en = wmb_read_ptr_clk_en || io.in.fromWmbCe.create_wmb_gateclk_en

  //depd clk is used for wakeup queue
  val wmb_wakeup_queue_clk_en  = io.in.fromLoadDA.wmb_discard_vld || wmb_pop_depd_ff || io.in.fromRTU.yy_xx_flush

  //==========================================================
  //                      Empty cnt
  //==========================================================
  //------------------empty signal----------------------------
  wmb_empty := !ParallelORR(wmb_entry_out.map(entry => entry.vld))
  io.out.wmb_empty := wmb_empty
  //==========================================================
  //                 Non-cacheable FIFO
  //==========================================================
  val wmb_idfifo_nc_create_vld = wmb_biu_nc_req_grnt
  val wmb_idfifo_nc_pop_vld    = wmb_b_nc_id_hit

  val idfifo_nc = Module(new IdFifo8)
  idfifo_nc.io.in.cp0_lsu_icg_en      := io.in.fromCp0.lsu_icg_en
  idfifo_nc.io.in.cp0_yy_clk_en       := io.in.fromCp0.yy_clk_en
  idfifo_nc.io.in.idfifo_create_id    := wmb_write_ptr_encode
  idfifo_nc.io.in.idfifo_create_id_oh := wmb_write_ptr.asUInt
  idfifo_nc.io.in.idfifo_create_vld   := wmb_idfifo_nc_create_vld
  idfifo_nc.io.in.idfifo_pop_vld      := wmb_idfifo_nc_pop_vld
  idfifo_nc.io.in.pad_yy_icg_scan_en  := io.in.fromPad.yy_icg_scan_en


  val wmb_nc_no_pending = idfifo_nc.io.out.idfifo_empty

  val wmb_entry_next_nc_bypass = idfifo_nc.io.out.idfifo_pop_id_oh
  for(i <- 0 until WMB_ENTRY){
    wmb_entry_in(i).next_nc_bypass := wmb_entry_next_nc_bypass(i)
  }
  //==========================================================
  //                 Strong order FIFO
  //==========================================================
  val wmb_idfifo_so_create_vld = wmb_biu_so_req_grnt
  val wmb_idfifo_so_pop_vld    = wmb_b_so_id_hit

  val idfifo_so = Module(new IdFifo8)
  idfifo_so.io.in.cp0_lsu_icg_en      := io.in.fromCp0.lsu_icg_en
  idfifo_so.io.in.cp0_yy_clk_en       := io.in.fromCp0.yy_clk_en
  idfifo_so.io.in.idfifo_create_id    := wmb_write_ptr_encode
  idfifo_so.io.in.idfifo_create_id_oh := wmb_write_ptr.asUInt
  idfifo_so.io.in.idfifo_create_vld   := wmb_idfifo_so_create_vld
  idfifo_so.io.in.idfifo_pop_vld      := wmb_idfifo_so_pop_vld
  idfifo_so.io.in.pad_yy_icg_scan_en  := io.in.fromPad.yy_icg_scan_en

  val wmb_so_no_pending = idfifo_so.io.out.idfifo_empty

  val wmb_entry_next_so_bypass = idfifo_so.io.out.idfifo_pop_id_oh
  for(i <- 0 until WMB_ENTRY){
    wmb_entry_in(i).next_so_bypass := wmb_entry_next_so_bypass(i)
  }
  //------------------pending---------------------------------
  io.out.wmb_rb_so_pending := !wmb_so_no_pending
  //==========================================================
  //          Instance of write merge buffer entry
  //==========================================================
  val wmb_entry = Seq.fill(WMB_ENTRY)(Module(new WmbEntry))
  for(i <- 0 until WMB_ENTRY){
    wmb_entry(i).io.in.fromBiu    := io.in.fromBiu
    wmb_entry(i).io.in.fromBusArb := io.in.fromBusArb
    wmb_entry(i).io.in.fromCp0    := io.in.fromCp0
    wmb_entry(i).io.in.fromDCache := io.in.fromDcache
    wmb_entry(i).io.in.fromLoadDC := io.in.fromLoadDC
    wmb_entry(i).io.in.fromLm     := io.in.fromLm
    wmb_entry(i).io.in.fromPad    := io.in.fromPad
    wmb_entry(i).io.in.fromSnq    := io.in.fromSnq

    wmb_entry(i).io.in.fromSQ.pop_addr            := io.in.fromSQ.pop_addr
    wmb_entry(i).io.in.fromSQ.pop_priv_mode       := io.in.fromSQ.pop_priv_mode
    wmb_entry(i).io.in.fromSQ.wmb_merge_req       := io.in.fromSQ.wmb_merge_req
    wmb_entry(i).io.in.fromSQ.wmb_merge_stall_req := io.in.fromSQ.wmb_merge_stall_req
    wmb_entry(i).io.in.fromSQ.update_dcache_dirty := io.in.fromSQ.wmb_ce_update_dcache_dirty
    wmb_entry(i).io.in.fromSQ.update_dcache_share := io.in.fromSQ.wmb_ce_update_dcache_share
    wmb_entry(i).io.in.fromSQ.update_dcache_valid := io.in.fromSQ.wmb_ce_update_dcache_valid
    wmb_entry(i).io.in.fromSQ.update_dcache_way   := io.in.fromSQ.wmb_ce_update_dcache_way
    wmb_entry(i).io.in.fromVb.empty    := io.in.fromVB.empty
    wmb_entry(i).io.in.fromVb.rcl_done := io.in.fromVB.entry_rcl_done(i)

    wmb_entry(i).io.in.fromWmbCe := io.in.fromWmbCe
    wmb_entry(i).io.in.WmbEntry  := wmb_entry_in(i)

    wmb_entry(i).io.in.wmb_dcache_req_ptr  := wmb_dcache_req_ptr(i)
    wmb_entry(i).io.in.pfu_biu_req_addr    := io.in.pfu_biu_req_addr
    wmb_entry(i).io.in.pw_merge_stall      := pw_merge_stall
    wmb_entry(i).io.in.rb_biu_req_addr     := io.in.fromRB.biu_req_addr
    wmb_entry(i).io.in.rb_biu_req_unmask   := io.in.fromRB.biu_req_unmask
    wmb_entry(i).io.in.rtu_lsu_async_flush := io.in.fromRTU.lsu_async_flush
    wmb_entry(i).io.in.wmb_b_resp_exokay   := wmb_b_resp_exokay
    wmb_entry(i).io.in.wmb_biu_ar_id       := wmb_biu_ar_id
    wmb_entry(i).io.in.wmb_biu_aw_id       := wmb_biu_aw_id
    wmb_entry(i).io.in.wmb_biu_write_en    := wmb_biu_write_en
    wmb_entry(i).io.in.wmb_create_ptr_next1               := wmb_create_ptr_next1(i)
    wmb_entry(i).io.in.wmb_create_vb_success              := wmb_create_vb_success
    wmb_entry(i).io.in.wmb_data_ptr                       := wmb_data_ptr(i)
    wmb_entry(i).io.in.wmb_dcache_arb_req_unmask          := wmb_dcache_arb_req_unmask
    wmb_entry(i).io.in.wmb_dcache_inst_write_req_hit_idx  := wmb_dcache_inst_write_req_hit_idx
    wmb_entry(i).io.in.wmb_ce_create_vld                  := wmb_ce_create_vld
    wmb_entry(i).io.in.wmb_ce_update_same_dcache_line     := wmb_ce_update_same_dcache_line
    wmb_entry(i).io.in.wmb_ce_update_same_dcache_line_ptr := wmb_ce_update_same_dcache_line_ptr
    wmb_entry(i).io.in.wmb_ce_last_addr_plus              := wmb_ce_last_addr_plus
    wmb_entry(i).io.in.wmb_ce_last_addr_sub               := wmb_ce_last_addr_sub
    wmb_entry(i).io.in.wmb_same_line_resp_ready   := wmb_same_line_resp_ready
    wmb_entry(i).io.in.wmb_wakeup_queue_not_empty := wmb_wakeup_queue_not_empty

    wmb_entry(i).io.in.WmbRead.ptr_read_req_grnt    := wmb_read_ptr_read_req_grnt
    wmb_entry(i).io.in.WmbRead.ptr_shift_imme_grnt  := wmb_read_ptr_shift_imme_grnt
    wmb_entry(i).io.in.WmbRead.ptr                  := wmb_read_ptr(i)
    wmb_entry(i).io.in.WmbWrite.biu_dcache_line     := wmb_write_biu_dcache_line
    wmb_entry(i).io.in.WmbWrite.dcache_success      := wmb_write_dcache_success
    wmb_entry(i).io.in.WmbWrite.ptr_shift_imme_grnt := wmb_write_ptr_shift_imme_grnt
    wmb_entry(i).io.in.WmbWrite.ptr                 := wmb_write_ptr(i)

    wmb_entry_out(i) := wmb_entry(i).io.out
  }

  io.out.wmb_entry_vld := VecInit(wmb_entry_out.map(_.vld))



  //==========================================================
  //                  Maintain pointer
  //==========================================================
  //----------------------registers---------------------------
  //circular bit is to check whether write/read/data ptr equal to create_ptr
  //+------------+
  //| create_ptr |
  //+------------+
  when(wmb_create_vld){
    wmb_create_ptr := wmb_create_ptr_next1
  }

  when(wmb_create_vld && wmb_create_ptr(WMB_ENTRY-1)){
    wmb_create_ptr_circular := !wmb_create_ptr_circular
  }

  //+----------+
  //| read_ptr |
  //+----------+
  when(wmb_read_ptr_shift_vld){
    wmb_read_ptr := wmb_read_ptr_next1
  }

  when(wmb_read_ptr_shift_vld && wmb_read_ptr(WMB_ENTRY-1)){
    wmb_read_ptr_circular := !wmb_read_ptr_circular
  }

  //+-----------+
  //| write_ptr |
  //+-----------+
  when(wmb_write_ptr_shift_vld){
    wmb_write_ptr := wmb_write_ptr_next(1)
  }

  when(wmb_write_ptr_shift_vld && wmb_write_ptr(WMB_ENTRY-1)){
    wmb_write_ptr_circular := !wmb_write_ptr_circular
  }

  //+----------+
  //| data_ptr |
  //+----------+
  when(wmb_data_ptr_shift_vld){
    wmb_data_ptr := wmb_data_ptr_next1
  }

  when(wmb_data_ptr_shift_vld && wmb_data_ptr(WMB_ENTRY-1)){
    wmb_data_ptr_circular := !wmb_data_ptr_circular
  }

  //------------------write ptr set---------------------------
  //if snq clear read req, then read ptr must be set write_ptr_set

  //------------------pointer encode--------------------------
  val wmb_read_ptr_encode = OHToUInt(wmb_read_ptr.asUInt)(2,0)

  wmb_write_ptr_encode := OHToUInt(wmb_write_ptr.asUInt)(2,0)
  io.out.wmb_write_ptr_encode := wmb_write_ptr_encode

  val wmb_write_ptr_next3_encode = OHToUInt(wmb_write_ptr_next(3).asUInt)(2,0)

  //------------------condition wires-------------------------
  val wmb_write_burst_neg = wmb_write_req_addr(5,4) === 3.U(2.W)

  val wmb_write_req_addr_dcache_begin = wmb_write_req_addr(5,4) === 3.U(2.W) &&
    wmb_write_req_next_addr_sub(1) && wmb_write_req_next_addr_sub(2) && wmb_write_req_next_addr_sub(3) ||
    wmb_write_req_addr(5,4) === 0.U(2.W) &&
    wmb_write_req_next_addr_plus(1) && wmb_write_req_next_addr_plus(2) && wmb_write_req_next_addr_plus(3)

  //------------has read resp in next 4 entry-----------------
  val write_burst_en = !io.in.fromCp0.lsu_wr_burst_dis

  wmb_write_biu_dcache_line := write_burst_en && wmb_write_req_addr_dcache_begin && wmb_write_req_ready_to_dcache_line.asUInt.andR

  val wmb_read_grnt = io.in.fromBusArb.ar_grnt
  wmb_read_ptr_unconditional_shift_imme := ParallelORR(wmb_entry_out.map(_.read_ptr_unconditional_shift_imme))
  wmb_read_ptr_chk_idx_shift_imme := ParallelORR(wmb_entry_out.map(_.read_ptr_chk_idx_shift_imme))

  wmb_create_vb_success := wmb_vb_create_vld &&  io.in.fromVB.create_grnt
  val wmb_write_grnt = io.in.fromBusArb.aw_grnt || wmb_create_vb_success

  wmb_write_req := ParallelORR(wmb_entry_out.map(_.write_req))
  wmb_write_ptr_unconditional_shift_imme := ParallelORR(wmb_entry_out.map(_.write_ptr_unconditional_shift_imme))
  wmb_write_ptr_chk_idx_shift_imme := ParallelORR(wmb_entry_out.map(_.write_ptr_chk_idx_shift_imme))
  val wmb_data_grnt = io.in.fromBusArb.w_grnt

  wmb_data_req := ParallelORR(wmb_entry_out.map(_.data_req))
  wmb_data_ptr_after_write_shift_imme := ParallelORR(wmb_entry_out.map(_.data_ptr_after_write_shift_imme))
  wmb_data_ptr_with_write_shift_imme := ParallelORR(wmb_entry_out.map(_.data_ptr_with_write_shift_imme))
  //-----------------shift condition wires--------------------
  wmb_read_ptr_shift_imme_grnt := !wmb_read_ptr_met_create && (wmb_read_ptr_unconditional_shift_imme || wmb_read_ptr_chk_idx_shift_imme && !wmb_read_req_hit_idx)

  wmb_read_ptr_read_req_grnt := wmb_read_grnt && (!wmb_read_req_ctc_inst || wmb_read_req_ctc_end)

  wmb_read_ptr_shift_vld := wmb_read_ptr_read_req_grnt || wmb_read_ptr_shift_imme_grnt

  //all shift imme don't see wmb write imme, because write_imme only effective
  //to wo st
  wmb_write_ptr_shift_imme_grnt := !wmb_write_ptr_met_create && (wmb_write_ptr_unconditional_shift_imme || wmb_write_ptr_chk_idx_shift_imme && !wmb_write_req_hit_idx)

  wmb_write_ptr_shift_vld := wmb_write_grnt || wmb_write_ptr_shift_imme_grnt

  //set circular
  wmb_data_ptr_shift_vld := wmb_data_grnt &&  wmb_data_req ||
    !wmb_data_ptr_met_create && (wmb_data_ptr_after_write_shift_imme || wmb_data_ptr_with_write_shift_imme && wmb_write_ptr_shift_imme_grnt)

  //------------------other pointer---------------------------
  wmb_create_ptr_next1 := Cat(wmb_create_ptr.asUInt(WMB_ENTRY-2,0), wmb_create_ptr.asUInt(WMB_ENTRY-1)).asTypeOf(Vec(WMB_ENTRY, Bool()))

  wmb_read_ptr_next1 := Cat(wmb_read_ptr.asUInt(WMB_ENTRY-2,0), wmb_read_ptr.asUInt(WMB_ENTRY-1)).asTypeOf(Vec(WMB_ENTRY, Bool()))

  //mem set must use write_ptr_to_next_3
  wmb_write_ptr_next(0) := wmb_write_ptr
  wmb_write_ptr_next(1) := Cat(wmb_write_ptr.asUInt(WMB_ENTRY-2,0), wmb_write_ptr.asUInt(WMB_ENTRY-1)).asTypeOf(Vec(WMB_ENTRY, Bool()))
  wmb_write_ptr_next(2) := Cat(wmb_write_ptr.asUInt(WMB_ENTRY-3,0), wmb_write_ptr.asUInt(WMB_ENTRY-1,WMB_ENTRY-2)).asTypeOf(Vec(WMB_ENTRY, Bool()))
  wmb_write_ptr_next(3) := Cat(wmb_write_ptr.asUInt(WMB_ENTRY-4,0), wmb_write_ptr.asUInt(WMB_ENTRY-1,WMB_ENTRY-3)).asTypeOf(Vec(WMB_ENTRY, Bool()))
  wmb_write_ptr_next(4) := Cat(wmb_write_ptr.asUInt(WMB_ENTRY-5,0), wmb_write_ptr.asUInt(WMB_ENTRY-1,WMB_ENTRY-4)).asTypeOf(Vec(WMB_ENTRY, Bool()))
  wmb_write_ptr_next(5) := Cat(wmb_write_ptr.asUInt(WMB_ENTRY-6,0), wmb_write_ptr.asUInt(WMB_ENTRY-1,WMB_ENTRY-5)).asTypeOf(Vec(WMB_ENTRY, Bool()))
  wmb_write_ptr_next(6) := Cat(wmb_write_ptr.asUInt(WMB_ENTRY-7,0), wmb_write_ptr.asUInt(WMB_ENTRY-1,WMB_ENTRY-6)).asTypeOf(Vec(WMB_ENTRY, Bool()))
  wmb_write_ptr_next(7) := Cat(wmb_write_ptr.asUInt(0), wmb_write_ptr.asUInt(WMB_ENTRY-1,WMB_ENTRY-7)).asTypeOf(Vec(WMB_ENTRY, Bool()))

  val wmb_write_ptr_to_next3 = (wmb_write_ptr.asUInt | wmb_write_ptr_next(1).asUInt | wmb_write_ptr_next(2).asUInt | wmb_write_ptr_next(3).asUInt).asTypeOf(Vec(WMB_ENTRY, Bool()))

  wmb_data_ptr_next1 := Cat(wmb_data_ptr.asUInt(WMB_ENTRY-2,0), wmb_data_ptr.asUInt(WMB_ENTRY-1)).asTypeOf(Vec(WMB_ENTRY, Bool()))

  //-------------judge if meet create signal------------------
  wmb_read_ptr_met_create := (wmb_read_ptr.asUInt === wmb_create_ptr.asUInt) && (wmb_read_ptr_circular === wmb_create_ptr_circular)

  wmb_write_ptr_met_create := (wmb_write_ptr.asUInt === wmb_create_ptr.asUInt) && (wmb_write_ptr_circular === wmb_create_ptr_circular)

  wmb_data_ptr_met_create := (wmb_data_ptr.asUInt === wmb_create_ptr.asUInt) && (wmb_data_ptr_circular === wmb_create_ptr_circular)

  //==========================================================
  //          Generate signal for sq pop
  //==========================================================
  //----------------can't merge signal------------------------
  val wmb_has_dcache_inst = ParallelORR(wmb_entry_out.map(entry_x => entry_x.dcache_inst && entry_x.vld))

  //------------------merge signal----------------------------
  val wmb_merge_data_stall = ParallelORR(wmb_entry_out.map(_.merge_data_stall))
  val wmb_merge_data_addr_hit = ParallelORR(wmb_entry_out.map(_.merge_data_addr_hit))

  //------------------wmb ce create signal--------------------
  io.out.toWmbCe.create_merge := io.in.fromSQ.wmb_merge_req && (wmb_merge_data_addr_hit || io.in.fromWmbCe.merge_data_addr_hit)

  io.out.toWmbCe.create_stall := wmb_merge_data_stall || io.in.fromWmbCe.merge_data_stall ||
    (wmb_merge_data_addr_hit || io.in.fromWmbCe.merge_data_addr_hit) && (io.in.fromSQ.wmb_merge_stall_req || wmb_has_dcache_inst)

  for(i <- 0 until WMB_ENTRY){
    io.out.toWmbCe.create_same_dcache_line(i) :=
      io.in.fromWmbCe.hit_sq_pop_dcache_line && wmb_create_vld && wmb_create_ptr(i) || wmb_entry_out(i).hit_sq_pop_dcache_line
    io.out.toWmbCe.create_merge_ptr(i) :=
      Mux(io.in.fromWmbCe.create_wmb_dp_req && io.in.fromWmbCe.merge_data_addr_hit, wmb_create_ptr(i), wmb_entry_out(i).merge_data_addr_hit)
  }

  val wmb_pop_to_ce_permit = wmb_sq_pop_grnt || !io.in.fromWmbCe.vld

  wmb_ce_create_vld := io.in.fromSQ.wmb_pop_to_ce_req && wmb_pop_to_ce_permit
  io.out.toWmbCe.create_vld := wmb_ce_create_vld

  io.out.toWmbCe.create_dp_vld := io.in.fromSQ.wmb_pop_to_ce_dp_req && wmb_pop_to_ce_permit

  io.out.toWmbCe.create_gateclk_en := io.in.fromSQ.wmb_pop_to_ce_gateclk_en && wmb_pop_to_ce_permit

  //for wmb entry
  wmb_ce_update_same_dcache_line_ptr := io.in.fromWmbCe.same_dcache_line.zip(wmb_entry_out).map(x => x._1 && x._2.vld)

  wmb_ce_update_same_dcache_line := wmb_ce_update_same_dcache_line_ptr.asUInt.orR
  //----------------------to sq-------------------------------
  io.out.toSQ.pop_to_ce_grnt := io.in.fromSQ.wmb_pop_to_ce_req && wmb_pop_to_ce_permit

  //==========================================================
  //          Generate grnt signal for wmb ce
  //==========================================================
  //------------------grnt signal-----------------------------
  val wmb_create_not_vld = Wire(Vec(WMB_ENTRY, Bool()))
  wmb_create_not_vld := wmb_create_ptr.zip(wmb_entry_out).map(x => x._1 && !x._2.vld)

  val wmb_create_permit = wmb_create_not_vld.asUInt.orR

  wmb_create_vld := wmb_create_permit && io.in.fromWmbCe.create_wmb_req

  val wmb_merge_vld = io.in.fromWmbCe.merge_wmb_req

  for(i <- 0 until WMB_ENTRY){
    wmb_entry_in(i).merge_data_vld := wmb_merge_vld && io.in.fromWmbCe.merge_ptr(i)
    wmb_entry_in(i).merge_data_wait_not_vld_req := io.in.fromWmbCe.merge_wmb_wait_not_vld_req && io.in.fromWmbCe.merge_ptr(i)
  }

  wmb_sq_pop_grnt := wmb_create_vld || wmb_merge_vld
  io.out.toSQ.pop_grnt := wmb_sq_pop_grnt

  io.out.toWmbCe.pop_vld := io.out.toSQ.pop_grnt

  //------------------create signal---------------------------
  for(i <- 0 until WMB_ENTRY){
    wmb_entry_in(i).create_vld        := wmb_create_not_vld(i) && io.in.fromWmbCe.create_wmb_req
    wmb_entry_in(i).create_dp_vld     := wmb_create_not_vld(i) && io.in.fromWmbCe.create_wmb_dp_req
    wmb_entry_in(i).create_gateclk_en := wmb_create_not_vld(i) && io.in.fromWmbCe.create_wmb_gateclk_en
    wmb_entry_in(i).create_data_vld   := wmb_create_not_vld(i) && io.in.fromWmbCe.create_wmb_data_req
  }

  //==========================================================
  //        Select signal from read/write/data ptr
  //==========================================================
  //-----------------read req info----------------------------wmb_entry_inst_flush
  wmb_read_req_unmask := ParallelORR(wmb_entry_out.map(_.read_req))

  val wmb_entry_out_read_sel = Mux1H(Seq(
    wmb_read_ptr(0) -> wmb_entry_out(0),
    wmb_read_ptr(1) -> wmb_entry_out(1),
    wmb_read_ptr(2) -> wmb_entry_out(2),
    wmb_read_ptr(3) -> wmb_entry_out(3),
    wmb_read_ptr(4) -> wmb_entry_out(4),
    wmb_read_ptr(5) -> wmb_entry_out(5),
    wmb_read_ptr(6) -> wmb_entry_out(6),
    wmb_read_ptr(7) -> wmb_entry_out(7)
  ))
  val wmb_read_req_atomic = wmb_entry_out_read_sel.atomic
  val wmb_read_req_icc = wmb_entry_out_read_sel.icc
  val wmb_read_req_inst_is_dcache = wmb_entry_out_read_sel.inst_is_dcache
  val wmb_read_req_inst_type = wmb_entry_out_read_sel.inst_type
  val wmb_read_req_inst_size = wmb_entry_out_read_sel.inst_size
  val wmb_read_req_inst_mode = wmb_entry_out_read_sel.inst_mode
  val wmb_read_req_data = wmb_entry_out_read_sel.data(15,0)
  val wmb_read_req_priv_mode = wmb_entry_out_read_sel.priv_mode
  val wmb_read_req_page_share = wmb_entry_out_read_sel.page_share
  val wmb_read_req_page_sec = wmb_entry_out_read_sel.page_sec

  val wmb_entry_out_read_next1_sel = Mux1H(Seq(
    wmb_read_ptr_next1(0) -> wmb_entry_out(0),
    wmb_read_ptr_next1(1) -> wmb_entry_out(1),
    wmb_read_ptr_next1(2) -> wmb_entry_out(2),
    wmb_read_ptr_next1(3) -> wmb_entry_out(3),
    wmb_read_ptr_next1(4) -> wmb_entry_out(4),
    wmb_read_ptr_next1(5) -> wmb_entry_out(5),
    wmb_read_ptr_next1(6) -> wmb_entry_out(6),
    wmb_read_ptr_next1(7) -> wmb_entry_out(7)
  ))
  val wmb_read_dp_req_next1 = wmb_entry_out_read_next1_sel.read_dp_req
  val wmb_read_req_addr_next1 = wmb_entry_out_read_next1_sel.addr

  //-----------------write req info---------------------------
  val wmb_write_imme_bypass = ParallelORR(wmb_entry_out.map(_.write_imme_bypass))
  val wmb_write_imme_other_bypass = ParallelORR(wmb_entry_out.zip(wmb_write_ptr).map(x => x._1.write_imme_bypass && !x._2))
  wmb_write_biu_req_unmask := ParallelORR(wmb_entry_out.map(_.write_biu_req))
  val wmb_write_vb_req_unmask = ParallelORR(wmb_entry_out.map(_.write_vb_req))

  val wmb_entry_out_write_sel = Mux1H(Seq(
    wmb_write_ptr(0) -> wmb_entry_out(0),
    wmb_write_ptr(1) -> wmb_entry_out(1),
    wmb_write_ptr(2) -> wmb_entry_out(2),
    wmb_write_ptr(3) -> wmb_entry_out(3),
    wmb_write_ptr(4) -> wmb_entry_out(4),
    wmb_write_ptr(5) -> wmb_entry_out(5),
    wmb_write_ptr(6) -> wmb_entry_out(6),
    wmb_write_ptr(7) -> wmb_entry_out(7)
  ))
  val wmb_write_req_inst_type = wmb_entry_out_write_sel.inst_type
  val wmb_write_req_inst_size = wmb_entry_out_write_sel.inst_size
  val wmb_write_req_inst_mode = wmb_entry_out_write_sel.inst_mode
  val wmb_write_req_priv_mode = wmb_entry_out_write_sel.priv_mode
  wmb_write_req_page_so := wmb_entry_out_write_sel.page_so
  val wmb_write_req_page_wa = wmb_entry_out_write_sel.page_wa
  val wmb_write_req_page_buf = wmb_entry_out_write_sel.page_buf
  val wmb_write_req_page_sec = wmb_entry_out_write_sel.page_sec

  val wmb_entry_out_write_next1_sel = Mux1H(Seq(
    wmb_write_ptr_next(1)(0) -> wmb_entry_out(0),
    wmb_write_ptr_next(1)(1) -> wmb_entry_out(1),
    wmb_write_ptr_next(1)(2) -> wmb_entry_out(2),
    wmb_write_ptr_next(1)(3) -> wmb_entry_out(3),
    wmb_write_ptr_next(1)(4) -> wmb_entry_out(4),
    wmb_write_ptr_next(1)(5) -> wmb_entry_out(5),
    wmb_write_ptr_next(1)(6) -> wmb_entry_out(6),
    wmb_write_ptr_next(1)(7) -> wmb_entry_out(7)
  ))
  val wmb_write_biu_dp_req_next1     = wmb_entry_out_write_next1_sel.write_biu_dp_req
  val wmb_write_req_atomic_next1     = wmb_entry_out_write_next1_sel.atomic_and_vld
  val wmb_write_req_icc_next1        = wmb_entry_out_write_next1_sel.icc_and_vld
  val wmb_write_req_sync_fence_next1 = wmb_entry_out_write_next1_sel.sync_fence
  val wmb_write_req_page_ca_next1    = wmb_entry_out_write_next1_sel.page_ca
  val wmb_write_req_page_share_next1 = wmb_entry_out_write_next1_sel.page_share
  val wmb_write_req_addr_next1       = wmb_entry_out_write_next1_sel.addr

  //-----------------------inst type--------------------------
  val wmb_write_req_st_inst = !wmb_write_req_atomic && !wmb_write_req_sync_fence && !wmb_write_req_icc
  val wmb_write_req_stex_inst = wmb_write_req_atomic

  //for write dcache_line check
  //wmb_write_req_ready_to_dcache_line(0) := ParallelORR(wmb_write_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.ready_to_dcache_line))
  for(i <- 0 until 4){
    wmb_write_req_ready_to_dcache_line(i) := ParallelORR(wmb_write_ptr_next(i).zip(wmb_entry_out).map(x => x._1 && x._2.ready_to_dcache_line))
    wmb_write_req_next_addr_plus(i) := ParallelORR(wmb_write_ptr_next(i).zip(wmb_entry_out).map(x => x._1 && x._2.last_addr_plus))
    wmb_write_req_next_addr_sub(i) := ParallelORR(wmb_write_ptr_next(i).zip(wmb_entry_out).map(x => x._1 && x._2.last_addr_sub))
  }
  val wmb_write_req_page_nc_atomic = !wmb_write_req_page_ca  &&  wmb_write_req_atomic
  //-----------------data req info----------------------------
  val wmb_data_biu_req = ParallelORR(wmb_entry_out.map(_.data_biu_req))
  val wmb_data_req_wns = ParallelORR(wmb_entry_out.map(_.data_req_wns))

  val wmb_entry_out_data_sel = Mux1H(Seq(
    wmb_data_ptr(0) -> wmb_entry_out(0),
    wmb_data_ptr(1) -> wmb_entry_out(1),
    wmb_data_ptr(2) -> wmb_entry_out(2),
    wmb_data_ptr(3) -> wmb_entry_out(3),
    wmb_data_ptr(4) -> wmb_entry_out(4),
    wmb_data_ptr(5) -> wmb_entry_out(5),
    wmb_data_ptr(6) -> wmb_entry_out(6),
    wmb_data_ptr(7) -> wmb_entry_out(7)
  ))
  val wmb_data_req_biu_id = wmb_entry_out_data_sel.biu_id
  val wmb_data_req_data = wmb_entry_out_data_sel.data
  val wmb_data_req_bytes_vld = wmb_entry_out_data_sel.bytes_vld
  val wmb_data_req_w_last = wmb_entry_out_data_sel.w_last

  //==========================================================
  //              write imme signal pop entry
  //==========================================================
  //wmb vld                                 mechanism
  //<=5                                     write leisure(!ld_ag && !st_ag)
  //>=6                                     write imme
  //                                        if (st_ag && st_rf),
  //                                        then write 2 cycle
  //amr and >=4                             write amr
  //-----------------------registers--------------------------
  //if wmb too full and must write dcache, and st_ag/rf has inst, then write
  //2 cycle to reduce st out of order
  when(wmb_write_imme_hold_set){
    wmb_write_imme_hold := true.B
  }.otherwise{
    wmb_write_imme_hold := false.B
  }

  //if sq pop write imme, then wmb_write_imme set immediately
  when(wmb_create_write_imme_set || wmb_write_imme_other_bypass){
    wmb_write_imme := true.B
  }.elsewhen(wmb_write_imme_amr_clr || wmb_write_imme_clr){
    wmb_write_imme := false.B
  }.elsewhen(wmb_write_imme_set){
    wmb_write_imme := true.B
  }

  //------------------------signals---------------------------
  wmb_write_imme_hold_set  := !wmb_write_imme_hold && !wmb_write_imme_amr_clr && !wmb_write_imme_clr &&
    wmb_write_imme_bypass && io.in.st_rf_inst_vld && io.in.st_ag_inst_vld

  wmb_create_write_imme_set := io.in.fromWmbCe.create_wmb_req && io.in.fromWmbCe.write_imme

  val wmb_write_imme_ori = ParallelORR(wmb_entry_out.map(_.write_imme))
  wmb_write_imme_set := io.in.icc_wmb_write_imme || io.in.fromCp0.lsu_no_op_req|| wmb_write_imme_bypass || wmb_write_imme_hold || !wmb_write_imme && wmb_write_imme_ori

  val wmb_other_write_imme = ParallelORR(wmb_write_ptr.zip(wmb_entry_out).map(x => !x._1 && x._2.write_imme))
  val wmb_other4_write_imme = ParallelORR(wmb_write_ptr_to_next3.zip(wmb_entry_out).map(x => !x._1 && x._2.write_imme))

  wmb_write_imme_amr_clr := !wmb_other4_write_imme && wmb_mem_set_write_grnt

  wmb_write_imme_clr := !wmb_other_write_imme && !wmb_write_imme_hold && wmb_write_ptr_shift_vld || wmb_empty

  val wmb_write_dcache_permit= wmb_write_imme || !io.in.st_ag_inst_vld && !io.in.ld_ag_inst_vld

  //==========================================================
  //            generate reset_read_ptr_req logic
  //==========================================================
  //if (wmb_reset_read_ptr_req_ff && wmb_ctc_secd)
  //  then wmb_reset_read_ptr_req_ff hold, and send read req
  //else if (wmb_reset_read_ptr_req_ff && !wmb_ctc_secd)
  //  then wmb_reset_read_ptr_req_ff clear, and do not send read req
  //--------------------reset_ptr_req_ff----------------------
  //no need anymore
  //==========================================================
  //                last create pop entry
  //==========================================================
  //used for write burst
  when(wmb_create_vld){
    wmb_last_create_addr := io.in.fromWmbCe.addr
  }

  //---------------------entry create signal------------------------
  val wmb_ce_last_addr_eq_high = io.in.fromWmbCe.addr(PA_WIDTH-1,6) === wmb_last_create_addr(PA_WIDTH-1,6)

  val wmb_last_addr_plus = wmb_last_create_addr(5,4) + 1.U(2.W)
  val wmb_last_addr_sub = wmb_last_create_addr(5,4) - 1.U(2.W)

  wmb_ce_last_addr_plus := wmb_ce_last_addr_eq_high && io.in.fromWmbCe.addr(5,4) === wmb_last_addr_plus
  wmb_ce_last_addr_sub  := wmb_ce_last_addr_eq_high && io.in.fromWmbCe.addr(5,4) === wmb_last_addr_sub

  //==========================================================
  //                Read ptr pop entry
  //==========================================================
  //+-------------+
  //| read_dp_req |
  //+-------------+
  when(wmb_empty && !io.in.fromWmbCe.vld){
    wmb_read_dp_req := false.B
  }.elsewhen(io.in.fromDcache.vb_snq_gwen){
    wmb_read_dp_req := true.B
  }.elsewhen(wmb_read_pop_up_wmb_ce_vld){
    wmb_read_dp_req := io.in.fromWmbCe.read_dp_req
  }.elsewhen(wmb_read_ptr_next1_met_create &&  wmb_read_ptr_shift_vld){
    wmb_read_dp_req := false.B
  }.elsewhen(wmb_read_ptr_shift_vld){
    wmb_read_dp_req :=  wmb_read_dp_req_next1
  }

  //+---------------+
  //| read_req_addr |
  //+---------------+
  when(wmb_read_pop_up_wmb_ce_vld){
    wmb_read_req_addr := io.in.fromWmbCe.addr
  }.elsewhen(wmb_read_ptr_shift_vld){
    wmb_read_req_addr := wmb_read_req_addr_next1
  }

  //---------------------update signal------------------------
  wmb_read_ptr_next1_met_create := wmb_create_ptr.asUInt === wmb_read_ptr_next1.asUInt

  wmb_read_pop_up_wmb_ce_vld := io.in.fromWmbCe.create_wmb_dp_req &&
    (wmb_read_ptr_met_create || wmb_read_ptr_next1_met_create && wmb_read_ptr_shift_vld)

  //for same_dcache_line
  wmb_same_line_resp_ready := wmb_entry_out.map(_.read_resp_ready)
  //==========================================================
  //                Write ptr pop entry
  //==========================================================
  //+------------------+
  //| write_biu_dp_req |
  //+------------------+
  when(wmb_empty && !io.in.fromWmbCe.vld){
    wmb_write_biu_dp_req := false.B
  }.elsewhen(io.in.fromDcache.vb_snq_gwen || wmb_write_dcache_req_icc_inst){
    wmb_write_biu_dp_req := true.B
  }.elsewhen(wmb_write_pop_up_wmb_ce_vld){
    wmb_write_biu_dp_req := io.in.fromWmbCe.write_biu_dp_req
  }.elsewhen(wmb_mem_set_write_grnt){
    wmb_write_biu_dp_req := true.B
  }.elsewhen(wmb_write_ptr_shift_vld){
    wmb_write_biu_dp_req := wmb_write_biu_dp_req_next1
  }

  //+----------------+
  //| write_pop_addr |
  //+----------------+
  when(wmb_write_pop_up_wmb_ce_vld){
    wmb_write_req_atomic     := io.in.fromWmbCe.atomic
    wmb_write_req_icc        := io.in.fromWmbCe.icc
    wmb_write_req_sync_fence := io.in.fromWmbCe.sync_fence
    wmb_write_req_page_ca    := io.in.fromWmbCe.page_ca
    wmb_write_req_page_share := io.in.fromWmbCe.page_share
    wmb_write_req_addr       := io.in.fromWmbCe.addr
  }.elsewhen(wmb_write_ptr_shift_vld){
    wmb_write_req_atomic     := wmb_write_req_atomic_next1
    wmb_write_req_icc        := wmb_write_req_icc_next1
    wmb_write_req_sync_fence := wmb_write_req_sync_fence_next1
    wmb_write_req_page_ca    := wmb_write_req_page_ca_next1
    wmb_write_req_page_share := wmb_write_req_page_share_next1
    wmb_write_req_addr       := wmb_write_req_addr_next1
  }.otherwise{
    wmb_write_req_atomic     := wmb_write_req_atomic
    wmb_write_req_icc        := wmb_write_req_icc
    wmb_write_req_sync_fence := wmb_write_req_sync_fence
    wmb_write_req_page_ca    := wmb_write_req_page_ca
    wmb_write_req_page_share := wmb_write_req_page_share
    wmb_write_req_addr       := wmb_write_req_addr
  }

  //---------------------update signal------------------------
  val wmb_write_ptr_next1_met_create = wmb_create_ptr.asUInt === wmb_write_ptr_next(1).asUInt

  wmb_write_pop_up_wmb_ce_vld := io.in.fromWmbCe.create_wmb_dp_req &&
    (wmb_write_ptr_met_create || wmb_write_ptr_next1_met_create && wmb_write_ptr_shift_vld)

  wmb_write_pop_up_wmb_ce_gateclk_en := io.in.fromWmbCe.create_wmb_gateclk_en &&
    (wmb_write_ptr_met_create || wmb_write_ptr_next1_met_create)

  //==========================================================
  //        Request biu ar channel(include ctc request)
  //==========================================================
  //----------------------hit_idx-----------------------------
  wmb_read_req_hit_idx := io.in.fromSnq.create_wmb_read_req_hit_idx && !io.in.fromLm.state_is_amo_lock ||
    io.in.fromSnq.wmb_read_req_hit_idx || io.in.fromLfb.read_req_hit_idx

  //-----------------ctc register-----------------------------
  when(wmb_read_req_ctc_inst && !wmb_read_req_ctc_end && io.in.fromBusArb.ar_grnt){
    wmb_ctc_secd := true.B
  }.elsewhen(io.in.fromBusArb.ar_grnt){
    wmb_ctc_secd := false.B
  }

  //-----------------ctc end singal-----------------------------
  wmb_read_req_ctc_end := !wmb_biu_ar_addr(0)
  //-----------------inst type--------------------------------
  val wmb_read_req_dcache_va_l1_inst = !wmb_read_req_atomic && wmb_read_req_icc && wmb_read_req_inst_is_dcache &&
    (wmb_read_req_inst_mode === 1.U(2.W)) && (wmb_read_req_inst_size === 0.U(2.W))

  wmb_read_req_ctc_inst := !wmb_read_req_atomic && wmb_read_req_icc && (wmb_read_req_inst_type =/= 2.U(2.W))

  val wmb_read_req_tlbi_inst = !wmb_read_req_atomic && wmb_read_req_icc && (wmb_read_req_inst_type === 0.U(2.W))

  val wmb_read_req_tlbi_asid_inst  = wmb_read_req_tlbi_inst && wmb_read_req_inst_mode(0)

  val wmb_read_req_tlbi_va_inst    = wmb_read_req_tlbi_inst && wmb_read_req_inst_mode(1)

  val wmb_read_req_icache_inst     = !wmb_read_req_atomic && wmb_read_req_icc && (wmb_read_req_inst_type === 1.U(2.W))

  val wmb_read_req_icache_all_inst = wmb_read_req_icache_inst && (wmb_read_req_inst_mode === 0.U(2.W))

  val wmb_read_req_l2cache_inst    = !wmb_read_req_atomic && wmb_read_req_icc && (wmb_read_req_inst_type === 3.U(2.W))

  //-----------------interface to bus_arb---------------------
  io.out.toBiu.ar_req := wmb_read_req_unmask && (!wmb_read_req_hit_idx || wmb_read_req_ctc_inst)

  io.out.toBiu.ar_dp_req := wmb_read_req_unmask

  io.out.toBiu.ar_req_gateclk_en := wmb_read_dp_req

  wmb_biu_ar_id := Mux(wmb_read_req_ctc_inst, BIU_R_CTC_ID, Cat(BIU_R_NORM_ID_T, wmb_read_ptr_encode))
  io.out.toBiu.ar_id := wmb_biu_ar_id

  //-----------------addr-----------------
  val wmb_biu_ar_tlbi_first_addr =
      Cat(wmb_read_req_data(15,8),  //ASID[15:8]    39:32
      0.U(8.W),                     //
      wmb_read_req_data(7,0),       //ASID[7:0]     23:16
      0.U(1.W),                     //              15
      0.U(3.W),                     //TLBI          14:12
      2.U(2.W),                     //Guest?        11:10
      2.U(2.W),                     //non-sec       9:8
      0.U(1.W),                     //              7
      0.U(1.W),                     //`PA_WIDTH-1:24 not vld 6
      wmb_read_req_tlbi_asid_inst,  //23:16 vld     5
      0.U(4.W),                     //              4:1
      wmb_read_req_tlbi_va_inst)    //need secd     0

  val wmb_biu_ar_tlbi_secd_addr =
      Cat(wmb_read_req_addr(PA_WIDTH-1,4), //VA            PA_WIDTH-1:4
      0.U(3.W),                            //              3:1
      1.U(1.W))                            //end           0

  val wmb_biu_ar_icache_first_addr =
      Cat(0.U(24.W),                  //            PA_WIDTH-1:16
      0.U(1.W),                       //              15
      2.U(3.W),                       //ICACHEI       14:12
      0.U(2.W),                       //Guest?        11:10
      2.U(2.W),                       //              9:8
      0.U(1.W),                       //              7
      0.U(2.W),                       //              6:5
      0.U(4.W),                       //              4:1
      !wmb_read_req_icache_all_inst)  //need secd?    0

  val wmb_biu_ar_icache_secd_addr =
      Cat(wmb_read_req_addr(PA_WIDTH-1,4), //VA            PA_WIDTH-1:4
      0.U(3.W),                            //              3:1
      1.U(1.W))                            //end           0

  val wmb_biu_ar_l2cache_first_addr =
      Cat(0.U((PA_WIDTH-24).W),       //              PA_WIDTH-1:24
      0.U(6.W),                       //              23:18
      wmb_read_req_inst_size(0),      //CLEAR         17
      wmb_read_req_inst_size(1),      //INV           16
      0.U(1.W),                       //              15
      7.U(3.W),                       //L2CACHE       14:12
      2.U(2.W),                       //Guest?        11:10
      2.U(2.W),                       //non-sec       9:8
      0.U(1.W),                       //              7
      0.U(1.W),                       //PA_WIDTH-1:24 not vld 6
      0.U(1.W),                       //23:16 not vld 5
      0.U(4.W),                       //              4:1
      0.U(1.W))                       //end           0

  val wmb_biu_ar_addr_judge = Cat(wmb_ctc_secd, wmb_read_req_tlbi_inst, wmb_read_req_icache_inst, wmb_read_req_l2cache_inst)

  wmb_biu_ar_addr := MuxLookup(wmb_biu_ar_addr_judge, Cat(wmb_read_req_addr(PA_WIDTH-1,6), 0.U(6.W)), Seq(
    "b0100".U -> wmb_biu_ar_tlbi_first_addr,
    "b1100".U -> wmb_biu_ar_tlbi_secd_addr,
    "b0010".U -> wmb_biu_ar_icache_first_addr,
    "b1010".U -> wmb_biu_ar_icache_secd_addr,
    "b0001".U -> wmb_biu_ar_l2cache_first_addr
  ))
  io.out.toBiu.ar_addr := wmb_biu_ar_addr

  //-----------------others-----------------
  //ctc   : 1
  //other : 3
  io.out.toBiu.ar_len := Mux(wmb_read_req_ctc_inst, 0.U(2.W), 3.U(2.W))
  //128 bits
  io.out.toBiu.ar_size := 4.U(3.W)
  //increase
  io.out.toBiu.ar_burst := 1.U(2.W)
  //not exclusive
  io.out.toBiu.ar_lock := false.B

  //ctc 0010
  //dcache.l1 0011
  //others 1111
  io.out.toBiu.ar_cache := MuxLookup(Cat(wmb_read_req_ctc_inst,wmb_read_req_dcache_va_l1_inst), "b1111".U, Seq(
    "b10".U -> "b0010".U,
    "b01".U -> "b0011".U
  ))

  io.out.toBiu.ar_prot := Cat(0.U(1.W), wmb_read_req_page_sec, wmb_read_req_priv_mode(0))

  io.out.toBiu.ar_user := Cat(0.U(1.W), wmb_read_req_priv_mode(1), 0.U(1.W))

  //-----------------snoop----------------
  //st cleanunique 1011
  //dcache only clr cleanshared  1000
  //dcache only clr l1 cleanshared  1000
  //dcache clr inv  cleaninvalid 1001
  //dcache only inv makeinvalid  1101
  //ctc 1111
  val ar_snoop_sel_value = MuxLookup(wmb_read_req_inst_size, "b1011".U, Seq(
    "b01".U -> "b1000".U,//CleanShared
    "b00".U -> "b1000".U,//CleanShared
    "b10".U -> "b1101".U,//MakeInvalid
    "b11".U -> "b1001".U//CleanInvalid
  ))

  val ar_snoop_sel = Cat(wmb_read_req_atomic,wmb_read_req_icc,wmb_read_req_inst_type)

  io.out.toBiu.ar_snoop := MuxLookup(ar_snoop_sel, "b1011".U, Seq(
    "b0_1_10".U -> ar_snoop_sel_value,
    "b0_1_00".U -> "b1111".U,//CTC
    "b0_1_01".U -> "b1111".U,
    "b0_1_11".U -> "b1111".U
  ))

  io.out.toBiu.ar_domain := Cat(0.U(1.W), wmb_read_req_page_share)

  io.out.toBiu.ar_bar := 0.U(2.W)

  //==========================================================
  //                    Request dcache
  //==========================================================
  //-----------------dcache req ptr---------------------------
  val wmb_write_dcache_pop_up = wmb_write_dcache_success || !wmb_write_dcache_pop_req && wmb_dcache_req_next

  wmb_dcache_req_ptr := Mux(wmb_write_dcache_pop_req, wmb_write_dcache_ptr, VecInit(Seq.fill(WMB_ENTRY)(false.B)))
  val wmb_dcache_req_set = Wire(Vec(WMB_ENTRY, Bool()))
  for(i <- 0 until WMB_ENTRY){
    wmb_dcache_req_set(i) := wmb_entry_out(i).write_dcache_req && !wmb_dcache_req_ptr(i)
  }

  wmb_dcache_req_next := wmb_dcache_req_set.asUInt.orR

  when(wmb_write_dcache_pop_up){
    wmb_write_dcache_pop_req := wmb_dcache_req_next
    wmb_write_dcache_ptr := wmb_write_dcache_ptr_set
    wmb_write_dcache_addr := wmb_write_dcache_addr_set
  }

  //sel dcache req entry
  val wmb_write_dcache_priority = PriorityEncoderOH(MuxLookup(wmb_write_ptr.asUInt, 0.U(8.W), Seq(
    "b00000001".U -> wmb_dcache_req_set.asUInt,
    "b00000010".U -> Cat(wmb_dcache_req_set.asUInt(0),wmb_dcache_req_set.asUInt(7,1)),
    "b00000100".U -> Cat(wmb_dcache_req_set.asUInt(1,0),wmb_dcache_req_set.asUInt(7,2)),
    "b00001000".U -> Cat(wmb_dcache_req_set.asUInt(2,0),wmb_dcache_req_set.asUInt(7,3)),
    "b00010000".U -> Cat(wmb_dcache_req_set.asUInt(3,0),wmb_dcache_req_set.asUInt(7,4)),
    "b00100000".U -> Cat(wmb_dcache_req_set.asUInt(4,0),wmb_dcache_req_set.asUInt(7,5)),
    "b01000000".U -> Cat(wmb_dcache_req_set.asUInt(5,0),wmb_dcache_req_set.asUInt(7,6)),
    "b10000000".U -> Cat(wmb_dcache_req_set.asUInt(6,0),wmb_dcache_req_set.asUInt(7))
  )))

  wmb_write_dcache_ptr_set := MuxLookup(wmb_write_dcache_priority.asUInt, VecInit(Seq.fill(WMB_ENTRY)(false.B)), Seq(
    "b00000001".U -> wmb_write_ptr,
    "b00000010".U -> wmb_write_ptr_next(1),
    "b00000100".U -> wmb_write_ptr_next(2),
    "b00001000".U -> wmb_write_ptr_next(3),
    "b00010000".U -> wmb_write_ptr_next(4),
    "b00100000".U -> wmb_write_ptr_next(5),
    "b01000000".U -> wmb_write_ptr_next(6),
    "b10000000".U -> wmb_write_ptr_next(7)
  ))

  wmb_write_dcache_addr_set := Mux1H(Seq(
    wmb_write_dcache_ptr_set(0) -> wmb_entry_out(0).addr,
    wmb_write_dcache_ptr_set(1) -> wmb_entry_out(1).addr,
    wmb_write_dcache_ptr_set(2) -> wmb_entry_out(2).addr,
    wmb_write_dcache_ptr_set(3) -> wmb_entry_out(3).addr,
    wmb_write_dcache_ptr_set(4) -> wmb_entry_out(4).addr,
    wmb_write_dcache_ptr_set(5) -> wmb_entry_out(5).addr,
    wmb_write_dcache_ptr_set(6) -> wmb_entry_out(6).addr,
    wmb_write_dcache_ptr_set(7) -> wmb_entry_out(7).addr
  ))

  //dcache pop signal
  val wmb_write_req_dcache_way = (wmb_write_dcache_ptr.asUInt & VecInit(wmb_entry_out.map(_.dcache_way)).asUInt).orR

  val wmb_write_dcache_bytes_vld = Mux1H(Seq(
    wmb_write_dcache_ptr(0) -> wmb_entry_out(0).bytes_vld,
    wmb_write_dcache_ptr(1) -> wmb_entry_out(1).bytes_vld,
    wmb_write_dcache_ptr(2) -> wmb_entry_out(2).bytes_vld,
    wmb_write_dcache_ptr(3) -> wmb_entry_out(3).bytes_vld,
    wmb_write_dcache_ptr(4) -> wmb_entry_out(4).bytes_vld,
    wmb_write_dcache_ptr(5) -> wmb_entry_out(5).bytes_vld,
    wmb_write_dcache_ptr(6) -> wmb_entry_out(6).bytes_vld,
    wmb_write_dcache_ptr(7) -> wmb_entry_out(7).bytes_vld
  ))

  val wmb_write_dcache_data = Mux1H(Seq(
    wmb_write_dcache_ptr(0) -> wmb_entry_out(0).data,
    wmb_write_dcache_ptr(1) -> wmb_entry_out(1).data,
    wmb_write_dcache_ptr(2) -> wmb_entry_out(2).data,
    wmb_write_dcache_ptr(3) -> wmb_entry_out(3).data,
    wmb_write_dcache_ptr(4) -> wmb_entry_out(4).data,
    wmb_write_dcache_ptr(5) -> wmb_entry_out(5).data,
    wmb_write_dcache_ptr(6) -> wmb_entry_out(6).data,
    wmb_write_dcache_ptr(7) -> wmb_entry_out(7).data
  ))

  val wmb_write_dcache_stall = (wmb_write_dcache_ptr.asUInt & VecInit(wmb_entry_out.map(_.write_stall)).asUInt).orR

  val wmb_write_dcache_sync_fence = (wmb_write_dcache_ptr.asUInt & VecInit(wmb_entry_out.map(_.sync_fence)).asUInt).orR
  val wmb_write_dcache_atomic     = (wmb_write_dcache_ptr.asUInt & VecInit(wmb_entry_out.map(_.atomic_and_vld)).asUInt).orR
  val wmb_write_dcache_icc        = (wmb_write_dcache_ptr.asUInt & VecInit(wmb_entry_out.map(_.icc_and_vld)).asUInt).orR

  wmb_write_dcache_req_icc_inst := !wmb_write_dcache_atomic && !wmb_write_dcache_sync_fence && wmb_write_dcache_icc

  //----------------------hit_idx-----------------------------
  wmb_write_req_hit_idx := io.in.fromLfb.write_req_hit_idx || io.in.fromVB.write_req_hit_idx

  //for dcache_inst
  wmb_dcache_inst_write_req_hit_idx := io.in.fromLfb.write_req_hit_idx || io.in.fromSnq.create_wmb_write_req_hit_idx ||
    io.in.fromSnq.wmb_write_req_hit_idx ||io.in.fromVB.write_req_hit_idx

  //-----------------for ecc ---------------------------------
  val pw_read           = false.B
  val pw_ecc_idle       = true.B
  pw_merge_stall       := false.B
  val wmb_ecc_write_req = false.B
  val wmb_ecc_fatal_err = false.B

  io.out.toDcacheArb.ld_borrow_req := wmb_dcache_arb_req && pw_read
  io.out.toDcacheArb.data_way := wmb_dcache_data_high_sel

  val wmb_write_dcache_wen = Mux(wmb_ecc_write_req, 0xffff.U, wmb_write_dcache_bytes_vld)

  //----------------------cache interface---------------------
  wmb_dcache_arb_req_unmask :=
    wmb_write_dcache_pop_req && pw_ecc_idle && !wmb_write_dcache_stall && wmb_write_dcache_permit || wmb_ecc_write_req

  wmb_dcache_arb_req := wmb_dcache_arb_req_unmask
  io.out.toDcacheArb.ld_req := wmb_dcache_arb_req
  io.out.toDcacheArb.st_req := wmb_dcache_arb_req

  val wmb_write_dcache_success_ori = wmb_dcache_arb_req && io.in.fromDcache.arb_wmb_ld_grnt

  //for ecc pw read
  wmb_write_dcache_success := wmb_write_dcache_success_ori && (!pw_read || wmb_ecc_write_req) || wmb_ecc_fatal_err
  //----------------tag array-------------
  io.out.toDcacheArb.ld_tag_gateclk_en := wmb_dcache_arb_req_unmask && !pw_read && wmb_write_dcache_req_icc_inst

  io.out.toDcacheArb.ld_tag_req := wmb_dcache_arb_req_unmask && !pw_read && wmb_write_dcache_req_icc_inst

  io.out.toDcacheArb.ld_tag_idx := wmb_write_dcache_addr(14,6)
  io.out.toDcacheArb.ld_tag_wen := Cat(wmb_write_req_dcache_way, !wmb_write_req_dcache_way)
  //---------------dirty array------------
  io.out.toDcacheArb.st_dirty_gateclk_en := wmb_dcache_arb_req_unmask && !pw_read

  io.out.toDcacheArb.st_dirty_req := wmb_dcache_arb_req_unmask && !pw_read

  io.out.toDcacheArb.st_dirty_idx := wmb_write_dcache_addr(14,6)
  //fifo,dirty1,share1,valid1,dirty0,share0,valid0
  io.out.toDcacheArb.st_dirty_din := Mux(wmb_write_dcache_req_icc_inst, 0.U(7.W), "b0101101".U)
  io.out.toDcacheArb.st_dirty_wen :=
    Cat(0.U(1.W), VecInit(Seq.fill(3)(wmb_write_req_dcache_way)).asUInt, VecInit(Seq.fill(3)(!wmb_write_req_dcache_way)).asUInt)

  //---------------data array-------------
  //pre signal
  val wmb_dcache_data_region = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    wmb_dcache_data_region(i) := wmb_write_dcache_bytes_vld(i*4+3,i*4).orR
  }

  wmb_dcache_data_high_sel := wmb_write_req_dcache_way ^ wmb_write_dcache_addr(4)

  val wmb_dcache_arb_ld_data_req_unmask =
    Cat(wmb_dcache_data_region.asUInt,wmb_dcache_data_region.asUInt) &
    Cat(VecInit(Seq.fill(4)(wmb_dcache_data_high_sel)).asUInt,VecInit(Seq.fill(4)(!wmb_dcache_data_high_sel)).asUInt) &
    VecInit(Seq.fill(8)(wmb_dcache_arb_req_unmask)).asUInt

  io.out.toDcacheArb.ld_data_req := wmb_dcache_arb_ld_data_req_unmask
  io.out.toDcacheArb.ld_data_gateclk_en := wmb_dcache_arb_ld_data_req_unmask

  io.out.toDcacheArb.ld_data_idx := Cat(wmb_write_dcache_addr(14,5),wmb_write_req_dcache_way)
  io.out.toDcacheArb.ld_data_low_din  := wmb_write_dcache_data
  io.out.toDcacheArb.ld_data_high_din := wmb_write_dcache_data
  io.out.toDcacheArb.ld_data_gwen := Mux(pw_read, 0.U(8.W), wmb_dcache_arb_ld_data_req_unmask)

  io.out.toDcacheArb.ld_data_wen := Cat(wmb_write_dcache_wen,wmb_write_dcache_wen) &
    Cat(VecInit(Seq.fill(16)(wmb_dcache_data_high_sel)).asUInt,VecInit(Seq.fill(16)(!wmb_dcache_data_high_sel)).asUInt)

  //==========================================================
  //                  Request victim buffer
  //==========================================================
  //req signal is used for arbitration
  io.out.toVB.create_req := wmb_write_vb_req_unmask

  //only create vb should check snq
  wmb_vb_create_vld := wmb_write_vb_req_unmask && !wmb_dcache_inst_write_req_hit_idx
  io.out.toVB.create_vld := wmb_vb_create_vld

  io.out.toVB.create_dp_vld := wmb_write_vb_req_unmask
  io.out.toVB.create_gateclk_en := wmb_write_vb_req_unmask
  io.out.toVB.inv := wmb_write_req_inst_size(1)
  io.out.toVB.set_way_mode := wmb_write_req_inst_mode === 2.U(2.W)
  io.out.toVB.addr_tto6 := wmb_write_req_addr(PA_WIDTH-1,6)

  //==========================================================
  //      Request biu aw channel(include mem set request)
  //==========================================================
  //-----------------------inst type--------------------------
  val wmb_write_req_sync_fence_inst  = wmb_write_req_sync_fence && !wmb_write_req_atomic

  val wmb_write_req_sync_inst  = wmb_write_req_sync_fence && !wmb_write_req_atomic && wmb_write_req_inst_type === 0.U(2.W)

  //-----------------interface to bus_arb---------------------
  wmb_biu_write_en := wmb_write_imme || wmb_write_biu_dcache_line

  io.out.toBiu.aw_req := wmb_write_biu_dp_req && wmb_biu_write_en && wmb_write_biu_req_unmask &&
    (!io.in.fromRB.wmb_so_pending || !wmb_write_req_page_so) && (!wmb_write_req_hit_idx || wmb_write_req_sync_fence_inst)

  io.out.toBiu.aw_dp_req := wmb_write_biu_dp_req && wmb_biu_write_en && wmb_write_biu_req_unmask &&
    (!io.in.fromRB.wmb_so_pending || !wmb_write_req_page_so)

  io.out.toBiu.aw_req_gateclk_en := wmb_write_biu_dp_req
  //-----------------id-------------------
  wmb_biu_aw_id := MuxCase(0.U(5.W) ,Seq(
    wmb_write_req_sync_fence_inst                         -> BIU_B_SYNC_FENCE_ID,
    wmb_write_req_page_so                                 -> BIU_B_SO_ID,
    (wmb_write_req_page_ca && !wmb_write_biu_dcache_line) -> Cat(BIU_R_NORM_ID_T,wmb_write_ptr_encode),
    (wmb_write_req_page_ca && wmb_write_biu_dcache_line)  -> Cat(BIU_R_NORM_ID_T,wmb_write_ptr_next3_encode),
    wmb_write_req_atomic                                  -> BIU_B_NC_ATOM_ID,
    (!wmb_write_req_sync_fence_inst && wmb_write_req_page_so && !wmb_write_req_page_ca && !wmb_write_req_atomic) -> BIU_B_NC_ID
  ))
  io.out.toBiu.aw_id := wmb_biu_aw_id

  val wmb_biu_aw_size_maintain = (wmb_write_req_page_so || wmb_write_req_page_nc_atomic) && !wmb_write_req_sync_fence_inst

  io.out.toBiu.aw_addr := Cat(wmb_write_req_addr(PA_WIDTH-1,4), Mux(wmb_biu_aw_size_maintain, wmb_write_req_addr(3,0), 0.U(4.W)))

  io.out.toBiu.aw_len := Mux(wmb_write_biu_dcache_line, 3.U(2.W), 0.U(2.W))

  io.out.toBiu.aw_size := Mux(wmb_biu_aw_size_maintain, wmb_write_req_inst_size, 4.U(3.W))

  io.out.toBiu.aw_burst := Mux(wmb_write_biu_dcache_line &&  wmb_write_burst_neg, 3.U(2.W), 1.U(2.W))

  io.out.toBiu.aw_lock := !wmb_write_req_page_ca && wmb_write_req_atomic

  //cache
  //if sync/fence use normal, noncacheable
  io.out.toBiu.aw_cache := Mux(wmb_write_req_sync_fence_inst, "b0011".U,
    Cat(wmb_write_req_page_wa && !io.in.amr_l2_mem_set, wmb_write_req_page_ca, !wmb_write_req_page_so, wmb_write_req_page_buf))

  //prot:supv,sec,inst
  io.out.toBiu.aw_prot := Cat(0.U(1.W) ,wmb_write_req_page_sec, wmb_write_req_priv_mode(0))

  io.out.toBiu.aw_user := wmb_write_req_priv_mode(1)

  //-----------------snoop----------------
  //for single core,send wu or wlu when not wns_en
  val wmb_write_req_no_wns = true.B

  io.out.toBiu.aw_snoop := Mux(wmb_write_req_page_ca && wmb_write_req_no_wns && wmb_write_biu_dcache_line, 1.U(3.W), 0.U(3.W))

  val wmb_write_req_default_domain = !wmb_write_req_page_ca && (wmb_write_req_stex_inst || wmb_write_req_st_inst)

  io.out.toBiu.aw_domain := Mux(wmb_write_req_default_domain, 3.U(2.W), Cat(0.U(1.W), wmb_write_req_no_wns))

  io.out.toBiu.aw_bar := Cat(wmb_write_req_sync_inst,wmb_write_req_sync_fence_inst)

  //-----------------mem_set counter--------------------------
  wmb_mem_set_write_grnt := wmb_write_biu_dcache_line && io.in.fromBusArb.aw_grnt

  wmb_entry_in.zip(wmb_write_ptr_to_next3).foreach(x => x._1.mem_set_write_grnt := wmb_mem_set_write_grnt && x._2)

  //for timing use mem_set write gateclk en for biu_id clk
  val wmb_mem_set_write_gateclk_en = wmb_write_biu_req_unmask && wmb_write_biu_dcache_line

  wmb_entry_in.zip(wmb_write_ptr_to_next3).foreach(x => x._1.mem_set_write_gateclk_en := wmb_mem_set_write_gateclk_en && x._2)

  //-----------------biu grnt signal--------------------------
  wmb_biu_nc_req_grnt := io.in.fromBusArb.aw_grnt && !wmb_write_req_page_ca && !wmb_write_req_page_so && !wmb_write_req_atomic && !wmb_write_req_sync_fence_inst

  wmb_biu_so_req_grnt := io.in.fromBusArb.aw_grnt && wmb_write_req_page_so && !wmb_write_req_sync_fence_inst

  //set wmb_entry_w_last signal
  for(i <- 0 until WMB_ENTRY){
    wmb_entry_in(i).w_last_set := Mux(wmb_write_biu_dcache_line, wmb_write_ptr_next(3)(i), wmb_write_ptr(i))
  }

  //==========================================================
  //                Request biu w channel
  //==========================================================
  io.out.toBiu.w_req          := wmb_data_biu_req
  //w_id is used for debug
  io.out.toBiu.w_id           := wmb_data_req_biu_id
  io.out.toBiu.w_data         := wmb_data_req_data
  io.out.toBiu.w_strb         := wmb_data_req_bytes_vld
  io.out.toBiu.w_last         := wmb_data_req_w_last
  io.out.toBiu.w_vld          := wmb_data_biu_req
  io.out.toBiu.w_wns          := wmb_data_req_wns

  //==========================================================
  //                Request wb cmplt part
  //==========================================================
  val wmb_st_wb_cmplt_ptr = VecInit(PriorityEncoderOH(wmb_entry_out.map(_.wb_cmplt_req)))

  //-----------------cmplt req info---------------------------
  io.out.toStoreWB.cmplt_req  := VecInit(wmb_entry_out.map(_.wb_cmplt_req)).asUInt.orR
  io.out.toStoreWB.inst_flush := VecInit(wmb_st_wb_cmplt_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.inst_flush)).asUInt.orR
  io.out.toStoreWB.spec_fail  := VecInit(wmb_st_wb_cmplt_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.spec_fail)).asUInt.orR
  io.out.toStoreWB.bkpta_data := VecInit(wmb_st_wb_cmplt_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.bkpta_data)).asUInt.orR
  io.out.toStoreWB.bkptb_data := VecInit(wmb_st_wb_cmplt_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.bkptb_data)).asUInt.orR
  io.out.toStoreWB.iid        := Mux1H(Seq(
    wmb_st_wb_cmplt_ptr(0) -> wmb_entry_out(0).iid,
    wmb_st_wb_cmplt_ptr(1) -> wmb_entry_out(1).iid,
    wmb_st_wb_cmplt_ptr(2) -> wmb_entry_out(2).iid,
    wmb_st_wb_cmplt_ptr(3) -> wmb_entry_out(3).iid,
    wmb_st_wb_cmplt_ptr(4) -> wmb_entry_out(4).iid,
    wmb_st_wb_cmplt_ptr(5) -> wmb_entry_out(5).iid,
    wmb_st_wb_cmplt_ptr(6) -> wmb_entry_out(6).iid,
    wmb_st_wb_cmplt_ptr(7) -> wmb_entry_out(7).iid
  ))

  //-------------------cmplt grnt-----------------------------
  wmb_entry_in.zip(wmb_st_wb_cmplt_ptr).foreach(x => x._1.wb_cmplt_grnt := io.in.st_wb_wmb_cmplt_grnt && x._2)

  //==========================================================
  //                Request wb data part
  //==========================================================
  val wmb_ld_wb_data_ptr = VecInit(wmb_entry_out.map(_.wb_data_req))

  io.out.toLoadWB.data_req := VecInit(wmb_entry_out.map(_.wb_data_req)).asUInt.orR

  val wmb_entry_ld_wb_data_ptr_sel = Mux1H(Seq(
    wmb_ld_wb_data_ptr(0) -> wmb_entry_out(0),
    wmb_ld_wb_data_ptr(1) -> wmb_entry_out(1),
    wmb_ld_wb_data_ptr(2) -> wmb_entry_out(2),
    wmb_ld_wb_data_ptr(3) -> wmb_entry_out(3),
    wmb_ld_wb_data_ptr(4) -> wmb_entry_out(4),
    wmb_ld_wb_data_ptr(5) -> wmb_entry_out(5),
    wmb_ld_wb_data_ptr(6) -> wmb_entry_out(6),
    wmb_ld_wb_data_ptr(7) -> wmb_entry_out(7)
  ))

  io.out.toLoadWB.preg := wmb_entry_ld_wb_data_ptr_sel.preg

  io.out.toLoadWB.data_addr := wmb_entry_ld_wb_data_ptr_sel.addr

  io.out.toLoadWB.data_iid := wmb_entry_ld_wb_data_ptr_sel.iid

  io.out.toLoadWB.vreg_sign_sel := 0.U(2.W)

  val wmb_ld_wb_data_success = wmb_entry_ld_wb_data_ptr_sel.sc_wb_success
  io.out.toLoadWB.data := Cat(0.U(63.W), !wmb_ld_wb_data_success)

  //don't expand sign
  io.out.toLoadWB.preg_sign_sel := "b0001".U
  io.out.toLoadWB.vreg := 0.U(6.W)
  io.out.toLoadWB.inst_vfls := false.B
  //-------------------atomic inst----------------------------
  val wmb_st_wb_atomic = VecInit(wmb_st_wb_cmplt_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.atomic)).asUInt.orR
  io.out.wmb_lm_state_clr := io.out.toStoreWB.cmplt_req && wmb_st_wb_atomic
  //-------------------data grnt------------------------------
  wmb_entry_in.zip(wmb_ld_wb_data_ptr).foreach(x => x._1.wb_data_grnt := io.in.ld_wb_wmb_data_grnt && x._2)

  //==========================================================
  //            maintain restart wakeup queue
  //==========================================================
  //---------------------registers----------------------------
  //+--------------+
  //| wakeup_queue |
  //+--------------+
  //the queue stores the instructions waiting for wakeup
  when(io.in.fromRTU.yy_xx_flush){
    wmb_wakeup_queue := VecInit(Seq.fill(LSIQ_ENTRY)(false.B))
  }.elsewhen(io.in.fromLoadDA.wmb_discard_vld || wmb_pop_depd_ff){
    wmb_wakeup_queue := wmb_wakeup_queue_next
  }

  //+-------------+
  //| depd_pop_ff |
  //+-------------+
  //if depd pop, this will set to 1, and clear wakeup_queue next cycle
  when(wmb_pop_depd || wmb_pop_discard_req || wmb_pop_fwd_req){
    wmb_pop_depd_ff := true.B
  }.otherwise{
    wmb_pop_depd_ff := false.B
  }

  wmb_wakeup_queue_not_empty := wmb_wakeup_queue.asUInt.orR
  //------------------request---------------------------------
  //------------wmb_pop_depd--------------
  wmb_pop_depd := VecInit(wmb_entry_out.map(x => x.pop_vld && x.depd)).asUInt.orR
  //---------interface to ld_dc-----------
  val wmb_discard_req = VecInit(wmb_entry_out.map(_.discard_req)).asUInt.orR
  io.out.toLoadDC.discard_req := wmb_discard_req
  val wmb_fwd_req = VecInit(wmb_entry_out.map(_.fwd_req)).asUInt.orR

  val wmb_entry_fwd_data_pe_req = VecInit(wmb_entry_out.map(_.fwd_data_pe_req)).asUInt

  io.out.wmb_fwd_bytes_vld := MuxLookup(wmb_entry_fwd_data_pe_req, 0.U(16.W), Seq(
    "b00000001".U -> wmb_entry_out(0).fwd_bytes_vld,
    "b00000010".U -> wmb_entry_out(1).fwd_bytes_vld,
    "b00000100".U -> wmb_entry_out(2).fwd_bytes_vld,
    "b00001000".U -> wmb_entry_out(3).fwd_bytes_vld,
    "b00010000".U -> wmb_entry_out(4).fwd_bytes_vld,
    "b00100000".U -> wmb_entry_out(5).fwd_bytes_vld,
    "b01000000".U -> wmb_entry_out(6).fwd_bytes_vld,
    "b10000000".U -> wmb_entry_out(7).fwd_bytes_vld
  ))

  io.out.toLoadDC.fwd_req := wmb_fwd_req

  io.out.toLoadDC.cancel_acc_req := VecInit(wmb_entry_out.map(_.cancel_acc_req)).asUInt.orR
  //---------interface to wmb_entry--------
  //for timing, discard req and set write imme/depd signal, and set wakeup queue
  //at next cycle

  //assign wmb_entry_discard_grnt[WMB_ENTRY-1:0]  = {WMB_ENTRY{ld_da_wmb_discard_vld}}
  //                                                & wmb_entry_discard_req[WMB_ENTRY-1:0];
  //-------forward to depd_pop_ff-------
  wmb_pop_discard_req := VecInit(wmb_entry_out.map(x => x.pop_vld && x.discard_req)).asUInt.orR
  wmb_pop_fwd_req     := VecInit(wmb_entry_out.map(x => x.pop_vld && x.fwd_req)).asUInt.orR

  //------------------update wakeup queue---------------------
  val wmb_wakeup_queue_grnt = Wire(Vec(LSIQ_ENTRY,Bool()))

  for(i <- 0 until LSIQ_ENTRY){
    wmb_wakeup_queue_grnt(i) := wmb_wakeup_queue(i) || io.in.fromLoadDA.wmb_discard_vld && io.in.fromLoadDA.lsid(i)
    wmb_wakeup_queue_next(i) := Mux(wmb_pop_depd_ff, false.B, wmb_wakeup_queue_grnt(i))
  }

  //-------------------------wakeup---------------------------
  io.out.wmb_depd_wakeup  := Mux(wmb_pop_depd_ff, wmb_wakeup_queue_grnt, VecInit(Seq.fill(LSIQ_ENTRY)(false.B)))

  //==========================================================
  //                Monitor biu b channel
  //==========================================================
  wmb_b_nc_id_hit := io.in.fromBiu.b_vld && (io.in.fromBiu.b_id ===  BIU_B_NC_ID)

  wmb_b_so_id_hit := io.in.fromBiu.b_vld && (io.in.fromBiu.b_id ===  BIU_B_SO_ID)

  wmb_b_resp_exokay := io.in.fromBiu.b_resp === EXOKAY

  //==========================================================
  //              Forward data to ld_da
  //==========================================================
  val wmb_fwd_data_sel = MuxLookup(wmb_entry_fwd_data_pe_req, 0.U(128.W), Seq(
    "b00000001".U -> wmb_entry_out(0).data,
    "b00000010".U -> wmb_entry_out(1).data,
    "b00000100".U -> wmb_entry_out(2).data,
    "b00001000".U -> wmb_entry_out(3).data,
    "b00010000".U -> wmb_entry_out(4).data,
    "b00100000".U -> wmb_entry_out(5).data,
    "b01000000".U -> wmb_entry_out(6).data,
    "b10000000".U -> wmb_entry_out(7).data
  ))

  val wmb_fwd_data_pe_req = wmb_entry_fwd_data_pe_req.orR && io.in.fromLoadDC.chk_ld_inst_vld && !io.in.fromLoadDA.fwd_ecc_stall

  wmb_fwd_data_pe_gateclk_en := VecInit(wmb_entry_out.map(_.fwd_data_pe_gateclk_en)).asUInt.orR && io.in.fromLoadDC.chk_ld_inst_vld

  when(wmb_fwd_data_pe_req){
    wmb_fwd_data := wmb_fwd_data_sel
  }

  io.out.wmb_ld_da_fwd_data := wmb_fwd_data
  //==========================================================
  //                      Hit index
  //==========================================================
  io.out.wmb_pfu_biu_req_hit_idx := VecInit(wmb_entry_out.map(_.pfu_biu_req_hit_idx)).asUInt.orR
  io.out.toSnq.depd := VecInit(wmb_entry_out.map(_.snq_depd))

  io.out.toSnq.depd_remove := VecInit(wmb_entry_out.map(_.snq_depd_remove))

  //hit cache line signal
  io.out.wmb_rb_biu_req_hit_idx := VecInit(wmb_entry_out.map(_.rb_biu_req_hit_idx)).asUInt.orR

  //==========================================================
  //              Interface to other module
  //==========================================================
  io.out.wmb_sync_fence_biu_req_success := VecInit(wmb_entry_out.map(_.sync_fence_biu_req_success)).asUInt.orR

  io.out.wmb_has_sync_fence := VecInit(wmb_entry_out.map(x => x.vld && x.sync_fence_inst)).asUInt.orR

  io.out.wmb_no_op := VecInit(wmb_entry_out.map(_.no_op)).asUInt.orR

  io.out.toHad.write_imme := wmb_write_imme
  io.out.toHad.entry_vld  := VecInit(wmb_entry_out.map(_.vld))
  io.out.toHad.ar_pending := VecInit(wmb_entry_out.map(_.ar_pending)).asUInt.orR
  io.out.toHad.aw_pending := VecInit(wmb_entry_out.map(_.aw_pending)).asUInt.orR
  io.out.toHad.w_pending  := VecInit(wmb_entry_out.map(_.w_pending)).asUInt.orR
  io.out.toHad.create_ptr := wmb_create_ptr
  io.out.toHad.write_ptr  := wmb_write_ptr
  io.out.toHad.read_ptr   := wmb_read_ptr
  io.out.toHad.data_ptr   := wmb_data_ptr

}
