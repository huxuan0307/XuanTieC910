package Core.LSU
import Utils.ParallelORR
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

  val ptr_init = Seq(false.B,false.B,false.B,false.B,false.B,false.B,false.B,true.B)
  //Reg
  val wmb_create_ptr = RegInit(VecInit(ptr_init))
  val wmb_read_ptr = RegInit(VecInit(ptr_init))
  val wmb_write_ptr = RegInit(VecInit(ptr_init))
  val wmb_data_ptr = RegInit(VecInit(ptr_init))
  val wmb_create_ptr_circular = RegInit(false.B)
  val wmb_read_ptr_circular = RegInit(false.B)
  val wmb_write_ptr_circular = RegInit(false.B)
  val wmb_data_ptr_circular = RegInit(false.B)

  val wmb_write_req_addr = Reg(UInt(LSUConfig.PA_WIDTH.W))


  //Wire
  val wmb_empty = Wire(Bool())
  val wmb_pop_depd_ff = Wire(Bool())
  val wmb_read_dp_req = Wire(Bool())
  val wmb_write_biu_dp_req = Wire(Bool())
  val wmb_write_imme = Wire(Bool())
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
  val wmb_write_dcache_pop_req = Wire(Bool())

  val wmb_entry_in = Wire(Vec(LSUConfig.WMB_ENTRY, new WmbEntryIn))
  val wmb_entry_out = Wire(Vec(LSUConfig.WMB_ENTRY, new WmbEntryOutput))
  val wmb_dcache_req_ptr = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  val pw_merge_stall = WireInit(false.B)

  val wmb_b_resp_exokay = Wire(Bool())
  val wmb_biu_ar_id    = Wire(UInt(5.W))
  val wmb_biu_aw_id    = Wire(UInt(5.W))
  val wmb_biu_write_en = Wire(Bool())

  val wmb_create_vb_success = Wire(Bool())
  val wmb_dcache_inst_write_req_hit_idx = Wire(Bool())

  val wmb_ce_create_vld = Wire(Bool())
  val wmb_ce_update_same_dcache_line = Wire(Bool())
  val wmb_ce_update_same_dcache_line_ptr = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  val wmb_ce_last_addr_plus = Wire(Bool())
  val wmb_ce_last_addr_sub = Wire(Bool())

  val wmb_same_line_resp_ready = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  val wmb_wakeup_queue_not_empty = Wire(Bool())

  val wmb_read_ptr_read_req_grnt = Wire(Bool())
  val wmb_read_ptr_shift_imme_grnt = Wire(Bool())
  val wmb_write_biu_dcache_line = Wire(Bool())
  val wmb_write_dcache_success = Wire(Bool())
  val wmb_write_ptr_shift_imme_grnt = Wire(Bool())

  val wmb_create_vld = Wire(Bool())
  val wmb_create_ptr_next1 = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  val wmb_read_ptr_shift_vld = Wire(Bool())
  val wmb_read_ptr_next1 = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  val wmb_write_ptr_shift_vld = Wire(Bool())
  val wmb_write_ptr_next = Wire(Vec(8, Vec(LSUConfig.WMB_ENTRY, Bool())))
  val wmb_write_ptr_set = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  val wmb_write_ptr_circular_set = Wire(Bool())
  val wmb_data_ptr_shift_vld = Wire(Bool())
  val wmb_data_ptr_next1 = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))

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

  //==========================================================
  //                 Non-cacheable FIFO
  //==========================================================
  //TODO: implement
  val wmb_nc_no_pending = Wire(true.B)

  val wmb_entry_next_nc_bypass = Wire(VecInit(Seq.fill(LSUConfig.WMB_ENTRY)(false.B)))

  //==========================================================
  //                 Strong order FIFO
  //==========================================================v
  //TODO: implement
  val wmb_so_no_pending = Wire(true.B)

  val wmb_entry_next_so_bypass = Wire(VecInit(Seq.fill(LSUConfig.WMB_ENTRY)(false.B)))

  //==========================================================
  //          Instance of write merge buffer entry
  //==========================================================
  val wmb_entry = Seq.fill(LSUConfig.WMB_ENTRY)(new WmbEntry)
  for(i <- 0 until LSUConfig.WMB_ENTRY){
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

  when(wmb_create_vld && wmb_create_ptr(LSUConfig.WMB_ENTRY-1)){
    wmb_create_ptr_circular := !wmb_create_ptr_circular
  }

  //+----------+
  //| read_ptr |
  //+----------+
  when(wmb_read_ptr_shift_vld){
    wmb_read_ptr := wmb_read_ptr_next1
  }

  when(wmb_read_ptr_shift_vld && wmb_read_ptr(LSUConfig.WMB_ENTRY-1)){
    wmb_read_ptr_circular := !wmb_read_ptr_circular
  }

  //+-----------+
  //| write_ptr |
  //+-----------+
  when(wmb_write_ptr_shift_vld){
    wmb_write_ptr := wmb_write_ptr_next(1)
  }

  when(wmb_write_ptr_shift_vld && wmb_write_ptr(LSUConfig.WMB_ENTRY-1)){
    wmb_write_ptr_circular := !wmb_write_ptr_circular
  }

  //+----------+
  //| data_ptr |
  //+----------+
  when(wmb_data_ptr_shift_vld){
    wmb_data_ptr := wmb_data_ptr_next1
  }

  when(wmb_data_ptr_shift_vld && wmb_data_ptr(LSUConfig.WMB_ENTRY-1)){
    wmb_data_ptr_circular := !wmb_data_ptr_circular
  }

  //------------------write ptr set---------------------------
  //if snq clear read req, then read ptr must be set write_ptr_set

  //------------------pointer encode--------------------------
  val wmb_read_ptr_encode = OHToUInt(wmb_read_ptr.asUInt)(2,0)

  val wmb_write_ptr_encode = OHToUInt(wmb_write_ptr.asUInt)(2,0)

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
  wmb_create_ptr_next1 := Cat(wmb_create_ptr.asUInt(LSUConfig.WMB_ENTRY-2,0), wmb_create_ptr.asUInt(LSUConfig.WMB_ENTRY-1)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))

  wmb_read_ptr_next1 := Cat(wmb_read_ptr.asUInt(LSUConfig.WMB_ENTRY-2,0), wmb_read_ptr.asUInt(LSUConfig.WMB_ENTRY-1)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))

  //mem set must use write_ptr_to_next_3
  wmb_write_ptr_next(1) := Cat(wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-2,0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_write_ptr_next(2) := Cat(wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-3,0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1,LSUConfig.WMB_ENTRY-2)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_write_ptr_next(3) := Cat(wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-4,0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1,LSUConfig.WMB_ENTRY-3)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_write_ptr_next(4) := Cat(wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-5,0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1,LSUConfig.WMB_ENTRY-4)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_write_ptr_next(5) := Cat(wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-6,0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1,LSUConfig.WMB_ENTRY-5)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_write_ptr_next(6) := Cat(wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-7,0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1,LSUConfig.WMB_ENTRY-6)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_write_ptr_next(7) := Cat(wmb_write_ptr.asUInt(0), wmb_write_ptr.asUInt(LSUConfig.WMB_ENTRY-1,LSUConfig.WMB_ENTRY-7)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))

  val wmb_write_ptr_to_next3 = (wmb_write_ptr.asUInt | wmb_write_ptr_next(1).asUInt | wmb_write_ptr_next(2).asUInt | wmb_write_ptr_next(3).asUInt).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))

  wmb_data_ptr_next1 := Cat(wmb_data_ptr.asUInt(LSUConfig.WMB_ENTRY-2,0), wmb_data_ptr.asUInt(LSUConfig.WMB_ENTRY-1)).asTypeOf(Vec(LSUConfig.WMB_ENTRY, Bool()))

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

  for(i <- 0 until LSUConfig.WMB_ENTRY){
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
  val wmb_create_not_vld = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  wmb_create_not_vld := wmb_create_ptr.zip(wmb_entry_out).map(x => x._1 && x._2.vld)

  val wmb_create_permit = wmb_create_not_vld.asUInt.orR

  wmb_create_vld := wmb_create_permit && io.in.fromWmbCe.create_wmb_req

  val wmb_merge_vld = io.in.fromWmbCe.merge_wmb_req




}
