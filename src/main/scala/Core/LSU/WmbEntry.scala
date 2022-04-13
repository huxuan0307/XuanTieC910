package Core.LSU
import chisel3._
import chisel3.util._

class WmbEntryInput extends Bundle{
  val fromBiu = new Bundle{
    val b_id = UInt(5.W)
    val b_vld = Bool()
    val r_id = UInt(5.W)
    val r_vld = Bool()
  }
  val fromBusArb = new Bundle{
    val aw_grnt = Bool()
    val w_grnt = Bool()
  }
  val fromCp0 = new Bundle{
    val lsu_icg_en = Bool()
    val yy_clk_en = Bool()
  }
  val fromDCache = new Bundle{
    val dirty_din = UInt(7.W)
    val dirty_gwen = Bool()
    val dirty_wen = UInt(7.W)
    val idx = UInt(9.W)
    val snq_st_sel = Bool()
    val tag_din = UInt(52.W)
    val tag_gwen = Bool()
    val tag_wen = UInt(2.W)
  }
  val fromLoadDC = new Bundle{
    val addr0 = UInt(40.W)
    val addr1_11to4 = UInt(8.W)
    val bytes_vld = UInt(16.W)
    val chk_atomic_inst_vld = Bool()
    val chk_ld_inst_vld = Bool()
  }
  val fromLm = new Bundle{
    val state_is_ex_wait_lock = Bool()
    val state_is_idle = Bool()
  }
  val fromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val pfu_biu_req_addr = UInt(40.W)
  val pw_merge_stall = Bool()
  val rb_biu_req_addr = UInt(40.W)
  val rb_biu_req_unmask = Bool()
  val rtu_lsu_async_flush = Bool()
  val fromSnq = new Bundle{
    val can_create_snq_uncheck = Bool()
    val create_addr = UInt(40.W)
  }
  val fromSQ = new Bundle{
    val pop_addr = UInt(40.W)
    val pop_priv_mode = UInt(2.W)
    val wmb_merge_req = Bool()
    val wmb_merge_stall_req = Bool()
  }
  val fromVb = new Bundle{
    val empty = Bool()
    val rcl_done = Bool()
  }
  val wmb_b_resp_exokay = Bool()
  val wmb_biu_ar_id = UInt(5.W)
  val wmb_biu_aw_id = UInt(5.W)
  val wmb_biu_write_en = Bool()

  val fromWmbCe = new FromWmbCe

  val wmb_create_ptr_next1 = Bool()
  val wmb_create_vb_success = Bool()
  val wmb_data_ptr = Bool()
  val wmb_dcache_arb_req_unmask = Bool()
  val wmb_dcache_inst_write_req_hit_idx = Bool()
  val wmb_dcache_req_ptr = Bool()

  val WmbEntry = new Bundle{
    val create_data_vld = Bool()
    val create_dp_vld = Bool()
    val create_gateclk_en = Bool()
    val create_vld = Bool()
    val mem_set_write_gateclk_en = Bool()
    val mem_set_write_grnt = Bool()
    val merge_data_vld = Bool()
    val merge_data_wait_not_vld_req = Bool()
    val next_nc_bypass = Bool()
    val next_so_bypass = Bool()
    val w_last_set = Bool()
    val wb_cmplt_grnt = Bool()
    val wb_data_grnt = Bool()
  }
  val WmbRead = new Bundle{
    val ptr_read_req_grnt = Bool()
    val ptr_shift_imme_grnt = Bool()
    val ptr = Bool()
  }
  val wmb_same_line_resp_ready = UInt(8.W)
  val wmb_wakeup_queue_not_empty = Bool()
  val WmbWrite = new Bundle{
    val biu_dcache_line = Bool()
    val dcache_success = Bool()
    val ptr_shift_imme_grnt = Bool()
    val ptr = Bool()
  }
}

class WmbEntryOutput extends Bundle{
  val addr = UInt(40.W)
  val ar_pending = Bool()
  val atomic_and_vld = Bool()
  val atomic = Bool()
  val aw_pending = Bool()
  val biu_id = UInt(5.W)
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val bytes_vld = UInt(16.W)
  val cancel_acc_req = Bool()
  val data_biu_req = Bool()
  val data_ptr_after_write_shift_imme = Bool()
  val data_ptr_with_write_shift_imme = Bool()
  val data_req_wns = Bool()
  val data_req = Bool()
  val data = UInt(128.W)
  val dcache_inst = Bool()
  val dcache_way = Bool()
  val depd = Bool()
  val discard_req = Bool()
  val fwd_bytes_vld = UInt(16.W)
  val fwd_data_pe_gateclk_en = Bool()
  val fwd_data_pe_req = Bool()
  val fwd_req = Bool()
  val hit_sq_pop_dcache_line = Bool()
  val icc_and_vld = Bool()
  val icc = Bool()
  val iid = UInt(7.W)
  val inst_flush = Bool()
  val inst_is_dcache = Bool()
  val inst_mode = UInt(2.W)
  val inst_size = UInt(3.W)
  val inst_type = UInt(2.W)
  val last_addr_plus = Bool()
  val last_addr_sub = Bool()
  val merge_data_addr_hit = Bool()
  val merge_data_stall = Bool()
  val no_op = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val page_wa = Bool()
  val pfu_biu_req_hit_idx = Bool()
  val pop_vld = Bool()
  val preg = UInt(7.W)
  val priv_mode = UInt(2.W)
  val rb_biu_req_hit_idx = Bool()
  val read_dp_req = Bool()
  val read_ptr_chk_idx_shift_imme = Bool()
  val read_ptr_unconditional_shift_imme = Bool()
  val read_req = Bool()
  val read_resp_ready = Bool()
  val ready_to_dcache_line = Bool()
  val sc_wb_success = Bool()
  val snq_depd_remove = Bool()
  val snq_depd = Bool()
  val spec_fail = Bool()
  val sync_fence_biu_req_success = Bool()
  val sync_fence_inst = Bool()
  val sync_fence = Bool()
  val vld = Bool()
  val vstart_vld = Bool()
  val w_last = Bool()
  val w_pending = Bool()
  val wb_cmplt_req = Bool()
  val wb_data_req = Bool()
  val write_biu_dp_req = Bool()
  val write_biu_req = Bool()
  val write_dcache_req = Bool()
  val write_imme_bypass = Bool()
  val write_imme = Bool()
  val write_ptr_chk_idx_shift_imme = Bool()
  val write_ptr_unconditional_shift_imme = Bool()
  val write_req = Bool()
  val write_stall = Bool()
  val write_vb_req = Bool()
}

class WmbEntryIO extends Bundle{
  val in = Input(new WmbEntryInput)
  val out = Output(new WmbEntryOutput)
}

class WmbEntry extends Module {
  val io = IO(new WmbEntryIO)

  //Reg
  val wmb_entry_vld = RegInit(false.B)

  //Wire
  val wmb_entry_read_req = Wire(Bool())
  val wmb_entry_write_biu_req = Wire(Bool())
  val wmb_entry_mem_set_write_gateclk_en = Wire(Bool())
  val wmb_entry_r_id_hit = Wire(Bool())
  val wmb_entry_b_id_hit = Wire(Bool())
  val wmb_entry_pop_vld = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //----------entry gateclk---------------
  val wmb_entry_clk_en = wmb_entry_vld || io.in.WmbEntry.create_gateclk_en

  //-----------create gateclk-------------
  val wmb_entry_create_clk_en = io.in.WmbEntry.create_gateclk_en

  //----------data gateclk----------------

  //biu_id_gate_clk
  val wmb_entry_biu_id_clk_en = io.in.WmbEntry.create_gateclk_en || wmb_entry_read_req || wmb_entry_write_biu_req ||
    wmb_entry_mem_set_write_gateclk_en || wmb_entry_r_id_hit || wmb_entry_b_id_hit

  //==========================================================
  //                 Register
  //==========================================================
  //+-----------+
  //| entry_vld |
  //+-----------+
  when(wmb_entry_pop_vld){
    wmb_entry_vld := false.B
  }.elsewhen(io.in.WmbEntry.create_vld){
    wmb_entry_vld := true.B
  }

  //+-------------------------+
  //| instruction information |
  //+-------------------------+

}
