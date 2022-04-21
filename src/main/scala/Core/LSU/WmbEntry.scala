package Core.LSU
import chisel3._
import chisel3.util._

class WmbEntryIn extends Bundle{
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

class InstInfo extends Bundle{
  val sync_fence     = Bool()
  val atomic         = Bool()
  val icc            = Bool()
  val inst_flush     = Bool()
  val inst_is_dcache = Bool()
  val inst_type      = UInt(2.W)
  val inst_size      = UInt(3.W)
  val inst_mode      = UInt(2.W)
  val fence_mode     = UInt(4.W)
  val iid            = UInt(7.W)
  val priv_mode      = UInt(2.W)
  val page_share     = Bool()
  val page_so        = Bool()
  val page_ca        = Bool()
  val page_wa        = Bool()
  val page_buf       = Bool()
  val page_sec       = Bool()
  val merge_en       = Bool()
  val addr           = UInt(LSUConfig.PA_WIDTH.W)
  val spec_fail      = Bool()
  val bkpta_data     = Bool()
  val bkptb_data     = Bool()
  val vstart_vld     = Bool()
}

class DcacheInfo extends Bundle{
  val valid = Bool()
  val share = Bool()
  val dirty = Bool()
  val way   = Bool()
}

class WmbEntryInput extends Bundle{
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
  val fromDCache = new Bundle{
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
  val wmb_dcache_req_ptr = Bool()
  val fromLoadDC = new Bundle{
    val addr0 = UInt(40.W)
    val addr1_11to4 = UInt(8.W)
    val bytes_vld = UInt(16.W)
    val chk_atomic_inst_vld = Bool()
    val chk_ld_inst_vld = Bool()
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
  val pw_merge_stall = Bool()
  val rb_biu_req_addr = UInt(40.W)
  val rb_biu_req_unmask = Bool()
  val rtu_lsu_async_flush = Bool()
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
    val update_dcache_dirty = Bool()
    val update_dcache_share = Bool()
    val update_dcache_valid = Bool()
    val update_dcache_way = Bool()
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


  val wmb_ce_create_vld = Bool()
  val wmb_ce_update_same_dcache_line = Bool()
  val wmb_ce_update_same_dcache_line_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
  val wmb_ce_last_addr_plus = Bool()
  val wmb_ce_last_addr_sub = Bool()

  val WmbEntry = new WmbEntryIn

  val WmbRead = new Bundle{
    val ptr_read_req_grnt = Bool()
    val ptr_shift_imme_grnt = Bool()
    val ptr = Bool()
  }
  val wmb_same_line_resp_ready = Vec(LSUConfig.WMB_ENTRY, Bool())
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
  val inst_info = RegInit(0.U.asTypeOf(new InstInfo))
  val dcache_info = RegInit(0.U.asTypeOf(new DcacheInfo))
  val wmb_entry_write_stall = RegInit(false.B)
  val wmb_entry_bytes_vld_full = RegInit(false.B)
  val wmb_entry_bytes_vld = RegInit(0.U(16.W))
  val wmb_entry_data = RegInit(VecInit(Seq.fill(4)(0.U(32.W))))

  val wmb_entry_read_req_success = RegInit(false.B)
  val wmb_entry_read_resp = RegInit(false.B)
  val wmb_entry_write_req_success = RegInit(false.B)
  val wmb_entry_write_resp = RegInit(false.B)
  val wmb_entry_data_req_success = RegInit(false.B)

  val wmb_entry_biu_id = RegInit(0.U(5.W))
  val wmb_entry_biu_r_id_vld = RegInit(false.B)
  val wmb_entry_biu_b_id_vld = RegInit(false.B)
  val wmb_entry_w_last = RegInit(false.B)
  val wmb_entry_mem_set_req = RegInit(false.B)
  val wmb_entry_wb_cmplt_success = RegInit(false.B)
  val wmb_entry_wb_data_success = RegInit(false.B)

  val wmb_entry_same_dcache_line = RegInit(false.B)
  val wmb_entry_same_dcache_line_ptr = RegInit(VecInit(Seq.fill(LSUConfig.WMB_ENTRY)(false.B)))
  val wmb_entry_write_imme = RegInit(false.B)
  val wmb_entry_depd = RegInit(false.B)
  val wmb_entry_sc_wb_vld = RegInit(false.B)
  val wmb_entry_sc_wb_success = RegInit(false.B)
  val wmb_entry_last_addr_plus = RegInit(false.B)
  val wmb_entry_last_addr_sub = RegInit(false.B)
  //Wire
  val wmb_entry_data_clk_en = Wire(UInt(4.W))
  val wmb_entry_bytes_vld_clk_en = Wire(Bool())

  val wmb_entry_read_req = Wire(Bool())
  val wmb_entry_write_biu_req = Wire(Bool())
  val wmb_entry_mem_set_write_gateclk_en = Wire(Bool())
  val wmb_entry_r_id_hit = Wire(Bool())
  val wmb_entry_b_id_hit = Wire(Bool())
  val wmb_entry_pop_vld = Wire(Bool())
  val wmb_entry_dcache_update_vld = Wire(Bool())
  val update_dcache_info = Wire(new DcacheInfo)
  val wmb_entry_merge_data_grnt = Wire(Bool())
  val wmb_entry_bytes_vld_full_and = Wire(Bool())
  val wmb_entry_bytes_vld_and = Wire(UInt(16.W))
  val wmb_entry_create_data = Wire(UInt(4.W))
  val wmb_entry_merge_data = Wire(UInt(4.W))
  val wmb_entry_data_next = Wire(Vec(16, UInt(8.W)))

  val wmb_entry_read_req_success_set = Wire(Bool())
  val wmb_entry_read_resp_set = Wire(Bool())
  val wmb_entry_write_req_success_set = Wire(Bool())
  val wmb_entry_write_resp_set = Wire(Bool())
  val wmb_entry_data_req_success_set = Wire(Bool())

  val wmb_entry_mem_set_write_grnt = Wire(Bool())
  val wmb_entry_biu_r_id_vld_set = Wire(Bool())
  val wmb_entry_r_resp_vld = Wire(Bool())
  val wmb_entry_biu_b_id_vld_set = Wire(Bool())
  val wmb_entry_b_resp_vld = Wire(Bool())
  val wmb_entry_w_last_set = Wire(Bool())
  val wmb_entry_wb_cmplt_grnt = Wire(Bool())
  val wmb_entry_wb_data_grnt = Wire(Bool())

  val wmb_entry_same_dcache_line_clr = Wire(Bool())
  val wmb_entry_write_imme_set = Wire(Bool())
  val wmb_entry_discard_req = Wire(Bool())
  val wmb_entry_fwd_req = Wire(Bool())
  val wmb_entry_sc_wb_set = Wire(Bool())
  val wmb_entry_sc_wb_success_set = Wire(Bool())

  val wmb_entry_st_inst = Wire(Bool())
  val wmb_entry_wo_st_write_biu_req = Wire(Bool())
  val wmb_entry_data_req = Wire(Bool())
  val wmb_dcache_req_ptr = Wire(Bool())
  val wmb_entry_wo_st_inst = Wire(Bool())

  val wmb_entry_rb_biu_req_hit_idx = Wire(Bool())
  val wmb_create_ptr_next1 = Wire(Bool())
  val wmb_entry_snq_set_write_imme = Wire(Bool())
  val wmb_read_ptr = Wire(Bool())
  val wmb_write_ptr = Wire(Bool())
  val wmb_entry_same_dcache_line_ready = Wire(Bool())

  val wmb_data_ptr = Wire(Bool())
  val wmb_entry_next_nc_bypass = Wire(Bool())
  val wmb_entry_next_so_bypass = Wire(Bool())
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
  when(io.in.WmbEntry.create_dp_vld){
    inst_info.sync_fence     :=  io.in.fromWmbCe.sync_fence
    inst_info.atomic         :=  io.in.fromWmbCe.atomic
    inst_info.icc            :=  io.in.fromWmbCe.icc
    inst_info.inst_flush     :=  io.in.fromWmbCe.inst_flush
    inst_info.inst_is_dcache :=  io.in.fromWmbCe.dcache_inst
    inst_info.inst_type      :=  io.in.fromWmbCe.inst_type
    inst_info.inst_size      :=  io.in.fromWmbCe.inst_size
    inst_info.inst_mode      :=  io.in.fromWmbCe.inst_mode
    inst_info.fence_mode     :=  io.in.fromWmbCe.fence_mode
    inst_info.iid            :=  io.in.fromWmbCe.iid
    inst_info.priv_mode      :=  io.in.fromWmbCe.priv_mode
    inst_info.page_share     :=  io.in.fromWmbCe.page_share
    inst_info.page_so        :=  io.in.fromWmbCe.page_so
    inst_info.page_ca        :=  io.in.fromWmbCe.page_ca
    inst_info.page_wa        :=  io.in.fromWmbCe.page_wa
    inst_info.page_buf       :=  io.in.fromWmbCe.page_buf
    inst_info.page_sec       :=  io.in.fromWmbCe.page_sec
    inst_info.merge_en       :=  io.in.fromWmbCe.merge_en
    inst_info.addr           :=  io.in.fromWmbCe.addr
    inst_info.spec_fail      :=  io.in.fromWmbCe.spec_fail
    inst_info.bkpta_data     :=  io.in.fromWmbCe.bkpta_data
    inst_info.bkptb_data     :=  io.in.fromWmbCe.bkptb_data
    inst_info.vstart_vld     :=  io.in.fromWmbCe.vstart_vld
  }

  //+-------------------+
  //| cache_information |
  //+-------------------+
  when(io.in.WmbEntry.create_dp_vld){
    dcache_info.valid := io.in.fromSQ.update_dcache_dirty
    dcache_info.share := io.in.fromSQ.update_dcache_share
    dcache_info.dirty := io.in.fromSQ.update_dcache_valid
    dcache_info.way   := io.in.fromSQ.update_dcache_way
  }.elsewhen(wmb_entry_dcache_update_vld){
    dcache_info := update_dcache_info
  }

  //+--------------------+
  //| already merge grnt |
  //+--------------------+
  when(wmb_entry_merge_data_grnt && io.in.fromSQ.wmb_merge_req && io.in.wmb_ce_create_vld){
    wmb_entry_write_stall := true.B
  }.otherwise{
    wmb_entry_write_stall := false.B
  }

  //+--------------------+
  //| data and bytes_vld |
  //+--------------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_bytes_vld_full := io.in.fromWmbCe.bytes_vld_full
    wmb_entry_bytes_vld      := io.in.fromWmbCe.bytes_vld
  }.elsewhen(io.in.WmbEntry.merge_data_vld){
    wmb_entry_bytes_vld_full :=  wmb_entry_bytes_vld_full_and
    wmb_entry_bytes_vld      :=  wmb_entry_bytes_vld_and
  }

  for(i <- 0 until 4){
    when(wmb_entry_create_data(i) || wmb_entry_merge_data(i)){
      wmb_entry_data(i) := wmb_entry_data_next.asUInt(32*i+31,32*i)
    }
  }

  //------------------success/resp signal---------------------
  //+------------------+
  //| read_req_success |
  //+------------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_read_req_success := false.B
  }.elsewhen(wmb_entry_read_req_success_set){
    wmb_entry_read_req_success := true.B
  }

  //+-----------+
  //| read_resp |
  //+-----------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_read_resp := false.B
  }.elsewhen(wmb_entry_read_resp_set){
    wmb_entry_read_resp := true.B
  }

  //+-------------------+
  //| write_req_success |
  //+-------------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_write_req_success := false.B
  }.elsewhen(wmb_entry_write_req_success_set){
    wmb_entry_write_req_success := true.B
  }

  //+-----------+
  //| write_resp |
  //+-----------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_write_resp := false.B
  }.elsewhen(wmb_entry_write_resp_set){
    wmb_entry_write_resp := true.B
  }

  //+------------------+
  //| data_req_success |
  //+------------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_data_req_success := false.B
  }.elsewhen(wmb_entry_data_req_success_set){
    wmb_entry_data_req_success := true.B
  }

  //----------------------biu id signal-----------------------
  //+--------+
  //| biu_id |
  //+--------+
  when(wmb_entry_read_req){
    wmb_entry_biu_id := io.in.wmb_biu_ar_id
  }.elsewhen(wmb_entry_write_biu_req || wmb_entry_mem_set_write_grnt){
    wmb_entry_biu_id := io.in.wmb_biu_aw_id
  }

  //+------------+
  //| biu_id_vld |
  //+------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_biu_r_id_vld := false.B
  }.elsewhen(wmb_entry_biu_r_id_vld_set){
    wmb_entry_biu_r_id_vld := true.B
  }.elsewhen(wmb_entry_r_resp_vld){
    wmb_entry_biu_r_id_vld := false.B
  }

  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_biu_b_id_vld := false.B
  }.elsewhen(wmb_entry_biu_b_id_vld_set){
    wmb_entry_biu_b_id_vld := true.B
  }.elsewhen(wmb_entry_b_resp_vld){
    wmb_entry_biu_b_id_vld := false.B
  }

  //+--------+
  //| w_last |
  //+--------+
  when(wmb_entry_write_biu_req || wmb_entry_mem_set_write_grnt){
    wmb_entry_w_last := wmb_entry_w_last_set
  }

  //+-------------+
  //| mem_set_req |
  //+-------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_mem_set_req := false.B
  }.elsewhen(wmb_entry_mem_set_write_grnt){
    wmb_entry_mem_set_req := io.in.WmbWrite.biu_dcache_line
  }

  //-------------cmplt/data req success signal----------------
  //+------------------+
  //| wb_cmplt_success |
  //+------------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_wb_cmplt_success := io.in.fromWmbCe.wb_cmplt_success
  }.elsewhen(wmb_entry_wb_cmplt_grnt || io.in.rtu_lsu_async_flush){
    wmb_entry_wb_cmplt_success := true.B
  }

  //+-----------------+
  //| wb_data_success |
  //+-----------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_wb_data_success := io.in.fromWmbCe.wb_data_success
  }.elsewhen(wmb_entry_wb_data_grnt || io.in.rtu_lsu_async_flush){
    wmb_entry_wb_data_success := true.B
  }

  //------------same cache line write imme and depd-----------
  //+-----------------+
  //| same_dcache_line |
  //+-----------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_same_dcache_line := io.in.wmb_ce_update_same_dcache_line
  }.elsewhen(wmb_entry_same_dcache_line_clr){
    wmb_entry_same_dcache_line := false.B
  }

  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_same_dcache_line_ptr := io.in.wmb_ce_update_same_dcache_line_ptr
  }

  //+------------+
  //| write_imme |
  //+------------+
  //if write req grnt, clear write imme
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_write_imme := io.in.fromWmbCe.write_imme
  }.elsewhen(wmb_entry_write_req_success_set){
    wmb_entry_write_imme := false.B
  }.elsewhen(wmb_entry_write_imme_set){
    wmb_entry_write_imme := true.B
  }

  //+------+
  //| depd |
  //+------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_depd := false.B
  }.elsewhen(wmb_entry_discard_req || wmb_entry_fwd_req){
    wmb_entry_depd := true.B
  }

  //----------------------stex signal-------------------------
  //+-----------+---------------+
  //| sc_wb_vld | sc_wb_success |
  //+-----------+---------------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_sc_wb_vld := io.in.fromWmbCe.sc_wb_vld
    wmb_entry_sc_wb_success := false.B
  }.elsewhen(wmb_entry_sc_wb_set){
    wmb_entry_sc_wb_vld := true.B
    wmb_entry_sc_wb_success := wmb_entry_sc_wb_success_set
  }

  //----------------------write burst judge signal-------------------------
  //+-----------+----------+
  //| addr_plus | addr_sub |
  //+-----------+----------+
  when(io.in.WmbEntry.create_dp_vld){
    wmb_entry_last_addr_plus := io.in.wmb_ce_last_addr_plus
    wmb_entry_last_addr_sub := io.in.wmb_ce_last_addr_sub
  }

  //==========================================================
  //                  Create/merge signal
  //==========================================================
  //wmb_entry_hit_sq_pop_cache_line is used for same_dcache_line
  val wmb_entry_hit_sq_pop_addr_tto6 = inst_info.addr(LSUConfig.PA_WIDTH-1,6) === io.in.fromSQ.pop_addr(LSUConfig.PA_WIDTH-1,6)
  val wmb_entry_hit_sq_pop_addr_5to4 = inst_info.addr(5,4) === io.in.fromSQ.pop_addr(5,4)
  val wmb_entry_hit_sq_pop_addr_tto4   = wmb_entry_hit_sq_pop_addr_tto6 && wmb_entry_hit_sq_pop_addr_5to4

  val wmb_entry_hit_sq_pop_dcache_line =  wmb_entry_hit_sq_pop_addr_tto6 && wmb_entry_st_inst && wmb_entry_vld

  //if supv mode or page info is not hit, then set write_imme and donot grnt
  //signal to sq
  val wmb_entry_merge_data_addr_hit = wmb_entry_hit_sq_pop_addr_tto4 && inst_info.merge_en && wmb_entry_vld

  val wmb_entry_merge_data_permit = (inst_info.priv_mode === io.in.fromSQ.pop_priv_mode) &&
    !(wmb_entry_wo_st_write_biu_req || wmb_entry_data_req || wmb_dcache_req_ptr && (io.in.pw_merge_stall || io.in.wmb_dcache_arb_req_unmask)) &&
    !wmb_entry_write_req_success && !wmb_entry_data_req_success

  val wmb_entry_merge_data_stall = wmb_entry_merge_data_addr_hit && !wmb_entry_merge_data_permit

  wmb_entry_merge_data_grnt := wmb_entry_merge_data_addr_hit && wmb_entry_merge_data_permit

  val wmb_entry_merge_data_write_imme_set = wmb_entry_merge_data_addr_hit && io.in.fromSQ.wmb_merge_stall_req

  wmb_entry_merge_data := Mux(io.in.WmbEntry.merge_data_vld, io.in.fromWmbCe.data_vld, 0.U(4.W))

  wmb_entry_create_data := Mux(io.in.WmbEntry.create_data_vld, io.in.fromWmbCe.data_vld, 0.U(4.W))

  val wmb_entry_create_merge_data_gateclk_en = io.in.WmbEntry.create_gateclk_en || io.in.WmbEntry.merge_data_vld

  wmb_entry_data_clk_en := Mux(wmb_entry_create_merge_data_gateclk_en, io.in.fromWmbCe.data_vld, 0.U(4.W))

  wmb_entry_bytes_vld_clk_en := wmb_entry_create_merge_data_gateclk_en

  //------------------merge data------------------------------
  for(i <- 0 until 16){
    wmb_entry_data_next(i) := Mux(io.in.fromWmbCe.bytes_vld(i), io.in.fromWmbCe.data128(8*i+7,8*i), wmb_entry_data.asUInt(8*i+7,8*i))
  }

  //------------------merge bytes_vld-------------------------
  wmb_entry_bytes_vld_and := wmb_entry_bytes_vld | io.in.fromWmbCe.bytes_vld
  wmb_entry_bytes_vld_full_and := wmb_entry_bytes_vld_and.andR

  //----------ready to send dcache line of this entry---------
  val wmb_entry_ready_to_dcache_line = wmb_entry_vld && wmb_entry_wo_st_inst && inst_info.page_ca &&
    wmb_entry_bytes_vld_full && !wmb_entry_write_req_success && wmb_entry_read_resp

  //==========================================================
  //        Generate inst type
  //==========================================================
  val wmb_entry_atomic_and_vld = inst_info.atomic && wmb_entry_vld

  val wmb_entry_icc_and_vld = inst_info.icc && wmb_entry_vld

  val wmb_entry_sync_fence_inst  = !inst_info.atomic && inst_info.sync_fence

  val wmb_entry_ctc_inst = inst_info.icc && !inst_info.atomic && (inst_info.inst_type =/= 2.U(2.W))

  val wmb_entry_dcache_inst = inst_info.inst_is_dcache

  wmb_entry_st_inst := !inst_info.icc && !inst_info.atomic && !inst_info.sync_fence

  wmb_entry_wo_st_inst := wmb_entry_st_inst && !inst_info.page_so

  val wmb_entry_so_st_inst = wmb_entry_st_inst &&  inst_info.page_so

  val wmb_entry_stamo_inst = inst_info.atomic && (inst_info.inst_type === 0.U(2.W))

  val wmb_entry_sc_inst = inst_info.atomic && (inst_info.inst_type === 1.U(2.W))


  val wmb_entry_dcache_all_inst = wmb_entry_dcache_inst && (inst_info.inst_mode === 0.U(2.W))

  val wmb_entry_dcache_1line_inst = wmb_entry_dcache_inst && (inst_info.inst_mode =/= 0.U(2.W))

  val wmb_entry_dcache_addr_inst = wmb_entry_dcache_inst && inst_info.inst_mode(0)

  val wmb_entry_dcache_sw_inst = wmb_entry_dcache_inst && (inst_info.inst_mode === 2.U(2.W))

  val wmb_entry_dcache_clr_addr_inst = wmb_entry_dcache_addr_inst && (inst_info.inst_size(1,0) =/= 2.U(2.W))

  val wmb_entry_dcache_clr_sw_inst = wmb_entry_dcache_sw_inst && (inst_info.inst_size(1,0) =/= 2.U(2.W))

  val wmb_entry_dcache_clr_1line_inst = wmb_entry_dcache_clr_addr_inst || wmb_entry_dcache_clr_sw_inst

  val wmb_entry_dcache_addr_l1_inst = wmb_entry_dcache_addr_inst && (inst_info.inst_size(1,0) === 0.U(2.W))

  val wmb_entry_dcache_addr_not_l1_inst = wmb_entry_dcache_addr_inst && (inst_info.inst_size(1,0) =/= 0.U(2.W))

  val wmb_entry_dcache_only_inv_addr_inst = wmb_entry_dcache_addr_inst && (inst_info.inst_size(1,0) === 2.U(2.W))

  val wmb_entry_dcache_only_inv_sw_inst = wmb_entry_dcache_sw_inst && (inst_info.inst_size(1,0) === 2.U(2.W))

  val wmb_entry_dcache_only_inv_1line_inst = wmb_entry_dcache_only_inv_addr_inst || wmb_entry_dcache_only_inv_sw_inst

  val wmb_entry_dcache_except_only_inv_1line_inst  = wmb_entry_dcache_inst && !wmb_entry_dcache_only_inv_1line_inst

  //==========================================================
  //            Compare dcache write port(dcwp)
  //==========================================================
  //TODO: Module ct_lsu_dcache_info_update
  update_dcache_info := dcache_info
  val wmb_entry_dcache_hit_idx = WireInit(false.B)
  val wmb_entry_dcache_update_vld_unmask = WireInit(false.B)

  wmb_entry_dcache_update_vld := wmb_entry_dcache_update_vld_unmask && wmb_entry_vld

  //==========================================================
  //                 Dependency check
  //==========================================================

  // situat ld pipe         sq/wmb          addr    bytes_vld data_vld  manner
  // --------------------------------------------------------------------------
  // 1      ld              st              :4      part      x         discard
  // 2      ld atomic       any             x       x         x         discard
  // 3      ld              atomic          :4      do        x         discard
  // 4      ld              st              :4      whole     x         forward
  // 5      ld(addr1)       st              :4      x         x         !acclr_en

  //------------------compare signal--------------------------
  //-----------addr compare---------------
  //addr compare
  val wmb_entry_depd_addr_tto12_hit = inst_info.addr(LSUConfig.PA_WIDTH-1,12) === io.in.fromLoadDC.addr0(LSUConfig.PA_WIDTH-1,12)
  val wmb_entry_depd_addr_11to4_hit = inst_info.addr(11,4) === io.in.fromLoadDC.addr0(11,4)
  val wmb_entry_depd_addr1_11to4_hit = inst_info.addr(11,4) === io.in.fromLoadDC.addr1_11to4

  val wmb_entry_depd_addr_tto4_hit  = wmb_entry_depd_addr_tto12_hit && wmb_entry_depd_addr_11to4_hit
  val wmb_entry_depd_addr1_tto4_hit = wmb_entry_depd_addr_tto12_hit && wmb_entry_depd_addr1_11to4_hit

  //-----------bytes_vld compare----------
  val wmb_entry_and_ld_dc_bytes_vld = wmb_entry_bytes_vld & io.in.fromLoadDC.bytes_vld
  val wmb_entry_and_ld_dc_bytes_vld_hit = wmb_entry_and_ld_dc_bytes_vld.orR

  //example:
  //depd_bytes_vld          ld_dc_bytes_vld     depd kinds
  //1111                    0011                do & whole
  //0011                    0011                do & whole
  //0110                    0011                do & part
  //0110                    1111                do & part
  //1100                    0011                /

  val wmb_entry_depd_do_hit = wmb_entry_and_ld_dc_bytes_vld_hit

  //------------------cancel amr------------------------------

  //------------------fwd data pop entry----------------------
  val wmb_entry_fwd_data_pe_req = wmb_entry_vld && wmb_entry_wo_st_inst && wmb_entry_depd_addr_tto4_hit
  io.out.fwd_data_pe_req := wmb_entry_fwd_data_pe_req

  io.out.fwd_data_pe_gateclk_en := wmb_entry_vld && wmb_entry_wo_st_inst&& wmb_entry_depd_addr_11to4_hit

  //------------------situation 1-----------------------------
  val wmb_entry_fwd_data_pre = wmb_entry_fwd_data_pe_req && io.in.fromLoadDC.chk_ld_inst_vld

  val wmb_entry_depd_hit1 = wmb_entry_fwd_data_pre && wmb_entry_depd_do_hit

  io.out.fwd_bytes_vld := Mux(wmb_entry_fwd_data_pre, wmb_entry_and_ld_dc_bytes_vld, 0.U(16.W))

  //------------------situation 2-----------------------------
  val wmb_entry_depd_hit2  = wmb_entry_vld && io.in.fromLoadDC.chk_atomic_inst_vld

  //------------------situation 3-----------------------------
  val wmb_entry_depd_hit3  = wmb_entry_vld && inst_info.atomic && io.in.fromLoadDC.chk_ld_inst_vld &&
    wmb_entry_depd_addr_tto4_hit && wmb_entry_depd_do_hit

  //for cache buffer acceleration
  val wmb_entry_depd_hit5  = wmb_entry_vld && wmb_entry_depd_addr1_tto4_hit
  //------------------combine---------------------------------
  wmb_entry_discard_req := wmb_entry_depd_hit2 || wmb_entry_depd_hit3
  io.out.discard_req := wmb_entry_discard_req

  wmb_entry_fwd_req := wmb_entry_depd_hit1
  io.out.fwd_req := wmb_entry_fwd_req

  io.out.cancel_acc_req := wmb_entry_depd_hit5

  //==========================================================
  //                 Set write_imme
  //==========================================================
  //-------------request ar channel if need-------------------
  //if has write out, then clear write imme
  wmb_entry_write_imme_set := wmb_entry_vld && !wmb_entry_write_req_success &&
    (wmb_entry_discard_req || io.in.WmbEntry.merge_data_wait_not_vld_req || wmb_entry_rb_biu_req_hit_idx ||
      wmb_create_ptr_next1 || wmb_entry_snq_set_write_imme || io.in.wmb_wakeup_queue_not_empty && wmb_entry_depd ||
      wmb_entry_merge_data_write_imme_set)

  //for timing, use write_imme_bypass to set wmb_write_imme
  val wmb_entry_write_imme_bypass = wmb_entry_vld && !wmb_entry_write_req_success && wmb_create_ptr_next1

  //==========================================================
  //                    Request read
  //==========================================================
  val wmb_entry_read_ptr_not_already_success = wmb_entry_vld && wmb_read_ptr && !wmb_entry_read_req_success

  //-------------request ar channel if need-------------------
  wmb_entry_read_req := wmb_entry_read_ptr_not_already_success &&
    (wmb_entry_st_inst && inst_info.page_ca && inst_info.page_share && dcache_info.share && !wmb_entry_same_dcache_line ||
      wmb_entry_ctc_inst ||
      wmb_entry_dcache_addr_inst && inst_info.page_ca && (wmb_entry_dcache_addr_not_l1_inst || inst_info.page_share) &&
        inst_info.page_ca && wmb_entry_write_resp ||
      wmb_entry_sc_inst && wmb_write_ptr && !wmb_entry_sc_wb_vld && inst_info.page_ca && inst_info.page_share &&
        (dcache_info.valid && dcache_info.share))

  val wmb_entry_read_dp_req  = wmb_entry_vld && !wmb_entry_read_req_success &&
    (wmb_entry_st_inst && inst_info.page_ca && inst_info.page_share && (dcache_info.share || !dcache_info.valid) ||
      wmb_entry_ctc_inst ||
      wmb_entry_dcache_addr_inst && (wmb_entry_dcache_addr_not_l1_inst || inst_info.page_share) && inst_info.page_ca ||
      wmb_entry_sc_inst && inst_info.page_ca && inst_info.page_share && !wmb_entry_sc_wb_vld)

  val wmb_entry_read_ptr_chk_idx_shift_imme = wmb_entry_read_ptr_not_already_success &&
    (wmb_entry_st_inst && inst_info.page_ca && (wmb_entry_same_dcache_line || !dcache_info.valid || dcache_info.valid && !dcache_info.share || !inst_info.page_share) ||
      wmb_entry_stamo_inst ||
      wmb_entry_sc_inst && wmb_write_ptr && !wmb_entry_sc_wb_vld && inst_info.page_ca &&
        (dcache_info.valid && !dcache_info.share || !dcache_info.valid || !inst_info.page_share))

  //if has sent read req and other condition, don't compare index
  val wmb_entry_read_ptr_unconditional_shift_imme = wmb_entry_vld && wmb_read_ptr &&
    (wmb_entry_read_req_success && !inst_info.page_ca || wmb_entry_dcache_sw_inst && wmb_entry_write_resp ||
      wmb_entry_dcache_all_inst || wmb_entry_sync_fence_inst ||
      wmb_entry_dcache_addr_l1_inst && !inst_info.page_share && wmb_entry_write_resp ||
      wmb_entry_dcache_addr_inst && !inst_info.page_ca && wmb_entry_write_resp ||
      wmb_entry_sc_inst && (!inst_info.page_ca || wmb_entry_sc_wb_vld))

  //-------------read req_success/resp set--------------------
  val wmb_entry_read_ptr_shift_imme = wmb_entry_read_ptr_unconditional_shift_imme || wmb_entry_read_ptr_chk_idx_shift_imme

  wmb_entry_read_req_success_set := !wmb_entry_read_req_success &&
    (wmb_entry_read_req && io.in.WmbRead.ptr_read_req_grnt ||
      wmb_entry_read_ptr_shift_imme && io.in.WmbRead.ptr_shift_imme_grnt)

  //if ctc has sent, then set read_resp
  wmb_entry_read_resp_set := !wmb_entry_read_resp &&
    (wmb_entry_read_ptr_shift_imme && io.in.WmbRead.ptr_shift_imme_grnt && !wmb_entry_read_req_success && !wmb_entry_sync_fence_inst &&
      !(wmb_entry_st_inst && inst_info.page_ca && inst_info.page_share && dcache_info.valid &&
        dcache_info.share && wmb_entry_same_dcache_line && !wmb_entry_same_dcache_line_ready) ||
      wmb_entry_read_req_success && wmb_entry_st_inst && inst_info.page_ca && wmb_entry_same_dcache_line && wmb_entry_same_dcache_line_ready ||
      wmb_entry_r_resp_vld || wmb_entry_read_req && wmb_entry_ctc_inst && io.in.WmbRead.ptr_read_req_grnt)

  //for same dcache line
  io.out.read_resp_ready := !(wmb_entry_vld && wmb_entry_read_req_success && !wmb_entry_read_resp)

  wmb_entry_same_dcache_line_ready := (wmb_entry_same_dcache_line_ptr.asUInt & io.in.wmb_same_line_resp_ready.asUInt)

  //==========================================================
  //                    Request write
  //==========================================================
  //-------------request dcache/vb/aw channel if need---------
  val wmb_entry_page_ca_dcache_valid = inst_info.page_ca && dcache_info.valid

  //write req is used for write ptr shift
  io.out.write_req := wmb_entry_vld && wmb_write_ptr && !wmb_entry_write_req_success && !wmb_entry_ctc_inst && !wmb_entry_dcache_all_inst

  wmb_entry_write_biu_req := wmb_entry_vld && wmb_write_ptr && !wmb_entry_write_req_success && !wmb_entry_write_stall &&
    (wmb_entry_so_st_inst && wmb_entry_read_resp ||
      wmb_entry_wo_st_inst && wmb_entry_read_resp && !wmb_entry_page_ca_dcache_valid ||
      wmb_entry_sync_fence_inst && wmb_entry_read_req_success && io.in.fromVb.empty ||
      wmb_entry_stamo_inst && wmb_entry_read_resp && !dcache_info.valid ||
      wmb_entry_sc_inst && wmb_entry_read_resp && !wmb_entry_sc_wb_vld && (!inst_info.page_ca || !dcache_info.valid))

  //if write imme, then must write this cycle or next cycle
  wmb_entry_wo_st_write_biu_req := wmb_entry_vld && wmb_write_ptr && inst_info.merge_en && !wmb_entry_write_req_success &&
    wmb_entry_wo_st_inst && io.in.wmb_biu_write_en && wmb_entry_read_resp && !wmb_entry_page_ca_dcache_valid

  val wmb_entry_write_biu_dp_req = wmb_entry_vld && !wmb_entry_write_req_success &&
    (wmb_entry_so_st_inst ||
      wmb_entry_wo_st_inst && !wmb_entry_page_ca_dcache_valid ||
      wmb_entry_sync_fence_inst ||
      wmb_entry_stamo_inst && !dcache_info.valid ||
      wmb_entry_sc_inst && (!inst_info.page_ca || !dcache_info.valid) && !wmb_entry_sc_wb_vld)

  val wmb_entry_write_dcache_req   = wmb_entry_vld && !wmb_entry_write_resp &&
    (wmb_entry_page_ca_dcache_valid && !wmb_entry_write_req_success &&
      (wmb_entry_st_inst && wmb_entry_read_resp ||
        wmb_entry_stamo_inst && wmb_entry_read_resp ||
        wmb_entry_sc_inst && wmb_entry_read_resp && !wmb_entry_sc_wb_vld) ||
      wmb_entry_dcache_only_inv_1line_inst && wmb_entry_page_ca_dcache_valid && wmb_entry_write_req_success)

  val wmb_entry_write_vb_req = wmb_entry_vld && wmb_write_ptr && !wmb_entry_write_req_success &&
    wmb_entry_dcache_clr_1line_inst && wmb_entry_page_ca_dcache_valid

  //if already mem_set success, then write ptr shift imme
  val wmb_entry_write_ptr_unconditional_shift_imme = wmb_write_ptr &&
    (!wmb_entry_vld || wmb_dcache_req_ptr && io.in.WmbWrite.dcache_success || wmb_entry_write_resp ||
      wmb_entry_write_req_success && wmb_entry_mem_set_req ||
      !wmb_entry_write_req_success && (wmb_entry_dcache_all_inst || wmb_entry_ctc_inst ||
        wmb_entry_sc_inst && wmb_entry_read_resp && wmb_entry_sc_wb_vld))

  val wmb_entry_write_ptr_chk_idx_shift_imme = wmb_entry_vld && wmb_write_ptr && !wmb_entry_write_req_success &&
    (wmb_entry_dcache_except_only_inv_1line_inst && !wmb_entry_page_ca_dcache_valid)

  //-------------write req_success/resp set-------------------
  val wmb_entry_write_ptr_shift_imme = wmb_entry_write_ptr_chk_idx_shift_imme || wmb_entry_write_ptr_unconditional_shift_imme

  wmb_entry_write_req_success_set := !wmb_entry_write_req_success &&
    (wmb_entry_write_biu_req && io.in.fromBusArb.aw_grnt ||
      wmb_entry_write_vb_req && io.in.wmb_create_vb_success ||
      wmb_dcache_req_ptr && io.in.WmbWrite.dcache_success ||
      wmb_entry_vld && wmb_entry_dcache_only_inv_1line_inst && io.in.wmb_dcache_inst_write_req_hit_idx && wmb_write_ptr ||
      wmb_entry_mem_set_write_grnt || wmb_entry_write_ptr_shift_imme && io.in.WmbWrite.ptr_shift_imme_grnt)

  wmb_entry_write_resp_set := !wmb_entry_write_resp && (io.in.fromVb.rcl_done ||
    wmb_dcache_req_ptr && io.in.WmbWrite.dcache_success ||
    wmb_entry_vld && wmb_entry_dcache_only_inv_1line_inst && wmb_entry_write_req_success && !wmb_entry_page_ca_dcache_valid ||
    wmb_entry_b_resp_vld || wmb_entry_write_ptr_shift_imme && !wmb_entry_write_req_success && io.in.WmbWrite.ptr_shift_imme_grnt)

  //==========================================================
  //                    Request data
  //==========================================================
  //-------------request data channel if need-----------------
  wmb_entry_data_req := wmb_entry_vld && wmb_data_ptr && !wmb_entry_data_req_success &&
    (wmb_entry_read_resp || !(wmb_entry_st_inst || inst_info.atomic)) && wmb_entry_write_req_success

  val wmb_entry_data_biu_req = wmb_data_ptr && wmb_entry_write_req_success && !wmb_entry_data_req_success &&
    !wmb_entry_write_resp && (wmb_entry_st_inst || inst_info.atomic || wmb_entry_sync_fence_inst)

  val wmb_entry_data_req_wns = wmb_data_ptr && !wmb_entry_sync_fence_inst && !inst_info.page_ca

  val wmb_entry_data_ptr_after_write_shift_imme = wmb_data_ptr && (!wmb_entry_vld ||
    wmb_dcache_req_ptr && io.in.WmbWrite.dcache_success || wmb_entry_write_resp ||
    wmb_entry_write_req_success && (wmb_entry_dcache_except_only_inv_1line_inst ||
      wmb_entry_ctc_inst || wmb_entry_sc_inst && wmb_entry_sc_wb_vld))

  val wmb_entry_data_ptr_with_write_shift_imme = false.B

  //-------------write req_success/resp set-------------------
  wmb_entry_data_req_success_set := wmb_entry_data_biu_req && io.in.fromBusArb.w_grnt ||
    wmb_dcache_req_ptr && io.in.WmbWrite.dcache_success || wmb_entry_data_ptr_after_write_shift_imme ||
    wmb_entry_data_ptr_with_write_shift_imme && io.in.WmbWrite.ptr_shift_imme_grnt

  //==========================================================
  //                Compare biu r/b channel
  //==========================================================
  //---------------------biu id vld set-----------------------
  wmb_entry_biu_r_id_vld_set := wmb_entry_read_req && !wmb_entry_ctc_inst || wmb_entry_sync_fence_inst && wmb_entry_write_biu_req

  wmb_entry_biu_b_id_vld_set := wmb_entry_write_biu_req || wmb_entry_mem_set_write_grnt

  //-----------------compare biu r channel--------------------
  wmb_entry_r_id_hit := io.in.fromBiu.r_vld && wmb_entry_biu_r_id_vld && (wmb_entry_biu_id === io.in.fromBiu.r_id)

  wmb_entry_r_resp_vld := wmb_entry_r_id_hit

  //-----------------compare biu b channel--------------------
  wmb_entry_b_id_hit := io.in.fromBiu.b_vld && wmb_entry_biu_b_id_vld && (wmb_entry_biu_id === io.in.fromBiu.b_id)

  wmb_entry_b_resp_vld := wmb_entry_b_id_hit && (inst_info.page_ca || inst_info.atomic && !inst_info.page_so ||
    wmb_entry_next_nc_bypass || wmb_entry_next_so_bypass || wmb_entry_dcache_clr_1line_inst || wmb_entry_sync_fence_inst)

  //==========================================================
  //                 Request wb cmplt/data
  //==========================================================
  //stex write data first, then request cmplt to ensure there is only 1 stex
  //inst in wmb
  val wmb_entry_wb_cmplt_req = wmb_entry_vld && !wmb_entry_wb_cmplt_success && wmb_entry_wb_data_success &&
    (wmb_entry_dcache_all_inst || wmb_entry_sync_fence_inst && wmb_entry_read_resp && wmb_entry_write_resp ||
      wmb_entry_so_st_inst && wmb_entry_write_req_success || wmb_entry_sc_inst && wmb_entry_sc_wb_vld ||
      wmb_entry_stamo_inst && wmb_entry_write_resp)

  val wmb_entry_wb_data_req  = wmb_entry_vld && !wmb_entry_wb_data_success && wmb_entry_sc_inst && wmb_entry_sc_wb_vld

  //==========================================================
  //                 sc execute
  //==========================================================
  wmb_entry_sc_wb_set := wmb_entry_vld && wmb_entry_sc_inst && !wmb_entry_sc_wb_vld &&
    (io.in.fromLm.state_is_idle || wmb_entry_page_ca_dcache_valid && wmb_entry_write_resp ||
      inst_info.page_ca && !dcache_info.valid && wmb_entry_b_resp_vld || !inst_info.page_ca && wmb_entry_b_resp_vld)

  wmb_entry_sc_wb_success_set := io.in.fromLm.state_is_ex_wait_lock && (inst_info.page_ca || !inst_info.page_ca && io.in.wmb_b_resp_exokay)

  val wmb_entry_preg = Cat(inst_info.icc, inst_info.inst_mode, inst_info.fence_mode)

  //==========================================================
  //                 Compare index
  //==========================================================
  val wmb_entry_idx_cmpare_inst  = wmb_entry_vld && (wmb_entry_st_inst || inst_info.atomic || wmb_entry_dcache_1line_inst) && inst_info.page_ca

  //for snq dep
  val wmb_entry_idx_snq_dep_inst = wmb_entry_vld &&
    (wmb_entry_st_inst || wmb_entry_stamo_inst || wmb_entry_dcache_only_inv_1line_inst ||
      wmb_entry_sc_inst && !wmb_entry_sc_wb_vld)

  //------------------compare rb biu req----------------------
  //because if hit index of rb_biu_req, this entry must set write_imme bit, so it
  //must compare with req_unmask signal
  wmb_entry_rb_biu_req_hit_idx := wmb_entry_idx_cmpare_inst && io.in.rb_biu_req_unmask && (inst_info.addr(13,6) === io.in.rb_biu_req_addr(13,6))

  //------------------compare pfu pop entry--------------------
  val wmb_entry_pfu_biu_req_hit_idx = wmb_entry_idx_cmpare_inst && (inst_info.addr(13,6) === io.in.pfu_biu_req_addr(13,6))

  //------------------compare snq create port-----------------
  //if hit snq create addr, then same_dcache_line will be cleared
  //if wmb entry has not write, and has read_resp, then this entry must clr
  val wmb_entry_snq_create_addr_hit_idx = inst_info.addr(13,6) === io.in.fromSnq.create_addr(13,6)

  val wmb_entry_snq_create_hit_idx  = wmb_entry_idx_snq_dep_inst && io.in.fromSnq.can_create_snq_uncheck && wmb_entry_snq_create_addr_hit_idx

  //if wmb entry has write and not resp, snq must wait
  //assign wmb_entry_read_resp_already_write  = wmb_entry_read_req_success
  //                                            &&  wmb_entry_read_resp
  //                                            &&  wmb_entry_write_req_success;

  //in this situation, then snq must wait, and set write imme of this entry
  //assign wmb_entry_read_resp_hit_write_ptr  = wmb_entry_read_req_success
  //                                            &&  wmb_entry_read_resp
  //                                            &&  !wmb_entry_write_req_success
  //                                            &&  wmb_write_ptr;

  //read_req_success and read_resp and reset read_ptr
  val wmb_entry_read_resp_not_write = wmb_entry_read_req_success && wmb_entry_read_resp && !wmb_entry_write_req_success

  //set snq signal
  val wmb_entry_snq_depd = wmb_entry_snq_create_hit_idx &&
    (wmb_entry_read_resp_not_write && !wmb_entry_dcache_only_inv_1line_inst && wmb_entry_page_ca_dcache_valid ||
      wmb_entry_vld && wmb_entry_write_req_success && wmb_entry_dcache_only_inv_1line_inst)

  wmb_entry_snq_set_write_imme := wmb_entry_snq_create_hit_idx && wmb_entry_read_resp_not_write

  val wmb_entry_snq_depd_remove = !wmb_entry_vld || wmb_entry_write_resp

  //------------------compare snq dcache port-----------------
  wmb_entry_same_dcache_line_clr := wmb_entry_vld && io.in.fromDCache.snq_st_sel && !wmb_entry_read_req_success && wmb_entry_dcache_hit_idx

  //==========================================================
  //                Generate no_op signal
  //==========================================================
  //if not vld/ read resp & not write & not write imme
  val wmb_entry_no_op = !wmb_entry_vld || wmb_entry_read_resp && !wmb_entry_write_imme && !wmb_entry_write_req_success

  //==========================================================
  //                 Generate pop signal
  //==========================================================
  //if write dcache line and is not the last entry of dcache line, then fast pop
  wmb_entry_pop_vld := wmb_entry_vld && wmb_entry_read_resp &&
    (wmb_entry_write_resp || wmb_entry_write_resp_set || wmb_entry_mem_set_req && !wmb_entry_w_last) &&
    (wmb_entry_data_req_success  ||  wmb_entry_data_req_success_set) && wmb_entry_wb_cmplt_success && wmb_entry_wb_data_success

  //==========================================================
  //                 Interface to rb
  //==========================================================
  val wmb_entry_sync_fence_biu_req_success = wmb_entry_vld && wmb_entry_sync_fence_inst && wmb_entry_write_req_success

  //==========================================================
  //                 Interface to had
  //==========================================================
  val wmb_entry_ar_pending = wmb_entry_vld && wmb_entry_read_req_success && !wmb_entry_read_resp

  val wmb_entry_aw_pending = wmb_entry_vld && wmb_entry_write_req_success && !wmb_entry_write_resp

  val wmb_entry_w_pending  = wmb_entry_vld && wmb_entry_data_req_success && !wmb_entry_write_resp

  //==========================================================
  //                 Generate interface
  //==========================================================
  //-----------------------input------------------------------
  //-----------create signal--------------

  //---------grnt/done signal-------------

  wmb_entry_next_nc_bypass           := io.in.WmbEntry.next_nc_bypass
  wmb_entry_next_so_bypass           := io.in.WmbEntry.next_so_bypass
  wmb_entry_wb_cmplt_grnt            := io.in.WmbEntry.wb_cmplt_grnt
  wmb_entry_wb_data_grnt             := io.in.WmbEntry.wb_data_grnt

  //-----------pointer--------------------
  wmb_create_ptr_next1         := io.in.wmb_create_ptr_next1
  wmb_data_ptr                 := io.in.wmb_data_ptr
  wmb_read_ptr                 := io.in.WmbRead.ptr
  wmb_write_ptr                := io.in.WmbWrite.ptr
  wmb_dcache_req_ptr           := io.in.wmb_dcache_req_ptr
  wmb_entry_mem_set_write_grnt := io.in.WmbEntry.mem_set_write_grnt
  wmb_entry_w_last_set         := io.in.WmbEntry.w_last_set

  //-----------merge signal---------------

  //-----------gateclk signal-------------
  wmb_entry_mem_set_write_gateclk_en := io.in.WmbEntry.mem_set_write_gateclk_en

  //-----------------------output-----------------------------
  //-----------entry signal---------------
  io.out.vld             := wmb_entry_vld
  io.out.sync_fence      := inst_info.sync_fence
  io.out.atomic          := inst_info.atomic
  io.out.atomic_and_vld  := wmb_entry_atomic_and_vld
  io.out.icc             := inst_info.icc
  io.out.icc_and_vld     := wmb_entry_icc_and_vld
  io.out.inst_flush      := inst_info.inst_flush
  io.out.inst_is_dcache  := inst_info.inst_is_dcache
  io.out.dcache_inst     := wmb_entry_dcache_inst
  io.out.inst_type       := inst_info.inst_type
  io.out.inst_size       := inst_info.inst_size
  io.out.inst_mode       := inst_info.inst_mode
  io.out.iid             := inst_info.iid
  io.out.priv_mode       := inst_info.priv_mode
  io.out.page_share      := inst_info.page_share
  io.out.page_so         := inst_info.page_so
  io.out.page_ca         := inst_info.page_ca
  io.out.page_wa         := inst_info.page_wa
  io.out.page_buf        := inst_info.page_buf
  io.out.page_sec        := inst_info.page_sec
  io.out.addr            := inst_info.addr
  io.out.spec_fail       := inst_info.spec_fail
  io.out.bkpta_data      := inst_info.bkpta_data
  io.out.bkptb_data      := inst_info.bkptb_data
  io.out.vstart_vld      := inst_info.vstart_vld
  io.out.dcache_way      := dcache_info.way
  io.out.data            := wmb_entry_data.asUInt
  io.out.biu_id          := wmb_entry_biu_id
  io.out.w_last          := wmb_entry_w_last
  io.out.bytes_vld       := wmb_entry_bytes_vld
  io.out.write_imme      := wmb_entry_write_imme
  io.out.depd            := wmb_entry_depd
  io.out.sc_wb_success   := wmb_entry_sc_wb_success
  io.out.preg            := wmb_entry_preg
  io.out.sync_fence_inst := wmb_entry_sync_fence_inst
  //-----------request--------------------
  io.out.fwd_data_pe_req        := wmb_entry_fwd_data_pe_req
  //io.out.fwd_data_pe_gateclk_en := wmb_entry_fwd_data_pe_gateclk_en
  io.out.discard_req            := wmb_entry_discard_req
  io.out.fwd_req                := wmb_entry_fwd_req
  //io.out.fwd_bytes_vld          := wmb_entry_fwd_bytes_vld
  io.out.wb_cmplt_req           := wmb_entry_wb_cmplt_req
  io.out.wb_data_req            := wmb_entry_wb_data_req
  io.out.read_req               := wmb_entry_read_req
  io.out.read_dp_req            := wmb_entry_read_dp_req
  //io.out.write_req              := wmb_entry_write_req
  io.out.write_biu_req          := wmb_entry_write_biu_req
  io.out.write_biu_dp_req       := wmb_entry_write_biu_dp_req
  io.out.write_dcache_req       := wmb_entry_write_dcache_req
  io.out.write_vb_req           := wmb_entry_write_vb_req
  io.out.data_req               := wmb_entry_data_req
  io.out.data_biu_req           := wmb_entry_data_biu_req
  io.out.data_req_wns           := wmb_entry_data_req_wns
  io.out.pop_vld                := wmb_entry_pop_vld
  //io.out.cancel_acc_req         := wmb_entry_cancel_acc_req
  io.out.merge_data_stall       := wmb_entry_merge_data_stall
  io.out.merge_data_addr_hit    := wmb_entry_merge_data_addr_hit
  io.out.write_stall            := wmb_entry_write_stall
  //-------maintain pointer---------------
  io.out.read_ptr_unconditional_shift_imme  := wmb_entry_read_ptr_unconditional_shift_imme
  io.out.read_ptr_chk_idx_shift_imme        := wmb_entry_read_ptr_chk_idx_shift_imme
  io.out.write_ptr_unconditional_shift_imme := wmb_entry_write_ptr_unconditional_shift_imme
  io.out.write_ptr_chk_idx_shift_imme       := wmb_entry_write_ptr_chk_idx_shift_imme
  io.out.data_ptr_after_write_shift_imme    := wmb_entry_data_ptr_after_write_shift_imme
  io.out.data_ptr_with_write_shift_imme     := wmb_entry_data_ptr_with_write_shift_imme
  //-----------hit idx--------------------
  io.out.pfu_biu_req_hit_idx     := wmb_entry_pfu_biu_req_hit_idx
  io.out.rb_biu_req_hit_idx      := wmb_entry_rb_biu_req_hit_idx
  io.out.snq_depd                := wmb_entry_snq_depd
  io.out.hit_sq_pop_dcache_line  := wmb_entry_hit_sq_pop_dcache_line
  //-----------other signal---------------
  io.out.write_imme_bypass       := wmb_entry_write_imme_bypass
  io.out.ready_to_dcache_line    := wmb_entry_ready_to_dcache_line
  io.out.last_addr_plus          := wmb_entry_last_addr_plus
  io.out.last_addr_sub           := wmb_entry_last_addr_sub
  io.out.no_op                   := wmb_entry_no_op
  //io.out.read_resp_ready         := wmb_entry_read_resp_ready
  io.out.snq_depd_remove         := wmb_entry_snq_depd_remove
  //--------to other module signal--------
  io.out.sync_fence_biu_req_success := wmb_entry_sync_fence_biu_req_success
  io.out.ar_pending              := wmb_entry_ar_pending
  io.out.aw_pending              := wmb_entry_aw_pending
  io.out.w_pending               := wmb_entry_w_pending
}
