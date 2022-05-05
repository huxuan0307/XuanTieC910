package Core.LSU.Rb
import Core.LSU.IdFifo8
import Core.{BIUConfig, LsuConfig}
import chisel3._
import chisel3.util._

class RbFromCp0 extends Bundle{
  val lsu_dcache_en = Bool()
  val lsu_icg_en = Bool()
  val yy_clk_en = Bool()
  val yy_priv_mode = UInt(2.W)
}

class RbFromLoadDA extends Bundle with LsuConfig{
  val addr = UInt(PA_WIDTH.W)
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val boundary_after_mask = Bool()
  val bytes_vld = UInt(16.W)
  val data_ori = UInt(64.W)
  val data_rot_sel = UInt(8.W)
  val dcache_hit = Bool()
  val idx = UInt(8.W)
  val iid = UInt(7.W)
  val inst_size = UInt(3.W)
  val inst_vfls = Bool()
  val mcic_borrow_mmu = Bool()
  val old = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val preg = UInt(7.W)
  val rb_atomic = Bool()
  val rb_cmit = Bool()
  val rb_cmplt_success = Bool()
  val rb_create_dp_vld = Bool()
  val rb_create_gateclk_en = Bool()
  val rb_create_judge_vld = Bool()
  val rb_create_lfb = Bool()
  val rb_create_vld = Bool()
  val rb_data_vld = Bool()
  val rb_dest_vld = Bool()
  val rb_discard_grnt = Bool()
  val rb_ldamo = Bool()
  val rb_merge_dp_vld = Bool()
  val rb_merge_expt_vld = Bool()
  val rb_merge_gateclk_en = Bool()
  val rb_merge_vld = Bool()
  val sign_extend = Bool()
  val vreg = UInt(6.W)
  val vreg_sign_sel = Bool()
}

class RbFromStoreDA extends Bundle with LsuConfig {
  val addr = UInt(PA_WIDTH.W)
  val dcache_hit = Bool()
  val fence_inst = Bool()
  val fence_mode = UInt(4.W)
  val iid = UInt(7.W)
  val inst_size = UInt(3.W)
  val old = Bool()
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val rb_cmit = Bool()
  val rb_create_dp_vld = Bool()
  val rb_create_gateclk_en = Bool()
  val rb_create_lfb = Bool()
  val rb_create_vld = Bool()
  val sync_fence = Bool()
  val sync_inst = Bool()
}

class RbFromWmb extends Bundle with LsuConfig {
  val ce_addr = UInt(PA_WIDTH.W)
  val ce_page_ca = Bool()
  val ce_page_so = Bool()
  val has_sync_fence = Bool()
  val rb_biu_req_hit_idx = Bool()
  val rb_so_pending = Bool()
  val sync_fence_biu_req_success = Bool()
}

class RbFromRTU extends Bundle {
  val lsu_async_flush = Bool()
  val yy_xx_commit = Vec(3, Bool())
  val yy_xx_commit_iid = Vec(3, UInt(7.W))
  val yy_xx_flush = Bool()
}


class RbInput extends Bundle with LsuConfig {
  val fromBiu = new Bundle{
    val b_id = UInt(5.W)
    val b_vld = Bool()
    val r_data = UInt(128.W)
    val r_id = UInt(5.W)
    val r_resp = UInt(4.W)
    val r_vld = Bool()
  }

  val bus_arb_rb_ar_grnt = Bool()

  val fromCp0 = new RbFromCp0
  val fromLoadDA = new RbFromLoadDA

  val ld_wb_rb_cmplt_grnt = Bool()
  val ld_wb_rb_data_grnt = Bool()

  val fromLfb = new Bundle{
    val addr_full = Bool()
    val rb_biu_req_hit_idx = Bool()
    val rb_ca_rready_grnt = Bool()
    val rb_create_id = UInt(5.W)
    val rb_nc_rready_grnt = Bool()
  }

  val lm_already_snoop = Bool()
  val lsu_special_clk = Bool()
  val pad_yy_icg_scan_en = Bool()
  val pfu_biu_req_addr = UInt(40.W)

  val fromRTU = new RbFromRTU
  val fromSQ = new Bundle{
    val pop_addr = UInt(PA_WIDTH.W)
    val pop_page_ca = Bool()
    val pop_page_so = Bool()
  }
  val fromStoreDA = new RbFromStoreDA

  val vb_rb_biu_req_hit_idx = Bool()

  val fromWmb = new RbFromWmb
}

class RbOutput extends Bundle with LsuConfig {
  val toHad = new Bundle{
    val rb_entry_fence = Vec(RB_ENTRY, Bool())
    val rb_entry_state = Vec(RB_ENTRY, UInt(4.W))
  }

  val lsu_has_fence = Bool()
  val lsu_idu_no_fence = Bool()
  val lsu_idu_rb_not_full = Bool()
  val lsu_rtu_all_commit_ld_data_vld = Bool()

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
    val req_addr = UInt(40.W)
    val req_unmask = Bool()
  }

  val rb_empty = Bool()
  val rb_fence_ld = Bool()

  val toLoadDA = new Bundle {
    val full = Bool()
    val hit_idx = Bool()
    val merge_fail = Bool()
  }
  val toLoadWB = new Bundle{
    val bkpta_data = Bool()
    val bkptb_data = Bool()
    val bus_err = Bool()
    val bus_err_addr = UInt(40.W)
    val cmplt_req = Bool()
    val data = UInt(64.W)
    val data_iid = UInt(7.W)
    val data_req = Bool()
    val expt_gateclk = Bool()
    val expt_vld = Bool()
    val iid = UInt(7.W)
    val inst_vfls = Bool()
    val preg = UInt(7.W)
    val preg_sign_sel = UInt(4.W)
    val vreg = UInt(6.W)
    val vreg_sign_sel = UInt(2.W)
  }
  val toLfb = new Bundle{
    val addr_tto4 = UInt(36.W)
    val atomic = Bool()
    val boundary_depd_wakeup = Bool()
    val create_dp_vld = Bool()
    val create_gateclk_en = Bool()
    val create_req = Bool()
    val create_vld = Bool()
    val depd = Bool()
    val ldamo = Bool()
  }
  val toLm = new Bundle{
    val ar_id = UInt(5.W)
    val atomic_next_resp = Bool()
    val wait_resp_dp_vld = Bool()
    val wait_resp_vld = Bool()
  }
  val toMcic = new Bundle{
    val ar_id = UInt(5.W)
    val biu_req_success = Bool()
    val ecc_err = Bool()
    val not_full = Bool()
  }

  val rb_pfu_biu_req_hit_idx = Bool()
  val rb_pfu_nc_no_pending = Bool()
  val rb_sq_pop_hit_idx = Bool()
  val rb_st_da_full = Bool()
  val rb_st_da_hit_idx = Bool()
  val rb_wmb_ce_hit_idx = Bool()
  val rb_wmb_so_pending = Bool()
}

class RbIO extends Bundle {
  val in = Input(new RbInput)
  val out = Output(new RbOutput)
}


class Rb extends Module with LsuConfig with BIUConfig{
  val io = IO(new RbIO)

  //Reg
  val lsu_has_fence = RegInit(false.B)
  val rb_biu_req_unmask = RegInit(false.B)
  val rb_biu_req_ptr = RegInit(VecInit(Seq.fill(RB_ENTRY)(false.B)))



  //Wire
  val rb_read_req_grnt_gateclk_en = Wire(Bool())
  val rb_biu_req_flush_clear = Wire(Bool())
  val rb_biu_pe_req_gateclk_en = Wire(Bool())

  val rb_biu_req_ptr_encode = Wire(UInt(3.W))
  val rb_biu_nc_req_grnt = Wire(Bool())
  val rb_r_nc_id_hit = Wire(Bool())
  val rb_biu_so_req_grnt = Wire(Bool())
  val rb_r_so_id_hit = Wire(Bool())

  val rb_entry_in = Wire(Vec(RB_ENTRY, new RbEntryIn))
  val rb_entry_out = Wire(Vec(RB_ENTRY, new RbEntryOutput))

  val rb_biu_ar_id = Wire(UInt(5.W))
  val rb_fence_ld = Wire(Bool())
  val rb_ld_biu_pe_req_grnt = Wire(Bool())
  val rb_r_resp_err = Wire(Bool())
  val rb_r_resp_okay = Wire(Bool())
  val rb_ready_all_req_biu_success = Wire(Bool())
  val rb_ready_ld_req_biu_success = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //pop entry signal
  val rb_pe_clk_en = Mux(rb_biu_req_unmask, (rb_read_req_grnt_gateclk_en || rb_biu_req_flush_clear),
    (io.in.fromLoadDA.rb_create_gateclk_en || rb_biu_pe_req_gateclk_en))

  //data pop entry signal

  //==========================================================
  //                 Non-cacheable FIFO
  //==========================================================
  val rb_idfifo_nc_create_vld = rb_biu_nc_req_grnt
  val rb_idfifo_nc_pop_vld    = rb_r_nc_id_hit

  val idfifo_nc = Module(new IdFifo8)
  idfifo_nc.io.in.cp0_lsu_icg_en      := io.in.fromCp0.lsu_icg_en
  idfifo_nc.io.in.cp0_yy_clk_en       := io.in.fromCp0.yy_clk_en
  idfifo_nc.io.in.idfifo_create_id    := rb_biu_req_ptr_encode
  idfifo_nc.io.in.idfifo_create_id_oh := rb_biu_req_ptr.asUInt
  idfifo_nc.io.in.idfifo_create_vld   := rb_idfifo_nc_create_vld
  idfifo_nc.io.in.idfifo_pop_vld      := rb_idfifo_nc_pop_vld
  idfifo_nc.io.in.pad_yy_icg_scan_en  := io.in.pad_yy_icg_scan_en

  val rb_nc_no_pending = idfifo_nc.io.out.idfifo_empty

  val rb_entry_next_nc_bypass = idfifo_nc.io.out.idfifo_pop_id_oh

  //==========================================================
  //                 Strong order FIFO
  //==========================================================
  val rb_idfifo_so_create_vld = rb_biu_so_req_grnt
  val rb_idfifo_so_pop_vld    = rb_r_so_id_hit

  val idfifo_so = Module(new IdFifo8)
  idfifo_so.io.in.cp0_lsu_icg_en      := io.in.fromCp0.lsu_icg_en
  idfifo_so.io.in.cp0_yy_clk_en       := io.in.fromCp0.yy_clk_en
  idfifo_so.io.in.idfifo_create_id    := rb_biu_req_ptr_encode
  idfifo_so.io.in.idfifo_create_id_oh := rb_biu_req_ptr.asUInt
  idfifo_so.io.in.idfifo_create_vld   := rb_idfifo_so_create_vld
  idfifo_so.io.in.idfifo_pop_vld      := rb_idfifo_so_pop_vld
  idfifo_so.io.in.pad_yy_icg_scan_en  := io.in.pad_yy_icg_scan_en

  val rb_so_no_pending = idfifo_so.io.out.idfifo_empty

  val rb_entry_next_so_bypass = idfifo_so.io.out.idfifo_pop_id_oh

  //------------------pending---------------------------------
  io.out.rb_wmb_so_pending := !rb_so_no_pending

  //-----------------biu req ptr------------------------------
  rb_biu_req_ptr_encode := OHToUInt(rb_biu_req_ptr.asUInt)(2,0)

  //-----------------biu data mask------------------------------
  val biu_lsu_r_data_mask = Mux(io.in.fromBiu.r_resp(1), 0.U(128.W), io.in.fromBiu.r_data)

  //==========================================================
  //                 Instance read buffer entry
  //==========================================================
  val rb_entry = Seq.fill(RB_ENTRY)(Module(new RbEntry))
  for(i <- 0 until RB_ENTRY){
    rb_entry(i).io.in.fromBiu.b_id        := io.in.fromBiu.b_id
    rb_entry(i).io.in.fromBiu.b_vld       := io.in.fromBiu.b_vld
    rb_entry(i).io.in.fromBiu.r_data_mask := biu_lsu_r_data_mask
    rb_entry(i).io.in.fromBiu.r_id        := io.in.fromBiu.r_id
    rb_entry(i).io.in.fromBiu.r_vld       := io.in.fromBiu.r_vld

    rb_entry(i).io.in.fromCp0    := io.in.fromCp0
    rb_entry(i).io.in.fromLoadDA := io.in.fromLoadDA

    rb_entry(i).io.in.lm_already_snoop   := io.in.lm_already_snoop
    rb_entry(i).io.in.lsu_has_fence      := lsu_has_fence
    rb_entry(i).io.in.lsu_special_clk    := io.in.lsu_special_clk
    rb_entry(i).io.in.pad_yy_icg_scan_en := io.in.pad_yy_icg_scan_en
    rb_entry(i).io.in.pfu_biu_req_addr   := io.in.pfu_biu_req_addr
    rb_entry(i).io.in.rb_biu_ar_id       := rb_biu_ar_id

    rb_entry(i).io.in.RbEntry := rb_entry_in(i)

    rb_entry(i).io.in.rb_fence_ld                  := rb_fence_ld
    rb_entry(i).io.in.rb_ld_biu_pe_req_grnt        := rb_ld_biu_pe_req_grnt
    rb_entry(i).io.in.rb_r_resp_err                := rb_r_resp_err
    rb_entry(i).io.in.rb_r_resp_okay               := rb_r_resp_okay
    rb_entry(i).io.in.rb_ready_all_req_biu_success := rb_ready_all_req_biu_success
    rb_entry(i).io.in.rb_ready_ld_req_biu_success  := rb_ready_ld_req_biu_success

    rb_entry(i).io.in.fromRTU := io.in.fromRTU
    rb_entry(i).io.in.fromSQ := io.in.fromSQ
    rb_entry(i).io.in.fromStoreDA := io.in.fromStoreDA
    rb_entry(i).io.in.fromWmb := io.in.fromWmb

    rb_entry_out(i) := rb_entry(i).io.out
  }

  //==========================================================
  //                      Create signal
  //==========================================================
  //+------------+
  //| create_ptr |
  //+------------+
  val rb_create_ptr0 = VecInit(PriorityEncoderOH(rb_entry_out.map(!_.vld)))
  val rb_create_ptr1 = VecInit(PriorityEncoderOH(rb_entry_out.map(!_.vld).reverse))

  rb_entry_in.zip(rb_create_ptr0).foreach{case(entry_in, ptr0) => entry_in.create_ptr0 := ptr0}

  //------------------full signal-----------------------------
  val rb_full = VecInit(rb_entry_out.map(_.vld)).asUInt.andR
  val rb_empty_less2 = (VecInit(rb_entry_out.map(_.vld)).asUInt | rb_create_ptr0.asUInt).andR
  val rb_empty_less3 = (VecInit(rb_entry_out.map(_.vld)).asUInt | rb_create_ptr0.asUInt | rb_create_ptr1.asUInt).andR

  val rb_ld_da_full = rb_full || !io.in.fromLoadDA.old && rb_empty_less2
  io.out.toLoadDA.full := rb_ld_da_full

  val rb_st_da_full = rb_full || (!io.in.fromStoreDA.old || io.in.fromLoadDA.rb_create_judge_vld) && rb_empty_less2 ||
    io.in.fromLoadDA.rb_create_judge_vld && !io.in.fromStoreDA.old && rb_empty_less3
  io.out.rb_st_da_full :=rb_st_da_full

  //------------------empty signal----------------------------
  val rb_not_empty = VecInit(rb_entry_out.map(_.vld)).asUInt.orR
  io.out.rb_empty := !rb_not_empty
  //------------------merge signal----------------------------
  io.out.toLoadDA.merge_fail := VecInit(rb_entry_out.map(_.merge_fail)).asUInt.orR

  //------------------create vld------------------------------

}
