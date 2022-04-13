package Core.LSU
import chisel3._
import chisel3.util._

class SQPopInfo extends Bundle{
  val addr = UInt(40.W)
  val atomic = Bool()
  val bytes_vld = UInt(16.W)
  val icc = Bool()
  val inst_flush = Bool()
  val inst_mode = UInt(2.W)
  val inst_size = UInt(3.W)
  val inst_type = UInt(2.W)
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val page_wa = Bool()
  val priv_mode = UInt(2.W)
  val sq_ptr = Vec(LSUConfig.SQ_ENTRY, Bool())
  val sync_fence = Bool()
  val wo_st = Bool()
}

class WmbCeInput extends Bundle{
  val fromCp0 = new Bundle{
    val lsu_icg_en = Bool()
    val yy_clk_en = Bool()
  }
  val lm_sq_sc_fail = Bool()
  val pad_yy_icg_scan_en = Bool()
  val rb_wmb_ce_hit_idx = Bool()
  val rtu_lsu_async_flush = Bool()
  val SQPop= new SQPopInfo
  val fromSQ = new Bundle{
    val wmb_ce_create_hit_rb_idx = Bool()
    val wmb_ce_dcache_share = Bool()
    val wmb_ce_dcache_valid = Bool()
  }
  val fromWmb = new Bundle{
    val create_dp_vld = Bool()
    val create_gateclk_en = Bool()
    val create_merge = Bool()
    val create_merge_ptr = Vec(LSUConfig.WMB_ENTRY, Bool())
    val create_same_dcache_line = Vec(LSUConfig.WMB_ENTRY, Bool())
    val create_stall = Bool()
    val create_vld = Bool()
    val pop_vld = Bool()
    val entry_vld = Vec(LSUConfig.WMB_ENTRY, Bool())
  }
}

class WmbCeOutput extends Bundle{
  val toWmb = new Bundle{
    val addr = UInt(40.W)
    val atomic = Bool()
    val bytes_vld = UInt(16.W)
    val bytes_vld_full = Bool()
    val ca_st_inst = Bool()
    val create_wmb_data_req = Bool()
    val create_wmb_dp_req = Bool()
    val create_wmb_gateclk_en = Bool()
    val create_wmb_req = Bool()
    val data_vld = UInt(4.W)
    val dcache_inst = Bool()
    val dcache_sw_inst = Bool()
    val hit_sq_pop_dcache_line = Bool()
    val icc = Bool()
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
    val sync_fence = Bool()
    val vld = Bool()
    val wb_cmplt_success = Bool()
    val wb_data_success = Bool()
    val write_biu_dp_req = Bool()
    val write_imme = Bool()
  }
  val wmb_ce_sq_ptr = Vec(LSUConfig.SQ_ENTRY, Bool())
}

class WmbCeIO extends Bundle{
  val in  = Input(new WmbCeInput)
  val out = Output(new WmbCeOutput)
}

class WmbCe extends Module {
  val io = IO(new WmbCeIO)

  //Reg
  val wmb_ce_vld = RegInit(false.B)
  val wmb_ce_hit_rb_idx = RegInit(false.B)
  val wmb_ce_hit_rb_idx_ff = RegInit(false.B)
  val pop_info = RegInit(0.U.asTypeOf(new SQPopInfo))
  val wmb_ce_merge = RegInit(false.B)
  val wmb_ce_stall = RegInit(false.B)
  val wmb_ce_merge_ptr = RegInit(VecInit(Seq.fill(LSUConfig.WMB_ENTRY)(false.B)))
  val wmb_ce_same_dcache_line = RegInit(VecInit(Seq.fill(LSUConfig.WMB_ENTRY)(false.B)))

  //Wire
  val wmb_ce_hit_rb_idx_set = Wire(Bool())
  val wmb_ce_dcache_1line_inst = Wire(Bool())
  val wmb_ce_write_data_inst = Wire(Bool())
  val wmb_ce_st_inst = Wire(Bool())
  val wmb_ce_tlbi_asid_inst = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val wmb_ce_create_clk_en = io.in.fromWmb.create_gateclk_en

  //==========================================================
  //                 Register
  //==========================================================
  //+-----+
  //| vld |
  //+-----+
  when(io.in.rtu_lsu_async_flush){
    wmb_ce_vld := false.B
  }.elsewhen(io.in.fromWmb.create_vld){
    wmb_ce_vld := true.B
  }.elsewhen(io.in.fromWmb.pop_vld){
    wmb_ce_vld := false.B
  }
  io.out.toWmb.vld := wmb_ce_vld

  //+------------+
  //| hit rb idx |
  //+------------+
  when(io.in.fromWmb.create_dp_vld){
    wmb_ce_hit_rb_idx := io.in.fromSQ.wmb_ce_create_hit_rb_idx
  }.elsewhen(wmb_ce_vld){
    wmb_ce_hit_rb_idx := wmb_ce_hit_rb_idx_set
  }

  //+---------------+
  //| hit rb idx ff |
  //+---------------+
  when(io.in.fromWmb.create_dp_vld){
    wmb_ce_hit_rb_idx_ff := io.in.fromSQ.wmb_ce_create_hit_rb_idx
  }.elsewhen(wmb_ce_vld){
    wmb_ce_hit_rb_idx_ff := wmb_ce_hit_rb_idx
  }

  //+-------------------------+
  //| instruction information |
  //+-------------------------+
  when(io.in.fromWmb.create_dp_vld){
    pop_info := io.in.SQPop
  }
  io.out.toWmb.addr       := pop_info.addr
  io.out.toWmb.page_ca    := pop_info.page_ca
  io.out.toWmb.page_wa    := pop_info.page_wa
  io.out.toWmb.page_so    := pop_info.page_so
  io.out.toWmb.page_sec   := pop_info.page_sec
  io.out.toWmb.page_buf   := pop_info.page_buf
  io.out.toWmb.page_share := pop_info.page_share
  io.out.toWmb.atomic     := pop_info.atomic
  io.out.toWmb.icc        := pop_info.icc
  io.out.toWmb.sync_fence := pop_info.sync_fence
  io.out.toWmb.inst_flush := pop_info.inst_flush
  io.out.toWmb.inst_type  := pop_info.inst_type
  io.out.toWmb.inst_size  := pop_info.inst_size
  io.out.toWmb.inst_mode  := pop_info.inst_mode
  io.out.toWmb.bytes_vld  := pop_info.bytes_vld
  io.out.toWmb.priv_mode  := pop_info.priv_mode
  io.out.wmb_ce_sq_ptr    := pop_info.sq_ptr

  //+---------------------------+
  //| create/merge/stall signal |
  //+---------------------------+
  when(io.in.fromWmb.create_dp_vld){
    wmb_ce_merge     :=  io.in.fromWmb.create_merge
    wmb_ce_stall     :=  io.in.fromWmb.create_stall
    wmb_ce_merge_ptr :=  io.in.fromWmb.create_merge_ptr
  }
  io.out.toWmb.merge_ptr := wmb_ce_merge_ptr

  when(io.in.fromWmb.create_dp_vld){
    wmb_ce_same_dcache_line := io.in.fromWmb.create_same_dcache_line
  }
  io.out.toWmb.same_dcache_line := wmb_ce_same_dcache_line


  //==========================================================
  //                      Set Wires
  //==========================================================
  wmb_ce_hit_rb_idx_set := wmb_ce_hit_rb_idx && io.in.rb_wmb_ce_hit_idx &&
    (pop_info.wo_st || pop_info.atomic || wmb_ce_dcache_1line_inst)

  //==========================================================
  //                    Request Wires
  //==========================================================
  //-----------------------pop req wires----------------------
  val wmb_ce_merge_ptr_and_not_vld = Wire(Vec(LSUConfig.WMB_ENTRY, Bool()))
  for(i <- 0 until LSUConfig.WMB_ENTRY){
    wmb_ce_merge_ptr_and_not_vld(i) := wmb_ce_merge_ptr(i) && !io.in.fromWmb.entry_vld(i)
  }

  val wmb_ce_merge_not_vld = wmb_ce_merge_ptr_and_not_vld.asUInt.orR

  //if not stall, then not hit rb idx
  io.out.toWmb.merge_wmb_req := wmb_ce_vld && wmb_ce_merge && !wmb_ce_stall

  io.out.toWmb.merge_wmb_wait_not_vld_req := wmb_ce_vld && wmb_ce_merge && wmb_ce_stall

  io.out.toWmb.create_wmb_req := wmb_ce_vld && !wmb_ce_hit_rb_idx_ff && (!wmb_ce_merge || wmb_ce_merge_not_vld)

  io.out.toWmb.create_wmb_dp_req := wmb_ce_vld && (!wmb_ce_merge || wmb_ce_stall)

  io.out.toWmb.create_wmb_data_req := io.out.toWmb.create_wmb_dp_req && wmb_ce_write_data_inst

  io.out.toWmb.create_wmb_gateclk_en := io.out.toWmb.create_wmb_dp_req

  //==========================================================
  //                    Data Wires
  //==========================================================
  //------------------inst type-------------------------------
  wmb_ce_write_data_inst := pop_info.atomic || wmb_ce_st_inst || wmb_ce_tlbi_asid_inst

  wmb_ce_st_inst := !pop_info.atomic && !pop_info.icc & !pop_info.sync_fence

  val wmb_ce_stamo_inst = pop_info.atomic && (pop_info.inst_type === 0.U(2.W))

  val wmb_ce_sc_inst = pop_info.atomic && (pop_info.inst_type === 1.U(2.W))

  io.out.toWmb.ca_st_inst := wmb_ce_st_inst && pop_info.page_ca

  val wmb_ce_so_st_inst = wmb_ce_st_inst && pop_info.page_so

  val wmb_ce_wo_st_inst = wmb_ce_st_inst && !pop_info.page_so

  val wmb_ce_sync_fence_inst = !pop_info.atomic && pop_info.sync_fence

  val wmb_ce_tlbi_inst = !pop_info.atomic && pop_info.icc && (pop_info.inst_type === 0.U(2.W))

  wmb_ce_tlbi_asid_inst := wmb_ce_tlbi_inst && pop_info.inst_mode(0)

  val wmb_ce_dcache_inst = !pop_info.atomic && pop_info.icc && (pop_info.inst_type === 2.U(2.W))
  io.out.toWmb.dcache_inst := wmb_ce_dcache_inst

  wmb_ce_dcache_1line_inst := wmb_ce_dcache_inst && (pop_info.inst_mode =/= 0.U(2.W))

  io.out.toWmb.dcache_sw_inst := wmb_ce_dcache_inst && (pop_info.inst_mode === 2.U(2.W))

  val wmb_ce_dcache_addr_inst = wmb_ce_dcache_inst && pop_info.inst_mode(0)

  val wmb_ce_dcache_addr_not_l1_inst = wmb_ce_dcache_addr_inst && (pop_info.inst_size(1,0) =/= 0.U(2.W))

  val wmb_ce_ctc_inst = !pop_info.atomic && pop_info.icc && (pop_info.inst_type =/= 2.U(2.W))

  val wmb_ce_dcache_all_inst = wmb_ce_dcache_inst && (pop_info.inst_mode === 0.U(2.W))

  //------------------pop info for wmb------------------------
  val wmb_ce_merge_en = pop_info.wo_st
  io.out.toWmb.merge_en := wmb_ce_merge_en
  io.out.toWmb.wb_cmplt_success := pop_info.wo_st || wmb_ce_ctc_inst || wmb_ce_dcache_inst && !wmb_ce_dcache_all_inst
  io.out.toWmb.wb_data_success := !wmb_ce_sc_inst

  //------------------data request----------------------------
  //----------get data_vld signal---------
  val wmb_ce_data_vld_vec = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    wmb_ce_data_vld_vec(i) := pop_info.bytes_vld(4*i+3, 4*i).orR
  }
  io.out.toWmb.data_vld := wmb_ce_data_vld_vec.asUInt

  io.out.toWmb.bytes_vld_full := pop_info.bytes_vld.andR

  //-----------sc signal----------------
  val wmb_ce_sc_wb_vld = io.in.lm_sq_sc_fail
  io.out.toWmb.sc_wb_vld := wmb_ce_sc_wb_vld

  //-------------------wmb status signal----------------------
  io.out.toWmb.write_imme := !pop_info.wo_st

  io.out.toWmb.read_dp_req := wmb_ce_st_inst && pop_info.page_ca && pop_info.page_share && (io.in.fromSQ.wmb_ce_dcache_share  || !io.in.fromSQ.wmb_ce_dcache_valid) ||
    wmb_ce_ctc_inst ||
    wmb_ce_dcache_addr_inst && pop_info.page_ca && (wmb_ce_dcache_addr_not_l1_inst  || pop_info.page_share) && pop_info.page_ca ||
    wmb_ce_sc_inst && pop_info.page_ca && pop_info.page_share

  //for write gateclk
  io.out.toWmb.write_biu_dp_req := wmb_ce_so_st_inst ||
    wmb_ce_wo_st_inst && !(pop_info.page_ca  && io.in.fromSQ.wmb_ce_dcache_valid) ||
    wmb_ce_sync_fence_inst ||
    wmb_ce_stamo_inst && !(pop_info.page_ca  && io.in.fromSQ.wmb_ce_dcache_valid) ||
    wmb_ce_sc_inst && !(pop_info.page_ca  && io.in.fromSQ.wmb_ce_dcache_valid) && !wmb_ce_sc_wb_vld

  //==========================================================
  //                  Compare with sq pop
  //==========================================================
  //wmb_ce_hit_sq_pop_cache_line is used for same_dcache_line
  val wmb_ce_hit_sq_pop_addr_tto6 = pop_info.addr(LSUConfig.PA_WIDTH-1,6) === io.in.SQPop.addr(LSUConfig.PA_WIDTH-1,6)
  val wmb_ce_hit_sq_pop_addr_5to4 = pop_info.addr(5,4) === io.in.SQPop.addr(5,4)
  val wmb_ce_hit_sq_pop_addr_tto4 = wmb_ce_hit_sq_pop_addr_tto6 && wmb_ce_hit_sq_pop_addr_5to4

  io.out.toWmb.hit_sq_pop_dcache_line := wmb_ce_hit_sq_pop_addr_tto6 && wmb_ce_st_inst && wmb_ce_vld

  //if supv mode or page info is not hit, then set write_imme and donot grnt
  //signal to sq
  io.out.toWmb.merge_data_addr_hit := wmb_ce_hit_sq_pop_addr_tto4 && wmb_ce_merge_en && wmb_ce_vld

  val wmb_ce_merge_data_permit = pop_info.priv_mode === io.in.SQPop.priv_mode

  io.out.toWmb.merge_data_stall := io.out.toWmb.merge_data_addr_hit && !wmb_ce_merge_data_permit

}
