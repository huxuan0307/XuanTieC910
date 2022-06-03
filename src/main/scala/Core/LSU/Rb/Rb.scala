package Core.LSU.Rb
import Core.LSU.{IdFifo8, RotData}
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
  val rb_biu_req_create_lfb = RegInit(false.B)
  val rb_biu_req_addr = RegInit(0.U(PA_WIDTH.W))
  val rb_ld_wb_data_ptr = RegInit(VecInit(Seq.fill(RB_ENTRY)(false.B)))

  io.out.toBiu.req_unmask := rb_biu_req_unmask
  io.out.toBiu.req_addr := rb_biu_req_addr
  io.out.lsu_has_fence := lsu_has_fence
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

  val rb_pipe_biu_pe_req = Wire(Bool())
  val rb_read_req_grnt = Wire(Bool())
  val rb_biu_pe_req_permit = Wire(Bool())
  val rb_biu_pe_req = Wire(Bool())
  val rb_biu_pe_req_ptr = Wire(Vec(RB_ENTRY, Bool()))
  val rb_biu_pe_create_lfb = Wire(Bool())
  val rb_biu_pe_req_addr = Wire(UInt(PA_WIDTH.W))

  val rb_nc_fifo_empty = Wire(Bool())
  val rb_ld_wb_cmplt_ptr = Wire(Vec(RB_ENTRY, Bool()))

  io.out.rb_fence_ld := rb_fence_ld
  io.out.toBiu.ar_id := rb_biu_ar_id
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

  for(i <- 0 until RB_ENTRY){
    rb_entry_in(i).next_nc_bypass := rb_entry_next_nc_bypass(i)
  }
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

  for(i <- 0 until RB_ENTRY){
    rb_entry_in(i).next_so_bypass := rb_entry_next_so_bypass(i)
  }
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
  val rb_create_ptr1 = VecInit(PriorityEncoderOH(rb_entry_out.map(!_.vld).reverse).reverse)

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
  val rb_create_ld_success = io.in.fromLoadDA.rb_create_vld && !rb_ld_da_full && !io.in.fromRTU.yy_xx_flush

  val rb_create_st_success = io.in.fromStoreDA.rb_create_vld && !rb_st_da_full && !io.in.fromRTU.yy_xx_flush

  rb_entry_in.zip(rb_create_ptr0).foreach{case(entry_in, ptr) => entry_in.ld_create_vld := ptr && rb_create_ld_success}
  rb_entry_in.zip(rb_create_ptr1).foreach{case(entry_in, ptr) => entry_in.st_create_vld := ptr && rb_create_st_success}

  //------------------create dp vld---------------------------
  rb_entry_in.zip(rb_create_ptr0).foreach{
    case(entry_in, ptr) => entry_in.ld_create_dp_vld := ptr && !rb_ld_da_full && io.in.fromLoadDA.rb_create_dp_vld
  }
  rb_entry_in.zip(rb_create_ptr1).foreach{
    case(entry_in, ptr) => entry_in.st_create_dp_vld := ptr && !rb_st_da_full && io.in.fromStoreDA.rb_create_dp_vld
  }

  //------------------create gateclk vld----------------------
  rb_entry_in.zip(rb_create_ptr0).foreach{
    case(entry_in, ptr) => entry_in.ld_create_gateclk_en := ptr && !rb_ld_da_full && io.in.fromLoadDA.rb_create_gateclk_en
  }
  rb_entry_in.zip(rb_create_ptr1).foreach{
    case(entry_in, ptr) => entry_in.st_create_gateclk_en := ptr && !rb_st_da_full && io.in.fromStoreDA.rb_create_gateclk_en
  }

  //==========================================================
  //                    info for bar ready
  //==========================================================
  //success neglect mmu request
  val rb_ready_ld_req_biu = VecInit(rb_entry_out.map(entry_out => !entry_out.st && !entry_out.mcic_req && entry_out.biu_req)).asUInt.orR

  rb_ready_ld_req_biu_success := !rb_ready_ld_req_biu
  val rb_ready_req_biu = VecInit(rb_entry_out.map(entry_out => !entry_out.mcic_req && entry_out.biu_req)).asUInt.orR
  rb_ready_all_req_biu_success := !rb_ready_req_biu

  //==========================================================
  //                        Fence signal
  //==========================================================
  val rb_has_sync_fence = VecInit(rb_entry_out.map(entry_out => !entry_out.vld && entry_out.sync_fence)).asUInt.orR
  val lsu_has_fence_set = rb_has_sync_fence || io.in.fromWmb.has_sync_fence
  io.out.lsu_idu_no_fence := !lsu_has_fence

  when(rb_create_st_success  &&  !lsu_has_fence){
    lsu_has_fence := io.in.fromStoreDA.sync_fence
  }.otherwise{
    lsu_has_fence := lsu_has_fence_set
  }

  //==========================================================
  //                  Request biu pop entry
  //==========================================================
  //------------------------registers-------------------------
  //+---------+
  //| biu_req |
  //+---------+
  when(rb_pipe_biu_pe_req){
    rb_biu_req_unmask := true.B
  }.elsewhen(rb_read_req_grnt || rb_biu_req_flush_clear){
    rb_biu_req_unmask := false.B
  }

  //+---------+------+------------+
  //| pop ptr | addr | lfb_create |
  //+---------+------+------------+
  when(rb_biu_pe_req_permit  &&  rb_biu_pe_req){
    rb_biu_req_ptr        :=  rb_biu_pe_req_ptr
    rb_biu_req_create_lfb :=  rb_biu_pe_create_lfb
    rb_biu_req_addr       :=  rb_biu_pe_req_addr
  }.elsewhen(rb_ld_biu_pe_req_grnt){
    rb_biu_req_ptr        :=  rb_create_ptr0
    rb_biu_req_create_lfb :=  io.in.fromLoadDA.rb_create_lfb
    rb_biu_req_addr       :=  io.in.fromLoadDA.addr
  }

  //-----------------------biu req ptr------------------------
  rb_biu_pe_req_gateclk_en := VecInit(rb_entry_out.map(_.biu_pe_req_gateclk_en)).asUInt.orR
  rb_biu_pe_req := VecInit(rb_entry_out.map(_.biu_pe_req)).asUInt.orR

  rb_biu_pe_req_ptr := VecInit(PriorityEncoderOH(rb_entry_out.map(_.biu_pe_req)))

  rb_biu_pe_create_lfb := VecInit(rb_entry_out.zip(rb_biu_pe_req_ptr).map(x => x._1.create_lfb && x._2)).asUInt.orR
  rb_biu_pe_req_addr := VecInit(rb_entry_out.zip(rb_biu_pe_req_ptr).map(x => Mux(x._2, x._1.addr, 0.U(PA_WIDTH.W)))).reduce(_ | _)

  //-------------------ld/st biu pop req----------------------
  val rb_ld_biu_pe_req    = rb_create_ld_success && !io.in.fromLoadDA.rb_data_vld && !io.in.fromLoadDA.page_so && !lsu_has_fence

  rb_pipe_biu_pe_req := rb_biu_pe_req || rb_ld_biu_pe_req

  //---------------------pe_req_grnt-------------------------
  rb_biu_pe_req_permit := !rb_biu_req_unmask || rb_read_req_grnt || rb_biu_req_flush_clear
  rb_entry_in.zip(rb_biu_pe_req_ptr).foreach{
    case(entry_in, ptr) => entry_in.biu_pe_req_grnt := rb_biu_pe_req_permit && ptr
  }

  rb_ld_biu_pe_req_grnt := rb_biu_pe_req_permit && !rb_biu_pe_req && rb_ld_biu_pe_req

  //-----------------flush clear in coror----------------------
  rb_biu_req_flush_clear := VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.flush_clear && x._2)).asUInt.orR && rb_biu_req_unmask

  //==========================================================
  //                      Request biu
  //==========================================================
  //------------------barrier---------------------------------
  rb_fence_ld := VecInit(rb_entry_out.map(_.fence_ld_vld)).asUInt.orR

  //------------------biu req info----------------------------
  val rb_biu_req_mcic_req   = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.mcic_req && x._2)).asUInt.orR
  val rb_biu_req_page_ca    = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.page_ca && x._2)).asUInt.orR
  val rb_biu_req_page_so    = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.page_so && x._2)).asUInt.orR
  val rb_biu_req_page_sec   = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.page_sec && x._2)).asUInt.orR
  val rb_biu_req_page_buf   = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.page_buf && x._2)).asUInt.orR
  val rb_biu_req_page_share = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.page_share && x._2)).asUInt.orR
  val rb_biu_req_sync       = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.sync && x._2)).asUInt.orR
  val rb_biu_req_fence      = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.fence && x._2)).asUInt.orR
  val rb_biu_req_atomic     = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.atomic && x._2)).asUInt.orR
  val rb_biu_req_ldamo      = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.ldamo && x._2)).asUInt.orR
  val rb_biu_req_st         = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._1.st && x._2)).asUInt.orR

  val rb_biu_req_inst_size = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => Mux(x._2, x._1.inst_size, 0.U(3.W)))).reduce(_ | _)
  val rb_biu_req_priv_mode = VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => Mux(x._2, x._1.priv_mode, 0.U(2.W)))).reduce(_ | _)

  val rb_biu_req_page_ca_dcache_en = rb_biu_req_page_ca && io.in.fromCp0.lsu_dcache_en
  val rb_biu_req_page_nc_atomic    = !rb_biu_req_page_ca &&  rb_biu_req_atomic

  val rb_biu_req_hit_idx = io.in.fromWmb.rb_biu_req_hit_idx || io.in.fromLfb.rb_biu_req_hit_idx || io.in.vb_rb_biu_req_hit_idx

  io.out.toBiu.ar_req := rb_biu_req_unmask && !rb_biu_req_flush_clear &&
    (!rb_biu_req_hit_idx && rb_biu_req_page_ca && !io.in.fromLfb.addr_full && (io.in.fromLfb.rb_ca_rready_grnt || rb_nc_fifo_empty) ||
      !rb_biu_req_page_ca && io.in.fromLfb.rb_nc_rready_grnt)

  io.out.toBiu.ar_dp_req := rb_biu_req_unmask && !rb_biu_req_flush_clear &&
    (rb_biu_req_page_ca && !io.in.fromLfb.addr_full && (io.in.fromLfb.rb_ca_rready_grnt || rb_nc_fifo_empty) ||
      !rb_biu_req_page_ca && io.in.fromLfb.rb_nc_rready_grnt)

  io.out.toBiu.ar_req_gateclk_en := rb_biu_req_unmask

  //----------ar_id-----------------------
  val rb_biu_req_sync_fence = rb_biu_req_sync  ||  rb_biu_req_fence

  rb_biu_ar_id := PriorityMux(Seq(
    rb_biu_req_sync_fence -> BIU_R_SYNC_FENCE_ID,
    rb_biu_req_page_so -> BIU_R_SO_ID,
    rb_biu_req_page_ca -> io.in.fromLfb.rb_create_id,
    rb_biu_req_atomic -> BIU_R_NC_ATOM_ID,
    (!rb_biu_req_sync_fence &&
    !rb_biu_req_page_so &&
    !rb_biu_req_page_ca &&
    !rb_biu_req_atomic) -> BIU_R_NC_ID,
    true.B -> 0.U(5.W)
  ))

  //----------ar others-------------------
  val rb_biu_ar_size_maintain = (rb_biu_req_page_so || rb_biu_req_page_nc_atomic) && !rb_biu_req_sync_fence

  io.out.toBiu.ar_addr := Cat(rb_biu_req_addr(PA_WIDTH-1,4), Mux(rb_biu_ar_size_maintain, rb_biu_req_addr(3,0), 0.U(4.W)))

  //len: a cache line(4x) or 1x
  val rb_biu_len3_sel = rb_biu_req_page_ca_dcache_en && !rb_biu_req_sync_fence ||
    rb_biu_req_page_ca && rb_biu_req_page_share && rb_biu_req_atomic

  io.out.toBiu.ar_len := Mux(rb_biu_len3_sel, 3.U(2.W), 0.U(2.W))

  //size:
  //  ca/nc : 111
  //  so    : so_size
  io.out.toBiu.ar_size := Mux(rb_biu_ar_size_maintain, rb_biu_req_inst_size, "b100".U)

  //burst:incr1 or wrap4
  io.out.toBiu.ar_burst := Mux(rb_biu_req_page_ca_dcache_en && !rb_biu_req_sync_fence, 2.U(2.W), 1.U(2.W))

  io.out.toBiu.ar_lock := rb_biu_req_page_nc_atomic

  //cache:
  //if sync/bar use normal, noncacheable
  io.out.toBiu.ar_cache := Mux(rb_biu_req_sync_fence, "b0011".U,
    Cat(rb_biu_req_page_ca, rb_biu_req_page_ca, !rb_biu_req_page_so, rb_biu_req_page_buf))

  //prot:2:inst,1:sec,0:supv
  io.out.toBiu.ar_prot := Cat(0.U(1.W), rb_biu_req_page_sec, rb_biu_req_priv_mode(0))

  //if request both biu and lfb, then the rb_entry must get two grnt signal, else
  //it will not request biu or lfb. it is realized in rb top module.
  io.out.toBiu.ar_user := Cat(0.U(1.W), rb_biu_req_priv_mode(1), rb_biu_req_mcic_req)
  //----------ar snoop--------------------
  val rb_atomic_readunique = rb_biu_req_page_ca && rb_biu_req_page_share && rb_biu_req_ldamo

  //lr should send read shared when cache is off (for l2 snoop filter)
  val rb_biu_share_refill = rb_biu_req_page_ca && rb_biu_req_page_share && (io.in.fromCp0.lsu_dcache_en || rb_biu_req_atomic)

  io.out.toBiu.ar_snoop := PriorityMux(Seq(
    rb_atomic_readunique -> "b0111".U,//ReadUnique
    (rb_biu_share_refill && rb_biu_req_st) -> "b0111".U,//ReadUnique
    (rb_biu_share_refill && !rb_biu_req_st) -> "b0001".U,//ReadShared
    true.B -> 0.U(4.W)//ReadNoSnoop & ReadOnce
  ))

  //if ld non-cacheable then domain = 2'b11
  io.out.toBiu.ar_domain := Mux(!rb_biu_req_page_ca && !rb_biu_req_st, "b11".U, Cat(0.U(1.W), rb_biu_req_page_share))

  val rb_biu_req_not_wait_fence = rb_biu_req_mcic_req || rb_biu_req_st

  io.out.toBiu.ar_bar := MuxLookup(Cat(rb_biu_req_not_wait_fence,rb_biu_req_sync,rb_biu_req_fence), 0.U(2.W), Seq(
    "b100".U -> "b10".U,//mmu req
    "b110".U -> "b11".U,//sync req
    "b101".U -> "b01".U //fence req
  ))

  //-----------------biu grnt signal--------------------------
  rb_biu_nc_req_grnt := io.in.bus_arb_rb_ar_grnt && !rb_biu_req_page_ca && !rb_biu_req_page_so &&
    !rb_biu_req_atomic && !rb_biu_req_sync_fence

  rb_biu_so_req_grnt := io.in.bus_arb_rb_ar_grnt && rb_biu_req_page_so && !rb_biu_req_sync_fence

  rb_read_req_grnt := io.in.bus_arb_rb_ar_grnt

  //for timing, use shorter route of req_success for gateclk
  rb_read_req_grnt_gateclk_en := rb_biu_req_unmask
  rb_entry_in.zip(rb_biu_req_ptr).foreach{
    case(entry_in, ptr) => entry_in.biu_id_gateclk_en := rb_read_req_grnt_gateclk_en && ptr
  }
  rb_entry_in.zip(rb_biu_req_ptr).foreach{
    case(entry_in, ptr) => entry_in.read_req_grnt := rb_read_req_grnt && ptr
  }

  //==========================================================
  //                  Request ld_wb stage
  //==========================================================
  //------------------wb cmplt part signal--------------------
  //because only so/ex load request cmplt, so there is only 1 complete request
  io.out.toLoadWB.cmplt_req := VecInit(rb_entry_out.map(_.wb_cmplt_req)).asUInt.orR
  io.out.toLoadWB.iid := VecInit(rb_entry_out.zip(rb_ld_wb_cmplt_ptr).map(x => Mux(x._2, x._1.iid, 0.U(7.W)))).reduce(_ | _)
  io.out.toLoadWB.bkpta_data := VecInit(rb_entry_out.zip(rb_ld_wb_cmplt_ptr).map(x => x._2 && x._1.bkpta_data)).asUInt.orR
  io.out.toLoadWB.bkptb_data := VecInit(rb_entry_out.zip(rb_ld_wb_cmplt_ptr).map(x => x._2 && x._1.bkptb_data)).asUInt.orR

  //----------wb cmplt part grnt----------
  rb_entry_in.zip(rb_ld_wb_cmplt_ptr).foreach{
    case(entry_in, ptr) => entry_in.wb_cmplt_grnt := io.in.ld_wb_rb_cmplt_grnt && ptr
  }

  io.out.toLoadWB.expt_vld     := false.B
  io.out.toLoadWB.expt_gateclk := false.B

  //------------------wb data part signal---------------------
  //use static arbitrate
  val rb_ld_wb_data_ptr_pre = VecInit(PriorityEncoderOH(rb_entry_out.map(_.wb_data_pre_sel)))

  //------------------wb cmplt part signal---------------------
  //use static arbitrate
  rb_ld_wb_cmplt_ptr := VecInit(PriorityEncoderOH(rb_entry_out.map(_.wb_cmplt_req)))

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //data_ptr pop signal
  when(rb_not_empty){
    rb_ld_wb_data_ptr := rb_ld_wb_data_ptr_pre
  }

  val rb_ld_wb_data_req_unmask = VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => x._2 && x._1.wb_data_req)).asUInt.orR

  val rb_ld_wb_inst_size = VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.inst_size, 0.U(3.W)))).reduce(_ | _)

  io.out.toLoadWB.preg := VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.preg, 0.U(7.W)))).reduce(_ | _)

  val rb_ld_wb_sign_extend = VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => x._2 && x._1.sign_extend)).asUInt.orR
  io.out.toLoadWB.bus_err := VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => x._2 && x._1.bus_err)).asUInt.orR
  io.out.toLoadWB.data_req := rb_ld_wb_data_req_unmask

  io.out.toLoadWB.vreg := VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.vreg, 0.U(6.W)))).reduce(_ | _)

  io.out.toLoadWB.bus_err_addr := VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.addr, 0.U(PA_WIDTH.W)))).reduce(_ | _)

  io.out.toLoadWB.data_iid := VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.iid, 0.U(7.W)))).reduce(_ | _)

  io.out.toLoadWB.inst_vfls := VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => x._2 && x._1.inst_vfls)).asUInt.orR
  val rb_ld_wb_vreg_sign = VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => x._2 && x._1.vreg_sign_sel)).asUInt.orR

  //----------wb data part grnt-----------
  rb_entry_in.zip(rb_ld_wb_data_ptr).foreach{
    case(entry_in, ptr) => entry_in.wb_data_grnt := io.in.ld_wb_rb_data_grnt && !io.in.fromRTU.yy_xx_flush && ptr
  }

  //==========================================================
  //            Settle data to register mode
  //==========================================================
  val rb_ld_rot_sel = VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.rot_sel, 0.U(8.W)))).reduce(_ | _)

  val rb_wb_data = VecInit(rb_entry_out.zip(rb_ld_wb_data_ptr).map(x => Mux(x._2, x._1.data, 0.U(64.W)))).reduce(_ | _)

  val rb_wb_data_unsettle = Cat(rb_wb_data, rb_wb_data)

  //rotate data
  val wb_data_rot = Module(new RotData)
  wb_data_rot.io.dataIn := rb_wb_data_unsettle
  wb_data_rot.io.rotSel := rb_ld_rot_sel
  val rb_ld_wb_data_128 = wb_data_rot.io.dataSettle

  io.out.toLoadWB.data := rb_ld_wb_data_128(63,0)

  //------------------select sign bit-------------------------
  io.out.toLoadWB.preg_sign_sel := MuxLookup(Cat(rb_ld_wb_sign_extend,rb_ld_wb_inst_size), "b0001".U, Seq(
    Cat(1.U(1.W),BYTE.U) -> "b0010".U,
    Cat(1.U(1.W),HALF.U) -> "b0100".U,
    Cat(1.U(1.W),WORD.U) -> "b1000".U
  ))

  io.out.toLoadWB.vreg_sign_sel :=
    Cat(rb_ld_wb_vreg_sign && rb_ld_wb_inst_size === "b10".U, rb_ld_wb_vreg_sign && rb_ld_wb_inst_size === "b01".U)

  //==========================================================
  //                    compare index
  //==========================================================
  io.out.rb_sq_pop_hit_idx      := VecInit(rb_entry_out.map(_.sq_pop_hit_idx)).asUInt.orR
  io.out.rb_wmb_ce_hit_idx      := VecInit(rb_entry_out.map(_.wmb_ce_hit_idx)).asUInt.orR
  io.out.rb_pfu_biu_req_hit_idx := VecInit(rb_entry_out.map(_.pfu_biu_req_hit_idx)).asUInt.orR
  io.out.toLoadDA.hit_idx       := VecInit(rb_entry_out.map(_.ld_da_hit_idx)).asUInt.orR
  io.out.rb_st_da_hit_idx       := VecInit(rb_entry_out.map(_.st_da_hit_idx)).asUInt.orR

  //==========================================================
  //                    Compare r_id
  //==========================================================
  rb_r_nc_id_hit := io.in.fromBiu.r_vld && io.in.fromBiu.r_id === BIU_R_NC_ID

  rb_r_so_id_hit := io.in.fromBiu.r_vld && io.in.fromBiu.r_id === BIU_R_SO_ID

  //ecc err(DECERR) will not carry bus err expt
  rb_r_resp_err := io.in.fromBiu.r_vld && io.in.fromBiu.r_resp === SLVERR
  rb_r_resp_okay := io.in.fromBiu.r_vld && io.in.fromBiu.r_resp === OKAY
  val rb_r_resp_ecc_err = io.in.fromBiu.r_vld && io.in.fromBiu.r_resp === DECERR

  //==========================================================
  //                for avoid deadlock with no rready
  //==========================================================
  rb_nc_fifo_empty := rb_so_no_pending  && rb_nc_no_pending
  val rb_nc_ar_req = rb_biu_req_unmask && !rb_biu_req_page_ca
  io.out.rb_pfu_nc_no_pending := rb_nc_fifo_empty  && !rb_nc_ar_req

  //==========================================================
  //        interface to other module (except biu_arb)
  //==========================================================
  //------------------------lfb-------------------------------
  val rb_lfb_create_vld_unmask  = rb_biu_req_unmask & rb_biu_req_create_lfb
  io.out.toLfb.depd            := VecInit(rb_entry_out.zip(rb_biu_req_ptr).map(x => x._2 && (x._1.depd || x._1.discard_vld))).asUInt.orR
  io.out.toLfb.atomic          := rb_biu_req_atomic
  io.out.toLfb.ldamo           := rb_biu_req_ldamo
  io.out.toLfb.addr_tto4       := rb_biu_req_addr

  //create req signal is used for artribute
  io.out.toLfb.create_req           := rb_lfb_create_vld_unmask
  io.out.toLfb.create_vld           := io.in.bus_arb_rb_ar_grnt && rb_biu_req_create_lfb
  io.out.toLfb.create_dp_vld        := rb_lfb_create_vld_unmask
  io.out.toLfb.create_gateclk_en    := rb_lfb_create_vld_unmask
  io.out.toLfb.boundary_depd_wakeup := VecInit(rb_entry_out.map(_.boundary_wakeup)).asUInt.orR
  //------------------------mcic------------------------------
  io.out.toMcic.biu_req_success := io.in.bus_arb_rb_ar_grnt && rb_biu_req_mcic_req
  io.out.toMcic.ar_id           := rb_biu_ar_id
  io.out.toMcic.not_full        := !rb_full
  io.out.toMcic.ecc_err         := rb_r_resp_ecc_err
  //------------------------lm--------------------------------
  io.out.toLm.wait_resp_dp_vld  := rb_biu_req_unmask &&  rb_biu_req_atomic
  io.out.toLm.wait_resp_vld     := io.in.bus_arb_rb_ar_grnt && rb_biu_req_atomic
  io.out.toLm.ar_id             := rb_biu_ar_id
  io.out.toLm.atomic_next_resp  := VecInit(rb_entry_out.map(_.atomic_next_resp)).asUInt.orR
  //------------------------idu-------------------------------
  io.out.lsu_idu_rb_not_full    := !rb_empty_less2
  //------------------------rtu-------------------------------
  io.out.lsu_rtu_all_commit_ld_data_vld := VecInit(rb_entry_out.map(_.cmit_data_vld)).asUInt.andR
  //------------------------had-------------------------------
  for(i <- 0 until RB_ENTRY){
    io.out.toHad.rb_entry_state(i) := rb_entry_out(i).state
    io.out.toHad.rb_entry_fence(i) := rb_entry_out(i).fence
  }

}
