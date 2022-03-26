package Core.LSU
import chisel3._
import chisel3.util._

class LoadDC2DA extends Bundle {
  val addr0 = UInt(40.W)
  val ahead_predict = Bool()
  val ahead_preg_wb_vld = Bool()
  val ahead_vreg_wb_vld = Bool()
  val already_da = Bool()
  val atomic = Bool()
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val borrow_db = UInt(3.W)
  val borrow_icc = Bool()
  val borrow_icc_tag = Bool()
  val borrow_mmu = Bool()
  val borrow_sndb = Bool()
  val borrow_vb = Bool()
  val borrow_vld = Bool()
  val boundary = Bool()
  val da_bytes_vld  = UInt(16.W)
  val da_bytes_vld1 = UInt(16.W)
  val da_cb_addr_create = Bool()
  val da_cb_merge_en = Bool()
  val da_data_rot_sel = UInt(8.W)
  val da_expt_vld_gate_en = Bool()
  val da_icc_tag_vld = Bool()
  val da_inst_vld = Bool()
  val da_old = Bool()
  val da_page_buf = Bool()
  val da_page_ca = Bool()
  val da_page_sec = Bool()
  val da_page_share = Bool()
  val da_page_so = Bool()
  val da_pf_inst = Bool()
  val da_tag_read = UInt(27.W)
  val dcache_hit = Bool()
  val expt_access_fault_extra = Bool()
  val expt_access_fault_mask = Bool()
  val expt_vec = UInt(5.W)
  val expt_vld_except_access_err = Bool()
  val fwd_bytes_vld = UInt(16.W)
  val fwd_sq_vld = Bool()
  val fwd_wmb_vld = Bool()
  val get_dcache_data = UInt(4.W)
  val hit_high_region = Bool()
  val hit_low_region = Bool()
  val iid = UInt(7.W)
  val inst_size = UInt(3.W)
  val inst_type = UInt(2.W)
  val inst_vfls = Bool()
  val inst_vld = Bool()
  val ldfifo_pc = UInt(15.W)
  val lsid = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val mmu_req = Bool()
  val mt_value = UInt(40.W)
  val no_spec = Bool()
  val no_spec_exist = Bool()
  val pfu_info_set_vld = Bool()
  val pfu_va = UInt(40.W)
  val preg = UInt(7.W)
  val preg_sign_sel = UInt(4.W)
  val secd = Bool()
  val settle_way = Bool()
  val sign_extend = Bool()
  val spec_fail = Bool()
  val split = Bool()
  val vector_nop = Bool()
  val vreg = UInt(6.W)
  val vreg_sign_sel = Bool()
  val wait_fence = Bool()
}

class LoadDCDataReg extends Bundle{
  val expt_vld_except_access_err = Bool()
  val pf_inst         = Bool()
  val vpn             = UInt(28.W)
  val addr1_tto4      = UInt((LSUConfig.PA_WIDTH-4).W)
  val split           = Bool()
  val inst_type       = UInt(2.W)
  val inst_size       = UInt(2.W)
  val secd            = Bool()
  val already_da      = Bool()
  val lsiq_spec_fail  = Bool()
  val lsiq_bkpta_data = Bool()
  val lsiq_bkptb_data = Bool()
  val sign_extend     = Bool()
  val atomic          = Bool()
  val iid             = UInt(7.W)
  val lsid            = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val old             = Bool()
  val bytes_vld       = UInt(16.W)
  val bytes_vld1      = UInt(16.W)
  val rot_sel         = UInt(4.W)
  val boundary        = Bool()
  val preg            = UInt(7.W)
  val preg_dup        = Vec(5, UInt(7.W))
  val ldfifo_pc       = UInt(LSUConfig.PC_LEN.W)
  val ahead_predict   = Bool()
  val page_so         = Bool()
  val page_ca         = Bool()
  val page_buf        = Bool()
  val page_sec        = Bool()
  val page_share      = Bool()
  val utlb_miss       = Bool()
  val tlb_busy        = Bool()
  val vreg            = UInt(7.W)
  val vreg_dup        = Vec(4, UInt(6.W))
  val inst_vfls       = Bool()
  val acclr_en        = Bool()
  val fwd_bypass_en   = Bool()
  val no_spec         = Bool()
  val no_spec_exist   = Bool()
  val raw_new         = Bool()
}

class LoadDCInput extends Bundle{
  val cb_ld_dc_addr_hit = Bool()
  val fromCp0 = new Bundle{
    val lsu_da_fwd_dis = Bool()
    val lsu_dcache_en = Bool()
    val lsu_icg_en = Bool()
    val yy_clk_en = Bool()
  }
  val ctrl_ld_clk = Bool()
  val fromDcacheArb = new Bundle{
    val ld_dc_borrow_db = UInt(3.W)
    val ld_dc_borrow_icc = Bool()
    val ld_dc_borrow_mmu = Bool()
    val ld_dc_borrow_sndb = Bool()
    val ld_dc_borrow_vb = Bool()
    val ld_dc_borrow_vld = Bool()
    val ld_dc_borrow_vld_gate = Bool()
    val ld_dc_settle_way = Bool()
    val idx = UInt(9.W)
  }
  val dcache_lsu_ld_tag_dout = Vec(2, UInt(27.W))
  val fromHad = new Bundle{
    val yy_xx_bkpta_base = UInt(40.W)
    val yy_xx_bkpta_mask = UInt(8.W)
    val yy_xx_bkpta_rc   = Bool()
    val yy_xx_bkptb_base = UInt(40.W)
    val yy_xx_bkptb_mask = UInt(8.W)
    val yy_xx_bkptb_rc   = Bool()
  }
  val icc_dcache_arb_ld_tag_read = Bool()
  val fromAG = new LoadAG2DC
  val fromLQ = new Bundle{
    val ld_dc_full = Bool()
    val ld_dc_inst_hit = Bool()
    val ld_dc_less2 = Bool()
    val ld_dc_spec_fail = Bool()
  }
  val lsu_dcache_ld_xx_gwen = Bool()
  val lsu_has_fence = Bool()
  val fromMMU = new Bundle{
    val data_req_size = Bool()
    val mmu_en = Bool()
    val tlb_busy = Bool()
  }
  val fromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val fromPFU = new Bundle{
    val pfb_empty = Bool()
    val sdb_create_gateclk_en = Bool()
    val sdb_empty = Bool()
  }
  val rb_fence_ld = Bool()
  val fromRTU = new Bundle{
    val yy_xx_flush = Bool()
  }
  val fromSQ = new Bundle{
    val ld_dc_addr1_dep_discard = Bool()
    val ld_dc_cancel_acc_req = Bool()
    val ld_dc_cancel_ahead_wb = Bool()
    val ld_dc_fwd_req = Bool()
    val ld_dc_has_fwd_req = Bool()
  }
  val fromStDC = new Bundle{
    val addr0 = UInt(40.W)
    val bytes_vld = UInt(16.W)
    val chk_st_inst_vld = Bool()
    val chk_statomic_inst_vld = Bool()
    val inst_vld = Bool()
  }
  val fromWmb = new Bundle {
    val fwd_bytes_vld = UInt(16.W)
    val ld_dc_cancel_acc_req = Bool()
    val ld_dc_discard_req = Bool()
    val ld_dc_fwd_req = Bool()
  }
}

class LoadDCOutput extends Bundle{
  val toDA = new LoadDC2DA
  val ld_dc_addr1 = UInt(40.W)
  val ld_dc_addr1_11to4 = UInt(8.W)
  val ld_dc_bytes_vld  = UInt(16.W)
  val ld_dc_bytes_vld1 = UInt(16.W)
  val toCB = new Bundle{
    val addr_create_gateclk_en = Bool()
    val addr_create_vld = Bool()
    val addr_tto4 = UInt(36.W)
  }
  val toChk = new Bundle{
    val atomic_inst_vld = Bool()
    val ld_addr1_vld    = Bool()
    val ld_bypass_vld   = Bool()
    val ld_inst_vld     = Bool()
  }
  val ld_dc_idu_lq_full  = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val ld_dc_idu_tlb_busy = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val ld_dc_imme_wakeup  = Vec(LSUConfig.LSIQ_ENTRY, Bool())
  val ld_dc_inst_chk_vld = Bool()
  val toLQ = new Bundle{
    val create1_dp_vld = Bool()
    val create1_gateclk_en = Bool()
    val create1_vld = Bool()
    val create_dp_vld = Bool()
    val create_gateclk_en = Bool()
    val create_vld = Bool()
    val full_gateclk_en = Bool()
  }
  val ld_dc_tlb_busy_gateclk_en = Bool()
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

class LoadDCIO extends Bundle{
  val in  = Input(new LoadDCInput)
  val out = Output(new LoadDCOutput)
}

class LoadDC extends Module {
  val io = IO(new LoadDCIO)

  //Reg
  val ld_dc_inst_vld = RegInit(false.B)
  val ld_dc_borrow_vld = RegInit(false.B)
  val ld_dc_load_inst_vld_dup = RegInit(VecInit(Seq.fill(5)(false.B)))
  val ld_dc_load_ahead_inst_vld_dup = RegInit(VecInit(Seq.fill(5)(false.B)))
  val ld_dc_vload_inst_vld_dup = RegInit(VecInit(Seq.fill(4)(false.B)))
  val ld_dc_vload_ahead_inst_vld = RegInit(false.B)

  val ld_dc_mmu_req                     = RegInit(false.B)
  val ld_dc_expt_misalign_no_page       = RegInit(false.B)
  val ld_dc_expt_page_fault             = RegInit(false.B)
  val ld_dc_expt_misalign_with_page     = RegInit(false.B)
  val ld_dc_expt_access_fault_with_page = RegInit(false.B)
  val ld_dc_expt_ldamo_not_ca           = RegInit(false.B)

  val ld_dc_borrow_db      = RegInit(0.U(LSUConfig.VB_DATA_ENTRY.W))
  val ld_dc_borrow_vb      = RegInit(false.B)
  val ld_dc_borrow_sndb    = RegInit(false.B)
  val ld_dc_borrow_mmu     = RegInit(false.B)
  val ld_dc_borrow_icc     = RegInit(false.B)
  val ld_dc_borrow_icc_tag = RegInit(false.B)
  val ld_dc_settle_way     = RegInit(false.B)

  val ld_dc_data = RegInit(0.U.asTypeOf(new LoadDCDataReg))

  val ld_dc_addr0 = RegInit(0.U(LSUConfig.PA_WIDTH.W))
  //Wire
  val ld_dc_depd_imme_restart_req = Wire(Bool())
  val ld_dc_borrow_mmu_vld = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val ld_dc_clk_en = io.in.fromAG.inst_vld || io.in.fromDcacheArb.ld_dc_borrow_vld_gate

  val ld_dc_inst_clk_en = io.in.fromAG.inst_vld

  val ld_dc_borrow_clk_en = io.in.fromDcacheArb.ld_dc_borrow_vld_gate


  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+------------+
  //| inst_vld | borrow_vld |
  //+----------+------------+
  when(io.in.fromAG.dc_inst_vld){
    ld_dc_inst_vld := true.B
    ld_dc_load_inst_vld_dup       := WireInit(VecInit(Seq.fill(5)(io.in.fromAG.dc_load_inst_vld)))
    ld_dc_load_ahead_inst_vld_dup := WireInit(VecInit(Seq.fill(5)(io.in.fromAG.dc_load_ahead_inst_vld)))
    ld_dc_vload_inst_vld_dup      := WireInit(VecInit(Seq.fill(4)(io.in.fromAG.dc_vload_inst_vld)))
    ld_dc_vload_ahead_inst_vld    := io.in.fromAG.dc_vload_ahead_inst_vld
  }.otherwise{
    ld_dc_inst_vld := false.B
    ld_dc_load_inst_vld_dup       := WireInit(VecInit(Seq.fill(5)(false.B)))
    ld_dc_load_ahead_inst_vld_dup := WireInit(VecInit(Seq.fill(5)(false.B)))
    ld_dc_vload_inst_vld_dup      := WireInit(VecInit(Seq.fill(4)(false.B)))
    ld_dc_vload_ahead_inst_vld    := false.B
  }

  when(io.in.fromDcacheArb.ld_dc_borrow_vld){
    ld_dc_borrow_vld := true.B
  }.otherwise{
    ld_dc_borrow_vld := false.B
  }
  io.out.toDA.inst_vld   := ld_dc_inst_vld
  io.out.toDA.borrow_vld := ld_dc_borrow_vld

  //------------------expt part-------------------------------
  //+-------+----------+--------+------+---------+
  //| tmiss | misalign | tfatal | deny | rd_tinv |
  //+-------+----------+--------+------+---------+
  when(io.in.fromAG.inst_vld){
    ld_dc_mmu_req                     := io.in.fromAG.dc_mmu_req
    ld_dc_expt_misalign_no_page       := io.in.fromAG.expt_misalign_no_page
    ld_dc_expt_page_fault             := io.in.fromAG.expt_page_fault
    ld_dc_expt_misalign_with_page     := io.in.fromAG.expt_misalign_with_page
    ld_dc_expt_access_fault_with_page := io.in.fromAG.expt_access_fault_with_page
    ld_dc_expt_ldamo_not_ca           := io.in.fromAG.expt_ldamo_not_ca
  }
  io.out.toDA.mmu_req := ld_dc_mmu_req

  //------------------borrow part-----------------------------
  //+-----+------+-----+------------+
  //| rcl | sndb | mmu | settle way |
  //+-----+------+-----+------------+
  when(io.in.fromDcacheArb.ld_dc_borrow_vld){
    ld_dc_borrow_db      := io.in.fromDcacheArb.ld_dc_borrow_db
    ld_dc_borrow_vb      := io.in.fromDcacheArb.ld_dc_borrow_vb
    ld_dc_borrow_sndb    := io.in.fromDcacheArb.ld_dc_borrow_sndb
    ld_dc_borrow_mmu     := io.in.fromDcacheArb.ld_dc_borrow_mmu
    ld_dc_borrow_icc     := io.in.fromDcacheArb.ld_dc_borrow_icc
    ld_dc_borrow_icc_tag := io.in.icc_dcache_arb_ld_tag_read
    ld_dc_settle_way     := io.in.fromDcacheArb.ld_dc_settle_way
  }
  io.out.toDA.borrow_db      := ld_dc_borrow_db
  io.out.toDA.borrow_vb      := ld_dc_borrow_vb
  io.out.toDA.borrow_sndb    := ld_dc_borrow_sndb
  io.out.toDA.borrow_mmu     := ld_dc_borrow_mmu
  io.out.toDA.borrow_icc     := ld_dc_borrow_icc
  io.out.toDA.borrow_icc_tag := ld_dc_borrow_icc_tag
  io.out.toDA.settle_way     := ld_dc_settle_way

  //------------------inst part----------------------------
  //+----------+---------+-----+
  //| expt_vld | pf_inst | vpn |
  //+----------+---------+-----+
  //+-----------+-----------+------+------------+----------------+
  //| inst_type | inst_size | secd | already_da | lsiq_spec_fail |
  //+-----------+-----------+------+------------+----------------+
  //+-------------+----+-----+------+-----+------------+------------+
  //| sign_extend | ex | iid | lsid | old | bytes_vld0 | bytes_vld1 |
  //+-------------+----+-----+------+-----+------------+------------+
  //+--------+----------+------+-------+
  //| deform | boundary | preg | split |
  //+--------+----------+------+-------+
  //+-----------+-------+-------+-----------+---------+
  //| ldfifo_pc | bkpta | bkptb | data_bias | pf_inst |
  //+-----------+-------+-------+-----------+---------+
  //+----+----+-----+-----+-------+
  //| so | ca | buf | sec | share |
  //+----+----+-----+-----+-------+
  when(io.in.fromAG.inst_vld){
    ld_dc_data.expt_vld_except_access_err := io.in.fromAG.expt_vld
    ld_dc_data.pf_inst         := io.in.fromAG.pf_inst
    ld_dc_data.vpn             := io.in.fromAG.vpn
    ld_dc_data.addr1_tto4      := io.in.fromAG.addr1_to4
    ld_dc_data.split           := io.in.fromAG.split
    ld_dc_data.inst_type       := io.in.fromAG.inst_type
    ld_dc_data.inst_size       := io.in.fromAG.dc_access_size
    ld_dc_data.secd            := io.in.fromAG.secd
    ld_dc_data.already_da      := io.in.fromAG.already_da
    ld_dc_data.lsiq_spec_fail  := io.in.fromAG.lsiq_spec_fail
    ld_dc_data.lsiq_bkpta_data := io.in.fromAG.lsiq_bkpta_data
    ld_dc_data.lsiq_bkptb_data := io.in.fromAG.lsiq_bkptb_data
    ld_dc_data.sign_extend     := io.in.fromAG.sign_extend
    ld_dc_data.atomic          := io.in.fromAG.atomic
    ld_dc_data.iid             := io.in.fromAG.iid
    ld_dc_data.lsid            := io.in.fromAG.lsid
    ld_dc_data.old             := io.in.fromAG.old
    ld_dc_data.bytes_vld       := io.in.fromAG.dc_bytes_vld
    ld_dc_data.bytes_vld1      := io.in.fromAG.dc_bytes_vld1
    ld_dc_data.rot_sel         := io.in.fromAG.dc_rot_sel
    ld_dc_data.boundary        := io.in.fromAG.boundary
    ld_dc_data.preg            := io.in.fromAG.preg
    ld_dc_data.preg_dup        := WireInit(VecInit(Seq.fill(5)(io.in.fromAG.preg)))
    ld_dc_data.ldfifo_pc       := io.in.fromAG.ldfifo_pc
    ld_dc_data.ahead_predict   := io.in.fromAG.ahead_predict
    ld_dc_data.page_so         := io.in.fromAG.page_so
    ld_dc_data.page_ca         := io.in.fromAG.page_ca
    ld_dc_data.page_buf        := io.in.fromAG.page_buf
    ld_dc_data.page_sec        := io.in.fromAG.page_sec
    ld_dc_data.page_share      := io.in.fromAG.page_share
    ld_dc_data.utlb_miss       := io.in.fromAG.utlb_miss
    ld_dc_data.tlb_busy        := io.in.fromMMU.tlb_busy
    ld_dc_data.vreg            := io.in.fromAG.vreg
    ld_dc_data.vreg_dup        := WireInit(VecInit(Seq.fill(4)(io.in.fromAG.vreg)))
    ld_dc_data.inst_vfls       := io.in.fromAG.inst_vfls
    ld_dc_data.acclr_en        := io.in.fromAG.dc_acclr_en
    ld_dc_data.fwd_bypass_en   := io.in.fromAG.dc_fwd_bypass_en
    ld_dc_data.no_spec         := io.in.fromAG.no_spec
    ld_dc_data.no_spec_exist   := io.in.fromAG.no_spec_exist
    ld_dc_data.raw_new         := io.in.fromAG.raw_new
  }
  io.out.toDA.expt_vld_except_access_err := ld_dc_data.expt_vld_except_access_err
  io.out.toDA.split := ld_dc_data.split
  io.out.toDA.inst_type := ld_dc_data.inst_type
  io.out.toDA.inst_size := ld_dc_data.inst_size
  io.out.toDA.secd := ld_dc_data.secd
  io.out.toDA.already_da := ld_dc_data.already_da
  io.out.toDA.sign_extend := ld_dc_data.sign_extend
  io.out.toDA.atomic := ld_dc_data.atomic
  io.out.toDA.iid := ld_dc_data.iid
  io.out.toDA.lsid := ld_dc_data.lsid
  io.out.toDA.boundary := ld_dc_data.boundary
  io.out.toDA.preg := ld_dc_data.preg
  io.out.toDA.ldfifo_pc := ld_dc_data.ldfifo_pc
  io.out.toDA.ahead_predict := ld_dc_data.ahead_predict
  io.out.toDA.vreg := ld_dc_data.vreg
  io.out.toDA.inst_vfls := ld_dc_data.inst_vfls
  io.out.toDA.no_spec := ld_dc_data.no_spec
  io.out.toDA.no_spec_exist := ld_dc_data.no_spec_exist

  io.out.ld_dc_bytes_vld  := ld_dc_data.bytes_vld
  io.out.ld_dc_bytes_vld1 := ld_dc_data.bytes_vld1

  //------------------inst/borrow share part------------------
  //+-------+
  //| addr0 |
  //+-------+
  when(io.in.fromAG.inst_vld || io.in.fromDcacheArb.ld_dc_borrow_vld){
    ld_dc_addr0 := io.in.fromAG.dc_addr0
  }
  io.out.toDA.addr0 := ld_dc_addr0

  //==========================================================
  //        Generate  va
  //==========================================================
  val ld_dc_va_offset = Mux(ld_dc_data.boundary && !ld_dc_data.secd && !ld_dc_expt_misalign_no_page, Cat(ld_dc_addr0(11,4), 0.U(4.W)), ld_dc_addr0(11,0))
  val ld_dc_va = Cat(ld_dc_data.vpn, ld_dc_va_offset)

  val ld_dc_addr1_11to4 = ld_dc_data.addr1_tto4
  io.out.ld_dc_addr1_11to4 := ld_dc_addr1_11to4
  // for preload addr check
  val ld_dc_pfu_va_11to4 = Mux(ld_dc_data.boundary && !ld_dc_data.secd, ld_dc_addr1_11to4, ld_dc_addr0(11,4))
  //if this inst cross 4k, this va is not accurate
  io.out.toDA.pfu_va := Cat(ld_dc_data.vpn, ld_dc_pfu_va_11to4, ld_dc_addr0(3,0))

  //==========================================================
  //        Exception generate
  //==========================================================
  io.out.toDA.expt_access_fault_mask  := ld_dc_expt_misalign_no_page || ld_dc_expt_page_fault
  io.out.toDA.expt_access_fault_extra := ld_dc_mmu_req && ld_dc_expt_ldamo_not_ca

  val ld_dc_expt_access_fault_short  = ld_dc_mmu_req

  //if utlb_miss and dmmu expt,
  //then st_dc_expt_vld_except_access_err is 0,
  //but st_dc_da_expt_vld is not certain
  io.out.toDA.da_expt_vld_gate_en := ld_dc_data.expt_vld_except_access_err || ld_dc_expt_access_fault_short

  val ld_dc_expt_vec = WireInit(0.U(5.W))
  val ld_dc_mt_value = WireInit(0.U(LSUConfig.PA_WIDTH.W))

  when(ld_dc_expt_misalign_no_page && !ld_dc_data.atomic){
    ld_dc_expt_vec := 4.U(5.W)
    ld_dc_mt_value := Cat(ld_dc_data.addr1_tto4, ld_dc_va(3,0))
  }.elsewhen(ld_dc_expt_misalign_no_page && ld_dc_data.atomic){
    ld_dc_expt_vec := 6.U(5.W)
    ld_dc_mt_value := ld_dc_va
  }.elsewhen(ld_dc_expt_page_fault && !ld_dc_data.atomic){
    ld_dc_expt_vec := 13.U(5.W)
    ld_dc_mt_value := ld_dc_va
  }.elsewhen(ld_dc_expt_page_fault && ld_dc_data.atomic){
    ld_dc_expt_vec := 15.U(5.W)
    ld_dc_mt_value := ld_dc_va
  }.elsewhen(ld_dc_expt_misalign_with_page){
    ld_dc_expt_vec := 4.U(5.W)
    ld_dc_mt_value := ld_dc_va
  }.elsewhen(ld_dc_expt_access_fault_with_page){
    ld_dc_expt_vec := 5.U(5.W)
    ld_dc_mt_value := 0.U(LSUConfig.PA_WIDTH.W)
  }
  io.out.toDA.expt_vec := ld_dc_expt_vec
  io.out.toDA.mt_value := ld_dc_mt_value

  //==========================================================
  //        Generate inst type
  //==========================================================
  val ld_dc_vector_nop = false.B
  io.out.toDA.vector_nop := ld_dc_vector_nop

  val ld_dc_ld_inst = !ld_dc_data.atomic && ld_dc_data.inst_type === 0.U(2.W)

  //==========================================================
  //                 Create load queue
  //==========================================================
  //lq_create_vld is not accurate, comparing iid is a must precedure to create lq
  //to reduce spec fail, create lq should be more strict

  //for timing, create lq do not see access deny
  io.out.toLQ.create_vld  := io.out.toLQ.create_dp_vld && !ld_dc_depd_imme_restart_req && !io.in.fromSQ.ld_dc_addr1_dep_discard

  io.out.toLQ.create1_vld := io.out.toLQ.create1_dp_vld && !ld_dc_depd_imme_restart_req && io.in.cb_ld_dc_addr_hit && !io.in.fromSQ.ld_dc_addr1_dep_discard

  io.out.toLQ.create_dp_vld  := ld_dc_inst_vld && ld_dc_ld_inst && !ld_dc_vector_nop && !ld_dc_data.old &&
    !ld_dc_data.page_so && !ld_dc_data.utlb_miss && io.in.fromLQ.ld_dc_inst_hit && !ld_dc_data.expt_vld_except_access_err

  io.out.toLQ.create1_dp_vld := io.out.toLQ.create_dp_vld && ld_dc_data.acclr_en

  io.out.toLQ.create_gateclk_en  := ld_dc_inst_vld && ld_dc_ld_inst && !ld_dc_data.old &&
    !ld_dc_data.page_so && !ld_dc_data.utlb_miss && !ld_dc_data.expt_vld_except_access_err

  io.out.toLQ.create1_gateclk_en := io.out.toLQ.create_gateclk_en && ld_dc_data.acclr_en

  val ld_dc_addr1 = Cat(ld_dc_addr0,ld_dc_addr1_11to4,0.U(4.W))
  io.out.ld_dc_addr1 := ld_dc_addr1

  //==========================================================
  //                 Generate check signal to lq/sq/wmb
  //==========================================================
  io.out.toChk.atomic_inst_vld := ld_dc_inst_vld && ld_dc_ld_inst &&
    !ld_dc_data.page_so && !ld_dc_data.utlb_miss && !ld_dc_data.expt_vld_except_access_err

  io.out.toChk.ld_addr1_vld    := io.out.toChk.atomic_inst_vld && ld_dc_data.fwd_bypass_en

  io.out.toChk.ld_bypass_vld   := ld_dc_inst_vld && ld_dc_ld_inst && ld_dc_data.acclr_en &&
    !ld_dc_data.page_so && !ld_dc_data.utlb_miss && !ld_dc_data.expt_vld_except_access_err

  io.out.toChk.ld_inst_vld     := ld_dc_inst_vld && !ld_dc_vector_nop && ld_dc_data.atomic &&
    !ld_dc_data.utlb_miss && !ld_dc_data.expt_vld_except_access_err

  //==========================================================
  //                 RAW speculation check
  //==========================================================
  // st_dc stage should check raw speculation for ld_dc stage

  // situat st pipe             ld pipe           addr      bytes_vld
  // ----------------------------------------------------------------
  // 2      st/stex             ld                31:4       x

  val ld_dc_inst_chk_vld = ld_dc_inst_vld && ld_dc_ld_inst &&
    !ld_dc_data.page_so && !ld_dc_data.utlb_miss && !ld_dc_data.expt_vld_except_access_err
  io.out.ld_dc_inst_chk_vld := ld_dc_inst_chk_vld

  //-----------addr compare---------------
  //addr0 compare
  val ld_dc_cmp_st_dc_addr0 = io.in.fromStDC.addr0
  val ld_dc_raw_addr_tto4_hit  = ld_dc_addr0 === ld_dc_cmp_st_dc_addr0
  val ld_dc_raw_addr1_tto4_hit = ld_dc_addr1 === ld_dc_cmp_st_dc_addr0

  //-----------bytes_vld compare----------
  val ld_dc_raw_do_hit = (ld_dc_data.bytes_vld & io.in.fromStDC.bytes_vld).orR

  //------------------situation 2-----------------------------
  val ld_dc_depd_st_dc2 = ld_dc_inst_chk_vld && ld_dc_data.raw_new &&
    (io.in.fromStDC.chk_st_inst_vld || io.in.fromStDC.chk_statomic_inst_vld) &&
    ld_dc_raw_addr_tto4_hit && ld_dc_raw_do_hit

  //------------------situation 3-----------------------------
  // when ld addr hit st, then do not get data from merge buffer
  val ld_dc_depd_st_dc3 = ld_dc_inst_chk_vld && ld_dc_data.raw_new &&
    io.in.fromStDC.inst_vld && ld_dc_raw_addr1_tto4_hit

  //------------------combine---------------------------------
  val ld_dc_depd_st_dc = ld_dc_depd_st_dc2

  //==========================================================
  //                 Dependency check
  //==========================================================
  // dependency check is done in sq/wmb entry file
  //------------------arbitrate-------------------------------
  //-----------forward arbitrate----------
  //bypass: pass data from ex1
  //fwd: pass data from sq/wmb
  //if ld_dc_fwd_sq_bypass_vld=1, and ld_dc_fwd_sq_vld=1,
  //then see as multi depd in da
  io.out.toDA.fwd_sq_vld  := io.in.fromSQ.ld_dc_fwd_req
  io.out.toDA.fwd_wmb_vld := !io.in.fromSQ.ld_dc_has_fwd_req && io.in.fromWmb.ld_dc_fwd_req

  when(io.in.fromSQ.ld_dc_has_fwd_req){
    io.out.toDA.fwd_bytes_vld := -1.S(16.W).asUInt
  }.elsewhen(io.in.fromWmb.ld_dc_fwd_req){
    io.out.toDA.fwd_bytes_vld := io.in.fromWmb.fwd_bytes_vld
  }.otherwise{
    io.out.toDA.fwd_bytes_vld := 0.U(16.W)
  }

  //==========================================================
  //                 Restart signal
  //==========================================================
  //-----------arbiter----------------------------------------
  //prioritize:
  //1. utlb_miss(include tlb_busy)
  //2. depd st_dc/sq imme restart
  //3. lq_full
  //
  //for timing, discard to da stage

  //for timing, use create_dp signal to replay
  val ld_dc_lq_full_req = io.out.toLQ.create_dp_vld && io.in.fromLQ.ld_dc_full ||
    io.out.toLQ.create1_dp_vld && io.in.fromLQ.ld_dc_less2

  ld_dc_depd_imme_restart_req := ld_dc_depd_st_dc

  val ld_dc_utlb_miss_vld = ld_dc_inst_vld && !ld_dc_data.expt_vld_except_access_err && ld_dc_data.utlb_miss

  val ld_dc_depd_imme_restart_vld = !ld_dc_utlb_miss_vld && ld_dc_depd_imme_restart_req

  val ld_dc_lq_full_vld = ld_dc_lq_full_req && !ld_dc_depd_imme_restart_req && !ld_dc_data.utlb_miss

  io.out.toLQ.full_gateclk_en := io.out.toLQ.create_gateclk_en && io.in.fromLQ.ld_dc_less2

  val ld_dc_restart_vld = ld_dc_lq_full_req || ld_dc_depd_imme_restart_req || ld_dc_utlb_miss_vld

  //---------------------restart kinds------------------------
  val ld_dc_imme_restart_vld = ld_dc_utlb_miss_vld && !ld_dc_data.tlb_busy || ld_dc_depd_imme_restart_vld

  val ld_dc_tlb_busy_restart_vld = ld_dc_utlb_miss_vld && ld_dc_data.tlb_busy

  io.out.ld_dc_tlb_busy_gateclk_en := ld_dc_tlb_busy_restart_vld

  //==========================================================
  //                Generage had signal
  //==========================================================
  val ld_dc_had_bkpta_addr = Cat(io.in.fromHad.yy_xx_bkpta_base(LSUConfig.PA_WIDTH-1,8), io.in.fromHad.yy_xx_bkpta_base(7,0) & io.in.fromHad.yy_xx_bkpta_mask)
  val ld_dc_had_bkptb_addr = Cat(io.in.fromHad.yy_xx_bkptb_base(LSUConfig.PA_WIDTH-1,8), io.in.fromHad.yy_xx_bkptb_base(7,0) & io.in.fromHad.yy_xx_bkptb_mask)

  val ld_dc_bkpta_addr_tto12  = ld_dc_va(LSUConfig.PA_WIDTH-1,12)
  val ld_dc_bkpta_addr0_3to0  = ld_dc_va(3,0) & io.in.fromHad.yy_xx_bkpta_mask(3,0)
  val ld_dc_bkpta_addr1_3to0  = ld_dc_addr0(3,0) & io.in.fromHad.yy_xx_bkpta_mask(3,0)
  val ld_dc_bkpta_addr0_11to4 = Cat(ld_dc_va(11,8), ld_dc_va(7,4) & io.in.fromHad.yy_xx_bkpta_mask(7,4))
  val ld_dc_bkpta_addr1_11to4 = Cat(ld_dc_addr1_11to4(7,4), ld_dc_addr1_11to4(3,0) & io.in.fromHad.yy_xx_bkpta_mask(7,4))

  val ld_dc_bkpta_addr0 = Cat(ld_dc_bkpta_addr_tto12, ld_dc_bkpta_addr0_11to4, ld_dc_bkpta_addr0_3to0)
  val ld_dc_bkpta_addr1 = Cat(ld_dc_bkpta_addr_tto12, ld_dc_bkpta_addr1_11to4, ld_dc_bkpta_addr1_3to0)

  val ld_dc_bkptb_addr_tto12  = ld_dc_va(LSUConfig.PA_WIDTH-1,12)
  val ld_dc_bkptb_addr0_3to0  = ld_dc_va(3,0) & io.in.fromHad.yy_xx_bkptb_mask(3,0)
  val ld_dc_bkptb_addr1_3to0  = ld_dc_addr0(3,0) & io.in.fromHad.yy_xx_bkptb_mask(3,0)
  val ld_dc_bkptb_addr0_11to4 = Cat(ld_dc_va(11,8), ld_dc_va(7,4) & io.in.fromHad.yy_xx_bkptb_mask(7,4))
  val ld_dc_bkptb_addr1_11to4 = Cat(ld_dc_addr1_11to4(7,4), ld_dc_addr1_11to4(3,0) & io.in.fromHad.yy_xx_bkptb_mask(7,4))

  val ld_dc_bkptb_addr0 = Cat(ld_dc_bkptb_addr_tto12, ld_dc_bkptb_addr0_11to4, ld_dc_bkptb_addr0_3to0)
  val ld_dc_bkptb_addr1 = Cat(ld_dc_bkptb_addr_tto12, ld_dc_bkptb_addr1_11to4, ld_dc_bkptb_addr1_3to0)


  val ld_dc_bkpta_trap = io.in.fromHad.yy_xx_bkpta_rc ^
    (ld_dc_had_bkpta_addr === ld_dc_bkpta_addr0 || ld_dc_had_bkpta_addr === ld_dc_bkpta_addr1 &&
      ld_dc_data.bytes_vld1.orR && ld_dc_data.acclr_en)

  val ld_dc_bkptb_trap = io.in.fromHad.yy_xx_bkptb_rc ^
    (ld_dc_had_bkptb_addr === ld_dc_bkptb_addr0 || ld_dc_had_bkptb_addr === ld_dc_bkptb_addr1 &&
      ld_dc_data.bytes_vld1.orR && ld_dc_data.acclr_en)

  val ld_dc_pipe_bkpta_data = (ld_dc_ld_inst || ld_dc_data.atomic) && !ld_dc_vector_nop && ld_dc_bkpta_trap
  val ld_dc_pipe_bkptb_data = (ld_dc_ld_inst || ld_dc_data.atomic) && !ld_dc_vector_nop && ld_dc_bkptb_trap

  //==========================================================
  //        Combine signal of spec_fail/bkpt
  //==========================================================
  io.out.toDA.spec_fail := io.in.fromLQ.ld_dc_spec_fail || ld_dc_data.lsiq_spec_fail

  io.out.toDA.bkpta_data := ld_dc_data.lsiq_bkpta_data || ld_dc_pipe_bkpta_data
  io.out.toDA.bkptb_data := ld_dc_data.lsiq_bkptb_data || ld_dc_pipe_bkptb_data

  //==========================================================
  //            Generage get dcache signal
  //==========================================================
  //this data_bias_sel is for wmb/dcache
  val mmu_bytes_vld = WireInit(0.U(16.W))
  when(!io.in.fromMMU.data_req_size){
    mmu_bytes_vld := MuxLookup(ld_dc_addr0(3,2), 0.U(16.W), Seq(
      "b00".U -> 0x000f.U,
      "b01".U -> 0x00f0.U,
      "b10".U -> 0x0f00.U,
      "b11".U -> 0xf000.U
    ))
  }.elsewhen(io.in.fromMMU.data_req_size){
    mmu_bytes_vld := MuxLookup(ld_dc_addr0(3,2), 0.U(16.W), Seq(
      "b00".U -> 0x00ff.U,
      "b10".U -> 0xff00.U
    ))
  }

  val ld_dc_da_bytes_vld = Mux(ld_dc_borrow_mmu_vld, mmu_bytes_vld, ld_dc_data.bytes_vld)
  io.out.toDA.da_bytes_vld := ld_dc_da_bytes_vld

  val ld_dc_da_bytes_vld1 = ld_dc_data.bytes_vld1
  io.out.toDA.da_bytes_vld1 := ld_dc_da_bytes_vld1

  val ld_dc_data_bias_sel = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    ld_dc_data_bias_sel(i) := ld_dc_da_bytes_vld(4*i+3, 4*i).orR
  }

  //if deform/mmu double word, then open 2 groups of banks
  val ld_dc_dup_dcache_data_en = ld_dc_inst_vld || ld_dc_borrow_mmu_vld

  val ld_dc_get_dcache_data_inst_mmu = Mux(ld_dc_dup_dcache_data_en, ld_dc_data_bias_sel.asUInt, 0.U(4.W))

  val ld_dc_get_dcache_data_all = ld_dc_borrow_vld && !ld_dc_borrow_mmu

  io.out.toDA.get_dcache_data := Mux(ld_dc_get_dcache_data_all || ld_dc_data.acclr_en && ld_dc_inst_vld, "b1111".U, ld_dc_get_dcache_data_inst_mmu)

  //==========================================================
  //            Generage to DA stage signal
  //==========================================================
  io.out.toDA.da_inst_vld := ld_dc_inst_vld && !ld_dc_restart_vld

  //------------------page info sel if mmu req----------------
  ld_dc_borrow_mmu_vld := ld_dc_borrow_vld && ld_dc_borrow_mmu

  io.out.toDA.da_page_so    := Mux(ld_dc_borrow_vld, false.B, ld_dc_data.page_so)
  io.out.toDA.da_page_ca    := Mux(ld_dc_borrow_vld,  true.B, ld_dc_data.page_ca)
  io.out.toDA.da_page_buf   := Mux(ld_dc_borrow_vld,  true.B, ld_dc_data.page_buf)
  io.out.toDA.da_page_sec   := Mux(ld_dc_borrow_vld, false.B, ld_dc_data.page_sec)
  io.out.toDA.da_page_share := Mux(ld_dc_borrow_vld,  true.B, ld_dc_data.page_share)

  //------------------regard mmu request as old---------------
  //because the old inst may cause tlb refill
  io.out.toDA.da_old := Mux(ld_dc_borrow_mmu_vld, true.B, ld_dc_data.old)

  //------------------dcache tag pre_compare----------------
  val ld_dc_dcache_tag_array = io.in.dcache_lsu_ld_tag_dout
  val ld_dc_way_tag_hit = Wire(Vec(2, Bool()))
  val ld_dc_dcache_valid = Wire(Vec(2, Bool()))
  val ld_dc_hit_way = Wire(Vec(2, Bool()))

  for(i <- 0 until 2){
    ld_dc_way_tag_hit(i) := ld_dc_addr0(LSUConfig.PA_WIDTH-1, 14) === ld_dc_dcache_tag_array(i)(25,0)
    ld_dc_dcache_valid(i) := ld_dc_dcache_tag_array(i)(26) && io.in.fromCp0.lsu_dcache_en && io.out.toDA.da_page_ca
    ld_dc_hit_way(i) := ld_dc_dcache_valid(i) && ld_dc_way_tag_hit(i)
  }

  val ld_dc_dcache_hit = ld_dc_hit_way.asUInt.orR
  io.out.toDA.dcache_hit := ld_dc_dcache_hit

  io.out.toDA.hit_low_region  := Mux(ld_dc_addr0(4), ld_dc_hit_way(1), ld_dc_hit_way(0))
  io.out.toDA.hit_high_region := Mux(ld_dc_addr0(4), ld_dc_hit_way(0), ld_dc_hit_way(1))

  //------------------icc read tag info----------------
  io.out.toDA.da_icc_tag_vld := ld_dc_borrow_vld && ld_dc_borrow_icc && ld_dc_borrow_icc_tag
  io.out.toDA.da_tag_read := Mux(ld_dc_settle_way, ld_dc_dcache_tag_array(1), ld_dc_dcache_tag_array(0))

  //------------------data pre_select----------------
  val mmu_rot_sel = Mux(io.in.fromMMU.data_req_size, Cat(ld_dc_addr0(3), 0.U(3.W)), Cat(ld_dc_addr0(3,2), 0.U(2.W)))

  val ld_dc_rot_sel_final = Mux(ld_dc_borrow_mmu_vld, mmu_rot_sel, ld_dc_data.rot_sel)

  io.out.toDA.da_data_rot_sel := MuxLookup(ld_dc_rot_sel_final(2,0), 0.U(8.W), Seq(
    0.U(3.W) -> "b00000001".U,
    1.U(3.W) -> "b00000010".U,
    2.U(3.W) -> "b00000100".U,
    3.U(3.W) -> "b00001000".U,
    4.U(3.W) -> "b00010000".U,
    5.U(3.W) -> "b00100000".U,
    6.U(3.W) -> "b01000000".U,
    7.U(3.W) -> "b10000000".U
  ))

  //----------sign_sel--------------------
  //3: word sign extend
  //2: half sign extend
  //1: byte sign extend
  //0: not extend
  when(ld_dc_data.sign_extend){
    io.out.toDA.preg_sign_sel := MuxLookup(ld_dc_data.inst_size, 1.U(4.W), Seq(
      "b00".U -> "b0010".U,
      "b01".U -> "b0100".U,
      "b10".U -> "b1000".U
    ))
  }.otherwise{
    io.out.toDA.preg_sign_sel := 1.U(4.W)
  }

  io.out.toDA.vreg_sign_sel := ld_dc_data.inst_size =/= "b11".U
  val ld_dc_inst_vls_dup = WireInit(VecInit(Seq.fill(4)(false.B)))

  //==========================================================
  //            Generage pfu signal
  //==========================================================
  io.out.toDA.da_pf_inst := ld_dc_data.pf_inst && !ld_dc_vector_nop && ld_dc_data.page_ca

  val ld_dc_pf_inst_short = ld_dc_data.pf_inst && !ld_dc_vector_nop && ld_dc_data.page_ca &&
    !ld_dc_data.utlb_miss && !ld_dc_data.expt_vld_except_access_err

  io.out.toDA.pfu_info_set_vld := ld_dc_inst_vld && ld_dc_pf_inst_short &&
    (!io.in.fromPFU.sdb_empty || !io.in.fromPFU.pfb_empty || io.in.fromPFU.sdb_create_gateclk_en)

  //==========================================================
  //            Generage to cache buffer signal
  //==========================================================
  //------------------addr prepare----------------
  io.out.toCB.addr_tto4 := ld_dc_addr0(LSUConfig.PA_WIDTH-1,4)

  val cb_create_hit_idx = ld_dc_addr0(13,6) === io.in.fromDcacheArb.idx

  io.out.toCB.addr_create_vld := ld_dc_inst_vld && ld_dc_ld_inst && ld_dc_data.acclr_en &&
    !ld_dc_vector_nop && !ld_dc_data.expt_vld_except_access_err && !ld_dc_restart_vld &&
    !(io.in.lsu_dcache_ld_xx_gwen && cb_create_hit_idx) && !io.in.fromRTU.yy_xx_flush

  io.out.toCB.addr_create_gateclk_en := ld_dc_inst_vld && ld_dc_ld_inst && ld_dc_data.acclr_en &&
    !ld_dc_data.expt_vld_except_access_err && !io.in.fromRTU.yy_xx_flush

  io.out.toDA.da_cb_addr_create := io.out.toCB.addr_create_vld

  io.out.toDA.da_cb_merge_en := ld_dc_data.acclr_en && io.in.cb_ld_dc_addr_hit && !ld_dc_vector_nop &&
    !ld_dc_depd_st_dc3 && !io.in.fromSQ.ld_dc_cancel_acc_req && !io.in.fromWmb.ld_dc_cancel_acc_req &&
    !io.in.fromLQ.ld_dc_inst_hit && !ld_dc_data.expt_vld_except_access_err

  //==========================================================
  //              Generate forward write back
  //==========================================================
  io.out.toDA.wait_fence := io.in.lsu_has_fence && io.in.rb_fence_ld

  val ld_dc_ahead_wb_vld = ld_dc_inst_vld && !io.in.fromCp0.lsu_da_fwd_dis && ld_dc_data.page_ca &&
    ld_dc_ld_inst && !ld_dc_data.expt_vld_except_access_err && io.out.toDA.wait_fence &&
    !ld_dc_data.boundary && ld_dc_dcache_hit

  io.out.toDA.ahead_preg_wb_vld := ld_dc_ahead_wb_vld && !io.in.fromSQ.ld_dc_cancel_ahead_wb &&
    !io.in.fromWmb.ld_dc_discard_req && !ld_dc_data.inst_vfls

  io.out.toDA.ahead_vreg_wb_vld := false.B

  //==========================================================
  //      Generage lsiq signal (renamed in lsu_restart.vp)
  //==========================================================
  val ld_dc_mask_lsid = Mux(ld_dc_inst_vld, ld_dc_data.lsid, VecInit(Seq.fill(LSUConfig.LSIQ_ENTRY)(false.B)))

  io.out.ld_dc_idu_lq_full  := Mux(ld_dc_lq_full_vld,          ld_dc_mask_lsid, VecInit(Seq.fill(LSUConfig.LSIQ_ENTRY)(false.B)))
  io.out.ld_dc_imme_wakeup  := Mux(ld_dc_imme_restart_vld,     ld_dc_mask_lsid, VecInit(Seq.fill(LSUConfig.LSIQ_ENTRY)(false.B)))
  io.out.ld_dc_idu_tlb_busy := Mux(ld_dc_tlb_busy_restart_vld, ld_dc_mask_lsid, VecInit(Seq.fill(LSUConfig.LSIQ_ENTRY)(false.B)))

  //==========================================================
  //                Generage signal to idu
  //==========================================================
  io.out.toIDU.pipe3_load_inst_vld_dup := ld_dc_load_inst_vld_dup
  io.out.toIDU.pipe3_load_fwd_inst_vld_dup := ld_dc_load_ahead_inst_vld_dup
  io.out.toIDU.pipe3_preg_dup := ld_dc_data.preg_dup
  io.out.toIDU.pipe3_vload_inst_vld_dup := ld_dc_vload_inst_vld_dup
  io.out.toIDU.pipe3_vload_fwd_inst_vld := ld_dc_vload_ahead_inst_vld
  for(i <- 0 until 4){
    io.out.toIDU.pipe3_vreg_dup(i) := Cat(0.U(1.W), ld_dc_data.vreg_dup(i))
  }

  //==========================================================
  //        for mmu power
  //==========================================================
  io.out.lsu_mmu_vabuf0 := ld_dc_data.vpn(27,0)
}
