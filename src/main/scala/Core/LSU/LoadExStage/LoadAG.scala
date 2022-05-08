package Core.LSU.LoadExStage

import Core.LsuConfig
import Utils.Bits.sext
import chisel3._
import chisel3.util._

class Pipe3In extends Bundle with LsuConfig{
  val already_da = Bool()
  val atomic = Bool()
  val bkpta_data = Bool()
  val bkptb_data = Bool()
  val gateclk_sel = Bool()
  val iid = UInt(7.W)
  val inst_fls = Bool()
  val inst_ldr = Bool()
  val inst_size = UInt(2.W)
  val inst_type = UInt(2.W)
  val lch_entry = Vec(LSIQ_ENTRY, Bool())
  val lsfifo = Bool()
  val no_spec = Bool()
  val no_spec_exist = Bool()
  val off_0_extend = Bool()
  val offset = UInt(12.W)
  val offset_plus = UInt(13.W)
  val oldest = Bool()
  val pc = UInt(15.W)
  val preg = UInt(7.W)
  val sel = Bool()
  val shift = UInt(4.W)
  val sign_extend = Bool()
  val spec_fail = Bool()
  val split = Bool()
  val src0 = UInt(64.W)
  val src1 = UInt(64.W)
  val unalign_2nd = Bool()
  val vreg = UInt(7.W)
}

class LoadAG2DC extends Bundle with LsuConfig{
  val addr1_to4 = UInt(36.W)
  val ahead_predict = Bool()
  val already_da = Bool()
  val atomic = Bool()
  val boundary = Bool()
  val dc_access_size = UInt(3.W)
  val dc_acclr_en = Bool()
  val dc_addr0 = UInt(40.W)
  val dc_bytes_vld = UInt(16.W)
  val dc_bytes_vld1 = UInt(16.W)
  val dc_fwd_bypass_en = Bool()
  val dc_inst_vld = Bool()
  val dc_load_ahead_inst_vld = Bool()
  val dc_load_inst_vld = Bool()
  val dc_mmu_req = Bool()
  val dc_rot_sel = UInt(4.W)
  val dc_vload_ahead_inst_vld = Bool()
  val dc_vload_inst_vld = Bool()
  val expt_access_fault_with_page = Bool()
  val expt_ldamo_not_ca = Bool()
  val expt_misalign_no_page = Bool()
  val expt_misalign_with_page = Bool()
  val expt_page_fault = Bool()
  val expt_vld = Bool()
  val iid = UInt(7.W)
  val inst_type = UInt(2.W)
  val inst_vfls = Bool()
  val inst_vld = Bool()
  val ldfifo_pc = UInt(15.W)
  val lm_init_vld = Bool()
  val lr_inst = Bool()
  val lsid = Vec(LSIQ_ENTRY, Bool())
  val lsiq_bkpta_data = Bool()
  val lsiq_bkptb_data = Bool()
  val lsiq_spec_fail = Bool()
  val no_spec = Bool()
  val no_spec_exist = Bool()
  val old = Bool()
  val pa = UInt(40.W)
  val page_buf = Bool()
  val page_ca = Bool()
  val page_sec = Bool()
  val page_share = Bool()
  val page_so = Bool()
  val pf_inst = Bool()
  val preg = UInt(7.W)
  val raw_new = Bool()
  val secd = Bool()
  val sign_extend = Bool()
  val split = Bool()
  val stall_ori = Bool()
  val stall_restart_entry = Vec(LSIQ_ENTRY, Bool())
  val utlb_miss = Bool()
  val vpn = UInt(28.W)
  val vreg = UInt(6.W)
}

class LoadAGDataReg extends Bundle with LsuConfig{
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
  val lsid            = Vec(LSIQ_ENTRY, Bool())
  val old             = Bool()
  val preg            = UInt(7.W)
  val preg_dup        = Vec(5, UInt(7.W))
  val ldfifo_pc       = UInt(LSU_PC_WIDTH.W)
  val vreg            = UInt(7.W)
  val vreg_dup        = Vec(4, UInt(6.W))
  val inst_ldr        = Bool()
  val inst_fls        = Bool()
  val lsfifo          = Bool()
  val no_spec         = Bool()
  val no_spec_exist   = Bool()
}

class LoadAGInput extends Bundle with LsuConfig{
  val fromCp0 = new Bundle{
    val lsu_cb_aclr_dis = Bool()
    val lsu_da_fwd_dis = Bool()
    val lsu_dcache_en = Bool()
    val lsu_icg_en = Bool()
    val lsu_mm = Bool()
    val yy_clk_en = Bool()
  }
  val ctrl_ld_clk = Bool()
  val fromDcacheArb = new Bundle{
    val ag_ld_sel = Bool()
    val ld_ag_addr = UInt(40.W)
    val ld_ag_borrow_addr_vld = Bool()
  }
  val pipe3 = new Pipe3In
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
  val st_ag_iid = UInt(7.W)
}

class LoadAGOutput extends Bundle with LsuConfig{
  val toDcacheArb = new Bundle{
    val data_gateclk_en = UInt(8.W)
    val data_high_idx = UInt(11.W)
    val data_low_idx = UInt(11.W)
    val data_req = UInt(8.W)
    val tag_gateclk_en = Bool()
    val tag_idx = UInt(9.W)
    val tag_req = Bool()
  }
  val toDC = new LoadAG2DC
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

class LoadAGIO extends Bundle{
  val in  = Input(new LoadAGInput)
  val out = Output(new LoadAGOutput)
}

class LoadAG extends Module with LsuConfig{
  val io = IO(new LoadAGIO)

  //Reg
  val ld_ag_inst_vld = RegInit(false.B)
  val ld_ag_data = RegInit(0.U.asTypeOf(new LoadAGDataReg))

  val ld_ag_already_cross_page_ldr_imme = RegInit(false.B)
  val ld_ag_offset_shift = RegInit(1.U(4.W))
  val ld_ag_offset = RegInit(VecInit(Seq.fill(2)(0.U(32.W))))
  val ld_ag_offset_plus = RegInit(0.U(13.W))
  val ld_ag_base = RegInit(0.U(64.W))
  //Wire
  val ld_ag_inst_stall_gateclk_en = Wire(Bool())
  val ld_ag_stall_vld = Wire(Bool())
  val ld_ag_cross_page_ldr_imme_stall_req = Wire(Bool())
  val ld_ag_cross_page_ldr_imme_stall_arb = Wire(Bool())
  val ld_ag_secd_imme_stall = Wire(Bool())
  val ld_ag_va = Wire(UInt(64.W))
  val ld_ag_ld_inst = Wire(Bool())
  val ld_ag_boundary_unmask = Wire(Bool())
  val ld_ag_atomic_no_cmit_restart_req = Wire(Bool())
  val ld_ag_dcache_stall_req = Wire(Bool())
  val ld_ag_expt_misalign_no_page = Wire(Bool())
  val ld_ag_4k_sum_ori = Wire(UInt(13.W))
  val ld_ag_stall_mask = Wire(Bool())
  //==========================================================
  //                        RF signal
  //==========================================================
  val ld_rf_inst_vld     = io.in.pipe3.gateclk_sel
  val ld_rf_inst_ldr     = io.in.pipe3.inst_ldr
  val ld_rf_off_0_extend = io.in.pipe3.off_0_extend

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val ld_ag_clk_en = io.in.pipe3.gateclk_sel || ld_ag_inst_stall_gateclk_en

  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+
  //| inst_vld |
  //+----------+
  //if there is a stall in the AG stage ,the inst keep valid,
  //elseif there is inst and no flush in RF stage,
  //the inst goes to the AG stage next cycle
  when(io.in.fromRTU.yy_xx_flush){
    ld_ag_inst_vld := false.B
  }.elsewhen(ld_ag_stall_vld || io.in.pipe3.sel){
    ld_ag_inst_vld := true.B
  }.otherwise{
    ld_ag_inst_vld := false.B
  }
  io.out.toDC.inst_vld := ld_ag_inst_vld

  //------------------data part-------------------------------
  //+-----------+-----------+------+------------+----------------+
  //| inst_type | inst_size | secd | already_da | lsiq_spec_fail |
  //+-----------+-----------+------+------------+----------------+
  //+-------------+----+-----+------+-----+------+
  //| sign_extend | ex | iid | lsid | old | preg |
  //+-------------+----+-----+------+-----+------+
  //+-----------+----------------+-------+
  //| ldfifo_pc | unalign_permit | split |
  //+-----------+----------------+-------+
  //+-------+-------+
  //| bkpta | bkptb |
  //+-------+-------+
  //if there is a stall in the AG stage ,the inst info keep unchanged,
  //elseif there is inst in RF stage, the inst goes to the AG stage next cycle
  when(!ld_ag_stall_vld && ld_rf_inst_vld){
    ld_ag_data.split           := io.in.pipe3.split
    ld_ag_data.inst_type       := io.in.pipe3.inst_type
    ld_ag_data.inst_size       := io.in.pipe3.inst_size
    ld_ag_data.secd            := io.in.pipe3.unalign_2nd
    ld_ag_data.already_da      := io.in.pipe3.already_da
    ld_ag_data.lsiq_spec_fail  := io.in.pipe3.spec_fail
    ld_ag_data.lsiq_bkpta_data := io.in.pipe3.bkpta_data
    ld_ag_data.lsiq_bkptb_data := io.in.pipe3.bkptb_data
    ld_ag_data.sign_extend     := io.in.pipe3.sign_extend
    ld_ag_data.atomic          := io.in.pipe3.atomic
    ld_ag_data.iid             := io.in.pipe3.iid
    ld_ag_data.lsid            := io.in.pipe3.lch_entry
    ld_ag_data.old             := io.in.pipe3.oldest
    ld_ag_data.preg            := io.in.pipe3.preg
    ld_ag_data.preg_dup        := WireInit(VecInit(Seq.fill(5)(io.in.pipe3.preg)))
    ld_ag_data.ldfifo_pc       := io.in.pipe3.pc
    ld_ag_data.vreg            := io.in.pipe3.vreg(5,0)
    ld_ag_data.vreg_dup        := WireInit(VecInit(Seq.fill(4)(io.in.pipe3.vreg(5,0))))
    ld_ag_data.inst_ldr        := io.in.pipe3.inst_ldr
    ld_ag_data.inst_fls        := io.in.pipe3.inst_fls
    ld_ag_data.lsfifo          := io.in.pipe3.lsfifo
    ld_ag_data.no_spec         := io.in.pipe3.no_spec
    ld_ag_data.no_spec_exist   := io.in.pipe3.no_spec_exist
  }
  io.out.toDC.split := ld_ag_data.split
  io.out.toDC.inst_type := ld_ag_data.inst_type
  io.out.toDC.secd := ld_ag_data.secd
  io.out.toDC.already_da := ld_ag_data.already_da
  io.out.toDC.lsiq_spec_fail := ld_ag_data.lsiq_spec_fail
  io.out.toDC.lsiq_bkpta_data := ld_ag_data.lsiq_bkpta_data
  io.out.toDC.lsiq_bkptb_data := ld_ag_data.lsiq_bkptb_data
  io.out.toDC.sign_extend := ld_ag_data.sign_extend
  io.out.toDC.atomic := ld_ag_data.atomic
  io.out.toDC.iid := ld_ag_data.iid
  io.out.toDC.lsid := ld_ag_data.lsid
  io.out.toDC.old := ld_ag_data.old
  io.out.toDC.preg := ld_ag_data.preg
  io.out.toDC.ldfifo_pc := ld_ag_data.ldfifo_pc
  io.out.toDC.vreg := ld_ag_data.vreg
  io.out.toDC.no_spec := ld_ag_data.no_spec
  io.out.toDC.no_spec_exist := ld_ag_data.no_spec_exist


  //+------------------+
  //| already_cross_4k |
  //+------------------+
  //already cross 4k means addr1 is wrong, and mustn't merge from cache buffer
  //TODO: figure out
  when(!ld_ag_stall_vld){
    ld_ag_already_cross_page_ldr_imme := false.B
  }.elsewhen(ld_ag_stall_vld && ld_ag_cross_page_ldr_imme_stall_req){
    ld_ag_already_cross_page_ldr_imme := true.B
  }

  //+--------------+
  //| offset_shift |
  //+--------------+
  //if there is a stall in the AG stage ,offset_shift is reset to 0
  //cache stall will not change shift
  when(!ld_ag_stall_vld && io.in.pipe3.sel){
    ld_ag_offset_shift := io.in.pipe3.shift
  }.elsewhen(ld_ag_stall_vld && ld_ag_cross_page_ldr_imme_stall_req){
    ld_ag_offset_shift := 1.U(4.W)
  }

  //+--------+
  //| offset |
  //+--------+
  //if the 1st time boundary 2nd instruction stall, the offset set 16 for bias, else
  //if stall, it set to 0, cache stall will not change offset
  when(ld_ag_cross_page_ldr_imme_stall_arb){
    ld_ag_offset(1) := 0.U(32.W)
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld  &&  !ld_rf_inst_ldr){
    ld_ag_offset(1) := Mux(io.in.pipe3.offset(11), -1.S(32.W).asUInt, 0.U(32.W))
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld  &&  ld_rf_inst_ldr  &&  ld_rf_off_0_extend){
    ld_ag_offset(1) := 0.U(32.W)
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld){
    ld_ag_offset(1) := io.in.pipe3.src1(63,32)
  }

  when(ld_ag_cross_page_ldr_imme_stall_arb  &&  ld_ag_secd_imme_stall){
    ld_ag_offset(0) := 16.U(32.W)
  }.elsewhen(ld_ag_cross_page_ldr_imme_stall_arb){
    ld_ag_offset(0) := 0.U(32.W)
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld  &&  ld_rf_inst_ldr){
    ld_ag_offset(0) := Cat(Mux(io.in.pipe3.offset(11), -1.S(20.W).asUInt, 0.U(20.W)), io.in.pipe3.offset)
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld){
    ld_ag_offset(0) := io.in.pipe3.src1(31,0)
  }

  //+-------------+
  //| offset_plus |
  //+-------------+
  //use this imm as offset when the ld/st inst need split and !secd
  when(ld_ag_cross_page_ldr_imme_stall_arb){
    ld_ag_offset_plus := 0.U(13.W)
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld){
    ld_ag_offset_plus := io.in.pipe3.offset_plus
  }

  //+------+
  //| base |
  //+------+
  //the base addr, if stall, the base is set the result from the adder
  when(ld_ag_cross_page_ldr_imme_stall_arb){
    ld_ag_base := ld_ag_va
  }.elsewhen(!ld_ag_stall_vld &&  ld_rf_inst_vld){
    ld_ag_base := io.in.pipe3.src0
  }

  //==========================================================
  //                      AG gateclk
  //==========================================================
  ld_ag_inst_stall_gateclk_en := ld_ag_inst_vld

  //==========================================================
  //               Generate virtual address
  //==========================================================
  // for first boundary inst, use addr+offset+128 as va instead of addr+offset
  //for secd boundary,use addr+offset as va
  val ld_ag_offset_aftershift = MuxLookup(ld_ag_offset_shift, 0.U(64.W), Seq(
    "b0001".U -> ld_ag_offset.asUInt,
    "b0010".U -> Cat(ld_ag_offset.asUInt(62,0), 0.U(1.W)),
    "b0100".U -> Cat(ld_ag_offset.asUInt(61,0), 0.U(2.W)),
    "b1000".U -> Cat(ld_ag_offset.asUInt(60,0), 0.U(3.W))
  ))

  val ld_ag_va_ori  = ld_ag_base + ld_ag_offset_aftershift
  val ld_ag_va_plus = ld_ag_base + sext(64, ld_ag_offset_plus)

  //if misalign without page, then select ori va
  val ld_ag_va_plus_sel = ld_ag_boundary_unmask && ld_ag_ld_inst &&  !ld_ag_data.secd && !ld_ag_data.inst_ldr

  ld_ag_va := Mux(ld_ag_va_plus_sel, ld_ag_va_plus, ld_ag_va_ori)

  io.out.toDC.vpn := ld_ag_va(PA_WIDTH-1, 12)

  //==========================================================
  //                Generate inst type
  //==========================================================
  //ld/ldr/lrw/pop/lrs is treated as ld inst
  ld_ag_ld_inst := !ld_ag_data.atomic && ld_ag_data.inst_type === "b00".U

  io.out.toDC.lr_inst := ld_ag_data.atomic && ld_ag_data.inst_type === "b01".U

  val ld_ag_ldamo_inst = ld_ag_data.atomic && ld_ag_data.inst_type === "b00".U

  //-------------need to prefetch inst------------------------
  io.out.toDC.pf_inst := ld_ag_ld_inst && !ld_ag_data.split && ld_ag_data.lsfifo && !ld_ag_data.secd

  val ld_ag_inst_vls      = false.B
  val ld_ag_inst_fof      = false.B
  val ld_ag_inst_vfls     = ld_ag_data.inst_fls
  io.out.toDC.inst_vfls := ld_ag_inst_vfls
  val ld_ag_vmb_merge_vld = false.B

  //==========================================================
  //            Generate unalign, bytes_vld
  //==========================================================
  //---------------inst access size---------------
  // access size is used to select bytes_vld and boundary judge
  val ld_ag_access_size_ori = MuxLookup(ld_ag_data.inst_size, 0.U(4.W), Seq(
    "b00".U -> "b0000".U,
    "b01".U -> "b0001".U,
    "b10".U -> "b0011".U,
    "b11".U -> "b0111".U
  ))
  val ld_ag_access_size = ld_ag_access_size_ori

  io.out.toDC.dc_access_size := MuxLookup(ld_ag_access_size, 0.U(3.W), Seq(
    "b0000".U -> "b000".U,//byte
    "b0001".U -> "b001".U,//half
    "b0011".U -> "b010".U,//word
    "b0111".U -> "b011".U,//dword
    "b1111".U -> "b100".U //qword
  ))

  //----------------generate unalign--------------------------
  //-----------unalign--------------------
  val ld_ag_align =
    ld_ag_data.inst_size === BYTE.U  ||
    ld_ag_data.inst_size === HALF.U  && ld_ag_va_ori(0) === 0.U   ||
    ld_ag_data.inst_size === WORD.U  && ld_ag_va_ori(1,0) === 0.U ||
    ld_ag_data.inst_size === DWORD.U && ld_ag_va_ori(2,0) === 0.U

  val ld_ag_unalign = !ld_ag_align

  val ld_ag_unalign_so = !ld_ag_align

  //---------------boundary---------------
  val ld_ag_va_add_access_size = Cat(0.U(1.W), ld_ag_va_ori(3,0)) + Cat(0.U(1.W), ld_ag_access_size_ori)
  ld_ag_boundary_unmask  := ld_ag_va_add_access_size(4)

  val ld_ag_boundary = (ld_ag_boundary_unmask || ld_ag_data.secd) && ld_ag_ld_inst
  io.out.toDC.boundary := ld_ag_boundary
  //----------------generate bytes_vld------------------------
  //-----------in le/bev2-----------------
  //the 2nd half boundary inst will +128, so va[3:0] of 2nd inst will not change
  val ld_ag_le_bytes_vld_high_bits_full = MuxLookup(ld_ag_va_ori, 0.U(16.W), Seq(
    "b0000".U -> "hffff".U,
    "b0001".U -> "hfffe".U,
    "b0010".U -> "hfffc".U,
    "b0011".U -> "hfff8".U,
    "b0100".U -> "hfff0".U,
    "b0101".U -> "hffe0".U,
    "b0110".U -> "hffc0".U,
    "b0111".U -> "hff80".U,
    "b1000".U -> "hff00".U,
    "b1001".U -> "hfe00".U,
    "b1010".U -> "hfc00".U,
    "b1011".U -> "hf800".U,
    "b1100".U -> "hf000".U,
    "b1101".U -> "he000".U,
    "b1110".U -> "hc000".U,
    "b1111".U -> "h8000".U
  ))

  val ld_ag_le_bytes_vld_low_bits_full = MuxLookup(ld_ag_va_add_access_size(3,0), 0.U(16.W), Seq(
    "b0000".U -> "h0001".U,
    "b0001".U -> "h0003".U,
    "b0010".U -> "h0007".U,
    "b0011".U -> "h000f".U,
    "b0100".U -> "h001f".U,
    "b0101".U -> "h003f".U,
    "b0110".U -> "h007f".U,
    "b0111".U -> "h00ff".U,
    "b1000".U -> "h01ff".U,
    "b1001".U -> "h03ff".U,
    "b1010".U -> "h07ff".U,
    "b1011".U -> "h0fff".U,
    "b1100".U -> "h1fff".U,
    "b1101".U -> "h3fff".U,
    "b1110".U -> "h7fff".U,
    "b1111".U -> "hffff".U
  ))

  val ld_ag_le_bytes_vld_cross          = ld_ag_le_bytes_vld_high_bits_full & ld_ag_le_bytes_vld_low_bits_full
  val ld_ag_le_bytes_vld_low_cross_bits = Mux(ld_ag_boundary_unmask, ld_ag_le_bytes_vld_low_bits_full, ld_ag_le_bytes_vld_cross)
  val ld_ag_le_bytes_vld_high_bits      = ld_ag_le_bytes_vld_high_bits_full

  //-----------select bytes_vld-----------
  val ld_ag_bytes_vld_low_cross_bits = ld_ag_le_bytes_vld_low_cross_bits
  val ld_ag_bytes_vld_high_bits      = ld_ag_le_bytes_vld_high_bits

  //used for
  //1.lq create
  //2.da data_merge when acclr_en
  //bytes_vld1 is the bytes_vld of lower addr when there is a first(bigger) boundary ld inst
  val ld_ag_bytes_vld1 = ld_ag_bytes_vld_high_bits
  val ld_ag_bytes_vld  = Mux(ld_ag_data.secd, ld_ag_bytes_vld_high_bits, ld_ag_bytes_vld_low_cross_bits)

  //==========================================================
  //        vector mask
  //==========================================================
  io.out.toDC.dc_bytes_vld  := ld_ag_bytes_vld
  io.out.toDC.dc_bytes_vld1 := ld_ag_bytes_vld1
  io.out.toDC.dc_rot_sel    := ld_ag_va_ori

  //==========================================================
  //        MMU interface
  //==========================================================
  //-----------mmu input--------------------------------------
  io.out.toMMU.va0_vld  := ld_ag_inst_vld
  io.out.toMMU.va0      := ld_ag_base
  io.out.toMMU.abort0   := ld_ag_cross_page_ldr_imme_stall_req ||
                           ld_ag_atomic_no_cmit_restart_req ||
                           ld_ag_dcache_stall_req ||
                           ld_ag_expt_misalign_no_page ||
                           io.in.fromRTU.yy_xx_flush
  io.out.toMMU.id0      := ld_ag_data.iid
  io.out.toMMU.st_inst0 := ld_ag_ldamo_inst

  //-----------mmu output-------------------------------------
  val ld_ag_pn            = io.in.fromMMU.pa0
  val ld_ag_page_so       = io.in.fromMMU.so0 && io.in.fromMMU.pa0_vld
  val ld_ag_page_ca       = io.in.fromMMU.ca0 && io.in.fromMMU.pa0_vld
  io.out.toDC.page_so    := ld_ag_page_so
  io.out.toDC.page_ca    := ld_ag_page_ca
  io.out.toDC.page_buf   := io.in.fromMMU.buf0
  io.out.toDC.page_sec   := io.in.fromMMU.sec0
  io.out.toDC.page_share := io.in.fromMMU.sh0
  val ld_ag_utlb_miss     = !io.in.fromMMU.pa0_vld
  io.out.toDC.utlb_miss  := ld_ag_utlb_miss
  val ld_ag_page_fault    = io.in.fromMMU.page_fault0

  //==========================================================
  //        Generate physical address
  //==========================================================
  val ld_ag_pa = Cat(ld_ag_pn, ld_ag_va(11,0))
  io.out.toDC.pa := ld_ag_pa

  //grs inst use va, rather than pa
  val ld_ag_addr0 = ld_ag_pa

  // used for boundary inst acceleration
  io.out.toDC.addr1_to4 := ld_ag_va_ori(PA_WIDTH-1,4)

  val ld_ag_acclr_en = ld_ag_boundary &&
                       !ld_ag_4k_sum_ori(12) &&
                       !io.in.fromCp0.lsu_cb_aclr_dis &&
                       !ld_ag_data.secd &&
                       !ld_ag_inst_fof &&
                       io.in.fromCp0.lsu_dcache_en &&
                       !ld_ag_already_cross_page_ldr_imme

  io.out.toDC.dc_acclr_en := ld_ag_acclr_en && io.in.fromMMU.pa0_vld && ld_ag_page_ca

  //fwd bypass is bypass data from pipe5 EX1 stage when ld is at AG stage
  // used for ld fwd bypass what means
  //only support byte,half,word,double word
  val ld_ag_bypass_en = ld_ag_access_size === "b0000".U ||
                        ld_ag_access_size === "b0001".U ||
                        ld_ag_access_size === "b0011".U ||
                        ld_ag_access_size === "b0111".U ||
                        ld_ag_access_size === "b1111".U

  io.out.toDC.dc_fwd_bypass_en := ld_ag_bypass_en && !ld_ag_inst_vls && !ld_ag_boundary

  //==========================================================
  //        Generage commit signal
  //==========================================================
  val ld_ag_cmit_hit = Wire(Vec(3, Bool()))
  for(i <- 0 until 3){
    ld_ag_cmit_hit(i) := io.in.fromRTU.yy_xx_commit(i) && io.in.fromRTU.yy_xx_commit_iid(i) === ld_ag_data.iid
  }
  val ld_ag_cmit = ld_ag_cmit_hit.asUInt.orR

  //==========================================================
  //        Generage dcache request information
  //==========================================================
  val ag_dcache_arb_ld_gateclk_en = ld_ag_inst_vld && io.in.fromCp0.lsu_dcache_en

  //for timing, delete mmu signal
  val ag_dcache_arb_ld_req = ld_ag_inst_vld && io.in.fromCp0.lsu_dcache_en

  //-----------tag array-------------------------------------
  io.out.toDcacheArb.tag_gateclk_en := ag_dcache_arb_ld_gateclk_en
  io.out.toDcacheArb.tag_req        := ag_dcache_arb_ld_req
  io.out.toDcacheArb.tag_idx        := ld_ag_pa(14,6)

  //-----------data array------------------------------------
  //------------data req signal-----------
  val bank_en_low_ori = WireInit(0.U(4.W))

  when(!ld_ag_boundary){
    bank_en_low_ori := MuxLookup(Cat(ld_ag_va_ori(3,2),ld_ag_va_add_access_size(3,2)), 0.U(4.W), Seq(
      "b00_00".U -> "b0001".U,
      "b00_01".U -> "b0011".U,
      "b00_10".U -> "b0111".U,
      "b00_11".U -> "b1111".U,
      "b01_01".U -> "b0010".U,
      "b01_10".U -> "b0110".U,
      "b01_11".U -> "b1110".U,
      "b10_10".U -> "b0100".U,
      "b10_11".U -> "b1100".U,
      "b11_11".U -> "b1000".U
    ))
  }.elsewhen(ld_ag_boundary && !ld_ag_data.secd){
    bank_en_low_ori := MuxLookup(ld_ag_va_add_access_size(3,2), 0.U(4.W), Seq(
      "b00".U -> "b0001".U,
      "b01".U -> "b0011".U,
      "b10".U -> "b0111".U,
      "b11".U -> "b1111".U
    ))
  }.elsewhen(ld_ag_boundary && ld_ag_data.secd){
    bank_en_low_ori := MuxLookup(ld_ag_va_ori(3,2), 0.U(4.W), Seq(
      "b00".U -> "b1111".U,
      "b01".U -> "b1110".U,
      "b10".U -> "b1100".U,
      "b11".U -> "b1000".U
    ))
  }

  //if accelate, it must access all banks for 128 bits
  val bank_en_low = Mux(ld_ag_acclr_en, "b1111".U, bank_en_low_ori)

  //-------------for gateclk--------------
  val bank_en_low_gateclk = bank_en_low

  io.out.toDcacheArb.data_gateclk_en := Mux(ag_dcache_arb_ld_gateclk_en, Cat(bank_en_low_gateclk,bank_en_low_gateclk), 0.U(8.W))

  //--------------for req-----------------
  io.out.toDcacheArb.data_req := Mux(ag_dcache_arb_ld_req, Cat(bank_en_low,bank_en_low), 0.U(8.W))

  //-----------data idx-------------------
  io.out.toDcacheArb.data_low_idx  := ld_ag_pa(14,4)
  io.out.toDcacheArb.data_high_idx := Cat(ld_ag_pa(14,5), ~ld_ag_pa(4))
  //TODO: figure out meaning of high_idx

  //==========================================================
  //        Generage exception signal
  //==========================================================
  //if the 1st boundary instruction is weak order and 2nd is strong order, then treat
  //this instruction as weak order
  ld_ag_expt_misalign_no_page := ld_ag_unalign && (ld_ag_data.atomic || ld_ag_inst_vls || ld_ag_ld_inst && !io.in.fromCp0.lsu_mm)
  io.out.toDC.expt_misalign_no_page := ld_ag_expt_misalign_no_page

  val ld_ag_expt_misalign_with_page = ld_ag_unalign_so && ld_ag_page_so && io.in.fromMMU.pa0_vld && ld_ag_ld_inst
  io.out.toDC.expt_misalign_with_page := ld_ag_expt_misalign_with_page

  val ld_ag_expt_page_fault = io.in.fromMMU.pa0_vld && ld_ag_page_fault
  io.out.toDC.expt_page_fault := ld_ag_expt_page_fault

  io.out.toDC.expt_ldamo_not_ca := io.in.fromMMU.pa0_vld && ld_ag_ldamo_inst && !ld_ag_page_ca

  //for vector strong order
  val ld_ag_expt_access_fault_with_page = io.in.fromMMU.pa0_vld && ld_ag_page_so && ld_ag_ld_inst && ld_ag_inst_vls
  io.out.toDC.expt_access_fault_with_page := ld_ag_expt_access_fault_with_page

  io.out.toDC.expt_vld := ld_ag_expt_misalign_no_page ||
                          ld_ag_expt_misalign_with_page ||
                          ld_ag_expt_access_fault_with_page ||
                          ld_ag_expt_page_fault

  //==========================================================
  //        Generage stall/restart signal
  //==========================================================
  //-----------restart----------------------------------------
  ld_ag_atomic_no_cmit_restart_req := ld_ag_inst_vld && ld_ag_data.atomic && !ld_ag_cmit

  //-----------stall------------------------------------------
  //get the stall signal if virtual address cross 4k address
  //for timing, if there is a carry adding last 12 bits, or there is '1' in high
  //bits, it will stall
  //---------------------cross 4k-----------------------------
  ld_ag_4k_sum_ori := Cat(0.U(1.W), ld_ag_base) + Cat(ld_ag_offset.asUInt(63),ld_ag_offset_aftershift(11,0))

  val ld_ag_4k_sum_plus = Cat(0.U(1.W), ld_ag_base) + ld_ag_offset_plus

  val ld_ag_off_4k_high_bits_all_0_ori = !ld_ag_offset_aftershift(63,12).orR
  val ld_ag_off_4k_high_bits_all_1_ori = ld_ag_offset_aftershift(63,12).andR
  val ld_ag_off_4k_high_bits_not_eq    = !ld_ag_off_4k_high_bits_all_0_ori && !ld_ag_off_4k_high_bits_all_1_ori

  val ld_ag_4k_sum_12 = Mux(ld_ag_va_plus_sel, ld_ag_4k_sum_plus(12), ld_ag_4k_sum_ori(12))

  val ld_ag_cross_4k = ld_ag_4k_sum_12 || ld_ag_off_4k_high_bits_not_eq

  //only ldr will trigger secd stall, and will stall at the first split
  val ld_ag_boundary_stall = ld_ag_data.inst_ldr && ld_ag_boundary && !ld_ag_data.secd
  ld_ag_secd_imme_stall := ld_ag_boundary_stall && !ld_ag_already_cross_page_ldr_imme

  val ld_ag_dcache_stall_unmask    = !io.in.fromDcacheArb.ag_ld_sel
  //because corss_4k to mmu, so there doesn't exist prior stall
  ld_ag_cross_page_ldr_imme_stall_req := (ld_ag_cross_4k || ld_ag_secd_imme_stall) && !ld_ag_expt_misalign_no_page && ld_ag_inst_vld

  ld_ag_dcache_stall_req :=  ld_ag_dcache_stall_unmask && ld_ag_inst_vld
  val ld_ag_mmu_stall_req = io.in.fromMMU.stall0

  //-----------arbiter----------------------------------------
  //prioritize:
  //  1. prior_restart  : ldex_no_cmit
  //  2. cross_page_ldr_imme_stall    : cross_4k, secd_imme
  //  3. dcache_stall    : cache
  //  other restart flop to dc stage
  //  other_restart  : utlb_miss, tlb_busy

  val ld_ag_stall_restart = ld_ag_atomic_no_cmit_restart_req ||
                            ld_ag_cross_page_ldr_imme_stall_req ||
                            ld_ag_dcache_stall_req ||
                            ld_ag_mmu_stall_req

  val ld_ag_atomic_no_cmit_restart_arb = ld_ag_atomic_no_cmit_restart_req
  ld_ag_cross_page_ldr_imme_stall_arb := !ld_ag_atomic_no_cmit_restart_req && ld_ag_cross_page_ldr_imme_stall_req && !ld_ag_stall_mask

  //-----------generate total siangl--------------------------
  io.out.toDC.stall_ori := (ld_ag_cross_page_ldr_imme_stall_req ||
                            ld_ag_dcache_stall_req ||
                            ld_ag_mmu_stall_req) &&
                            !ld_ag_atomic_no_cmit_restart_req

  ld_ag_stall_vld := io.out.toDC.stall_ori && !ld_ag_stall_mask

  //for performance,when ag stall,let oldest inst go
  val rf_iid_older_than_ld_ag = (ld_ag_data.iid(6) ^ io.in.pipe3.iid(6)) ^ (ld_ag_data.iid(5,0) > io.in.pipe3.iid(5,0))

  ld_ag_stall_mask := io.in.pipe3.sel && rf_iid_older_than_ld_ag

  io.out.toDC.stall_restart_entry := Mux(ld_ag_stall_mask, ld_ag_data.lsid, io.in.pipe3.lch_entry)

  //==========================================================
  //        Generage to DC stage signal
  //==========================================================
  io.out.toDC.dc_inst_vld := ld_ag_inst_vld && !ld_ag_stall_restart

  io.out.toDC.dc_mmu_req := !io.out.toMMU.abort0

  //this logic may be redundant
  io.out.toDC.dc_addr0 := Mux(io.in.fromDcacheArb.ld_ag_borrow_addr_vld, io.in.fromDcacheArb.ld_ag_addr, ld_ag_addr0)

  //for idu timing
  val ld_ag_ahead_predict = true.B
  io.out.toDC.ahead_predict := ld_ag_ahead_predict
  io.out.toDC.dc_load_inst_vld := ld_ag_inst_vld && !ld_ag_inst_vfls && !(ld_ag_boundary && !ld_ag_acclr_en)
  //if boundary and acclr en, then set load_inst_vld and clr
  //load_ahead_inst_vld, because boundary don't write back in 3 cycles
  io.out.toDC.dc_load_ahead_inst_vld := ld_ag_inst_vld && !ld_ag_inst_vfls && !ld_ag_boundary && !io.in.fromCp0.lsu_da_fwd_dis && ld_ag_ahead_predict

  io.out.toDC.dc_vload_inst_vld := ld_ag_inst_vld && !ld_ag_inst_vfls && !ld_ag_vmb_merge_vld && !(ld_ag_boundary && !ld_ag_acclr_en)

  io.out.toDC.dc_vload_ahead_inst_vld := false.B

  //-----------for timing--------------------------
  //compare iid ahead for dc restart timing
  //compare the instruction in the entry is newer or older
  val ld_ag_raw_new = (ld_ag_data.iid(6) ^ io.in.st_ag_iid(6)) ^ (ld_ag_data.iid(5,0) > io.in.st_ag_iid(5,0))
  io.out.toDC.raw_new := ld_ag_raw_new
  //==========================================================
  //              Interface to other module
  //==========================================================
  //-----------interface to cmit monitor----------------------
  //assign ld_ag_inst_wait_cmit_vld = ld_ag_inst_vld
  //                                  &&  ld_ag_atomic;
  //-----------interface to local monitor---------------------
  io.out.toDC.lm_init_vld := ld_ag_inst_vld &&ld_ag_data.atomic && ld_ag_cmit

  //==========================================================
  //        Generate restart/lsiq signal
  //==========================================================
  //-----------lsiq signal----------------
  val ld_ag_mask_lsid = Mux(ld_ag_inst_vld, ld_ag_data.lsid, VecInit(Seq.fill(LSIQ_ENTRY)(false.B)))

  io.out.toIDU.ag_wait_old_gateclk_en := ld_ag_atomic_no_cmit_restart_arb
  io.out.toIDU.ag_wait_old := Mux(ld_ag_atomic_no_cmit_restart_arb, ld_ag_mask_lsid, VecInit(Seq.fill(LSIQ_ENTRY)(false.B)))

  //==========================================================
  //        for idu timing
  //==========================================================

  io.out.toIDU.pipe3_load_inst_vld := ld_ag_inst_vld && !ld_ag_inst_vfls && !io.in.fromCp0.lsu_da_fwd_dis && ld_ag_ahead_predict

  io.out.toIDU.pipe3_vload_inst_vld := io.out.toIDU.pipe3_load_inst_vld && !ld_ag_inst_vls

  io.out.toIDU.pipe3_preg_dup := ld_ag_data.preg_dup
  for(i <- 0 until 4){
    io.out.toIDU.pipe3_vreg_dup(i) := Cat(0.U(1.W), ld_ag_data.vreg_dup(i))
  }


  //==========================================================
  //        interface to hpcp
  //==========================================================
  io.out.toHpcp.cross_4k_stall := ld_ag_inst_vld && ld_ag_already_cross_page_ldr_imme && !ld_ag_stall_vld && !ld_ag_utlb_miss && !ld_ag_data.already_da
  io.out.toHpcp.other_stall    := ld_ag_inst_vld && !ld_ag_cross_4k && ld_ag_stall_vld && !ld_ag_utlb_miss && !ld_ag_data.already_da

}




