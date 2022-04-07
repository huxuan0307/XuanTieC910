package Core.LSU
import chisel3._
import chisel3.util._
import com.sun.webkit.dom.UIEventImpl

//class LQInput extends Bundle{
//  val fromCP0 = new Bundle{
//    val lsu_corr_dis = Bool()
//    val lsu_icg_en = Bool()
//    val yy_clk_en = Bool()
//  }
//  val fromLdDC = new Bundle {
//    val addr = Vec(2, UInt(40.W))
//    val bytes_vld = Vec(2, UInt(16.W))
//    val chk_ld_addr1_vld = Bool()
//    val iid = UInt(7.W)
//    val inst_chk_vld = Bool()
//    val secd = Bool()
//  }
//  val EntryCreate = new Bundle {
//    val dp_vld = Vec(2, Bool())
//    val gateclk_en = Vec(2, Bool())
//    val vld = Vec(2, Bool())
//  }
//  val fromPad = new Bundle{
//    val yy_icg_scan_en = Bool()
//  }
//  val fromRTU = new Bundle{
//    val yy_xx_commit = Vec(3, Bool())
//    val yy_xx_commit_iid = Vec(3, UInt(7.W))
//    val yy_xx_flush = Bool()
//  }
//  val fromStDC = new Bundle{
//    val addr0 = UInt(40.W)
//    val bytes_vld = UInt(16.W)
//    val chk_st_inst_vld = Bool()
//    val chk_statomic_inst_vld = Bool()
//    val iid = UInt(7.W)
//  }
//}

class LQEntryIO extends Bundle{
  val in = Input(new LQInput)
  val out = Output(new Bundle{
    val inst_hit      = Bool()
    val rar_spec_fail = Bool()
    val raw_spec_fail = Bool()
    val vld           = Bool()
  })
}

class LQEntry extends Module {
  val io = IO(new LQEntryIO)

  //Reg
  val lq_entry_addr0_tto4 = RegInit(0.U(36.W))
  val lq_entry_bytes_vld = RegInit(0.U(16.W))
  val lq_entry_iid = RegInit(0.U(7.W))
  val lq_entry_secd = RegInit(false.B)
  val lq_entry_vld = RegInit(false.B)

  //Wire
  val lq_entry_pop_vld = Wire(Bool())
  //val lq_entry_from_ld_dc_addr = Wire(Vec(2, UInt(LSUConfig.PA_WIDTH.W)))
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val lq_entry_create_clk_en = io.in.EntryCreate.gateclk_en.asUInt.orR

  //==========================================================
  //                 Register
  //==========================================================
  //+-----------+
  //| entry_vld |
  //+-----------+
  when(lq_entry_pop_vld || io.in.fromRTU.yy_xx_flush){
    lq_entry_vld := false.B
  }.elsewhen(io.in.EntryCreate.vld.asUInt.orR){
    lq_entry_vld := true.B
  }
  io.out.vld := lq_entry_vld

  //+-----------+------------+-----+--------+------+
  //| addr_tto2 | bytes_vld0 | iid | deform | secd |
  //+-----------+------------+-----+--------+------+
  when(io.in.EntryCreate.dp_vld(0)){
    lq_entry_addr0_tto4 :=  io.in.fromLdDC.addr(0)(LSUConfig.PA_WIDTH-1,4)
    lq_entry_bytes_vld  :=  io.in.fromLdDC.bytes_vld(0)
    lq_entry_iid        :=  io.in.fromLdDC.iid
    lq_entry_secd       :=  io.in.fromLdDC.secd
  }.elsewhen(io.in.EntryCreate.dp_vld(1)){
    lq_entry_addr0_tto4 :=  io.in.fromLdDC.addr(1)(LSUConfig.PA_WIDTH-1,4)
    lq_entry_bytes_vld  :=  io.in.fromLdDC.bytes_vld(1)
    lq_entry_iid        :=  io.in.fromLdDC.iid
    lq_entry_secd       :=  true.B
  }

  //==========================================================
  //                 Generate pop signal
  //==========================================================
  val lq_entry_cmit_hit = Wire(Vec(3, Bool()))
  for(i <- 0 until 3){
    lq_entry_cmit_hit(i) := io.in.fromRTU.yy_xx_commit(i) && lq_entry_vld &&
      (io.in.fromRTU.yy_xx_commit_iid(i) === lq_entry_iid)
  }
  val lq_entry_cmit_vld = lq_entry_cmit_hit.asUInt.orR && lq_entry_vld
  lq_entry_pop_vld := lq_entry_cmit_vld

  //==========================================================
  //                 lq iid check
  //==========================================================
  //check iid to judge whether to create lq
  io.out.inst_hit := lq_entry_vld && (lq_entry_secd === io.in.fromLdDC.secd) && (lq_entry_iid === io.in.fromLdDC.iid)

  //==========================================================
  //                 RAR speculation check
  //==========================================================

  // situat ld pipe             lq        addr      bytes_vld
  // --------------------------------------------------------
  // 0      ld/ldex             x         `PA_WIDTH-1:2      cross

  //------------------compare signal--------------------------
  //compare the instruction in the entry is newer or older
  val lq_entry_iid_newer_than_ld_dc = (lq_entry_iid(6) ^ io.in.fromLdDC.iid(6)) ^ (lq_entry_iid(5,0) > io.in.fromLdDC.iid(5,0))

  val lq_entry_newer_than_ld_dc = lq_entry_vld && lq_entry_iid_newer_than_ld_dc

  val lq_entry_rar_addr_tto4_hit = Wire(Vec(2, Bool()))
  val lq_entry_rar_do_hit = Wire(Vec(2, Bool()))
  for(i <- 0 until 2){
    lq_entry_rar_addr_tto4_hit(i) := lq_entry_addr0_tto4 === io.in.fromLdDC.addr(i)(LSUConfig.PA_WIDTH-1,4)
    lq_entry_rar_do_hit(i) := (lq_entry_bytes_vld & io.in.fromLdDC.bytes_vld(i)).orR
  }

  //------------------situation 0-----------------------------
  val lq_entry_rar_spec_fail = Wire(Vec(2, Bool()))
  lq_entry_rar_spec_fail(0) := lq_entry_newer_than_ld_dc && io.in.fromLdDC.inst_chk_vld &&
    lq_entry_rar_addr_tto4_hit(0) && lq_entry_rar_do_hit(0)
  lq_entry_rar_spec_fail(1) := lq_entry_newer_than_ld_dc && io.in.fromLdDC.chk_ld_addr1_vld &&
    lq_entry_rar_addr_tto4_hit(1) && lq_entry_rar_do_hit(1)

  //------------------combine---------------------------------
  val lq_entry_rar_spec_fail_unmask = lq_entry_rar_spec_fail.asUInt.orR
  io.out.rar_spec_fail := lq_entry_rar_spec_fail_unmask && !io.in.fromCP0.lsu_corr_dis

  //==========================================================
  //                 RAW speculation check
  //==========================================================
  // situat st pipe             lq        addr      bytes_vld
  // 1      st/stex             ld        `PA_WIDTH-1:4      x

  //------------------compare signal--------------------------
  //compare the instruction in the entry is newer or older
  val lq_entry_iid_newer_than_st_dc = (lq_entry_iid(6) ^ io.in.fromStDC.iid(6)) ^ (lq_entry_iid(5,0) > io.in.fromStDC.iid(5,0))

  val lq_entry_newer_than_st_dc  = lq_entry_vld && lq_entry_iid_newer_than_st_dc

  val lq_entry_raw_addr_tto4_hit = lq_entry_addr0_tto4 === io.in.fromStDC.addr0(LSUConfig.PA_WIDTH-1,4)

  val lq_entry_raw_do_hit = (lq_entry_bytes_vld & io.in.fromStDC.bytes_vld).orR

  //------------------situation 1-----------------------------
  val lq_entry_raw_spec_fail1 = lq_entry_newer_than_st_dc &&
    (io.in.fromStDC.chk_st_inst_vld  ||  io.in.fromStDC.chk_statomic_inst_vld) &&
    lq_entry_raw_addr_tto4_hit && lq_entry_raw_do_hit

  //------------------combine---------------------------------
  io.out.raw_spec_fail := lq_entry_raw_spec_fail1

  //==========================================================
  //                 Generate interface
  //==========================================================
}
