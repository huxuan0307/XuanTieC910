package Core.LSU

import chisel3._
import chisel3.util._

class LQInput extends Bundle{
  val fromCP0 = new Bundle{
    val lsu_corr_dis = Bool()
    val lsu_icg_en = Bool()
    val yy_clk_en = Bool()
  }
  val fromLdDC = new Bundle {
    val addr = Vec(2, UInt(40.W))
    val bytes_vld = Vec(2, UInt(16.W))
    val chk_ld_addr1_vld = Bool()
    val iid = UInt(7.W)
    val inst_chk_vld = Bool()
    val secd = Bool()
  }
  val EntryCreate = new Bundle {
    val dp_vld = Vec(2, Bool())
    val gateclk_en = Vec(2, Bool())
    val vld = Vec(2, Bool())
  }
  val fromPad = new Bundle{
    val yy_icg_scan_en = Bool()
  }
  val fromRTU = new Bundle{
    val yy_xx_commit = Vec(3, Bool())
    val yy_xx_commit_iid = Vec(3, UInt(7.W))
    val yy_xx_flush = Bool()
  }
  val fromStDC = new Bundle{
    val addr0 = UInt(40.W)
    val bytes_vld = UInt(16.W)
    val chk_st_inst_vld = Bool()
    val chk_statomic_inst_vld = Bool()
    val iid = UInt(7.W)
  }
}

class LQOutput extends Bundle{
  val lq_ld_dc_full       = Bool()
  val lq_ld_dc_inst_hit   = Bool()
  val lq_ld_dc_less2      = Bool()
  val lq_ld_dc_spec_fail  = Bool()
  val lq_st_dc_spec_fail  = Bool()
  val lsu_idu_lq_not_full = Bool()
}

class LQIO extends Bundle{
  val in  = Input(new LQInput)
  val out = Output(new LQOutput)
}

class LQ extends Module {
  val io = IO(new LQIO)

  //Wire
  val lq_empty = Wire(Bool())

  val entry_create = Wire(Vec(LSUConfig.LQ_ENTRY, new Bundle {
    val dp_vld = Vec(2, Bool())
    val gateclk_en = Vec(2, Bool())
    val vld = Vec(2, Bool())
  }))

  val lq_entry_inst_hit      = Wire(Vec(LSUConfig.LQ_ENTRY, Bool()))
  val lq_entry_rar_spec_fail = Wire(Vec(LSUConfig.LQ_ENTRY, Bool()))
  val lq_entry_raw_spec_fail = Wire(Vec(LSUConfig.LQ_ENTRY, Bool()))
  val lq_entry_vld           = Wire(Vec(LSUConfig.LQ_ENTRY, Bool()))

  val lq_create_ptr = Wire(Vec(2, Vec(LSUConfig.LQ_ENTRY, Bool())))
  lq_create_ptr(0) := VecInit(Seq.fill(LSUConfig.LQ_ENTRY)(false.B))
  lq_create_ptr(1) := VecInit(Seq.fill(LSUConfig.LQ_ENTRY)(false.B))
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val lq_clk_en = !lq_empty || io.in.EntryCreate.gateclk_en(0)

  //==========================================================
  //                 Instance load queue entry
  //==========================================================
  val lq_entry = Seq.fill(LSUConfig.LQ_ENTRY)(Module(new LQEntry))
  for(i <- 0 until LSUConfig.LQ_ENTRY){
    lq_entry(i).io.in.fromCP0  := io.in.fromCP0
    lq_entry(i).io.in.fromLdDC := io.in.fromLdDC
    lq_entry(i).io.in.fromPad  := io.in.fromPad
    lq_entry(i).io.in.fromRTU  := io.in.fromRTU
    lq_entry(i).io.in.fromStDC := io.in.fromStDC
    lq_entry(i).io.in.EntryCreate := entry_create(i)

    lq_entry_inst_hit(i)      := lq_entry(i).io.out.inst_hit
    lq_entry_rar_spec_fail(i) := lq_entry(i).io.out.rar_spec_fail
    lq_entry_raw_spec_fail(i) := lq_entry(i).io.out.raw_spec_fail
    lq_entry_vld(i)                      := lq_entry(i).io.out.vld
  }

  //==========================================================
  //                 Generate create pointer
  //==========================================================
  when(!lq_entry_vld(0)){
    lq_create_ptr(0)(0)  := true.B
  }
  when(!lq_entry_vld(15)){
    lq_create_ptr(1)(15)  := true.B
  }

  for(i <- 1 until LSUConfig.LQ_ENTRY){
    when(lq_entry_vld.asUInt(i,0) === Cat(0.U(1.W), WireInit(VecInit(Seq.fill(i)(true.B))).asUInt)){lq_create_ptr(0)(i)  := true.B}
    when(lq_entry_vld.asUInt(LSUConfig.LQ_ENTRY-1, LSUConfig.LQ_ENTRY-1-i) === Cat(WireInit(VecInit(Seq.fill(i)(true.B))).asUInt, 0.U(1.W))){lq_create_ptr(1)(LSUConfig.LQ_ENTRY-1-i)  := true.B}
  }

  lq_empty := !lq_entry_vld.asUInt.orR
  val lq_full = lq_entry_vld.asUInt.andR

  //==========================================================
  //                 Generate create pointer
  //==========================================================
  val lq_create_success = Wire(Vec(2, Bool()))
  lq_create_success(0) := io.in.EntryCreate.vld(0) && !io.in.fromRTU.yy_xx_flush &&
    (!io.out.lq_ld_dc_less2 || !io.out.lq_ld_dc_full && !io.in.EntryCreate.vld(1))

  lq_create_success(1) := lq_create_success(0) && io.in.EntryCreate.vld(1)

  for(i <- 0 until LSUConfig.LQ_ENTRY){
    for(j <- 0 until 2){
      entry_create(i).vld(j)        := lq_create_success(j) && lq_create_ptr(j)(i)
      entry_create(i).dp_vld(j)     := io.in.EntryCreate.dp_vld(j) && lq_create_ptr(j)(i)
      entry_create(i).gateclk_en(j) := io.in.EntryCreate.gateclk_en(j) && lq_create_ptr(j)(i)
    }
  }

  //==========================================================
  //                 Generate interface
  //==========================================================
 io.out.lq_ld_dc_full        := lq_full
 io.out.lq_ld_dc_less2       := (lq_create_ptr(0).asUInt | lq_entry_vld.asUInt).andR
 io.out.lq_ld_dc_inst_hit    := lq_entry_inst_hit.asUInt.orR
 io.out.lq_ld_dc_spec_fail   := lq_entry_rar_spec_fail.asUInt.orR
 io.out.lq_st_dc_spec_fail   := lq_entry_raw_spec_fail.asUInt.orR
 io.out.lsu_idu_lq_not_full  := !lq_full

}
