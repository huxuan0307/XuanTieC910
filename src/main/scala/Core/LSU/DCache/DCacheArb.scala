package Core.LSU.DCache
import Core.LsuConfig
import chisel3._
import chisel3.util._

class DCacheArbReqIn extends Bundle with LsuConfig{
  val borrow_addr = UInt(PA_WIDTH.W)
  val data_way = Bool()
  val dcache_replace = Bool()
  val ld_borrow_req = Bool()
  val ld_borrow_req_gate = Bool()
  val ld_data_gateclk_en = UInt(8.W)
  val ld_data_gwen = UInt(8.W)
  val ld_data_high_idx = UInt(11.W)
  val ld_data_low_idx = UInt(11.W)
  val ld_data_req = UInt(8.W)
  val ld_data_wen = UInt(32.W)
  val ld_data_high_din = UInt(128.W)
  val ld_data_idx = UInt(11.W)
  val ld_data_low_din = UInt(128.W)
  val ld_req = Bool()
  val ld_tag_din = UInt(54.W)
  val ld_tag_gateclk_en = Bool()
  val ld_tag_idx = UInt(9.W)
  val ld_tag_read = Bool()
  val ld_tag_req = Bool()
  val ld_tag_wen = UInt(2.W)
  val serial_req = Bool()
  val set_way_mode = Bool()
  val st_borrow_req = Bool()
  val st_dirty_din = UInt(7.W)
  val st_dirty_gateclk_en = Bool()
  val st_dirty_gwen = Bool()
  val st_dirty_idx = UInt(9.W)
  val st_dirty_req = Bool()
  val st_dirty_wen = UInt(7.W)
  val st_id = UInt(SNQ_ENTRY.W)
  val st_req = Bool()
  val st_tag_din = UInt(52.W)
  val st_tag_gateclk_en = Bool()
  val st_tag_idx = UInt(9.W)
  val st_tag_req = Bool()
  val st_tag_wen = UInt(2.W)
  val way = Bool()
  val req_addr = UInt(PA_WIDTH.W)
}

class DCacheArbInput extends Bundle with LsuConfig {
  val fromLoadAG = new Bundle{
    val ld_data_gateclk_en = UInt(8.W)
    val ld_data_high_idx = UInt(11.W)
    val ld_data_low_idx = UInt(11.W)
    val ld_data_req = UInt(8.W)
    val ld_tag_gateclk_en = Bool()
    val ld_tag_idx = UInt(9.W)
    val ld_tag_req = Bool()
  }
  val fromStoreAG = new Bundle{
    val st_dirty_gateclk_en = Bool()
    val st_dirty_idx = UInt(9.W)
    val st_dirty_req = Bool()
    val st_tag_gateclk_en = Bool()
    val st_tag_idx = UInt(9.W)
    val st_tag_req = Bool()
  }

  val cp0_lsu_icg_en = Bool()

  val fromIcc = new DCacheArbReqIn
  val fromLfb = new DCacheArbReqIn
  val fromMcic = new DCacheArbReqIn

  val pad_yy_icg_scan_en = Bool()

  val fromSnq = new DCacheArbReqIn
  val snq_dcache_sdb_id = UInt(VB_DATA_ENTRY.W)

  val fromVb = new DCacheArbReqIn
  val vb_rcl_sm_data_id = UInt(VB_DATA_ENTRY.W)

  val fromWmb = new DCacheArbReqIn

}

class DCacheArbOutput extends Bundle with LsuConfig {
  val dcache_arb_ag_ld_sel = Bool()
  val dcache_arb_ag_st_sel = Bool()
  val dcache_arb_icc_ld_grnt = Bool()
  val dcache_arb_ld_ag_addr = UInt(PA_WIDTH.W)
  val dcache_arb_ld_ag_borrow_addr_vld = Bool()
  val toLoadDC = new Bundle{
    val borrow_db = UInt(VB_DATA_ENTRY.W)
    val borrow_icc = Bool()
    val borrow_mmu = Bool()
    val borrow_sndb = Bool()
    val borrow_vb = Bool()
    val borrow_vld = Bool()
    val borrow_vld_gate = Bool()
    val settle_way = Bool()
  }
  val dcache_arb_lfb_ld_grnt = Bool()
  val dcache_arb_mcic_ld_grnt = Bool()
  val dcache_arb_snq_ld_grnt = Bool()
  val dcache_arb_snq_st_grnt = Bool()
  val dcache_arb_st_ag_addr = UInt(PA_WIDTH.W)
  val dcache_arb_st_ag_borrow_addr_vld = Bool()
  val toStoreDC = new Bundle{
    val borrow_icc = Bool()
    val borrow_snq = Bool()
    val borrow_snq_id = UInt(SNQ_ENTRY.W)
    val borrow_vld = Bool()
    val borrow_vld_gate = Bool()
    val dcache_replace = Bool()
    val dcache_sw = Bool()
  }
  val dcache_arb_vb_ld_grnt = Bool()
  val dcache_arb_vb_st_grnt = Bool()
  val dcache_arb_wmb_ld_grnt = Bool()
  val dcache_dirty_din = UInt(7.W)
  val dcache_dirty_gwen = Bool()
  val dcache_dirty_wen = UInt(7.W)
  val dcache_idx = UInt(9.W)
  val dcache_snq_st_sel = Bool()
  val dcache_tag_din = UInt(52.W)
  val dcache_tag_gwen = Bool()
  val dcache_tag_wen = UInt(2.W)
  val dcache_vb_snq_gwen = Bool()

  val ld_data = new Bundle{
    val gateclk_en = Vec(8, Bool())
    val gwen_b = Vec(8, Bool())
    val high_din = UInt(128.W)
    val high_idx = UInt(11.W)
    val low_din = UInt(128.W)
    val low_idx = UInt(11.W)
    val sel_b = Vec(8, Bool())
    val wen_b = Vec(8, UInt(4.W))
  }
  val ld_tag = new Bundle{
    val din = UInt(54.W)
    val gateclk_en = Bool()
    val gwen_b = Bool()
    val idx = UInt(9.W)
    val sel_b = Bool()
    val wen_b = UInt(2.W)
  }
  val lsu_dcache_ld_xx_gwen = Bool()
  val st_dirty = new Bundle{
    val din = UInt(7.W)
    val gateclk_en = Bool()
    val gwen_b = Bool()
    val idx = UInt(9.W)
    val sel_b = Bool()
    val wen_b = UInt(7.W)
  }
  val st_tag = new Bundle{
    val din = UInt(52.W)
    val gateclk_en = Bool()
    val gwen_b = Bool()
    val idx = UInt(9.W)
    val sel_b = Bool()
    val wen_b = UInt(2.W)
  }
}

class DCacheArbIO extends Bundle{
  val in = Input(new DCacheArbInput)
  val out = Output(new DCacheArbOutput)
}

class DCacheArb extends Module with LsuConfig {
  val io = IO(new DCacheArbIO)

  //Reg
  val dcache_arb_serial_vld = RegInit(false.B)
  val dcache_arb_serial_lfb = RegInit(false.B)
  val dcache_arb_serial_vb  = RegInit(false.B)
  val dcache_arb_serial_snq = RegInit(false.B)

  //Wire
  val dcache_arb_serial_req = Wire(Bool())
  val dcache_arb_lfb_ld_sel = Wire(Bool())
  val dcache_arb_vb_ld_sel  = Wire(Bool())
  val dcache_arb_snq_ld_sel = Wire(Bool())

  val dcache_arb_lfb_st_sel_unmask = Wire(Bool())
  val dcache_arb_vb_st_sel_unmask = Wire(Bool())
  val dcache_arb_snq_st_sel_unmask = Wire(Bool())
  val dcache_arb_wmb_st_sel_unmask = Wire(Bool())
  val dcache_arb_icc_st_sel_unmask = Wire(Bool())

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val dcache_arb_serial_clk_en = dcache_arb_serial_vld || dcache_arb_serial_req

  //==========================================================
  //                 Serial wires
  //==========================================================
  val dcache_arb_serial_lfb_sel = dcache_arb_lfb_ld_sel && io.in.fromLfb.serial_req
  val dcache_arb_serial_vb_sel  = dcache_arb_vb_ld_sel && io.in.fromVb.serial_req
  val dcache_arb_serial_snq_sel = dcache_arb_snq_ld_sel && io.in.fromSnq.serial_req

  dcache_arb_serial_req := dcache_arb_serial_lfb_sel || dcache_arb_serial_vb_sel || dcache_arb_serial_snq_sel
  //==========================================================
  //                 Serial request registers
  //==========================================================
  when(dcache_arb_serial_req){
    dcache_arb_serial_vld := true.B
    dcache_arb_serial_lfb := dcache_arb_serial_lfb_sel
    dcache_arb_serial_vb  := dcache_arb_serial_vb_sel
    dcache_arb_serial_snq := dcache_arb_serial_snq_sel
  }.otherwise{
    dcache_arb_serial_vld := false.B
    dcache_arb_serial_lfb := false.B
    dcache_arb_serial_vb  := false.B
    dcache_arb_serial_snq := false.B
  }

  //==========================================================
  //              Sel/Grnt signal for LD part
  //==========================================================
  //1. lfb state machine
  //2. vb state machine
  //3. snq/sndb req
  //4. mmu dcache issue controller
  //5. wmb
  //6. icc
  //7. ld ag stage

  //------------------unmask signal---------------------------
  //select signal send back to the source
  val dcache_arb_ld_sel = PriorityMux(Seq(
    (!dcache_arb_serial_vld && io.in.fromLfb.ld_req)  -> "b1000000".U,
    (!dcache_arb_serial_vld && io.in.fromVb.ld_req)   -> "b0100000".U,
    (!dcache_arb_serial_vld && io.in.fromSnq.ld_req)  -> "b0010000".U,
    (!dcache_arb_serial_vld && io.in.fromIcc.ld_req)  -> "b0001000".U,
    (!dcache_arb_serial_vld && io.in.fromWmb.ld_req)  -> "b0000100".U,
    (!dcache_arb_serial_vld && io.in.fromMcic.ld_req) -> "b0000010".U,
    true.B                                            -> "b0000001".U
  ))

  val dcache_arb_lfb_ld_sel_unmask  = dcache_arb_ld_sel(6)
  val dcache_arb_vb_ld_sel_unmask   = dcache_arb_ld_sel(5)
  val dcache_arb_snq_ld_sel_unmask  = dcache_arb_ld_sel(4)
  val dcache_arb_icc_ld_sel_unmask  = dcache_arb_ld_sel(3)
  val dcache_arb_wmb_ld_sel_unmask  = dcache_arb_ld_sel(2)
  val dcache_arb_mcic_ld_sel_unmask = dcache_arb_ld_sel(1)
  val dcache_arb_ag_ld_sel_unmask   = dcache_arb_ld_sel(0)

  //------------------masked signal---------------------------
  //because lfb/vb/snq/icc may request ld and st pipeline once a time,
  //to insure that they can get both sel signal simultaneously,
  //if they request 2 pipeline and get 1 sel, then it must be clr to 0.
  dcache_arb_lfb_ld_sel := dcache_arb_lfb_ld_sel_unmask && (!io.in.fromLfb.st_req || dcache_arb_lfb_st_sel_unmask) || dcache_arb_serial_lfb
  dcache_arb_vb_ld_sel  := dcache_arb_vb_ld_sel_unmask && (!io.in.fromVb.st_req || dcache_arb_vb_st_sel_unmask) || dcache_arb_serial_vb
  dcache_arb_snq_ld_sel := dcache_arb_snq_ld_sel_unmask&& (!io.in.fromSnq.st_req || dcache_arb_snq_st_sel_unmask) || dcache_arb_serial_snq

  val dcache_arb_wmb_ld_sel  = dcache_arb_wmb_ld_sel_unmask && dcache_arb_wmb_st_sel_unmask
  val dcache_arb_icc_ld_sel  = dcache_arb_icc_ld_sel_unmask && dcache_arb_icc_st_sel_unmask
  val dcache_arb_mcic_ld_sel = dcache_arb_mcic_ld_sel_unmask
  val dcache_arb_ag_ld_sel   = dcache_arb_ag_ld_sel_unmask
  io.out.dcache_arb_ag_ld_sel := dcache_arb_ag_ld_sel

  //----------shorten signal to select signal-----------------
  val dcache_arb_lfb_ld_dp_sel  = dcache_arb_lfb_ld_sel_unmask  ||  dcache_arb_serial_lfb
  val dcache_arb_vb_ld_dp_sel   = dcache_arb_vb_ld_sel_unmask   ||  dcache_arb_serial_vb
  val dcache_arb_snq_ld_dp_sel  = dcache_arb_snq_ld_sel_unmask  ||  dcache_arb_serial_snq
  val dcache_arb_wmb_ld_dp_sel  = dcache_arb_wmb_ld_sel_unmask
  val dcache_arb_icc_ld_dp_sel  = dcache_arb_icc_ld_sel_unmask
  val dcache_arb_mcic_ld_dp_sel = dcache_arb_mcic_ld_sel_unmask
  val dcache_arb_ag_ld_dp_sel   = dcache_arb_ag_ld_sel_unmask


  val dcache_arb_ld_dp_sel_id = Wire(Vec(7, Bool()))
  dcache_arb_ld_dp_sel_id(6) := dcache_arb_lfb_ld_sel_unmask  ||  dcache_arb_serial_lfb
  dcache_arb_ld_dp_sel_id(5) := dcache_arb_vb_ld_sel_unmask   ||  dcache_arb_serial_vb
  dcache_arb_ld_dp_sel_id(4) := dcache_arb_snq_ld_sel_unmask  ||  dcache_arb_serial_snq
  dcache_arb_ld_dp_sel_id(3) := dcache_arb_wmb_ld_sel_unmask
  dcache_arb_ld_dp_sel_id(2) := dcache_arb_icc_ld_sel_unmask
  dcache_arb_ld_dp_sel_id(1) := dcache_arb_mcic_ld_sel_unmask
  dcache_arb_ld_dp_sel_id(0) := dcache_arb_ag_ld_sel_unmask

  //------------------grnt   signal---------------------------
  io.out.dcache_arb_lfb_ld_grnt  := dcache_arb_lfb_ld_sel
  io.out.dcache_arb_vb_ld_grnt   := dcache_arb_vb_ld_sel
  io.out.dcache_arb_snq_ld_grnt  := dcache_arb_snq_ld_sel
  io.out.dcache_arb_wmb_ld_grnt  := dcache_arb_wmb_ld_sel
  io.out.dcache_arb_icc_ld_grnt  := dcache_arb_icc_ld_sel
  io.out.dcache_arb_mcic_ld_grnt := dcache_arb_mcic_ld_sel

  //==========================================================
  //        Borrow signal for LD part to DC stage
  //==========================================================
  //if vb request data, mmu request data, they will borrow ld dc/da stage
  //-----------------------borrow addr------------------------
  io.out.dcache_arb_ld_ag_borrow_addr_vld := dcache_arb_mcic_ld_sel
  io.out.dcache_arb_ld_ag_addr := io.in.fromMcic.req_addr
  //---------------------borrow signal------------------------
  io.out.toLoadDC.borrow_vld :=
    dcache_arb_vb_ld_sel  &&  io.in.fromVb.ld_borrow_req ||
    dcache_arb_snq_ld_sel  &&  io.in.fromSnq.ld_borrow_req ||
    dcache_arb_icc_ld_sel  &&  io.in.fromIcc.ld_borrow_req ||
    dcache_arb_wmb_ld_sel  &&  io.in.fromWmb.ld_borrow_req ||
    dcache_arb_mcic_ld_sel

  io.out.toLoadDC.borrow_vld_gate :=
    dcache_arb_vb_ld_sel  &&  io.in.fromVb.ld_borrow_req_gate ||
    dcache_arb_snq_ld_sel  &&  io.in.fromSnq.ld_borrow_req_gate ||
    dcache_arb_icc_ld_sel  &&  io.in.fromIcc.ld_borrow_req ||
    dcache_arb_wmb_ld_sel  &&  io.in.fromWmb.ld_borrow_req ||
    dcache_arb_mcic_ld_sel

  io.out.toLoadDC.borrow_db := Mux(dcache_arb_vb_ld_sel, io.in.vb_rcl_sm_data_id, io.in.snq_dcache_sdb_id)

  io.out.toLoadDC.borrow_vb   := dcache_arb_vb_ld_sel &&  io.in.fromVb.ld_borrow_req
  io.out.toLoadDC.borrow_sndb := dcache_arb_snq_ld_sel  &&  io.in.fromSnq.ld_borrow_req
  io.out.toLoadDC.borrow_mmu  := dcache_arb_mcic_ld_sel
  io.out.toLoadDC.borrow_icc  := dcache_arb_icc_ld_sel  &&  io.in.fromIcc.ld_borrow_req

  //borrow way is used
  io.out.toLoadDC.settle_way :=
    dcache_arb_vb_ld_sel  &&  io.in.fromVb.data_way ||
    dcache_arb_snq_ld_sel  &&  io.in.fromSnq.data_way ||
    dcache_arb_icc_ld_sel  &&  io.in.fromIcc.data_way ||
    dcache_arb_wmb_ld_sel  &&  io.in.fromWmb.data_way

  //==========================================================
  //        Input select for LD part
  //==========================================================
  //------------------tag   array-----------------------------
  //-----------gateclk--------------------
  io.out.ld_tag.gateclk_en :=
    io.in.fromLfb.ld_tag_gateclk_en   ||
    io.in.fromVb.ld_tag_gateclk_en    ||
    io.in.fromSnq.ld_tag_gateclk_en   ||
    io.in.fromWmb.ld_tag_gateclk_en   ||
    io.in.fromIcc.ld_tag_gateclk_en   ||
    io.in.fromMcic.ld_tag_gateclk_en  ||
    io.in.fromLoadAG.ld_tag_gateclk_en

  //-----------interface------------------
  val dcache_arb_ld_tag_req  =
    dcache_arb_lfb_ld_sel  &&  io.in.fromLfb.ld_tag_req ||
    dcache_arb_vb_ld_sel   &&  io.in.fromVb.ld_tag_req ||
    dcache_arb_snq_ld_sel  &&  io.in.fromSnq.ld_tag_req ||
    dcache_arb_wmb_ld_sel  &&  io.in.fromWmb.ld_tag_req ||
    dcache_arb_icc_ld_sel  &&  io.in.fromIcc.ld_tag_req ||
    dcache_arb_mcic_ld_sel ||
    dcache_arb_ag_ld_sel   &&  io.in.fromLoadAG.ld_tag_req

  io.out.ld_tag.sel_b := !dcache_arb_ld_tag_req

  io.out.ld_tag.idx := MuxLookup(dcache_arb_ld_dp_sel_id.asUInt, 0.U(9.W), Seq(
    "b100_0000".U -> io.in.fromLfb.ld_tag_idx,
    "b010_0000".U -> io.in.fromVb.ld_tag_idx,
    "b001_0000".U -> io.in.fromSnq.ld_tag_idx,
    "b000_1000".U -> io.in.fromWmb.ld_tag_idx,
    "b000_0100".U -> io.in.fromIcc.ld_tag_idx,
    "b000_0010".U -> io.in.fromMcic.ld_tag_idx,
    "b000_0001".U -> io.in.fromLoadAG.ld_tag_idx
  ))

  io.out.ld_tag.din := Mux(dcache_arb_lfb_ld_dp_sel, io.in.fromLfb.ld_tag_din, 0.U(54.W))

  val lsu_dcache_ld_tag_gwen =
    dcache_arb_lfb_ld_dp_sel ||
    dcache_arb_vb_ld_dp_sel ||
    dcache_arb_snq_ld_dp_sel ||
    dcache_arb_wmb_ld_dp_sel ||
    dcache_arb_icc_ld_dp_sel && !io.in.fromIcc.ld_tag_read
  io.out.ld_tag.gwen_b := lsu_dcache_ld_tag_gwen

  val lsu_dcache_ld_tag_wen = Mux1H(Seq(
    dcache_arb_lfb_ld_dp_sel ->  io.in.fromLfb.ld_tag_wen,
    dcache_arb_vb_ld_dp_sel  ->  io.in.fromVb.ld_tag_wen,
    dcache_arb_snq_ld_dp_sel ->  io.in.fromSnq.ld_tag_wen,
    dcache_arb_wmb_ld_dp_sel ->  io.in.fromWmb.ld_tag_wen,
    dcache_arb_icc_ld_dp_sel ->  3.U(2.W)
  ))
  io.out.ld_tag.wen_b := !lsu_dcache_ld_tag_wen

  //------------------for cache buffer-----------------------------
  io.out.lsu_dcache_ld_xx_gwen := lsu_dcache_ld_tag_gwen

  //------------------data  array-----------------------------
  //-----------gateclk--------------------
  io.out.ld_data.gateclk_en :=
    (io.in.fromLfb.ld_data_gateclk_en |
    io.in.fromVb.ld_data_gateclk_en |
    io.in.fromSnq.ld_data_gateclk_en |
    io.in.fromWmb.ld_data_gateclk_en |
    io.in.fromIcc.ld_data_gateclk_en |
    io.in.fromMcic.ld_data_gateclk_en |
    io.in.fromLoadAG.ld_data_gateclk_en).asTypeOf(Vec(8, Bool()))

  //-----------interface------------------
  val dcache_arb_ld_data_req = Mux1H(Seq(
    dcache_arb_lfb_ld_sel  -> "b1111_1111".U,
    dcache_arb_vb_ld_sel   -> "b1111_1111".U,
    dcache_arb_snq_ld_sel  -> "b1111_1111".U,
    dcache_arb_wmb_ld_sel  -> io.in.fromWmb.ld_data_req,
    dcache_arb_mcic_ld_sel -> io.in.fromMcic.ld_data_req,
    dcache_arb_ag_ld_sel   -> io.in.fromLoadAG.ld_data_req,
    dcache_arb_icc_ld_sel  -> io.in.fromIcc.ld_data_req
  ))
  io.out.ld_data.sel_b := (!dcache_arb_ld_data_req).asTypeOf(Vec(8, Bool()))

  io.out.ld_data.low_idx := MuxLookup(dcache_arb_ld_dp_sel_id.asUInt, 0.U(11.W), Seq(
    "b100_0000".U -> io.in.fromLfb.ld_data_idx,
    "b010_0000".U -> io.in.fromVb.ld_data_idx,
    "b001_0000".U -> io.in.fromSnq.ld_data_idx,
    "b000_1000".U -> io.in.fromWmb.ld_data_idx,
    "b000_0100".U -> io.in.fromIcc.ld_data_low_idx,
    "b000_0010".U -> io.in.fromMcic.ld_data_low_idx,
    "b000_0001".U -> io.in.fromLoadAG.ld_data_low_idx
  ))

  io.out.ld_data.high_idx := MuxLookup(dcache_arb_ld_dp_sel_id.asUInt, 0.U(11.W), Seq(
    "b100_0000".U -> io.in.fromLfb.ld_data_idx,
    "b010_0000".U -> io.in.fromVb.ld_data_idx,
    "b001_0000".U -> io.in.fromSnq.ld_data_idx,
    "b000_1000".U -> io.in.fromWmb.ld_data_idx,
    "b000_0100".U -> io.in.fromIcc.ld_data_high_idx,
    "b000_0010".U -> io.in.fromMcic.ld_data_high_idx,
    "b000_0001".U -> io.in.fromLoadAG.ld_data_high_idx
  ))

  io.out.ld_data.low_din := Mux(dcache_arb_lfb_ld_dp_sel, io.in.fromLfb.ld_data_low_din, io.in.fromWmb.ld_data_low_din)
  io.out.ld_data.high_din := Mux(dcache_arb_lfb_ld_dp_sel, io.in.fromLfb.ld_data_high_din, io.in.fromWmb.ld_data_high_din)

  val lsu_dcache_ld_data_gwen = Mux(dcache_arb_lfb_ld_dp_sel, "b1111_1111".U, 0.U(8.W)) |
    Mux(dcache_arb_wmb_ld_dp_sel, io.in.fromWmb.ld_data_gwen, 0.U(8.W))
  io.out.ld_data.gwen_b := (!lsu_dcache_ld_data_gwen).asTypeOf(Vec(8, Bool()))

  val lsu_dcache_ld_data_wen = Mux(dcache_arb_lfb_ld_dp_sel, VecInit(Seq.fill(32)(true.B)).asUInt, 0.U(32.W)) |
    Mux(dcache_arb_wmb_ld_dp_sel, io.in.fromWmb.ld_data_wen, 0.U(32.W))
  io.out.ld_data.wen_b := (!lsu_dcache_ld_data_wen).asTypeOf(Vec(8, UInt(4.W)))

  //==========================================================
  //        Sel/Grnt signal for ST part
  //==========================================================
  //1. lfb state machine
  //2. vb state machine
  //3. snq
  //4. wmb
  //5. icc
  //6. st ag stage

  //------------------unmask signal---------------------------
  val dcache_arb_st_sel = PriorityMux(Seq(
    (!dcache_arb_serial_vld && io.in.fromLfb.st_req)  -> "b100000".U,
    (!dcache_arb_serial_vld && io.in.fromVb.st_req)   -> "b010000".U,
    (!dcache_arb_serial_vld && io.in.fromSnq.st_req)  -> "b001000".U,
    (!dcache_arb_serial_vld && io.in.fromIcc.st_req)  -> "b000100".U,
    (!dcache_arb_serial_vld && io.in.fromWmb.st_req)  -> "b000010".U,
    true.B                                            -> "b000001".U
  ))

  dcache_arb_lfb_st_sel_unmask := dcache_arb_st_sel(5)
  dcache_arb_vb_st_sel_unmask  := dcache_arb_st_sel(4)
  dcache_arb_snq_st_sel_unmask := dcache_arb_st_sel(3)
  dcache_arb_icc_st_sel_unmask := dcache_arb_st_sel(2)
  dcache_arb_wmb_st_sel_unmask := dcache_arb_st_sel(1)
  val dcache_arb_ag_st_sel_unmask  = dcache_arb_st_sel(0)

  //------------------masked signal---------------------------
  //because lfb/vb/snq/icc may request ld and st pipeline once a time,
  //to insure that they can get both sel signal simultaneously,
  //if they request 2 pipeline and get 1 sel, then it must be clr to 0.
  val dcache_arb_lfb_st_sel  = dcache_arb_lfb_st_sel_unmask && (!io.in.fromLfb.ld_req || dcache_arb_lfb_ld_sel_unmask) || dcache_arb_serial_lfb
  val dcache_arb_vb_st_sel   = dcache_arb_vb_st_sel_unmask && (!io.in.fromVb.ld_req || dcache_arb_vb_ld_sel_unmask) || dcache_arb_serial_vb
  val dcache_arb_snq_st_sel  = dcache_arb_snq_st_sel_unmask&& (!io.in.fromSnq.ld_req || dcache_arb_snq_ld_sel_unmask) || dcache_arb_serial_snq

  val dcache_arb_wmb_st_sel  = dcache_arb_wmb_st_sel_unmask && dcache_arb_wmb_ld_sel_unmask
  val dcache_arb_icc_st_sel  = dcache_arb_icc_st_sel_unmask && dcache_arb_icc_ld_sel_unmask
  val dcache_arb_ag_st_sel   = dcache_arb_ag_st_sel_unmask
  io.out.dcache_arb_ag_st_sel := dcache_arb_ag_st_sel

  //----------shorten signal to select signal-----------------
  val dcache_arb_lfb_st_dp_sel  = dcache_arb_lfb_st_sel_unmask  ||  dcache_arb_serial_lfb
  val dcache_arb_vb_st_dp_sel   = dcache_arb_vb_st_sel_unmask   ||  dcache_arb_serial_vb
  val dcache_arb_snq_st_dp_sel  = dcache_arb_snq_st_sel_unmask  ||  dcache_arb_serial_snq
  val dcache_arb_wmb_st_dp_sel  = dcache_arb_wmb_st_sel_unmask
  val dcache_arb_icc_st_dp_sel  = dcache_arb_icc_st_sel_unmask
  val dcache_arb_ag_st_dp_sel   = dcache_arb_ag_st_sel_unmask

  val dcache_arb_st_dp_sel_id = Wire(Vec(6, Bool()))

  dcache_arb_st_dp_sel_id(5) := dcache_arb_lfb_st_sel_unmask  ||  dcache_arb_serial_lfb
  dcache_arb_st_dp_sel_id(4) := dcache_arb_vb_st_sel_unmask   ||  dcache_arb_serial_vb
  dcache_arb_st_dp_sel_id(3) := dcache_arb_snq_st_sel_unmask  ||  dcache_arb_serial_snq
  dcache_arb_st_dp_sel_id(2) := dcache_arb_wmb_st_sel_unmask
  dcache_arb_st_dp_sel_id(1) := dcache_arb_icc_st_sel_unmask
  dcache_arb_st_dp_sel_id(0) := dcache_arb_ag_st_sel_unmask

  //------------------grnt   signal---------------------------
  io.out.dcache_arb_vb_st_grnt  := dcache_arb_vb_st_sel
  io.out.dcache_arb_snq_st_grnt := dcache_arb_snq_st_sel

  //==========================================================
  //        Borrow signal for ST part to DC stage
  //==========================================================
  //if vb request tag/dirty, mmu request tag/dirty, they will borrow st dc/da stage
  //---------------------borrow addr--------------------------
  io.out.dcache_arb_st_ag_borrow_addr_vld :=
    dcache_arb_vb_st_sel && io.in.fromVb.st_borrow_req ||
    dcache_arb_snq_st_sel && io.in.fromSnq.st_borrow_req

  io.out.dcache_arb_st_ag_addr :=
    Mux(dcache_arb_vb_st_sel, io.in.fromVb.borrow_addr, 0.U(PA_WIDTH.W)) |
    Mux(dcache_arb_snq_st_sel, io.in.fromSnq.borrow_addr, 0.U(PA_WIDTH.W))

  //---------------------borrow signal------------------------
  io.out.toStoreDC.borrow_vld_gate := io.out.dcache_arb_st_ag_borrow_addr_vld ||
    dcache_arb_icc_st_sel && io.in.fromIcc.st_borrow_req

  io.out.toStoreDC.borrow_vld := io.out.dcache_arb_st_ag_borrow_addr_vld ||
    dcache_arb_icc_st_sel && io.in.fromIcc.st_borrow_req

  io.out.toStoreDC.borrow_snq := dcache_arb_snq_st_sel && io.in.fromSnq.st_borrow_req
  io.out.toStoreDC.borrow_snq_id := io.in.fromSnq.st_id
  io.out.toStoreDC.borrow_icc := dcache_arb_icc_st_sel && io.in.fromIcc.st_borrow_req
  //------------------borrow other signal---------------------
  io.out.toStoreDC.dcache_replace := dcache_arb_vb_st_sel  &&  io.in.fromVb.dcache_replace
  io.out.toStoreDC.dcache_sw :=
    dcache_arb_vb_st_sel && io.in.fromVb.set_way_mode ||
    dcache_arb_icc_st_sel && io.in.fromIcc.way

  //==========================================================
  //        Input select for ST part
  //==========================================================
  //------------------tag   array-----------------------------
  //-----------gateclk--------------------
  io.out.st_tag.gateclk_en :=
    io.in.fromLfb.st_tag_gateclk_en   ||
    io.in.fromVb.st_tag_gateclk_en    ||
    io.in.fromSnq.st_tag_gateclk_en   ||
    io.in.fromIcc.st_tag_gateclk_en   ||
    io.in.fromStoreAG.st_tag_gateclk_en

  //-----------interface------------------
  val dcache_arb_st_tag_req  =
    dcache_arb_lfb_st_sel &&  io.in.fromLfb.st_tag_req ||
    dcache_arb_vb_st_sel  &&  io.in.fromVb.st_tag_req  ||
    dcache_arb_snq_st_sel &&  io.in.fromSnq.st_tag_req ||
    dcache_arb_icc_st_sel &&  io.in.fromIcc.st_tag_req ||
    dcache_arb_ag_st_sel  &&  io.in.fromStoreAG.st_tag_req

  io.out.st_tag.sel_b := !dcache_arb_st_tag_req

  io.out.st_tag.idx := MuxLookup(dcache_arb_st_dp_sel_id.asUInt, 0.U(9.W), Seq(
    "b10_0000".U -> io.in.fromLfb.st_tag_idx,
    "b01_0000".U -> io.in.fromVb.st_tag_idx,
    "b00_1000".U -> io.in.fromSnq.st_tag_idx,
    "b00_0010".U -> io.in.fromIcc.st_tag_idx,
    "b00_0001".U -> io.in.fromStoreAG.st_tag_idx
  ))

  io.out.st_tag.din := io.in.fromLfb.st_tag_din
  val lsu_dcache_st_tag_gwen = dcache_arb_lfb_st_dp_sel
  val lsu_dcache_st_tag_wen = Mux(dcache_arb_lfb_st_dp_sel, io.in.fromLfb.st_tag_wen, 0.U(2.W))

  io.out.st_tag.gwen_b := !lsu_dcache_st_tag_gwen
  io.out.st_tag.wen_b  := !lsu_dcache_st_tag_wen

  //------------------dirty array-----------------------------
  //-----------gateclk--------------------
  io.out.st_dirty.gateclk_en :=
    io.in.fromLfb.st_dirty_gateclk_en   ||
    io.in.fromVb.st_dirty_gateclk_en    ||
    io.in.fromSnq.st_dirty_gateclk_en   ||
    io.in.fromWmb.st_dirty_gateclk_en   ||
    io.in.fromIcc.st_dirty_gateclk_en   ||
    io.in.fromStoreAG.st_dirty_gateclk_en

  //-----------interface------------------
  val dcache_arb_st_dirty_req =
    dcache_arb_lfb_st_sel &&  io.in.fromLfb.st_dirty_req ||
    dcache_arb_vb_st_sel  &&  io.in.fromVb.st_dirty_req  ||
    dcache_arb_snq_st_sel &&  io.in.fromSnq.st_dirty_req ||
    dcache_arb_wmb_st_sel &&  io.in.fromWmb.st_dirty_req ||
    dcache_arb_icc_st_sel &&  io.in.fromIcc.st_dirty_req ||
    dcache_arb_ag_st_sel  &&  io.in.fromStoreAG.st_dirty_req

  io.out.st_dirty.sel_b := !dcache_arb_st_dirty_req

  io.out.st_dirty.idx := MuxLookup(dcache_arb_st_dp_sel_id.asUInt, 0.U(9.W), Seq(
    "b10_0000".U -> io.in.fromLfb.st_dirty_idx,
    "b01_0000".U -> io.in.fromVb.st_dirty_idx,
    "b00_1000".U -> io.in.fromSnq.st_dirty_idx,
    "b00_0100".U -> io.in.fromWmb.st_dirty_idx,
    "b00_0010".U -> io.in.fromIcc.st_dirty_idx,
    "b00_0001".U -> io.in.fromStoreAG.st_dirty_idx
  ))

  io.out.st_dirty.din :=
    Mux(dcache_arb_lfb_st_dp_sel,  io.in.fromLfb.st_dirty_din, 0.U(7.W)) |
    Mux(dcache_arb_vb_st_dp_sel,   io.in.fromVb.st_dirty_din,  0.U(7.W)) |
    Mux(dcache_arb_snq_st_dp_sel,  io.in.fromSnq.st_dirty_din, 0.U(7.W)) |
    Mux(dcache_arb_wmb_st_dp_sel,  io.in.fromWmb.st_dirty_din, 0.U(7.W)) |
    Mux(dcache_arb_icc_st_dp_sel,  io.in.fromIcc.st_dirty_din, 0.U(7.W))

  val lsu_dcache_st_dirty_gwen =
    dcache_arb_lfb_st_dp_sel ||
    dcache_arb_vb_st_dp_sel    &&  io.in.fromVb.st_dirty_gwen ||
    dcache_arb_snq_st_dp_sel   &&  io.in.fromSnq.st_dirty_gwen ||
    dcache_arb_wmb_st_dp_sel ||
    dcache_arb_icc_st_dp_sel   &&  io.in.fromIcc.st_dirty_gwen

  io.out.st_dirty.gwen_b := !lsu_dcache_st_dirty_gwen

  val lsu_dcache_st_dirty_wen =
    Mux(dcache_arb_lfb_st_dp_sel,  io.in.fromLfb.st_dirty_wen, 0.U(7.W)) |
    Mux(dcache_arb_vb_st_dp_sel,   io.in.fromVb.st_dirty_wen,  0.U(7.W)) |
    Mux(dcache_arb_snq_st_dp_sel,  io.in.fromSnq.st_dirty_wen, 0.U(7.W)) |
    Mux(dcache_arb_wmb_st_dp_sel,  io.in.fromWmb.st_dirty_wen, 0.U(7.W)) |
    Mux(dcache_arb_icc_st_dp_sel,  io.in.fromIcc.st_dirty_wen, 0.U(7.W))

  io.out.st_dirty.wen_b := !lsu_dcache_st_dirty_wen

  //==========================================================
  //        Dcache write port information
  //==========================================================
  io.out.dcache_dirty_gwen :=
    dcache_arb_lfb_st_sel && io.in.fromLfb.st_dirty_req ||
    dcache_arb_vb_st_sel && io.in.fromVb.st_dirty_req && io.in.fromVb.st_dirty_gwen ||
    dcache_arb_snq_st_sel && io.in.fromSnq.st_dirty_req && io.in.fromSnq.st_dirty_gwen ||
    dcache_arb_wmb_st_sel && io.in.fromWmb.st_dirty_req

  io.out.dcache_snq_st_sel := dcache_arb_snq_st_sel

  io.out.dcache_vb_snq_gwen :=
    dcache_arb_vb_st_sel && io.in.fromVb.st_dirty_req && io.in.fromVb.st_dirty_gwen ||
    dcache_arb_snq_st_sel && io.in.fromSnq.st_dirty_req && io.in.fromSnq.st_dirty_gwen

  io.out.dcache_tag_gwen := dcache_arb_lfb_st_sel && io.in.fromLfb.st_tag_req

  //ATTENTION:there are 9 bits idx in dcache 32K, for dcwp hit, it must compare
  //only 8 bits in 32K and 9 bits in 64K
  when(dcache_arb_lfb_st_dp_sel){
    io.out.dcache_idx       := io.in.fromLfb.st_dirty_idx
    io.out.dcache_dirty_din := io.in.fromLfb.st_dirty_din
    io.out.dcache_dirty_wen := io.in.fromLfb.st_dirty_wen
  }.elsewhen(dcache_arb_vb_st_dp_sel){
    io.out.dcache_idx       := io.in.fromVb.st_dirty_idx
    io.out.dcache_dirty_din := io.in.fromVb.st_dirty_din
    io.out.dcache_dirty_wen := io.in.fromVb.st_dirty_wen
  }.elsewhen(dcache_arb_snq_st_dp_sel){
    io.out.dcache_idx       := io.in.fromSnq.st_dirty_idx
    io.out.dcache_dirty_din := io.in.fromSnq.st_dirty_din
    io.out.dcache_dirty_wen := io.in.fromSnq.st_dirty_wen
  }.otherwise{
    io.out.dcache_idx       := io.in.fromWmb.st_dirty_idx
    io.out.dcache_dirty_din := io.in.fromWmb.st_dirty_din
    io.out.dcache_dirty_wen := io.in.fromWmb.st_dirty_wen
  }

  io.out.dcache_tag_din := io.out.st_tag.din
  io.out.dcache_tag_wen := lsu_dcache_st_tag_wen

}
