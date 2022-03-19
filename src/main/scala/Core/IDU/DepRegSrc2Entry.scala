package Core.IDU
import chisel3._
import chisel3.util._

class CreateDepData extends Bundle{
  val lsu_match = Bool()
  val mla_rdy   = Bool()
  val preg      = UInt(7.W)
  val wb        = Bool()
  val rdy       = Bool()
}

class ReadDepData extends Bundle{
  val lsu_match      = Bool()
  val rdy_for_bypass = Bool()
  val rdy_for_issue  = Bool()
  val mla_rdy        = Bool()
  val preg           = UInt(7.W)
  val wb             = Bool()
  val rdy            = Bool()
}

class DepRegSrc2EntryInput extends Bundle {
  val alu_reg_fwd_vld = Vec(2, Bool())
  val fromCp0 = new Bundle{
    val IcgEn = Bool()
    val yyClkEn = Bool()
  }
  val fromRf = new Bundle{
    val pipe_preg_lch_vld_dupx = Vec(2, Bool())//pipe0 pipe1
    val pipe_dst_preg_dupx    = Vec(2, UInt(7.W))
  }
  val fromIU = new Bundle{
    val div_inst_vld                 = Bool()
    val div_preg_dupx                = UInt(7.W)
    val ex2_pipe0_wb_preg_dupx       = UInt(7.W)
    val ex2_pipe0_wb_preg_vld_dupx   = Bool()
    val ex2_pipe1_mult_inst_vld_dupx = Bool()
    val ex2_pipe1_preg_dupx          = UInt(7.W)
    val ex2_pipe1_wb_preg_dupx       = UInt(7.W)
    val ex2_pipe1_wb_preg_vld_dupx   = Bool()
  }
  val fromLSU = new Bundle{
    val ag_pipe3_load_inst_vld      = Bool()
    val ag_pipe3_preg_dupx          = UInt(7.W)
    val dc_pipe3_load_inst_vld_dupx = Bool()
    val dc_pipe3_preg_dupx          = UInt(7.W)
    val wb_pipe3_wb_preg_dupx       = UInt(7.W)
    val wb_pipe3_wb_preg_vld_dupx   = Bool()
  }
  val lsu_dc_pipe3_load_fwd_inst_vld_dupx = Bool()
  val mla_reg_fwd_vld = Bool()
  val pad_yy_icg_scan_en = Bool()
  val fromRTU = new Bundle{
    val flush_fe = Bool()
    val flush_is = Bool()
  }
  val fromVFPU = new Bundle{
    val ex1_pipe_mfvr_inst_vld_dupx = Vec(2, Bool())//pipe6 pipe7
    val ex1_pipe_preg_dupx          = Vec(2, UInt(7.W))
  }
  val createData = new CreateDepData
  val entry_mla = Bool()
  val rdy_clr = Bool()
  val write_en = Bool()
}

class DepRegSrc2EntryIO extends Bundle{
  val in = Input(new DepRegSrc2EntryInput)
  val read_data = Output(new ReadDepData)
}

class DepRegSrc2Entry extends Module{
  val io = IO(new DepRegSrc2EntryIO)

  val rdy       = RegInit(true.B)
  val mla_rdy   = RegInit(true.B)
  val wb        = RegInit(true.B)
  val lsu_match = RegInit(false.B)
  val preg      = RegInit(0.U(7.W))

  val mla_issue_data_ready = Wire(Bool())

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================

  //==========================================================
  //                  Create and Read Bus
  //==========================================================

  //==========================================================
  //                       Ready Bit
  //==========================================================
  //ready bit shows the result of source is predicted to be ready:
  //1 stands for the result may be forwarded
  val alu_data_ready = Wire(Vec(2, Bool()))
  for(i <- 0 until 2){
    alu_data_ready(i) := io.in.fromRf.pipe_preg_lch_vld_dupx(i) &&
                      io.in.fromRf.pipe_dst_preg_dupx(i) === preg
  }
  val mult_data_ready = io.in.fromIU.ex2_pipe1_mult_inst_vld_dupx &&
                        io.in.fromIU.ex2_pipe1_preg_dupx === preg
  val div_data_ready  = io.in.fromIU.div_inst_vld &&
                        io.in.fromIU.div_preg_dupx === preg
  val load_data_ready = io.in.fromLSU.dc_pipe3_load_inst_vld_dupx &&
                        io.in.fromLSU.dc_pipe3_preg_dupx === preg
  val vfpu_data_ready = Wire(Vec(2, Bool()))
  for(i <- 0 until 2){
    vfpu_data_ready(i) := io.in.fromVFPU.ex1_pipe_mfvr_inst_vld_dupx(i) &&
                          io.in.fromVFPU.ex1_pipe_preg_dupx(i) === preg
  }

  val load_issue_data_ready = io.in.lsu_dc_pipe3_load_fwd_inst_vld_dupx && lsu_match

  val data_ready = alu_data_ready.asUInt.orR || mult_data_ready || div_data_ready || load_data_ready || vfpu_data_ready.asUInt.orR
  //prepare wake up signal
  val wake_up = wb
  val rdy_clear = io.in.rdy_clr

  //1.if ready is already be 1, just hold 1
  //2.if producer are presumed to produce the result two cycles later,
  //  set ready to 1
  //3.if producer wake up, set ready to 1
  //4.clear ready to 0
  val rdy_update = (rdy || data_ready || wake_up) && !rdy_clear
  //ready read signal
  io.read_data.rdy := rdy_update
  //the following signals are for Issue Queue bypass/issue logic
  io.read_data.rdy_for_issue := rdy || mla_rdy || io.in.alu_reg_fwd_vld.asUInt.orR || load_issue_data_ready || mla_issue_data_ready
  io.read_data.rdy_for_bypass := rdy

  when(io.in.fromRTU.flush_fe || io.in.fromRTU.flush_is){
    rdy := true.B
  }.elsewhen(io.in.write_en){
    rdy := io.in.createData.rdy
  }.otherwise{
    rdy := rdy_update
  }

  //==========================================================
  //              Multiply Accumulate Ready Bit
  //==========================================================
  //ready bit for mla inst, ready eariler than normal ready

  //-------------Update value of Ready Bit--------------------
  //prepare data_ready signal
  mla_issue_data_ready := io.in.entry_mla && io.in.mla_reg_fwd_vld
  val mla_data_ready = mla_issue_data_ready

  //1.if ready is already be 1, just hold 1
  //2.if producer are presumed to produce the result two cycles later,
  //  set ready to 1
  //3.if producer wake up, set ready to 1
  //4.clear ready to 0
  val mla_rdy_update = (mla_rdy || mla_data_ready || wake_up) && !rdy_clear
  io.read_data.mla_rdy := mla_rdy_update

  when(io.in.fromRTU.flush_fe || io.in.fromRTU.flush_is){
    mla_rdy := true.B
  }.elsewhen(io.in.write_en){
    mla_rdy := io.in.createData.mla_rdy
  }.otherwise{
    mla_rdy := mla_rdy_update
  }

  //==========================================================
  //                     Write Back Valid
  //==========================================================
  //write back valid shows whether the result is written back
  //into PRF : 1 stands for the result is in PRF

  //-------------Update value of Write Back Bit---------------
  //prepare write back signal
  val pipe_wb = Wire(Vec(3, Bool()))
  pipe_wb(0) := io.in.fromIU.ex2_pipe0_wb_preg_vld_dupx && io.in.fromIU.ex2_pipe0_wb_preg_dupx === preg
  pipe_wb(1) := io.in.fromIU.ex2_pipe1_wb_preg_vld_dupx && io.in.fromIU.ex2_pipe1_wb_preg_dupx === preg
  pipe_wb(2) := io.in.fromLSU.wb_pipe3_wb_preg_vld_dupx && io.in.fromLSU.wb_pipe3_wb_preg_dupx === preg

  val wb_update = wb || pipe_wb.asUInt.orR
  io.read_data.wb := wb_update
  when(io.in.fromRTU.flush_fe || io.in.fromRTU.flush_is){
    wb := true.B
  }.elsewhen(io.in.write_en){
    wb := io.in.createData.wb
  }.otherwise{
    wb := wb_update
  }

  //==========================================================
  //              LSU reg Match for Bypass Ready
  //==========================================================
  val lsu_match_update = io.in.fromLSU.ag_pipe3_load_inst_vld && io.in.fromLSU.ag_pipe3_preg_dupx === preg
  io.read_data.lsu_match := lsu_match_update

  when(io.in.fromRTU.flush_fe || io.in.fromRTU.flush_is){
    lsu_match := false.B
  }.elsewhen(io.in.write_en){
    lsu_match := io.in.createData.lsu_match
  }.otherwise{
    lsu_match := lsu_match_update
  }
  //==========================================================
  //                         Preg
  //==========================================================
  io.read_data.preg := preg
  when(io.in.write_en){
    preg := io.in.createData.preg
  }
}
