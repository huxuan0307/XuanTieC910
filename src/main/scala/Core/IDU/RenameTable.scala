package Core.IDU
import chisel3._
import chisel3.util._

class MovDstInfo extends Bundle{
  val rdy = Bool()
  val mla_rdy = Bool()
  val wb = Bool()
  val preg = UInt(7.W)
  val match_inst0 = Bool()
  val match_inst1 = Bool()
  val match_inst2 = Bool()
}


class RenameInput extends Bundle{
  val fromCp0 = new Bundle{
    val IcgEn = Bool()
    val yyClkEn = Bool()
  }
  val ir_stall = Bool()
  val fromRf = new Bundle{
    val pipe_preg_lch_vld_dupx = Vec(2, Bool())//pipe0 pipe1
    val pipe_dst_preg_dupx    = Vec(2, UInt(7.W))
  }
  val rt_req = new RT_Req
  val ifu_xx_sync_reset = Bool()
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
  val pad_yy_icg_scan_en = Bool()
  val fromRTU = new Bundle{
    val flush_fe = Bool()
    val flush_is = Bool()
    val rt_recover_preg = Vec(32, UInt(7.W))
    val yy_xx_flush     = Bool()
  }
  val fromVFPU = new Bundle{
    val ex1_pipe_mfvr_inst_vld_dupx = Vec(2, Bool())//pipe6 pipe7
    val ex1_pipe_preg_dupx          = Vec(2, UInt(7.W))
  }
}

class RenameTableIO extends Bundle{
  val in = Input(new RenameInput)
  val out = Output(new RT_Resp)
}

class RenameTable extends Module{
  val io = IO(new RenameTableIO)

  val rt_recover_updt_vld = WireInit(false.B)
  val ir_stall = io.in.ir_stall
  //==========================================================
  //                   Instance Entries
  //==========================================================
  //------------------------Registers-------------------------
  val reg_rdy_clr = WireInit(VecInit(Seq.fill(33)(false.B)))
  val reg_entry_mla = WireInit(VecInit(Seq.fill(33)(false.B)))
  //--------------------bypass ready signal-------------------
  //ir insts do not need issue bypass singal, only is insts need
  //set bypass ready to 0 for timing
  val alu_reg_fwd_vld = WireInit(VecInit(Seq.fill(2)(false.B)))
  val mla_reg_fwd_vld = WireInit(false.B)
  val lsu_idu_dc_pipe3_load_fwd_inst_vld_dupx = WireInit(false.B)

  //------------------------x0 entry--------------------------
  // &Force ("nonport","reg_0_create_data"); @139
  // &Force ("nonport","reg_0_gateclk_idx_write_en"); @140
  // &Force ("nonport","reg_0_gateclk_write_en"); @141
  // &Force ("nonport","reg_0_rdy_clr"); @142
  // &Force ("nonport","reg_0_write_en"); @143
  // &Force ("nonport","reg_0_entry_mla"); @144
  //treat x0 always ready and wb
  val reg_creat_data = WireInit(VecInit(Seq.fill(33)(0.U.asTypeOf(new CreateDepData))))
  val reg_write_en = WireInit(VecInit(Seq.fill(33)(false.B)))
  val reg_read_data = WireInit(VecInit(Seq.fill(33)(0.U.asTypeOf(new ReadDepData))))
  reg_read_data(0) := "b0111000000011".U.asTypeOf(new ReadDepData)

  val rename_table = Seq.fill(32)(Module(new DepRegSrc2Entry))
  for(i <- 0 until 32){
    rename_table(i).io.in.alu_reg_fwd_vld    := alu_reg_fwd_vld
    rename_table(i).io.in.fromCp0            := io.in.fromCp0
    rename_table(i).io.in.fromRf             := io.in.fromRf
    rename_table(i).io.in.fromIU             := io.in.fromIU
    rename_table(i).io.in.fromLSU            := io.in.fromLSU
    rename_table(i).io.in.lsu_dc_pipe3_load_fwd_inst_vld_dupx := lsu_idu_dc_pipe3_load_fwd_inst_vld_dupx
    rename_table(i).io.in.mla_reg_fwd_vld    := mla_reg_fwd_vld
    rename_table(i).io.in.pad_yy_icg_scan_en := io.in.pad_yy_icg_scan_en
    rename_table(i).io.in.fromRTU.flush_fe   := io.in.fromRTU.flush_fe
    rename_table(i).io.in.fromRTU.flush_is   := io.in.fromRTU.flush_is
    rename_table(i).io.in.fromVFPU           := io.in.fromVFPU

    rename_table(i).io.in.createData := reg_creat_data(i+1)
    rename_table(i).io.in.entry_mla  := reg_entry_mla(i+1)
    rename_table(i).io.in.rdy_clr    := reg_rdy_clr(i+1)
    rename_table(i).io.in.write_en   := reg_write_en(i+1)

    reg_read_data(i+1) := rename_table(i).io.read_data
  }

  //==========================================================
  //                      Write Port
  //==========================================================
  //--------------------ir inst write enable------------------
  val inst_dst_reg_OH = Wire(Vec(4, UInt(32.W)))
  for(i <- 0 until 4){
    inst_dst_reg_OH(i) := UIntToOH(io.in.rt_req.instInfo(i).bits.dst_reg(4,0))(31,0)
  }
  val inst_write_en            = Wire(Vec(4, Bool()))
  val reg_write_en_vec         = Wire(Vec(4, UInt(33.W)))
  val inst_gateclk_write_en    = Wire(Vec(4, Bool()))
  val reg_gateclk_write_en_vec = Wire(Vec(4, UInt(33.W)))
  for(i <- 0 until 4){
    inst_write_en(i) := io.in.rt_req.instInfo(i).valid && !ir_stall && !rt_recover_updt_vld && io.in.rt_req.instInfo(i).bits.dst_vld
    reg_write_en_vec(i) := Cat(Mux(inst_write_en(i) && io.in.rt_req.instInfo(i).bits.dst_reg(5) ,inst_dst_reg_OH(i)(0), 0.U(1.W)),
      Mux(inst_write_en(i) && !io.in.rt_req.instInfo(i).bits.dst_reg(5) ,inst_dst_reg_OH(i), 0.U(32.W)))
    //reg_write_en_vec(i)(32)   := Mux(inst_write_en(i) && io.in.instReq(i).bits.dst_reg(5) ,inst_dst_reg_OH(i)(0), 0.U(1.W))

    inst_gateclk_write_en(i) := io.in.rt_req.instInfo(i).valid && !rt_recover_updt_vld && io.in.rt_req.instInfo(i).bits.dst_vld
    reg_gateclk_write_en_vec(i) := Cat(Mux(inst_gateclk_write_en(i) && io.in.rt_req.instInfo(i).bits.dst_reg(5) ,inst_dst_reg_OH(i)(0), 0.U(1.W)),
      Mux(inst_gateclk_write_en(i) && !io.in.rt_req.instInfo(i).bits.dst_reg(5) ,inst_dst_reg_OH(i), 0.U(32.W)))
    //reg_gateclk_write_en_vec(i)(32)   := Mux(inst_gateclk_write_en(i) && io.in.instReq(i).bits.dst_reg(5) ,inst_dst_reg_OH(i)(0), 0.U(1.W))
  }

  //-------------flush and reset write enable-----------------
  //reset: build initial mappings (r0~r31 <-> p0~p31)
  //flush: recover mappings from rtu pst
  val rt_reset_updt_preg = WireInit(VecInit((0 until 32).map(_.U(7.W))))
  rt_recover_updt_vld := io.in.ifu_xx_sync_reset || io.in.fromRTU.yy_xx_flush
  val rt_recover_updt_preg = Mux(io.in.ifu_xx_sync_reset, rt_reset_updt_preg, io.in.fromRTU.rt_recover_preg)

  //-----------------------Write value------------------------
  //generate write enable signal
  for(i <- 0 until 33){
    reg_write_en(i) := rt_recover_updt_vld ||
                       reg_write_en_vec(0)(i) || reg_write_en_vec(1)(i) ||
                       reg_write_en_vec(2)(i) || reg_write_en_vec(3)(i)
  }

  //------------------------write data------------------------
  //preg valid always initial as 0, except recover from pst
  //the write back data path use gateclk wen, ignoring id stall
  //priority is 3>2>1>0>pst_update
  for(i <- 0 until 32){
    reg_creat_data(i).lsu_match := false.B
    reg_creat_data(i).mla_rdy   := false.B
    reg_creat_data(i).preg := PriorityMux(Seq(
      reg_gateclk_write_en_vec(3)(i) -> io.in.rt_req.instInfo(3).bits.dst_preg,
      reg_gateclk_write_en_vec(2)(i) -> io.in.rt_req.instInfo(2).bits.dst_preg,
      reg_gateclk_write_en_vec(1)(i) -> io.in.rt_req.instInfo(1).bits.dst_preg,
      reg_gateclk_write_en_vec(0)(i) -> io.in.rt_req.instInfo(0).bits.dst_preg,
      true.B                         -> rt_recover_updt_preg(i)
    ))
    reg_creat_data(i).wb  := rt_recover_updt_vld
    reg_creat_data(i).rdy := rt_recover_updt_vld
  }
  reg_creat_data(32).lsu_match := false.B
  reg_creat_data(32).mla_rdy   := false.B
  reg_creat_data(32).preg := PriorityMux(Seq(
    reg_gateclk_write_en_vec(3)(32) -> io.in.rt_req.instInfo(3).bits.dst_preg,
    reg_gateclk_write_en_vec(2)(32) -> io.in.rt_req.instInfo(2).bits.dst_preg,
    reg_gateclk_write_en_vec(1)(32) -> io.in.rt_req.instInfo(1).bits.dst_preg,
    reg_gateclk_write_en_vec(0)(32) -> io.in.rt_req.instInfo(0).bits.dst_preg,
    true.B                         -> 0.U(7.W)
  ))
  reg_creat_data(32).wb  := rt_recover_updt_vld
  reg_creat_data(32).rdy := rt_recover_updt_vld

  //==========================================================
  //                       Read Port
  //==========================================================
  //val inst_src_match = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(4)(0.U.asTypeOf(new srcMatch))))))
  val inst_src0_read_data = Wire(Vec(4, new ReadDepData))
  val inst_src1_read_data = Wire(Vec(4, new ReadDepData))
  val inst_dst_read_data  = Wire(Vec(4, new ReadDepData))
  for(i <- 0 until 4){
    inst_src0_read_data(i) := MuxLookup(io.in.rt_req.instInfo(i).bits.src_reg(0),
      0.U.asTypeOf(new ReadDepData), (0 until 33).map(_.U).zip(reg_read_data))
    inst_src1_read_data(i) := MuxLookup(io.in.rt_req.instInfo(i).bits.src_reg(1),
      0.U.asTypeOf(new ReadDepData), (0 until 33).map(_.U).zip(reg_read_data))
    inst_dst_read_data(i) := MuxLookup(io.in.rt_req.instInfo(i).bits.dst_reg,
      0.U.asTypeOf(new ReadDepData), (0 until 33).map(_.U).zip(reg_read_data))
  }


  val inst_mov_dst = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new MovDstInfo))))

  //-----------------instruction 0 source 0-------------------
  io.out.instInfo(0).src0_data.rdy  := inst_src0_read_data(0).rdy || !io.in.rt_req.instInfo(0).bits.src_vld(0)
  io.out.instInfo(0).src0_data.wb   := inst_src0_read_data(0).wb  || !io.in.rt_req.instInfo(0).bits.src_vld(0)
  io.out.instInfo(0).src0_data.preg := inst_src0_read_data(0).preg

  inst_mov_dst(0).rdy     := inst_src0_read_data(0).rdy
  inst_mov_dst(0).mla_rdy := inst_src0_read_data(0).mla_rdy
  inst_mov_dst(0).wb      := inst_src0_read_data(0).wb
  inst_mov_dst(0).preg    := inst_src0_read_data(0).preg
  //-----------------instruction 0 source 1-------------------
  io.out.instInfo(0).src1_data.rdy  := inst_src1_read_data(0).rdy || !io.in.rt_req.instInfo(0).bits.src_vld(1)
  io.out.instInfo(0).src1_data.wb   := inst_src1_read_data(0).wb  || !io.in.rt_req.instInfo(0).bits.src_vld(1)
  io.out.instInfo(0).src1_data.preg := inst_src1_read_data(0).preg
  //---------instruction 0 src2/dest reg (for release)--------
  io.out.instInfo(0).src2_data.rdy  := inst_dst_read_data(0).rdy || !io.in.rt_req.instInfo(0).bits.src_vld(2)
  io.out.instInfo(0).src2_data.mla_rdy := inst_dst_read_data(0).mla_rdy && io.in.rt_req.instInfo(0).bits.mla || !io.in.rt_req.instInfo(0).bits.src_vld(2)
  io.out.instInfo(0).src2_data.wb   := inst_dst_read_data(0).wb  || !io.in.rt_req.instInfo(0).bits.src_vld(2)
  io.out.instInfo(0).src2_data.preg := inst_dst_read_data(0).preg

  io.out.instInfo(0).rel_preg := Mux(io.in.rt_req.instInfo(0).bits.dst_reg(5), io.in.rt_req.instInfo(0).bits.dst_preg, inst_dst_read_data(0).preg)

  //-----------------instruction 1 source 0-------------------
  val inst1_src0_match_inst0 =
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.src_vld(0) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(1).bits.src_reg(0) &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst01_src0_mask

  when(inst1_src0_match_inst0 && io.in.rt_req.instInfo(0).bits.mov){
    io.out.instInfo(1).src0_data.rdy  := inst_mov_dst(0).rdy
    io.out.instInfo(1).src0_data.wb   := inst_mov_dst(0).wb
    io.out.instInfo(1).src0_data.preg := inst_mov_dst(0).preg

    io.out.srcMatch(0).src0 := false.B
  }.elsewhen(inst1_src0_match_inst0){
    io.out.instInfo(1).src0_data.rdy  := false.B
    io.out.instInfo(1).src0_data.wb   := false.B
    io.out.instInfo(1).src0_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(0).src0 := true.B
  }.otherwise{
    io.out.instInfo(1).src0_data.rdy  := inst_src0_read_data(1).rdy || !io.in.rt_req.instInfo(1).bits.src_vld(0)
    io.out.instInfo(1).src0_data.wb   := inst_src0_read_data(1).wb  || !io.in.rt_req.instInfo(1).bits.src_vld(0)
    io.out.instInfo(1).src0_data.preg := inst_src0_read_data(1).preg

    io.out.srcMatch(0).src0 := false.B
  }
  //if inst1 is mov, bypass src0 dep info to inst in same packet
  when(inst1_src0_match_inst0){
    inst_mov_dst(1).rdy     := false.B
    inst_mov_dst(1).mla_rdy := false.B
    inst_mov_dst(1).wb      := false.B
    inst_mov_dst(1).preg    := io.in.rt_req.instInfo(0).bits.dst_preg
    inst_mov_dst(1).match_inst0 := true.B
  }.otherwise{
    inst_mov_dst(1).rdy     := inst_src0_read_data(1).rdy
    inst_mov_dst(1).mla_rdy := inst_src0_read_data(1).mla_rdy
    inst_mov_dst(1).wb      := inst_src0_read_data(1).wb
    inst_mov_dst(1).preg    := inst_src0_read_data(1).preg
    inst_mov_dst(1).match_inst0 := false.B
  }

  //-----------------instruction 1 source 1-------------------
  val inst1_src1_match_inst0 =
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.src_vld(1) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(1).bits.src_reg(1) &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst01_src1_mask

  when(inst1_src1_match_inst0 && io.in.rt_req.instInfo(0).bits.mov){
    io.out.instInfo(1).src1_data.rdy  := inst_mov_dst(0).rdy
    io.out.instInfo(1).src1_data.wb   := inst_mov_dst(0).wb
    io.out.instInfo(1).src1_data.preg := inst_mov_dst(0).preg

    io.out.srcMatch(0).src1 := false.B
  }.elsewhen(inst1_src1_match_inst0){
    io.out.instInfo(1).src1_data.rdy  := false.B
    io.out.instInfo(1).src1_data.wb   := false.B
    io.out.instInfo(1).src1_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(0).src1 := true.B
  }.otherwise{
    io.out.instInfo(1).src1_data.rdy  := inst_src1_read_data(1).rdy || !io.in.rt_req.instInfo(1).bits.src_vld(1)
    io.out.instInfo(1).src1_data.wb   := inst_src1_read_data(1).wb  || !io.in.rt_req.instInfo(1).bits.src_vld(1)
    io.out.instInfo(1).src1_data.preg := inst_src1_read_data(1).preg

    io.out.srcMatch(0).src1 := false.B
  }

  //---------instruction 1 src2/dest reg (for release)--------
  val inst1_src2_match_inst0 =
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.src_vld(2) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(1).bits.dst_reg &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U && (!io.in.rt_req.depInfo.inst01_src0_mask || !io.in.rt_req.depInfo.inst01_src1_mask)

  when(inst1_src2_match_inst0 && io.in.rt_req.instInfo(0).bits.mov){
    io.out.instInfo(1).src2_data.rdy     := inst_mov_dst(0).rdy
    io.out.instInfo(1).src2_data.mla_rdy := inst_mov_dst(0).mla_rdy
    io.out.instInfo(1).src2_data.wb      := inst_mov_dst(0).wb
    io.out.instInfo(1).src2_data.preg    := inst_mov_dst(0).preg

    io.out.srcMatch(0).src2 := false.B
  }.elsewhen(inst1_src2_match_inst0){
    io.out.instInfo(1).src2_data.rdy     := false.B
    io.out.instInfo(1).src2_data.mla_rdy := false.B
    io.out.instInfo(1).src2_data.wb      := false.B
    io.out.instInfo(1).src2_data.preg    := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(0).src2 := true.B
  }.otherwise{
    io.out.instInfo(1).src2_data.rdy     := inst_dst_read_data(1).rdy || !io.in.rt_req.instInfo(1).bits.src_vld(2)
    io.out.instInfo(1).src2_data.mla_rdy := inst_dst_read_data(1).mla_rdy && io.in.rt_req.instInfo(1).bits.mla || !io.in.rt_req.instInfo(1).bits.src_vld(2)
    io.out.instInfo(1).src2_data.wb      := inst_dst_read_data(1).wb  || !io.in.rt_req.instInfo(1).bits.src_vld(2)
    io.out.instInfo(1).src2_data.preg    := inst_dst_read_data(1).preg

    io.out.srcMatch(0).src2 := false.B
  }

  val inst1_dst_match_inst0 =
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(1).bits.dst_reg &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U

  when(io.in.rt_req.instInfo(1).bits.dst_reg(5)){
    io.out.instInfo(1).rel_preg := io.in.rt_req.instInfo(1).bits.dst_preg
  }.elsewhen(inst1_dst_match_inst0){
    io.out.instInfo(1).rel_preg := io.in.rt_req.instInfo(0).bits.dst_preg
  }.otherwise{
    io.out.instInfo(1).rel_preg := inst_dst_read_data(1).preg
  }

  //-----------------instruction 2 source 0-------------------
  val inst2_src0_match_inst0 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.src_vld(0) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(2).bits.src_reg(0) &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst02_preg_mask

  val inst2_src0_match_inst1 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.src_vld(0) &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(2).bits.src_reg(0) &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst12_src0_mask

  val inst0_mov_bypass_over_inst1 = io.in.rt_req.instInfo(0).bits.mov &&
    !(io.in.rt_req.instInfo(1).bits.dst_vld && io.in.rt_req.instInfo(0).bits.src_reg(0) === io.in.rt_req.instInfo(1).bits.dst_reg)

  when(inst2_src0_match_inst1 && io.in.rt_req.instInfo(1).bits.mov){
    io.out.instInfo(2).src0_data.rdy  := inst_mov_dst(1).rdy
    io.out.instInfo(2).src0_data.wb   := inst_mov_dst(1).wb
    io.out.instInfo(2).src0_data.preg := inst_mov_dst(1).preg

    io.out.srcMatch(3).src0 := false.B //12
    io.out.srcMatch(1).src0 := inst_mov_dst(1).match_inst0 //02
  }.elsewhen(inst2_src0_match_inst1){
    io.out.instInfo(2).src0_data.rdy  := false.B
    io.out.instInfo(2).src0_data.wb   := false.B
    io.out.instInfo(2).src0_data.preg := io.in.rt_req.instInfo(1).bits.dst_preg

    io.out.srcMatch(3).src0 := true.B
    io.out.srcMatch(1).src0 := false.B
  }.elsewhen(inst2_src0_match_inst0 && inst0_mov_bypass_over_inst1){
    io.out.instInfo(2).src0_data.rdy  := inst_mov_dst(0).rdy
    io.out.instInfo(2).src0_data.wb   := inst_mov_dst(0).wb
    io.out.instInfo(2).src0_data.preg := inst_mov_dst(0).preg

    io.out.srcMatch(3).src0 := false.B
    io.out.srcMatch(1).src0 := false.B
  }.elsewhen(inst2_src0_match_inst0){
    io.out.instInfo(2).src0_data.rdy  := false.B
    io.out.instInfo(2).src0_data.wb   := false.B
    io.out.instInfo(2).src0_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(3).src0 := false.B
    io.out.srcMatch(1).src0 := true.B
  }.otherwise{
    io.out.instInfo(2).src0_data.rdy  := inst_src0_read_data(2).rdy || !io.in.rt_req.instInfo(2).bits.src_vld(0)
    io.out.instInfo(2).src0_data.wb   := inst_src0_read_data(2).wb  || !io.in.rt_req.instInfo(2).bits.src_vld(0)
    io.out.instInfo(2).src0_data.preg := inst_src0_read_data(2).preg

    io.out.srcMatch(3).src0 := false.B
    io.out.srcMatch(1).src0 := false.B
  }

  //if inst2 is mov, bypass src0 dep info to inst in same packet
  when(io.in.rt_req.instInfo(2).bits.mov && inst2_src0_match_inst1){
    inst_mov_dst(2).rdy     := false.B
    inst_mov_dst(2).mla_rdy := false.B
    inst_mov_dst(2).wb      := false.B
    inst_mov_dst(2).preg    := io.in.rt_req.instInfo(1).bits.dst_preg

    inst_mov_dst(2).match_inst0 := false.B
    inst_mov_dst(2).match_inst1 := true.B
    inst_mov_dst(2).match_inst2 := false.B
  }.elsewhen(io.in.rt_req.instInfo(2).bits.mov && inst2_src0_match_inst0){
    inst_mov_dst(2).rdy     := false.B
    inst_mov_dst(2).mla_rdy := false.B
    inst_mov_dst(2).wb      := false.B
    inst_mov_dst(2).preg    := io.in.rt_req.instInfo(0).bits.dst_preg

    inst_mov_dst(2).match_inst0 := true.B
    inst_mov_dst(2).match_inst1 := false.B
    inst_mov_dst(2).match_inst2 := false.B
  }.elsewhen(io.in.rt_req.instInfo(2).bits.mov){
    inst_mov_dst(2).rdy     := inst_src0_read_data(2).rdy
    inst_mov_dst(2).mla_rdy := inst_src0_read_data(2).mla_rdy
    inst_mov_dst(2).wb      := inst_src0_read_data(2).wb
    inst_mov_dst(2).preg    := inst_src0_read_data(2).preg

    inst_mov_dst(2).match_inst0 := false.B
    inst_mov_dst(2).match_inst1 := false.B
    inst_mov_dst(2).match_inst2 := false.B
  }.otherwise{
    inst_mov_dst(2).rdy     := false.B
    inst_mov_dst(2).mla_rdy := false.B
    inst_mov_dst(2).wb      := false.B
    inst_mov_dst(2).preg    := io.in.rt_req.instInfo(2).bits.dst_preg

    inst_mov_dst(2).match_inst0 := false.B
    inst_mov_dst(2).match_inst1 := false.B
    inst_mov_dst(2).match_inst2 := true.B
  }

  //-----------------instruction 2 source 1-------------------
  val inst2_src1_match_inst0 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.src_vld(1) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(2).bits.src_reg(1) &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst02_preg_mask

  val inst2_src1_match_inst1 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.src_vld(1) &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(2).bits.src_reg(1) &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst12_src1_mask

  when(inst2_src1_match_inst1 && io.in.rt_req.instInfo(1).bits.mov){
    io.out.instInfo(2).src1_data.rdy  := inst_mov_dst(1).rdy
    io.out.instInfo(2).src1_data.wb   := inst_mov_dst(1).wb
    io.out.instInfo(2).src1_data.preg := inst_mov_dst(1).preg

    io.out.srcMatch(3).src1 := false.B //12
    io.out.srcMatch(1).src1 := inst_mov_dst(1).match_inst0 //02
  }.elsewhen(inst2_src1_match_inst1){
    io.out.instInfo(2).src1_data.rdy  := false.B
    io.out.instInfo(2).src1_data.wb   := false.B
    io.out.instInfo(2).src1_data.preg := io.in.rt_req.instInfo(1).bits.dst_preg

    io.out.srcMatch(3).src1 := true.B
    io.out.srcMatch(1).src1 := false.B
  }.elsewhen(inst2_src1_match_inst0 && inst0_mov_bypass_over_inst1){
    io.out.instInfo(2).src1_data.rdy  := inst_mov_dst(0).rdy
    io.out.instInfo(2).src1_data.wb   := inst_mov_dst(0).wb
    io.out.instInfo(2).src1_data.preg := inst_mov_dst(0).preg

    io.out.srcMatch(3).src1 := false.B
    io.out.srcMatch(1).src1 := false.B
  }.elsewhen(inst2_src1_match_inst0){
    io.out.instInfo(2).src1_data.rdy  := false.B
    io.out.instInfo(2).src1_data.wb   := false.B
    io.out.instInfo(2).src1_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(3).src1 := false.B
    io.out.srcMatch(1).src1 := true.B
  }.otherwise{
    io.out.instInfo(2).src1_data.rdy  := inst_src1_read_data(2).rdy || !io.in.rt_req.instInfo(2).bits.src_vld(1)
    io.out.instInfo(2).src1_data.wb   := inst_src1_read_data(2).wb  || !io.in.rt_req.instInfo(2).bits.src_vld(1)
    io.out.instInfo(2).src1_data.preg := inst_src1_read_data(2).preg

    io.out.srcMatch(3).src1 := false.B
    io.out.srcMatch(1).src1 := false.B
  }

  //---------instruction 2 src2/dest reg (for release)--------
  val inst2_src2_match_inst0 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.src_vld(2) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(2).bits.dst_reg &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst02_preg_mask

  val inst2_src2_match_inst1 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.src_vld(2) &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(2).bits.dst_reg &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U && !(io.in.rt_req.depInfo.inst12_src0_mask || io.in.rt_req.depInfo.inst12_src1_mask)

  when(inst2_src2_match_inst1 && io.in.rt_req.instInfo(1).bits.mov){
    io.out.instInfo(2).src2_data.rdy  := inst_mov_dst(1).rdy
    io.out.instInfo(2).src2_data.mla_rdy := inst_mov_dst(1).mla_rdy
    io.out.instInfo(2).src2_data.wb   := inst_mov_dst(1).wb
    io.out.instInfo(2).src2_data.preg := inst_mov_dst(1).preg

    io.out.srcMatch(3).src2 := false.B //12
    io.out.srcMatch(1).src2 := inst_mov_dst(1).match_inst0 //02
  }.elsewhen(inst2_src2_match_inst1){
    io.out.instInfo(2).src2_data.rdy  := false.B
    io.out.instInfo(2).src2_data.mla_rdy := false.B
    io.out.instInfo(2).src2_data.wb   := false.B
    io.out.instInfo(2).src2_data.preg := io.in.rt_req.instInfo(1).bits.dst_preg

    io.out.srcMatch(3).src2 := true.B
    io.out.srcMatch(1).src2 := false.B
  }.elsewhen(inst2_src2_match_inst0 && inst0_mov_bypass_over_inst1){
    io.out.instInfo(2).src2_data.rdy  := inst_mov_dst(0).rdy
    io.out.instInfo(2).src2_data.mla_rdy := inst_mov_dst(0).mla_rdy
    io.out.instInfo(2).src2_data.wb   := inst_mov_dst(0).wb
    io.out.instInfo(2).src2_data.preg := inst_mov_dst(0).preg

    io.out.srcMatch(3).src2 := false.B
    io.out.srcMatch(1).src2 := false.B
  }.elsewhen(inst2_src2_match_inst0){
    io.out.instInfo(2).src2_data.rdy  := false.B
    io.out.instInfo(2).src2_data.mla_rdy := false.B
    io.out.instInfo(2).src2_data.wb   := false.B
    io.out.instInfo(2).src2_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(3).src2 := false.B
    io.out.srcMatch(1).src2 := true.B
  }.otherwise{
    io.out.instInfo(2).src2_data.rdy  := inst_dst_read_data(2).rdy || !io.in.rt_req.instInfo(2).bits.src_vld(2)
    io.out.instInfo(2).src2_data.mla_rdy := inst_dst_read_data(2).mla_rdy && io.in.rt_req.instInfo(2).bits.mla || !io.in.rt_req.instInfo(2).bits.src_vld(2)
    io.out.instInfo(2).src2_data.wb   := inst_dst_read_data(2).wb  || !io.in.rt_req.instInfo(2).bits.src_vld(2)
    io.out.instInfo(2).src2_data.preg := inst_dst_read_data(2).preg

    io.out.srcMatch(3).src2 := false.B
    io.out.srcMatch(1).src2 := false.B
  }

  val inst2_dst_match_inst0 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.dst_vld &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(2).bits.dst_reg &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U

  val inst2_dst_match_inst1 =
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.dst_vld &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(2).bits.dst_reg &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U

  when(io.in.rt_req.instInfo(2).bits.dst_reg(5)){
    io.out.instInfo(2).rel_preg := io.in.rt_req.instInfo(2).bits.dst_preg
  }.elsewhen(inst2_dst_match_inst1){
    io.out.instInfo(2).rel_preg := io.in.rt_req.instInfo(1).bits.dst_preg
  }.elsewhen(inst2_dst_match_inst0){
    io.out.instInfo(2).rel_preg := io.in.rt_req.instInfo(0).bits.dst_preg
  }.otherwise{
    io.out.instInfo(2).rel_preg := inst_dst_read_data(2).preg
  }

  //-----------------instruction 3 source 0-------------------
  val inst3_src0_match_inst0 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(0) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(3).bits.src_reg(0) &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U

  val inst3_src0_match_inst1 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(0) &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(3).bits.src_reg(0) &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst13_preg_mask

  val inst3_src0_match_inst2 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(0) &&
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.dst_vld &&
    io.in.rt_req.instInfo(2).bits.dst_reg === io.in.rt_req.instInfo(3).bits.src_reg(0) &&
    io.in.rt_req.instInfo(2).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst23_src0_mask

  when(inst3_src0_match_inst2){
    io.out.instInfo(3).src0_data.rdy  := inst_mov_dst(2).rdy
    io.out.instInfo(3).src0_data.wb   := inst_mov_dst(2).wb
    io.out.instInfo(3).src0_data.preg := inst_mov_dst(2).preg

    io.out.srcMatch(5).src0 := inst_mov_dst(2).match_inst2
    io.out.srcMatch(4).src0 := inst_mov_dst(2).match_inst1
    io.out.srcMatch(2).src0 := inst_mov_dst(2).match_inst0
  }.elsewhen(inst3_src0_match_inst1){
    io.out.instInfo(3).src0_data.rdy  := false.B
    io.out.instInfo(3).src0_data.wb   := false.B
    io.out.instInfo(3).src0_data.preg := io.in.rt_req.instInfo(1).bits.dst_preg

    io.out.srcMatch(5).src0 := false.B
    io.out.srcMatch(4).src0 := true.B
    io.out.srcMatch(2).src0 := false.B
  }.elsewhen(inst3_src0_match_inst0){
    io.out.instInfo(3).src0_data.rdy  := false.B
    io.out.instInfo(3).src0_data.wb   := false.B
    io.out.instInfo(3).src0_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(5).src0 := false.B
    io.out.srcMatch(4).src0 := false.B
    io.out.srcMatch(2).src0 := true.B
  }.otherwise{
    io.out.instInfo(3).src0_data.rdy  := inst_src0_read_data(3).rdy || !io.in.rt_req.instInfo(3).bits.src_vld(0)
    io.out.instInfo(3).src0_data.wb   := inst_src0_read_data(3).wb  || !io.in.rt_req.instInfo(3).bits.src_vld(0)
    io.out.instInfo(3).src0_data.preg := inst_src0_read_data(3).preg

    io.out.srcMatch(5).src0 := false.B
    io.out.srcMatch(4).src0 := false.B
    io.out.srcMatch(2).src0 := false.B
  }

  //-----------------instruction 3 source 1-------------------
  val inst3_src1_match_inst0 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(1) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(3).bits.src_reg(1) &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U

  val inst3_src1_match_inst1 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(1) &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(3).bits.src_reg(1) &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst13_preg_mask

  val inst3_src1_match_inst2 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(1) &&
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.dst_vld &&
    io.in.rt_req.instInfo(2).bits.dst_reg === io.in.rt_req.instInfo(3).bits.src_reg(1) &&
    io.in.rt_req.instInfo(2).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst23_src1_mask

  when(inst3_src1_match_inst2){
    io.out.instInfo(3).src1_data.rdy  := inst_mov_dst(2).rdy
    io.out.instInfo(3).src1_data.wb   := inst_mov_dst(2).wb
    io.out.instInfo(3).src1_data.preg := inst_mov_dst(2).preg

    io.out.srcMatch(5).src1 := inst_mov_dst(2).match_inst2
    io.out.srcMatch(4).src1 := inst_mov_dst(2).match_inst1
    io.out.srcMatch(2).src1 := inst_mov_dst(2).match_inst0
  }.elsewhen(inst3_src1_match_inst1){
    io.out.instInfo(3).src1_data.rdy  := false.B
    io.out.instInfo(3).src1_data.wb   := false.B
    io.out.instInfo(3).src1_data.preg := io.in.rt_req.instInfo(1).bits.dst_preg

    io.out.srcMatch(5).src1 := false.B
    io.out.srcMatch(4).src1 := true.B
    io.out.srcMatch(2).src1 := false.B
  }.elsewhen(inst3_src1_match_inst0){
    io.out.instInfo(3).src1_data.rdy  := false.B
    io.out.instInfo(3).src1_data.wb   := false.B
    io.out.instInfo(3).src1_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(5).src1 := false.B
    io.out.srcMatch(4).src1 := false.B
    io.out.srcMatch(2).src1 := true.B
  }.otherwise{
    io.out.instInfo(3).src1_data.rdy  := inst_src1_read_data(3).rdy || !io.in.rt_req.instInfo(3).bits.src_vld(1)
    io.out.instInfo(3).src1_data.wb   := inst_src1_read_data(3).wb  || !io.in.rt_req.instInfo(3).bits.src_vld(1)
    io.out.instInfo(3).src1_data.preg := inst_src1_read_data(3).preg

    io.out.srcMatch(5).src1 := false.B
    io.out.srcMatch(4).src1 := false.B
    io.out.srcMatch(2).src1 := false.B
  }

  //---------instruction 3 src2/dest reg (for release)--------
  val inst3_src2_match_inst0 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(2) &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(3).bits.dst_reg &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U

  val inst3_src2_match_inst1 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(2) &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(3).bits.dst_reg &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U && !io.in.rt_req.depInfo.inst13_preg_mask

  val inst3_src2_match_inst2 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.src_vld(2) &&
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.dst_vld &&
    io.in.rt_req.instInfo(2).bits.dst_reg === io.in.rt_req.instInfo(3).bits.dst_reg &&
    io.in.rt_req.instInfo(2).bits.dst_reg =/= 0.U && !(io.in.rt_req.depInfo.inst23_src0_mask || io.in.rt_req.depInfo.inst23_src1_mask)

  when(inst3_src2_match_inst2){
    io.out.instInfo(3).src2_data.rdy  := inst_mov_dst(2).rdy
    io.out.instInfo(3).src2_data.mla_rdy := inst_mov_dst(2).mla_rdy
    io.out.instInfo(3).src2_data.wb   := inst_mov_dst(2).wb
    io.out.instInfo(3).src2_data.preg := inst_mov_dst(2).preg

    io.out.srcMatch(5).src2 := inst_mov_dst(2).match_inst2
    io.out.srcMatch(4).src2 := inst_mov_dst(2).match_inst1
    io.out.srcMatch(2).src2 := inst_mov_dst(2).match_inst0
  }.elsewhen(inst3_src2_match_inst1){
    io.out.instInfo(3).src2_data.rdy  := false.B
    io.out.instInfo(3).src2_data.mla_rdy := false.B
    io.out.instInfo(3).src2_data.wb   := false.B
    io.out.instInfo(3).src2_data.preg := io.in.rt_req.instInfo(1).bits.dst_preg

    io.out.srcMatch(5).src2 := false.B
    io.out.srcMatch(4).src2 := true.B
    io.out.srcMatch(2).src2 := false.B
  }.elsewhen(inst3_src2_match_inst0){
    io.out.instInfo(3).src2_data.rdy  := false.B
    io.out.instInfo(3).src2_data.mla_rdy := false.B
    io.out.instInfo(3).src2_data.wb   := false.B
    io.out.instInfo(3).src2_data.preg := io.in.rt_req.instInfo(0).bits.dst_preg

    io.out.srcMatch(5).src2 := false.B
    io.out.srcMatch(4).src2 := false.B
    io.out.srcMatch(2).src2 := true.B
  }.otherwise{
    io.out.instInfo(3).src2_data.rdy  := inst_dst_read_data(3).rdy || !io.in.rt_req.instInfo(3).bits.src_vld(2)
    io.out.instInfo(3).src2_data.mla_rdy := inst_dst_read_data(3).mla_rdy && io.in.rt_req.instInfo(3).bits.mla || !io.in.rt_req.instInfo(3).bits.src_vld(2)
    io.out.instInfo(3).src2_data.wb   := inst_dst_read_data(3).wb  || !io.in.rt_req.instInfo(3).bits.src_vld(2)
    io.out.instInfo(3).src2_data.preg := inst_dst_read_data(3).preg

    io.out.srcMatch(5).src2 := false.B
    io.out.srcMatch(4).src2 := false.B
    io.out.srcMatch(2).src2 := false.B
  }

  val inst3_dst_match_inst0 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.dst_vld &&
    io.in.rt_req.instInfo(0).valid && io.in.rt_req.instInfo(0).bits.dst_vld &&
    io.in.rt_req.instInfo(0).bits.dst_reg === io.in.rt_req.instInfo(3).bits.dst_reg &&
    io.in.rt_req.instInfo(0).bits.dst_reg =/= 0.U

  val inst3_dst_match_inst1 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.dst_vld &&
    io.in.rt_req.instInfo(1).valid && io.in.rt_req.instInfo(1).bits.dst_vld &&
    io.in.rt_req.instInfo(1).bits.dst_reg === io.in.rt_req.instInfo(3).bits.dst_reg &&
    io.in.rt_req.instInfo(1).bits.dst_reg =/= 0.U

  val inst3_dst_match_inst2 =
    io.in.rt_req.instInfo(3).valid && io.in.rt_req.instInfo(3).bits.dst_vld &&
    io.in.rt_req.instInfo(2).valid && io.in.rt_req.instInfo(2).bits.dst_vld &&
    io.in.rt_req.instInfo(2).bits.dst_reg === io.in.rt_req.instInfo(3).bits.dst_reg &&
    io.in.rt_req.instInfo(2).bits.dst_reg =/= 0.U

  when(io.in.rt_req.instInfo(3).bits.dst_reg(5)){
    io.out.instInfo(3).rel_preg := io.in.rt_req.instInfo(3).bits.dst_preg
  }.elsewhen(inst3_dst_match_inst2){
    io.out.instInfo(3).rel_preg := io.in.rt_req.instInfo(2).bits.dst_preg
  }.elsewhen(inst3_dst_match_inst1){
    io.out.instInfo(3).rel_preg := io.in.rt_req.instInfo(1).bits.dst_preg
  }.elsewhen(inst3_dst_match_inst0){
    io.out.instInfo(3).rel_preg := io.in.rt_req.instInfo(0).bits.dst_preg
  }.otherwise{
    io.out.instInfo(3).rel_preg := inst_dst_read_data(3).preg
  }

}