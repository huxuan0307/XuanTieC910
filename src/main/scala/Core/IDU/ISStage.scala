package Core.IDU
import Core.Config.XLEN
import Core.GlobalConfig.{DifftestEnable, NumFoldMax}
import Core.IntConfig.{InstBits, NumLogicRegsBits, NumPhysicRegsBits}
import Core.ROBConfig.NumCreateEntry
import chisel3._
import chisel3.util._

class ROBData extends Bundle{
  val INSTR           = UInt(32.W)
  val VL_PRED         = Bool() //39
  val VL              = UInt(8.W) //38
  val VEC_DIRTY       = Bool() //30
  val VSETVLI         = Bool() //29
  val VSEW            = UInt(3.W) //28
  val VLMUL           = UInt(2.W) //25
  val NO_SPEC_MISPRED = Bool() //23
  val NO_SPEC_MISS    = Bool() //22
  val NO_SPEC_HIT     = Bool() //21
  val LOAD            = Bool() //20
  val FP_DIRTY        = Bool() //19
  val INST_NUM        = UInt(2.W) //18
  val BKPTB_INST      = Bool() //16
  val BKPTA_INST      = Bool() //15
  val BKPTB_DATA      = Bool() //14
  val BKPTA_DATA      = Bool() //13
  val STORE           = Bool() //12
  val RAS             = Bool() //11
  val PCFIFO          = Bool() //10
  val BJU             = Bool() //9
  val INTMASK         = Bool() //8
  val SPLIT           = Bool() //7
  val PC_OFFSET       = UInt(3.W) //6
  val CMPLT_CNT       = UInt(2.W) //3
  val CMPLT           = Bool() //1
  val VLD             = Bool() //0
  val debug = if (DifftestEnable) new Bundle() {
    val pc    = Vec(NumFoldMax, UInt(XLEN.W))
    val inst  = Vec(NumFoldMax, UInt(InstBits.W))
    val RVC   = Vec(NumFoldMax, Bool())
    val rfwen = Vec(NumFoldMax, Bool())
    val wpdest= Vec(NumFoldMax, UInt(NumPhysicRegsBits.W))
    val wdest = Vec(NumFoldMax, UInt(NumLogicRegsBits.W))
  } else null
}

class VFPU2IS extends Bundle {
  val ex1_data_vld_dupx = Bool()
  val ex1_fmla_data_vld_dupx = Bool()
  val ex1_mfvr_inst_vld_dupx = Bool()
  val ex1_preg_dupx = UInt(7.W)
  val ex1_vreg_dupx = UInt(7.W)
  val ex2_data_vld_dupx = Bool()
  val ex2_fmla_data_vld_dupx = Bool()
  val ex2_vreg_dupx = UInt(7.W)
  val ex3_data_vld_dupx = Bool()
  val ex3_vreg_dupx = UInt(7.W)
  val ex5_wb_vreg_dupx = UInt(7.W)
  val ex5_wb_vreg_vld_dupx = Bool()
}

class ISStageInput extends Bundle {
  val fromCp0 = new Bundle {
    val icgEn = Bool()
    val yyClkEn = Bool()
  }

  val pre_dispatch = new IR_preDispatch

  val ir_pipedown = new Bundle{
    val pipedown = Bool()
    val gateclk = Bool()
    val instVld = Vec(4,Bool())

    val inst_src_match = Vec(6,new ir_srcMatch)
    val instData = Vec(4, new ISData)
  }

  //aiq0 aiq1 biq lsiq sdiq viq0 viq1 vmb
  val iq_cnt_info = Vec(8, new IQCntInfo)

  val iq_create_entry = new Bundle{
    val aiq0_aiq = Vec(2, UInt(8.W))
    val aiq1_aiq = Vec(2, UInt(8.W))
    val biq_aiq = Vec(2, UInt(12.W))
    val lsiq_aiq = Vec(2, UInt(12.W))
    val sdiq_aiq = Vec(2, UInt(12.W))
    val sdiq_dp = Vec(2, UInt(12.W))//the same with sdiq_aiq
    val viq0_viq = Vec(2, UInt(8.W))
    val viq1_viq = Vec(2, UInt(8.W))
  }
  val lsiq_dp_create_bypass_oldest = Bool()
  val lsiq_dp_no_spec_store_vld = Bool()

  val fromRf = new Bundle{
    val preg_lch_vld_dupx = Vec(2, Bool())//pipe0 pipe1
    val dst_preg_dupx    = Vec(2, UInt(7.W))
    val vmla_lch_vld_dupx = Vec(2, Bool())//pipe6 pipe7
    val dst_vreg_dupx    = Vec(2, UInt(7.W))
  }
  val fromLSU = new Bundle {
    val ag_pipe3_load_inst_vld = Bool()
    val ag_pipe3_preg_dupx = UInt(7.W)
    val ag_pipe3_vload_inst_vld = Bool()
    val ag_pipe3_vreg_dupx = UInt(7.W)
    val dc_pipe3_load_fwd_inst_vld_dupx = Bool()
    val dc_pipe3_load_inst_vld_dupx = Bool()
    val dc_pipe3_preg_dupx = UInt(7.W)
    val dc_pipe3_vload_fwd_inst_vld = Bool()
    val dc_pipe3_vload_inst_vld_dupx = Bool()
    val dc_pipe3_vreg_dupx = UInt(7.W)
    val vmb_create0_entry = UInt(8.W)
    val vmb_create1_entry = UInt(8.W)
    val wb_pipe3_wb_preg_dupx = UInt(7.W)
    val wb_pipe3_wb_preg_vld_dupx = Bool()
    val wb_pipe3_wb_vreg_dupx = UInt(7.W)
    val wb_pipe3_wb_vreg_vld_dupx = Bool()
  }
  val fromVFPU = Vec(2, new VFPU2IS)
  val fromRTU = new Bundle {
    val flush_fe = Bool()
    val flush_is = Bool()
    val flush_stall = Bool()
    val yy_xx_flush = Bool()
    val rob_full = Bool()

    val retire_int_vld = Bool()

    val rob_inst_idd = Vec(4, UInt(7.W))
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

    val pcfifo_dis_inst_pid = Vec(4, UInt(5.W))

    val yyxxCancel = Bool()
  }
  val fromPad = new Bundle{
    val yyIcgScanEn = Bool()
  }

  val ir_type_stall_inst2_vld = Bool()
  val ir_type_stall_inst3_vld = Bool()
}

class ISStageOutput extends Bundle{
  //aiq0 aiq1 biq lsiq sdiq viq0 viq1
  val iqCreateEn = Vec(7, Vec(2, new Bundle{
    val dp_en = Bool()
    val en = Bool()
    val gateclk_en = Bool()
    val sel = UInt(2.W)
  }))

  val toAiq0 = new Bundle {
    val bypass_data = new AIQ0Data
    val create_data = Vec(2, new AIQ0Data)
    val create_div = Bool()
    val src_rdy_for_bypass = Vec(3, Bool())
  }
  val toAiq1 = new Bundle {
    val bypass_data = new AIQ1Data
    val create_data = Vec(2, new AIQ1Data)
    val create_alu = Bool()
    val src_rdy_for_bypass = Vec(3, Bool())
  }
  val toAiq = new Bundle {
    val inst_src_preg = Vec(4, Vec(3, UInt(7.W)))
    val sdiq_create_src_sel = Vec(2, Bool())
  }
  val toBiq = new Bundle {
    val bypass_data = new BIQData
    val create_data = Vec(2, new BIQData)
    val src_rdy_for_bypass = Vec(2, Bool())
  }
  val toLsiq = new Bundle {
    val bypass_data = new LSIQData
    val create_data = Vec(2, new LSIQData)
    val create_bar = Vec(2, Bool())
    val create_load = Vec(2, Bool())
    val create_no_spec = Vec(2, Bool())
    val create_store = Vec(2, Bool())

    val create0_src_rdy_for_bypass = Vec(2, Bool())
    val create0_srcvm_rdy_for_bypass = Bool()
    val bar_inst_vld = Bool()
  }
  val sdiq_create_data = Vec(2, new SDIQData)
  val toViq0 = new Bundle{
    val bypass_data = new VIQData
    val create_data = Vec(2, new VIQData)
    val srcv_rdy_for_bypass = Vec(3, Bool())
    val srcvm_rdy_for_bypass = Bool()
    val create_vdiv = Bool()
  }
  val toViq1 = new Bundle{
    val bypass_data = new VIQData
    val create_data = Vec(2, new VIQData)
    val srcv_rdy_for_bypass = Vec(3, Bool())
    val srcvm_rdy_for_bypass = Bool()
  }
  val viq_inst_srcv2_vreg = Vec(4, UInt(7.W))

  val toFence = new Bundle{
    val is_pipe_empty = Bool()
  }
  val toHad = new Bundle{
    val iq_empty = Bool()
  }
  val toIU = new Bundle{
    val pcfifo_inst_num = UInt(3.W)
    val pcfifo_inst_vld = Bool()
  }
  val toLSU = new Bundle {
    val vmb_create = Vec(2, new Bundle{
      val dp_en = Bool()
      val en = Bool()
      val gateclk_en = Bool()
      val dst_ready = Bool()
      val sdiq_entry = UInt(12.W)
      val split_num = UInt(7.W)
      val unit_stride = Bool()
      val vamo = Bool()
      val vl = UInt(8.W)
      val vreg = UInt(6.W)
      val vsew = UInt(2.W)
    })
  }
  val toTop = new Bundle{
    val inst_vld = Vec(4, Bool())
    val dis_pipedown2 = Bool()
    val iq_full = Bool()
    val vmb_full = Bool()
  }
  val toIR = new Bundle{
    val dis_type_stall = Bool()
    val inst2_vld = Bool()
    val inst2_ctrl_info = new ISCtrl
    val inst3_vld = Bool()
    val inst3_ctrl_info = new ISCtrl
    val stall     = Bool()
    val inst0_sel = UInt(2.W)
    val inst_sel  = UInt(3.W)
  }
  val toRTU = new Bundle{
    val pst_dis = Vec(4, new Bundle{
      val ereg_vld = Bool()
      val freg_vld = Bool()
      val preg_vld = Bool()
      val vreg_vld = Bool()

      val dst_reg  = UInt(5.W)
      val dstv_reg = UInt(5.W)
      val ereg     = UInt(5.W)
      val ereg_iid = UInt(7.W)
      val preg     = UInt(7.W)
      val preg_iid = UInt(7.W)
      val rel_ereg = UInt(5.W)
      val rel_preg = UInt(7.W)
      val rel_vreg = UInt(6.W)
      val vreg     = UInt(6.W)
      val vreg_iid = UInt(7.W)
    })
    val rob_create = Vec(4, new Bundle{
      val dp_en = Bool()
      val en = Bool()
      val gateclk_en = Bool()
      val data = new ROBData
    })
  }
}

class ISStageIO extends Bundle{
  val in  = Input(new ISStageInput)
  val out = Output(new ISStageOutput)
}

class ISStage extends Module{
  val io = IO(new ISStageIO)

  //Reg
  val instVld = RegInit(VecInit(Seq.fill(4)(false.B)))
  val dis_info = RegInit(0.U.asTypeOf(Output(new IR_preDispatch)))

  val inst_src_match = RegInit(VecInit(Seq.fill(6)(0.U.asTypeOf(new ir_srcMatch))))

  val iq_full  = RegInit(false.B)
  val vmb_full = RegInit(false.B)
  //Wire
  val is_dis_stall = Wire(Bool())
  val is_dis_type_stall = Wire(Bool())

  val inst_create_data = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new ISData))))
  val inst_read_data = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new ISData))))

  val sdiq_vmb_create1_dp_en = Wire(Bool())
  val sdiq_vmb_create1_entry = Wire(UInt(12.W))
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val inst_clk_en = io.in.ir_pipedown.gateclk || instVld.asUInt.orR


  //==========================================================
  //                IS pipeline registers
  //==========================================================
  //----------------------------------------------------------
  //            Implement of is inst valid register
  //----------------------------------------------------------
  when(io.in.fromRTU.flush_fe || io.in.fromIU.yyxxCancel){
    instVld := WireInit(VecInit(Seq.fill(4)(false.B)))
  }.elsewhen(!is_dis_stall){
    instVld := io.in.ir_pipedown.instVld
  }

  io.out.toTop.inst_vld := instVld

  io.out.toIR.inst2_vld := instVld(2)
  io.out.toIR.inst3_vld := instVld(3)

  io.out.toFence.is_pipe_empty := !instVld(0)

  //----------------------------------------------------------
  //            Implement of dispatch control register
  //----------------------------------------------------------
  when(io.in.fromRTU.flush_fe || io.in.fromIU.yyxxCancel){
    dis_info := 0.U.asTypeOf(Output(new IR_preDispatch))
  }.elsewhen(!is_dis_stall){
    dis_info := io.in.pre_dispatch
  }
  io.out.toTop.dis_pipedown2 := dis_info.pipedown2

  //==========================================================
  //        Control signal for IS data path update
  //==========================================================
  //TODO: figure out the meaning of tpye_stall
  is_dis_type_stall :=
    dis_info.pipedown2 &&                             //if pipedown2, is inst1/2 must be valid
      (instVld(3) && io.in.ir_type_stall_inst2_vld || //  if is inst3 valid, type stall if ir inst2 valid
        !instVld(3) && io.in.ir_type_stall_inst3_vld) //  if next cycle is inst3 not valid, type stall if ir inst3 valid
  io.out.toIR.dis_type_stall := is_dis_type_stall
  //==========================================================
  //        Control signal for IS data path update
  //==========================================================
  //1.if pipedown2, is inst1/2 must be valid, is inst0 will sel is inst2
  //2.if pipedown4, is inst0 will sel ir inst0
  io.out.toIR.inst0_sel := Cat(!dis_info.pipedown2, dis_info.pipedown2)

  //1.if pipedown2, is inst1/2 must be valid
  //  1.1 if is inst3 valid, is inst1 sel is inst3,
  //      is inst2/3 sel ir inst0/1
  //  1.2 if is inst3 not valid, is inst1/2/3 sel ir inst0/1/2
  //2.if pipedown4, is inst1/2/3 will sel ir inst1/2/3
  io.out.toIR.inst_sel := Cat(!dis_info.pipedown2, dis_info.pipedown2 && !instVld(3), dis_info.pipedown2 && instVld(3))

  //==========================================================
  //                IR/IS pipeline registers
  //==========================================================
  //----------------------------------------------------------
  //           control singals for pipeline entry
  //----------------------------------------------------------
  //TODO: if need to implement

  //----------------------------------------------------------
  //             IS pipeline registers shift MUX
  //----------------------------------------------------------
  inst_create_data(0) := MuxLookup(io.out.toIR.inst0_sel, 0.U.asTypeOf(new ISData), Seq(
    "b01".U -> inst_read_data(2),
    "b10".U -> io.in.ir_pipedown.instData(0)
  ))

  inst_create_data(1) := MuxLookup(io.out.toIR.inst_sel, 0.U.asTypeOf(new ISData), Seq(
    "b001".U -> inst_read_data(3),
    "b010".U -> io.in.ir_pipedown.instData(0),
    "b100".U -> io.in.ir_pipedown.instData(1)
  ))

  inst_create_data(2) := MuxLookup(io.out.toIR.inst_sel, 0.U.asTypeOf(new ISData), Seq(
    "b001".U -> io.in.ir_pipedown.instData(0),
    "b010".U -> io.in.ir_pipedown.instData(1),
    "b100".U -> io.in.ir_pipedown.instData(2)
  ))

  inst_create_data(3) := MuxLookup(io.out.toIR.inst_sel, 0.U.asTypeOf(new ISData), Seq(
    "b001".U -> io.in.ir_pipedown.instData(1),
    "b010".U -> io.in.ir_pipedown.instData(2),
    "b100".U -> io.in.ir_pipedown.instData(3)
  ))

  //----------------------------------------------------------
  //            pipeline entry registers instance
  //----------------------------------------------------------
  val is_dp_inst = Seq.fill(4)(Module(new ct_idu_is_pipe_entry))
  for(i <- 0 until 4){
    is_dp_inst(i).io.cp0_idu_icg_en := io.in.fromCp0.icgEn
    is_dp_inst(i).io.cp0_yy_clk_en := io.in.fromCp0.yyClkEn
    is_dp_inst(i).io.cpurst_b := !reset.asBool
    is_dp_inst(i).io.ctrl_xx_rf_pipe0_preg_lch_vld_dupx := io.in.fromRf.preg_lch_vld_dupx(0)
    is_dp_inst(i).io.ctrl_xx_rf_pipe1_preg_lch_vld_dupx := io.in.fromRf.preg_lch_vld_dupx(1)
    is_dp_inst(i).io.ctrl_xx_rf_pipe6_vmla_lch_vld_dupx := io.in.fromRf.vmla_lch_vld_dupx(0)
    is_dp_inst(i).io.ctrl_xx_rf_pipe7_vmla_lch_vld_dupx := io.in.fromRf.vmla_lch_vld_dupx(1)
    is_dp_inst(i).io.dp_xx_rf_pipe0_dst_preg_dupx := io.in.fromRf.dst_preg_dupx(0)
    is_dp_inst(i).io.dp_xx_rf_pipe1_dst_preg_dupx := io.in.fromRf.dst_preg_dupx(1)
    is_dp_inst(i).io.dp_xx_rf_pipe6_dst_vreg_dupx := io.in.fromRf.dst_vreg_dupx(0)
    is_dp_inst(i).io.dp_xx_rf_pipe7_dst_vreg_dupx := io.in.fromRf.dst_vreg_dupx(1)
    is_dp_inst(i).io.forever_cpuclk := clock.asBool
    is_dp_inst(i).io.iu_idu_div_inst_vld                 := io.in.fromIU.div_inst_vld
    is_dp_inst(i).io.iu_idu_div_preg_dupx                := io.in.fromIU.div_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe0_wb_preg_dupx       := io.in.fromIU.ex2_pipe0_wb_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe0_wb_preg_vld_dupx   := io.in.fromIU.ex2_pipe0_wb_preg_vld_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_mult_inst_vld_dupx := io.in.fromIU.ex2_pipe1_mult_inst_vld_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_preg_dupx          := io.in.fromIU.ex2_pipe1_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_wb_preg_dupx       := io.in.fromIU.ex2_pipe1_wb_preg_dupx
    is_dp_inst(i).io.iu_idu_ex2_pipe1_wb_preg_vld_dupx   := io.in.fromIU.ex2_pipe1_wb_preg_vld_dupx
    is_dp_inst(i).io.lsu_idu_ag_pipe3_load_inst_vld          := io.in.fromLSU.ag_pipe3_load_inst_vld
    is_dp_inst(i).io.lsu_idu_ag_pipe3_preg_dupx              := io.in.fromLSU.ag_pipe3_preg_dupx
    is_dp_inst(i).io.lsu_idu_ag_pipe3_vload_inst_vld         := io.in.fromLSU.ag_pipe3_vload_inst_vld
    is_dp_inst(i).io.lsu_idu_ag_pipe3_vreg_dupx              := io.in.fromLSU.ag_pipe3_vreg_dupx
    is_dp_inst(i).io.lsu_idu_dc_pipe3_load_fwd_inst_vld_dupx := io.in.fromLSU.dc_pipe3_load_fwd_inst_vld_dupx
    is_dp_inst(i).io.lsu_idu_dc_pipe3_load_inst_vld_dupx     := io.in.fromLSU.dc_pipe3_load_inst_vld_dupx
    is_dp_inst(i).io.lsu_idu_dc_pipe3_preg_dupx              := io.in.fromLSU.dc_pipe3_preg_dupx
    is_dp_inst(i).io.lsu_idu_dc_pipe3_vload_fwd_inst_vld     := io.in.fromLSU.dc_pipe3_vload_fwd_inst_vld
    is_dp_inst(i).io.lsu_idu_dc_pipe3_vload_inst_vld_dupx    := io.in.fromLSU.dc_pipe3_vload_inst_vld_dupx
    is_dp_inst(i).io.lsu_idu_dc_pipe3_vreg_dupx              := io.in.fromLSU.dc_pipe3_vreg_dupx
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_preg_dupx           := io.in.fromLSU.wb_pipe3_wb_preg_dupx
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_preg_vld_dupx       := io.in.fromLSU.wb_pipe3_wb_preg_vld_dupx
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_vreg_dupx           := io.in.fromLSU.wb_pipe3_wb_vreg_dupx
    is_dp_inst(i).io.lsu_idu_wb_pipe3_wb_vreg_vld_dupx       := io.in.fromLSU.wb_pipe3_wb_vreg_vld_dupx
    is_dp_inst(i).io.pad_yy_icg_scan_en := io.in.fromPad.yyIcgScanEn
    is_dp_inst(i).io.rtu_idu_flush_fe   := io.in.fromRTU.flush_fe
    is_dp_inst(i).io.rtu_idu_flush_is   := io.in.fromRTU.flush_is
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_data_vld_dupx      := io.in.fromVFPU(0).ex1_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_fmla_data_vld_dupx := io.in.fromVFPU(0).ex1_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_mfvr_inst_vld_dupx := io.in.fromVFPU(0).ex1_mfvr_inst_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_preg_dupx          := io.in.fromVFPU(0).ex1_preg_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe6_vreg_dupx          := io.in.fromVFPU(0).ex1_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_data_vld_dupx      := io.in.fromVFPU(1).ex1_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_fmla_data_vld_dupx := io.in.fromVFPU(1).ex1_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_mfvr_inst_vld_dupx := io.in.fromVFPU(1).ex1_mfvr_inst_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_preg_dupx          := io.in.fromVFPU(1).ex1_preg_dupx
    is_dp_inst(i).io.vfpu_idu_ex1_pipe7_vreg_dupx          := io.in.fromVFPU(1).ex1_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe6_data_vld_dupx      := io.in.fromVFPU(0).ex2_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe6_fmla_data_vld_dupx := io.in.fromVFPU(0).ex2_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe6_vreg_dupx          := io.in.fromVFPU(0).ex2_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe7_data_vld_dupx      := io.in.fromVFPU(1).ex2_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe7_fmla_data_vld_dupx := io.in.fromVFPU(1).ex2_fmla_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex2_pipe7_vreg_dupx          := io.in.fromVFPU(1).ex2_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe6_data_vld_dupx      := io.in.fromVFPU(0).ex3_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe6_vreg_dupx          := io.in.fromVFPU(0).ex3_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe7_data_vld_dupx      := io.in.fromVFPU(1).ex3_data_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex3_pipe7_vreg_dupx          := io.in.fromVFPU(1).ex3_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe6_wb_vreg_dupx       := io.in.fromVFPU(0).ex5_wb_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe6_wb_vreg_vld_dupx   := io.in.fromVFPU(0).ex5_wb_vreg_vld_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe7_wb_vreg_dupx       := io.in.fromVFPU(1).ex5_wb_vreg_dupx
    is_dp_inst(i).io.vfpu_idu_ex5_pipe7_wb_vreg_vld_dupx   := io.in.fromVFPU(1).ex5_wb_vreg_vld_dupx
    is_dp_inst(i).io.x_create_data       := inst_create_data(i).asUInt
    is_dp_inst(i).io.x_create_dp_en      := io.in.ir_pipedown.pipedown && !is_dis_stall
    is_dp_inst(i).io.x_create_gateclk_en := io.in.ir_pipedown.gateclk
    is_dp_inst(i).io.x_entry_vld         := instVld(i)

    inst_read_data(i) := is_dp_inst(i).io.x_read_data.asTypeOf(new ISData)
  }

  //----------------------------------------------------------
  //               Output for Control Logic
  //----------------------------------------------------------
  for(i <- 0 until 4){
    io.out.toAiq.inst_src_preg(i)(0) := inst_read_data(i).src0_data.preg
    io.out.toAiq.inst_src_preg(i)(1) := inst_read_data(i).src1_data.preg
    io.out.toAiq.inst_src_preg(i)(2) := inst_read_data(i).src2_data.preg
    io.out.viq_inst_srcv2_vreg(i) := inst_read_data(i).srcv2_data.preg
  }

  //----------------------------------------------------------
  //             IS pipeline preg match create
  //----------------------------------------------------------
  val inst_create_src_match = WireInit(VecInit(Seq.fill(6)(0.U.asTypeOf(new ir_srcMatch))))
  when(io.out.toIR.inst_sel === "b001".U){
    inst_create_src_match(0) := inst_src_match(5)
    inst_create_src_match(1) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(2) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(3) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(4) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(5) := io.in.ir_pipedown.inst_src_match(0)
  }.elsewhen(io.out.toIR.inst_sel === "b010".U){
    inst_create_src_match(0) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(1) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(2) := 0.U.asTypeOf(new ir_srcMatch)
    inst_create_src_match(3) := io.in.ir_pipedown.inst_src_match(0)
    inst_create_src_match(4) := io.in.ir_pipedown.inst_src_match(1)
    inst_create_src_match(5) := io.in.ir_pipedown.inst_src_match(3)
  }.elsewhen(io.out.toIR.inst_sel === "b100".U){
    inst_create_src_match(0) := io.in.ir_pipedown.inst_src_match(0)
    inst_create_src_match(1) := io.in.ir_pipedown.inst_src_match(1)
    inst_create_src_match(2) := io.in.ir_pipedown.inst_src_match(2)
    inst_create_src_match(3) := io.in.ir_pipedown.inst_src_match(3)
    inst_create_src_match(4) := io.in.ir_pipedown.inst_src_match(4)
    inst_create_src_match(5) := io.in.ir_pipedown.inst_src_match(5)
  }

  //----------------------------------------------------------
  //              Instance of Gated Cell
  //----------------------------------------------------------
  val dp_inst_clk_en = io.in.ir_pipedown.gateclk

  //----------------------------------------------------------
  //             IS pipeline preg match create
  //----------------------------------------------------------
  val inst_creare_dp_en = io.in.ir_pipedown.pipedown && !is_dis_stall
  when(inst_creare_dp_en){
    inst_src_match := inst_create_src_match
  }

  //==========================================================
  //          Control signal for reorder buffer create
  //==========================================================
  //-------------create enable for reorder buffer-------------
  //output create enable for control path and data path
  //create 0 always from dis_inst0_vld
  io.out.toRTU.rob_create(0).en := !is_dis_stall && dis_info.inst_vld(0)
  io.out.toRTU.rob_create(1).en := !is_dis_stall && dis_info.rob_create.en1
  io.out.toRTU.rob_create(2).en := !is_dis_stall && dis_info.rob_create.en2
  io.out.toRTU.rob_create(3).en := !is_dis_stall && dis_info.rob_create.en3

  io.out.toRTU.rob_create(0).dp_en := !io.in.fromRTU.rob_full && dis_info.inst_vld(0)
  io.out.toRTU.rob_create(1).dp_en := !io.in.fromRTU.rob_full && dis_info.rob_create.en1
  io.out.toRTU.rob_create(2).dp_en := !io.in.fromRTU.rob_full && dis_info.rob_create.en2
  io.out.toRTU.rob_create(3).dp_en := !io.in.fromRTU.rob_full && dis_info.rob_create.en3

  io.out.toRTU.rob_create(0).gateclk_en := dis_info.inst_vld(0)
  io.out.toRTU.rob_create(1).gateclk_en := dis_info.rob_create.en1
  io.out.toRTU.rob_create(2).gateclk_en := dis_info.rob_create.en2
  io.out.toRTU.rob_create(3).gateclk_en := dis_info.rob_create.en3

  //==========================================================
  //                   Create Data for ROB
  //==========================================================
  val dis_inst_pc_offset = Wire(Vec(4, UInt(3.W)))
  val dis_inst_ras = Wire(Vec(4, Bool()))
  val dis_inst_fp_dirty = Wire(Vec(4, Bool()))
  val dis_inst_vec_dirty = Wire(Vec(4, Bool()))

  for(i <- 0 until 4){
    dis_inst_pc_offset(i) := Mux(inst_read_data(i).SPLIT || inst_read_data(i).BJU, 0.U, Mux(inst_read_data(i).LENGTH, 2.U, 1.U))
    dis_inst_ras(i)       := inst_read_data(i).RTS || inst_read_data(i).PCALL
    dis_inst_fp_dirty(i)  := inst_read_data(i).dstv_vld && !inst_read_data(i).dst_vreg(6) && inst_read_data(i).DSTV_IMP ||
      inst_read_data(i).dste_vld
    dis_inst_vec_dirty(i) := inst_read_data(i).dstv_vld && inst_read_data(i).dst_vreg(6) && inst_read_data(i).DSTV_IMP ||
      inst_read_data(i).VSETVLI || inst_read_data(i).VSETVL
  }


  val rob_create_data = WireInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new ROBData))))

  for(i <- 0 until 4){
    //rob_create_data(0).INSTR :=
    rob_create_data(i).VL_PRED := inst_read_data(i).VL_PRED
    rob_create_data(i).VL := inst_read_data(i).VL
    //rob_create_data(i).VEC_DIRTY :=
    rob_create_data(i).VSETVLI := inst_read_data(i).VSETVLI
    rob_create_data(i).VSEW := inst_read_data(i).VSEW
    rob_create_data(i).VLMUL := inst_read_data(i).VLMUL
    rob_create_data(i).NO_SPEC_MISPRED := false.B
    rob_create_data(i).NO_SPEC_MISS    := false.B
    rob_create_data(i).NO_SPEC_HIT     := false.B
    rob_create_data(i).LOAD := inst_read_data(i).LOAD
    //rob_create_data(i).FP_DIRTY :=
    //rob_create_data(i).INST_NUM :=
    //rob_create_data(i).BKPTB_INST :=
    //rob_create_data(i).BKPTA_INST :=
    rob_create_data(i).BKPTB_DATA := 0.U
    rob_create_data(i).BKPTA_DATA := 0.U
    rob_create_data(i).STORE := inst_read_data(i).STADDR
    //rob_create_data(i).RAS :=
    //rob_create_data(i).PCFIFO := inst_read_data(i).PCFIFO
    //rob_create_data(i).BJU := inst_read_data(i).BJU
    rob_create_data(i).INTMASK := inst_read_data(i).INTMASK
    rob_create_data(i).SPLIT := inst_read_data(i).SPLIT
    //rob_create_data(i).PC_OFFSET :=
    //rob_create_data(i).CMPLT_CNT :=
    rob_create_data(i).CMPLT := false.B
    rob_create_data(i).VLD := true.B
    if (DifftestEnable) {
      val idx_offset = Wire(Vec(NumCreateEntry, UInt(2.W)))
      idx_offset(0) := 0.U
      for (j <- 1 until NumCreateEntry) {
        idx_offset(j) := rob_create_data(j - 1).INST_NUM
      }
      for (j <- 0 until NumFoldMax) {
        if (i + j < inst_read_data.length) {
          rob_create_data(i).debug.pc(j)      := inst_read_data(idx_offset(i) + j.U).LSU_PC
          rob_create_data(i).debug.inst(j)    := inst_read_data(idx_offset(i) + j.U).opcode
          rob_create_data(i).debug.RVC(j)     := !inst_read_data(idx_offset(i) + j.U).LENGTH
          rob_create_data(i).debug.rfwen(j)   := inst_read_data(idx_offset(i) + j.U).dst_vld
          rob_create_data(i).debug.wdest(j)   := inst_read_data(idx_offset(i) + j.U).dst_reg
          rob_create_data(i).debug.wpdest(j)  := inst_read_data(idx_offset(i) + j.U).dst_preg
        } else {
          rob_create_data(i).debug.pc(j)      := BigInt("ffffffff", 16).U
          rob_create_data(i).debug.inst(j)    := BigInt("ffffffff", 16).U
          rob_create_data(i).debug.RVC(j)     := BigInt("ffffffff", 16).U
          rob_create_data(i).debug.rfwen(j)   := BigInt("ffffffff", 16).U
          rob_create_data(i).debug.wdest(j)   := BigInt("ffffffff", 16).U
          rob_create_data(i).debug.wpdest(j)  := BigInt("ffffffff", 16).U
        }
      }
    }
  }

  val dis_inst01_pc_offset  = dis_inst_pc_offset(0) + dis_inst_pc_offset(1)
  val dis_inst12_pc_offset  = dis_inst_pc_offset(1) + dis_inst_pc_offset(2)
  val dis_inst23_pc_offset  = dis_inst_pc_offset(2) + dis_inst_pc_offset(3)
  val dis_inst012_pc_offset = dis_inst_pc_offset(0) + dis_inst_pc_offset(1) + dis_inst_pc_offset(2)
  val dis_inst123_pc_offset = dis_inst_pc_offset(1) + dis_inst_pc_offset(2) + dis_inst_pc_offset(3)

  val dis_inst01_bkpta_inst  = inst_read_data(0).BKPTA_INST || inst_read_data(1).BKPTA_INST
  val dis_inst01_bkptb_inst  = inst_read_data(0).BKPTB_INST || inst_read_data(1).BKPTB_INST
  val dis_inst12_bkpta_inst  = inst_read_data(1).BKPTA_INST || inst_read_data(2).BKPTA_INST
  val dis_inst12_bkptb_inst  = inst_read_data(1).BKPTB_INST || inst_read_data(2).BKPTB_INST
  val dis_inst23_bkpta_inst  = inst_read_data(2).BKPTA_INST || inst_read_data(3).BKPTA_INST
  val dis_inst23_bkptb_inst  = inst_read_data(2).BKPTB_INST || inst_read_data(3).BKPTB_INST
  val dis_inst012_bkpta_inst = inst_read_data(0).BKPTA_INST || inst_read_data(1).BKPTA_INST || inst_read_data(2).BKPTA_INST
  val dis_inst012_bkptb_inst = inst_read_data(0).BKPTB_INST || inst_read_data(1).BKPTB_INST || inst_read_data(2).BKPTB_INST
  val dis_inst123_bkpta_inst = inst_read_data(1).BKPTA_INST || inst_read_data(2).BKPTA_INST || inst_read_data(3).BKPTA_INST
  val dis_inst123_bkptb_inst = inst_read_data(1).BKPTB_INST || inst_read_data(2).BKPTB_INST || inst_read_data(3).BKPTB_INST

  val dis_inst01_fp_dirty  = dis_inst_fp_dirty(0) || dis_inst_fp_dirty(1)
  val dis_inst12_fp_dirty  = dis_inst_fp_dirty(1) || dis_inst_fp_dirty(2)
  val dis_inst23_fp_dirty  = dis_inst_fp_dirty(2) || dis_inst_fp_dirty(3)
  val dis_inst012_fp_dirty = dis_inst_fp_dirty(0) || dis_inst_fp_dirty(1) || dis_inst_fp_dirty(2)
  val dis_inst123_fp_dirty = dis_inst_fp_dirty(1) || dis_inst_fp_dirty(2) || dis_inst_fp_dirty(3)

  val dis_inst01_vec_dirty  = dis_inst_vec_dirty(0) || dis_inst_vec_dirty(1)
  val dis_inst12_vec_dirty  = dis_inst_vec_dirty(1) || dis_inst_vec_dirty(2)
  val dis_inst23_vec_dirty  = dis_inst_vec_dirty(2) || dis_inst_vec_dirty(3)
  val dis_inst012_vec_dirty = dis_inst_vec_dirty(0) || dis_inst_vec_dirty(1) || dis_inst_vec_dirty(2)
  val dis_inst123_vec_dirty = dis_inst_vec_dirty(1) || dis_inst_vec_dirty(2) || dis_inst_vec_dirty(3)

  //----------------------------------------------------------
  //                  Create Data for Port 0
  //----------------------------------------------------------
  when(dis_info.rob_create.sel0 === 0.U){//inst0
    rob_create_data(0).INSTR      := inst_read_data(0).opcode
    rob_create_data(0).PCFIFO     := inst_read_data(0).PCFIFO
    rob_create_data(0).BJU        := inst_read_data(0).BJU
    rob_create_data(0).VEC_DIRTY  := dis_inst_vec_dirty(0)
    rob_create_data(0).FP_DIRTY   := dis_inst_fp_dirty(0)
    rob_create_data(0).INST_NUM   := 1.U
    rob_create_data(0).BKPTB_INST := inst_read_data(0).BKPTB_INST
    rob_create_data(0).BKPTA_INST := inst_read_data(0).BKPTA_INST
    rob_create_data(0).RAS        := dis_inst_ras(0)
    rob_create_data(0).PC_OFFSET  := dis_inst_pc_offset(0)
    rob_create_data(0).CMPLT_CNT  := 1.U
  }.elsewhen(dis_info.rob_create.sel0 === 1.U){//inst0 and inst1
    rob_create_data(0).INSTR      := inst_read_data(0).opcode
    rob_create_data(0).PCFIFO     := inst_read_data(0).PCFIFO
    rob_create_data(0).BJU        := inst_read_data(0).BJU
    rob_create_data(0).VEC_DIRTY  := dis_inst01_vec_dirty
    rob_create_data(0).FP_DIRTY   := dis_inst01_fp_dirty
    rob_create_data(0).INST_NUM   := 2.U
    rob_create_data(0).BKPTB_INST := dis_inst01_bkptb_inst
    rob_create_data(0).BKPTA_INST := dis_inst01_bkpta_inst
    rob_create_data(0).RAS        := dis_inst_ras(0)
    rob_create_data(0).PC_OFFSET  := dis_inst01_pc_offset
    rob_create_data(0).CMPLT_CNT  := 2.U
  }.elsewhen(dis_info.rob_create.sel0 === 2.U){//inst0, inst1 and inst2
    rob_create_data(0).INSTR      := inst_read_data(0).opcode
    rob_create_data(0).PCFIFO     := inst_read_data(0).PCFIFO
    rob_create_data(0).BJU        := inst_read_data(0).BJU
    rob_create_data(0).VEC_DIRTY  := dis_inst012_vec_dirty
    rob_create_data(0).FP_DIRTY   := dis_inst012_fp_dirty
    rob_create_data(0).INST_NUM   := 3.U
    rob_create_data(0).BKPTB_INST := dis_inst012_bkptb_inst
    rob_create_data(0).BKPTA_INST := dis_inst012_bkpta_inst
    rob_create_data(0).RAS        := dis_inst_ras(0)
    rob_create_data(0).PC_OFFSET  := dis_inst012_pc_offset
    rob_create_data(0).CMPLT_CNT  := 3.U
  }

  //----------------------------------------------------------
  //                  Create Data for Port 1
  //----------------------------------------------------------
  when(dis_info.rob_create.sel1 === 0.U){//inst1
    rob_create_data(1).INSTR      := inst_read_data(1).opcode
    rob_create_data(1).PCFIFO     := inst_read_data(1).PCFIFO
    rob_create_data(1).BJU        := inst_read_data(1).BJU
    rob_create_data(1).VEC_DIRTY  := dis_inst_vec_dirty(1)
    rob_create_data(1).FP_DIRTY   := dis_inst_fp_dirty(1)
    rob_create_data(1).INST_NUM   := 1.U
    rob_create_data(1).BKPTB_INST := inst_read_data(1).BKPTB_INST
    rob_create_data(1).BKPTA_INST := inst_read_data(1).BKPTA_INST
    rob_create_data(1).RAS        := dis_inst_ras(1)
    rob_create_data(1).PC_OFFSET  := dis_inst_pc_offset(1)
    rob_create_data(1).CMPLT_CNT  := 1.U
  }.elsewhen(dis_info.rob_create.sel1 === 1.U){//inst1 and inst2
    rob_create_data(1).INSTR      := inst_read_data(1).opcode
    rob_create_data(1).PCFIFO     := inst_read_data(1).PCFIFO
    rob_create_data(1).BJU        := inst_read_data(1).BJU
    rob_create_data(1).VEC_DIRTY  := dis_inst12_vec_dirty
    rob_create_data(1).FP_DIRTY   := dis_inst12_fp_dirty
    rob_create_data(1).INST_NUM   := 2.U
    rob_create_data(1).BKPTB_INST := dis_inst12_bkptb_inst
    rob_create_data(1).BKPTA_INST := dis_inst12_bkpta_inst
    rob_create_data(1).RAS        := dis_inst_ras(1)
    rob_create_data(1).PC_OFFSET  := dis_inst12_pc_offset
    rob_create_data(1).CMPLT_CNT  := 2.U
  }.elsewhen(dis_info.rob_create.sel1 === 2.U){//inst2
    rob_create_data(1).INSTR      := inst_read_data(2).opcode
    rob_create_data(1).PCFIFO     := inst_read_data(2).PCFIFO
    rob_create_data(1).BJU        := inst_read_data(2).BJU
    rob_create_data(1).VEC_DIRTY  := dis_inst_vec_dirty(2)
    rob_create_data(1).FP_DIRTY   := dis_inst_fp_dirty(2)
    rob_create_data(1).INST_NUM   := 1.U
    rob_create_data(1).BKPTB_INST := inst_read_data(2).BKPTB_INST
    rob_create_data(1).BKPTA_INST := inst_read_data(2).BKPTA_INST
    rob_create_data(1).RAS        := dis_inst_ras(2)
    rob_create_data(1).PC_OFFSET  := dis_inst_pc_offset(2)
    rob_create_data(1).CMPLT_CNT  := 1.U
  }.elsewhen(dis_info.rob_create.sel1 === 3.U){//inst3
    rob_create_data(1).INSTR      := inst_read_data(3).opcode
    rob_create_data(1).PCFIFO     := inst_read_data(3).PCFIFO
    rob_create_data(1).BJU        := inst_read_data(3).BJU
    rob_create_data(1).VEC_DIRTY  := dis_inst_vec_dirty(3)
    rob_create_data(1).FP_DIRTY   := dis_inst_fp_dirty(3)
    rob_create_data(1).INST_NUM   := 1.U
    rob_create_data(1).BKPTB_INST := inst_read_data(3).BKPTB_INST
    rob_create_data(1).BKPTA_INST := inst_read_data(3).BKPTA_INST
    rob_create_data(1).RAS        := dis_inst_ras(3)
    rob_create_data(1).PC_OFFSET  := dis_inst_pc_offset(3)
    rob_create_data(1).CMPLT_CNT  := 1.U
  }.elsewhen(dis_info.rob_create.sel1 === 4.U){//inst1, inst2 and inst3
    rob_create_data(1).INSTR      := inst_read_data(1).opcode
    rob_create_data(1).PCFIFO     := inst_read_data(1).PCFIFO
    rob_create_data(1).BJU        := inst_read_data(1).BJU
    rob_create_data(1).VEC_DIRTY  := dis_inst123_vec_dirty
    rob_create_data(1).FP_DIRTY   := dis_inst123_fp_dirty
    rob_create_data(1).INST_NUM   := 3.U
    rob_create_data(1).BKPTB_INST := dis_inst123_bkptb_inst
    rob_create_data(1).BKPTA_INST := dis_inst123_bkpta_inst
    rob_create_data(1).RAS        := dis_inst_ras(1)
    rob_create_data(1).PC_OFFSET  := dis_inst123_pc_offset
    rob_create_data(1).CMPLT_CNT  := 3.U
  }

  //----------------------------------------------------------
  //                  Create Data for Port 2
  //----------------------------------------------------------
  when(dis_info.rob_create.sel2 === 0.U){//inst2
    rob_create_data(2).INSTR      := inst_read_data(2).opcode
    rob_create_data(2).PCFIFO     := inst_read_data(2).PCFIFO
    rob_create_data(2).BJU        := inst_read_data(2).BJU
    rob_create_data(2).VEC_DIRTY  := dis_inst_vec_dirty(2)
    rob_create_data(2).FP_DIRTY   := dis_inst_fp_dirty(2)
    rob_create_data(2).INST_NUM   := 1.U
    rob_create_data(2).BKPTB_INST := inst_read_data(2).BKPTB_INST
    rob_create_data(2).BKPTA_INST := inst_read_data(2).BKPTA_INST
    rob_create_data(2).RAS        := dis_inst_ras(2)
    rob_create_data(2).PC_OFFSET  := dis_inst_pc_offset(2)
    rob_create_data(2).CMPLT_CNT  := 1.U
  }.elsewhen(dis_info.rob_create.sel2 === 2.U){//inst2 and inst3
    rob_create_data(2).INSTR      := inst_read_data(2).opcode
    rob_create_data(2).PCFIFO     := inst_read_data(2).PCFIFO
    rob_create_data(2).BJU        := inst_read_data(2).BJU
    rob_create_data(2).VEC_DIRTY  := dis_inst23_vec_dirty
    rob_create_data(2).FP_DIRTY   := dis_inst23_fp_dirty
    rob_create_data(2).INST_NUM   := 2.U
    rob_create_data(2).BKPTB_INST := dis_inst23_bkptb_inst
    rob_create_data(2).BKPTA_INST := dis_inst23_bkpta_inst
    rob_create_data(2).RAS        := dis_inst_ras(2)
    rob_create_data(2).PC_OFFSET  := dis_inst23_pc_offset
    rob_create_data(2).CMPLT_CNT  := 2.U
  }.elsewhen(dis_info.rob_create.sel2 === 3.U){//inst3
    rob_create_data(2).INSTR      := inst_read_data(3).opcode
    rob_create_data(2).PCFIFO     := inst_read_data(3).PCFIFO
    rob_create_data(2).BJU        := inst_read_data(3).BJU
    rob_create_data(2).VEC_DIRTY  := dis_inst_vec_dirty(3)
    rob_create_data(2).FP_DIRTY   := dis_inst_fp_dirty(3)
    rob_create_data(2).INST_NUM   := 1.U
    rob_create_data(2).BKPTB_INST := inst_read_data(3).BKPTB_INST
    rob_create_data(2).BKPTA_INST := inst_read_data(3).BKPTA_INST
    rob_create_data(2).RAS        := dis_inst_ras(3)
    rob_create_data(2).PC_OFFSET  := dis_inst_pc_offset(3)
    rob_create_data(2).CMPLT_CNT  := 1.U
  }

  //----------------------------------------------------------
  //                  Create Data for Port 3
  //----------------------------------------------------------
  //create port 3 is always from inst3
  rob_create_data(3).INSTR      := inst_read_data(3).opcode
  rob_create_data(3).PCFIFO     := inst_read_data(3).PCFIFO
  rob_create_data(3).BJU        := inst_read_data(3).BJU
  rob_create_data(3).VEC_DIRTY  := dis_inst_vec_dirty(3)
  rob_create_data(3).FP_DIRTY   := dis_inst_fp_dirty(3)
  rob_create_data(3).INST_NUM   := 1.U
  rob_create_data(3).BKPTB_INST := inst_read_data(3).BKPTB_INST
  rob_create_data(3).BKPTA_INST := inst_read_data(3).BKPTA_INST
  rob_create_data(3).RAS        := dis_inst_ras(3)
  rob_create_data(3).PC_OFFSET  := dis_inst_pc_offset(3)
  rob_create_data(3).CMPLT_CNT  := 1.U

  for(i <- 0 until 4){
    io.out.toRTU.rob_create(i).data := rob_create_data(i)
  }

  //----------------------------------------------------------
  //                       Assign IID
  //----------------------------------------------------------
  val inst_iid = Wire(Vec(4, UInt(7.W)))

  inst_iid(0) := io.in.fromRTU.rob_inst_idd(0)
  inst_iid(1) := Mux(dis_info.pst_create_iid_sel(0)(0), io.in.fromRTU.rob_inst_idd(0), io.in.fromRTU.rob_inst_idd(1))
  inst_iid(2) := MuxLookup(dis_info.pst_create_iid_sel(1), 0.U(7.W), Seq(
    "b001".U -> io.in.fromRTU.rob_inst_idd(0),
    "b010".U -> io.in.fromRTU.rob_inst_idd(1),
    "b100".U -> io.in.fromRTU.rob_inst_idd(2)
  ))
  inst_iid(3) := MuxLookup(dis_info.pst_create_iid_sel(2), 0.U(7.W), Seq(
    "b001".U -> io.in.fromRTU.rob_inst_idd(1),
    "b010".U -> io.in.fromRTU.rob_inst_idd(2),
    "b100".U -> io.in.fromRTU.rob_inst_idd(3)
  ))

  //==========================================================
  //                Control signal for PST
  //==========================================================
  val dis_inst_preg_vld = Wire(Vec(4, Bool()))
  val dis_inst_vreg_vld = Wire(Vec(4, Bool()))
  val dis_inst_freg_vld = Wire(Vec(4, Bool()))
  val dis_inst_ereg_vld = Wire(Vec(4, Bool()))

  for(i <- 0 until 4){
    dis_inst_preg_vld(i) := dis_info.inst_vld(i) && !is_dis_stall && inst_read_data(i).dst_vld
    dis_inst_vreg_vld(i) := dis_info.inst_vld(i) && !is_dis_stall && inst_read_data(i).dstv_vld && inst_read_data(i).dst_vreg(6)
    dis_inst_freg_vld(i) := dis_info.inst_vld(i) && !is_dis_stall && inst_read_data(i).dstv_vld && !inst_read_data(i).dst_vreg(6)
    dis_inst_ereg_vld(i) := dis_info.inst_vld(i) && !is_dis_stall && inst_read_data(i).dste_vld
  }

  for(i <- 0 until 4){
    io.out.toRTU.pst_dis(i).preg_vld := dis_inst_preg_vld(i)
    io.out.toRTU.pst_dis(i).vreg_vld := dis_inst_vreg_vld(i)
    io.out.toRTU.pst_dis(i).freg_vld := dis_inst_freg_vld(i)
    io.out.toRTU.pst_dis(i).ereg_vld := dis_inst_ereg_vld(i)
  }

  //==========================================================
  //                 Create Data for PST
  //==========================================================
  //----------------------------------------------------------
  //                     Output for RTU
  //----------------------------------------------------------
  //implicit dest should create iid+1, which is iid of split consumer
  val dis_inst_iid = Wire(Vec(4, UInt(7.W)))
  for(i <- 0 until 4){
    dis_inst_iid(i) := inst_iid(i) + Cat(0.U(3.W), inst_read_data(i).IID_PLUS)
  }

  //power optimization: operand mux for pst_create_iid
  //if inst expt, it should write ereg, split inst should always its iid without plus
  //no problem because split consumer never read ereg
  for(i <- 0 until 4){
    io.out.toRTU.pst_dis(i).preg_iid := Mux(dis_inst_preg_vld(i), dis_inst_iid(i), 0.U)
    io.out.toRTU.pst_dis(i).vreg_iid := Mux(dis_inst_vreg_vld(i) || dis_inst_freg_vld(i), dis_inst_iid(i), 0.U)
    io.out.toRTU.pst_dis(i).ereg_iid := Mux(dis_inst_ereg_vld(i), inst_iid(i), 0.U)
  }

  for(i <- 0 until 4){
    io.out.toRTU.pst_dis(i).dst_reg  := inst_read_data(i).dst_reg
    io.out.toRTU.pst_dis(i).preg     := inst_read_data(i).dst_preg
    io.out.toRTU.pst_dis(i).rel_preg := inst_read_data(i).dst_rel_preg
    io.out.toRTU.pst_dis(i).dstv_reg := inst_read_data(i).dstv_reg
    io.out.toRTU.pst_dis(i).vreg     := inst_read_data(i).dst_vreg
    io.out.toRTU.pst_dis(i).rel_vreg := inst_read_data(i).dst_rel_vreg
    io.out.toRTU.pst_dis(i).ereg     := inst_read_data(i).dst_ereg
    io.out.toRTU.pst_dis(i).rel_ereg := inst_read_data(i).dst_rel_ereg
  }

  //==========================================================
  //          Control Signal for LSU VMB Create
  //==========================================================
  for(i <- 0 until 2){
    io.out.toLSU.vmb_create(i).en         := dis_info.iq_create_sel(7)(i).valid && !is_dis_stall
    io.out.toLSU.vmb_create(i).dp_en      := dis_info.iq_create_sel(7)(i).valid && !io.in.iq_cnt_info(7).full
    io.out.toLSU.vmb_create(i).gateclk_en := dis_info.iq_create_sel(7)(i).valid
  }

  //==========================================================
  //                 Create Data for LSU VMB
  //==========================================================
  for(i <- 0 until 2){
    io.out.toLSU.vmb_create(i).split_num   := 0.U
    io.out.toLSU.vmb_create(i).unit_stride := false.B
    io.out.toLSU.vmb_create(i).vamo        := false.B
    io.out.toLSU.vmb_create(i).vl          := 0.U
    io.out.toLSU.vmb_create(i).vreg        := 0.U
    io.out.toLSU.vmb_create(i).vsew        := 0.U

    for(j <- 0 until 4){
      when(dis_info.iq_create_sel(7)(i).bits === j.U){
        io.out.toLSU.vmb_create(i).split_num   := inst_read_data(j).SPLIT_NUM
        io.out.toLSU.vmb_create(i).unit_stride := inst_read_data(j).UNIT_STRIDE
        io.out.toLSU.vmb_create(i).vamo        := inst_read_data(j).VAMO
        io.out.toLSU.vmb_create(i).vl          := inst_read_data(j).VL
        io.out.toLSU.vmb_create(i).vreg        := inst_read_data(j).dst_vreg(5,0)
        io.out.toLSU.vmb_create(i).vsew        := inst_read_data(j).VSEW
      }
    }
  }
  io.out.toLSU.vmb_create(0).dst_ready   := !dis_info.iq_create_sel(4)(0).valid
  io.out.toLSU.vmb_create(0).sdiq_entry  := io.in.iq_create_entry.sdiq_dp(0)
  io.out.toLSU.vmb_create(1).dst_ready   := !sdiq_vmb_create1_dp_en
  io.out.toLSU.vmb_create(1).sdiq_entry  := sdiq_vmb_create1_entry

  //==========================================================
  //                 Assign PCFIFO ID (PID)
  //==========================================================
  val inst_pcfifo = inst_read_data.map(_.PCFIFO)

  val inst_alloc_pid = Wire(Vec(4, UInt(5.W)))
  inst_alloc_pid(0) := io.in.fromIU.pcfifo_dis_inst_pid(0)
  inst_alloc_pid(1) := Mux(inst_pcfifo(0), io.in.fromIU.pcfifo_dis_inst_pid(1), io.in.fromIU.pcfifo_dis_inst_pid(0))

  when(inst_pcfifo(0) && inst_pcfifo(1)){
    inst_alloc_pid(2) := io.in.fromIU.pcfifo_dis_inst_pid(2)
  }.elsewhen(inst_pcfifo(0) || inst_pcfifo(1)){
    inst_alloc_pid(2) := io.in.fromIU.pcfifo_dis_inst_pid(1)
  }.otherwise{
    inst_alloc_pid(2) := io.in.fromIU.pcfifo_dis_inst_pid(0)
  }

  when(inst_pcfifo(0) && inst_pcfifo(1) && inst_pcfifo(2)){
    inst_alloc_pid(3) := io.in.fromIU.pcfifo_dis_inst_pid(3)
  }.elsewhen(inst_pcfifo(0) && inst_pcfifo(1) || inst_pcfifo(0) && inst_pcfifo(2) || inst_pcfifo(1) && inst_pcfifo(2)){
    inst_alloc_pid(3) := io.in.fromIU.pcfifo_dis_inst_pid(2)
  }.elsewhen(inst_pcfifo(0) || inst_pcfifo(1) || inst_pcfifo(2)){
    inst_alloc_pid(3) := io.in.fromIU.pcfifo_dis_inst_pid(1)
  }.otherwise{
    inst_alloc_pid(3) := io.in.fromIU.pcfifo_dis_inst_pid(0)
  }

  //power optimization: mask pipedown index if dst not valid
  val inst_pid = Wire(Vec(4, UInt(5.W)))
  for(i <- 0 until 4){
    inst_pid(i) := Mux(inst_pcfifo(i), inst_alloc_pid(i), 0.U)
  }

  //==========================================================
  //              Issue Queue Dispatch Control
  //==========================================================
  for(j <- 0 until 7){
    for(i <- 0 until 2){
      io.out.iqCreateEn(j)(i).en := dis_info.iq_create_sel(j)(i).valid && !is_dis_stall
      io.out.iqCreateEn(j)(i).dp_en := dis_info.iq_create_sel(j)(i).valid && !io.in.iq_cnt_info(j).full
      io.out.iqCreateEn(j)(i).gateclk_en := dis_info.iq_create_sel(j)(i).valid
      io.out.iqCreateEn(j)(i).sel := dis_info.iq_create_sel(j)(i).bits
    }
  }

  //==========================================================
  //               Create Launch Ready for IQ
  //==========================================================
  //----------------------------------------------------------
  //               Issue Queue Create entry
  //----------------------------------------------------------
  val aiq0_create_entry = Wire(Vec(2, UInt(8.W)))
  val aiq1_create_entry = Wire(Vec(2, UInt(8.W)))
  val biq_create_entry  = Wire(Vec(2, UInt(12.W)))
  val lsiq_create_entry = Wire(Vec(2, UInt(12.W)))
  val sdiq_create_entry = Wire(Vec(2, UInt(12.W)))
  val viq0_create_entry = Wire(Vec(2, UInt(8.W)))
  val viq1_create_entry = Wire(Vec(2, UInt(8.W)))

  for(i <- 0 until 2){
    aiq0_create_entry(i) := Mux(io.out.iqCreateEn(0)(i).dp_en, io.in.iq_create_entry.aiq0_aiq(i), 0.U)
    aiq1_create_entry(i) := Mux(io.out.iqCreateEn(1)(i).dp_en, io.in.iq_create_entry.aiq1_aiq(i), 0.U)
    biq_create_entry(i)  := Mux(io.out.iqCreateEn(2)(i).dp_en, io.in.iq_create_entry.biq_aiq(i), 0.U)
    lsiq_create_entry(i) := Mux(io.out.iqCreateEn(3)(i).dp_en, io.in.iq_create_entry.lsiq_aiq(i), 0.U)
    sdiq_create_entry(i) := Mux(io.out.iqCreateEn(4)(i).dp_en, io.in.iq_create_entry.sdiq_aiq(i), 0.U)
    viq0_create_entry(i) := Mux(io.out.iqCreateEn(5)(i).dp_en, io.in.iq_create_entry.viq0_viq(i), 0.U)
    viq1_create_entry(i) := Mux(io.out.iqCreateEn(6)(i).dp_en, io.in.iq_create_entry.viq1_viq(i), 0.U)
  }

  //----------------------------------------------------------
  //         Dispatch Inst Create Launch Ready
  //----------------------------------------------------------
  val iq_inst_create_src_match = Wire(Vec(3, Vec(7, Vec(2, new ir_srcMatch))))
  for(j <- 0 until 7){
    for(i <- 0 until 2){
      //inst0
      iq_inst_create_src_match(0)(j)(i) := MuxLookup(dis_info.iq_create_sel(j)(i).bits, 0.U.asTypeOf(new ir_srcMatch), Seq(
        0.U -> 0.U.asTypeOf(new ir_srcMatch),
        1.U -> inst_src_match(0),
        2.U -> inst_src_match(1),
        3.U -> inst_src_match(2)
      ))
      //inst1
      iq_inst_create_src_match(1)(j)(i) := MuxLookup(dis_info.iq_create_sel(j)(i).bits, 0.U.asTypeOf(new ir_srcMatch), Seq(
        0.U -> 0.U.asTypeOf(new ir_srcMatch),
        1.U -> 0.U.asTypeOf(new ir_srcMatch),
        2.U -> inst_src_match(3),
        3.U -> inst_src_match(4)
      ))
      iq_inst_create_src_match(2)(j)(i) := MuxLookup(dis_info.iq_create_sel(j)(i).bits, 0.U.asTypeOf(new ir_srcMatch), Seq(
        0.U -> 0.U.asTypeOf(new ir_srcMatch),
        1.U -> 0.U.asTypeOf(new ir_srcMatch),
        2.U -> 0.U.asTypeOf(new ir_srcMatch),
        3.U -> inst_src_match(5)
      ))
    }
  }

  val inst_lch_rdy_aiq0 = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(8)(0.U(3.W))))))
  val inst_lch_rdy_aiq1 = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(8)(0.U(3.W))))))
  val inst_lch_rdy_viq0 = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(8)(false.B)))))
  val inst_lch_rdy_viq1 = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(8)(false.B)))))
  for(i <- 0 until 3){//inst
    for(j <- 0 until 8){//iq entry sel
      for(k <- 0 until 2){//create sel
        when(aiq0_create_entry(k)(j)){
          inst_lch_rdy_aiq0(i)(j) := Cat(iq_inst_create_src_match(i)(0)(k).src2, iq_inst_create_src_match(i)(0)(k).src1, iq_inst_create_src_match(i)(0)(k).src0)
        }
        when(aiq1_create_entry(k)(j)){
          inst_lch_rdy_aiq1(i)(j) := Cat(iq_inst_create_src_match(i)(1)(k).src2, iq_inst_create_src_match(i)(1)(k).src1, iq_inst_create_src_match(i)(1)(k).src0)
        }
        when(viq0_create_entry(k)(j)){
          inst_lch_rdy_viq0(i)(j) := iq_inst_create_src_match(i)(5)(k).srcv2
        }
        when(viq1_create_entry(k)(j)){
          inst_lch_rdy_viq1(i)(j) := iq_inst_create_src_match(i)(6)(k).srcv2
        }
      }
    }
  }

  val dp_sdiq_create_sti_sel = Wire(Vec(2, Bool()))

  val inst_lch_rdy_biq = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(12)(0.U(2.W))))))
  val inst_lch_rdy_lsiq = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(12)(0.U(2.W))))))
  val inst_lch_rdy_sdiq = WireInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(12)(false.B)))))
  for(i <- 0 until 3){ //inst
    for (j <- 0 until 12){ //iq entry sel
      for (k <- 0 until 2){ //create sel
        when(biq_create_entry(k)(j)){
          inst_lch_rdy_biq(i)(j) := Cat(iq_inst_create_src_match(i)(2)(k).src1, iq_inst_create_src_match(i)(2)(k).src0)
        }
        when(lsiq_create_entry(k)(j)){
          inst_lch_rdy_lsiq(i)(j) := Cat(iq_inst_create_src_match(i)(3)(k).src1, iq_inst_create_src_match(i)(3)(k).src0)
        }
      }
      //sdiq
      when(sdiq_create_entry(0)(j)){
        inst_lch_rdy_sdiq(i)(j) := Mux(dp_sdiq_create_sti_sel(0), iq_inst_create_src_match(i)(4)(0).src1, iq_inst_create_src_match(i)(4)(0).src2)
      }
      when(sdiq_create_entry(1)(j) && dis_info.iq_create_sel(4)(1).bits === 3.U){
        inst_lch_rdy_sdiq(i)(j) := Mux(dp_sdiq_create_sti_sel(1), iq_inst_create_src_match(i)(4)(1).src1, iq_inst_create_src_match(i)(4)(1).src2)
      }.elsewhen(sdiq_create_entry(1)(j) && dis_info.iq_create_sel(4)(1).bits =/= 3.U){
        inst_lch_rdy_sdiq(i)(j) := Mux(dp_sdiq_create_sti_sel(0), iq_inst_create_src_match(i)(4)(1).src1, iq_inst_create_src_match(i)(4)(1).src2)
      }
    }
  }

  //----------------------------------------------------------
  //            Dispatch Inst3 Create Launch Ready
  //----------------------------------------------------------
  //inst3 is always zero


  //==========================================================
  //               Create Data for Issue Queue
  //==========================================================
  //----------------------------------------------------------
  //                  Create Data for AIQ0
  //----------------------------------------------------------
  val aiq0_create_data         = Wire(Vec(2, new ISData))
  val aiq0_create_iid          = Wire(Vec(2, UInt(7.W)))
  val aiq0_create_pid          = Wire(Vec(2, UInt(5.W)))
  val aiq0_create_lch_rdy_aiq0 = Wire(Vec(2, Vec(8, UInt(3.W))))
  val aiq0_create_lch_rdy_aiq1 = Wire(Vec(2, Vec(8, UInt(3.W))))
  val aiq0_create_lch_rdy_biq  = Wire(Vec(2, Vec(12, UInt(2.W))))
  val aiq0_create_lch_rdy_lsiq = Wire(Vec(2, Vec(12, UInt(2.W))))
  val aiq0_create_lch_rdy_sdiq = Wire(Vec(2, Vec(12, Bool())))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(0)(i).bits === 0.U){
      aiq0_create_data(i) := inst_read_data(0)
      aiq0_create_iid(i) := inst_iid(0)
      aiq0_create_pid(i) := inst_pid(0)
      aiq0_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(0)
      aiq0_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(0)
      aiq0_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(0)
      aiq0_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(0)
      aiq0_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(0)
    }.elsewhen(dis_info.iq_create_sel(0)(i).bits === 1.U){
      aiq0_create_data(i) := inst_read_data(1)
      aiq0_create_iid(i) := inst_iid(1)
      aiq0_create_pid(i) := inst_pid(1)
      aiq0_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(1)
      aiq0_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(1)
      aiq0_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(1)
      aiq0_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(1)
      aiq0_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(1)
    }.elsewhen(dis_info.iq_create_sel(0)(i).bits === 2.U){
      aiq0_create_data(i) := inst_read_data(2)
      aiq0_create_iid(i) := inst_iid(2)
      aiq0_create_pid(i) := inst_pid(2)
      aiq0_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(2)
      aiq0_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(2)
      aiq0_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(2)
      aiq0_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(2)
      aiq0_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(2)
    }.elsewhen(dis_info.iq_create_sel(0)(i).bits === 3.U){
      aiq0_create_data(i) := inst_read_data(3)
      aiq0_create_iid(i) := inst_iid(3)
      aiq0_create_pid(i) := inst_pid(3)
      aiq0_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(3)
      aiq0_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(3)
      aiq0_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(3)
      aiq0_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(3)
      aiq0_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(3)
    }.otherwise{
      aiq0_create_data(i) := 0.U.asTypeOf(new ISData)
      aiq0_create_iid(i) := 0.U
      aiq0_create_pid(i) := 0.U
      aiq0_create_lch_rdy_aiq0(i) := 0.U.asTypeOf(Vec(8, UInt(3.W)))
      aiq0_create_lch_rdy_aiq1(i) := 0.U.asTypeOf(Vec(8, UInt(3.W)))
      aiq0_create_lch_rdy_biq(i)  := 0.U.asTypeOf(Vec(12, UInt(2.W)))
      aiq0_create_lch_rdy_lsiq(i) := 0.U.asTypeOf(Vec(12, UInt(2.W)))
      aiq0_create_lch_rdy_sdiq(i) := 0.U.asTypeOf(Vec(12, Bool()))
    }
  }

  //----------------------------------------------------------
  //                Reorganize for AIQ0 create
  //----------------------------------------------------------
  for(i <- 0 until 2){
    io.out.toAiq0.create_data(i).VL           := aiq0_create_data(i).VL
    io.out.toAiq0.create_data(i).LCH_PREG     := aiq0_create_data(i).LCH_PREG
    io.out.toAiq0.create_data(i).SPECIAL      := aiq0_create_data(i).SPECIAL
    io.out.toAiq0.create_data(i).VSEW         := aiq0_create_data(i).VSEW
    io.out.toAiq0.create_data(i).VLMUL        := aiq0_create_data(i).VLMUL
    io.out.toAiq0.create_data(i).LCH_RDY_SDIQ := aiq0_create_lch_rdy_sdiq(i)
    io.out.toAiq0.create_data(i).LCH_RDY_LSIQ := aiq0_create_lch_rdy_lsiq(i)
    io.out.toAiq0.create_data(i).LCH_RDY_BIQ  := aiq0_create_lch_rdy_biq(i)
    io.out.toAiq0.create_data(i).LCH_RDY_AIQ1 := aiq0_create_lch_rdy_aiq1(i)
    io.out.toAiq0.create_data(i).LCH_RDY_AIQ0 := aiq0_create_lch_rdy_aiq0(i)
    io.out.toAiq0.create_data(i).ALU_SHORT    := aiq0_create_data(i).ALU_SHORT
    io.out.toAiq0.create_data(i).PID          := aiq0_create_pid(i)
    io.out.toAiq0.create_data(i).PCFIFO       := aiq0_create_data(i).PCFIFO
    io.out.toAiq0.create_data(i).MTVR         := aiq0_create_data(i).MTVR
    io.out.toAiq0.create_data(i).DIV          := aiq0_create_data(i).DIV
    io.out.toAiq0.create_data(i).HIGH_HW_EXPT := aiq0_create_data(i).EXPT(6)
    io.out.toAiq0.create_data(i).EXPT_VEC     := aiq0_create_data(i).EXPT(5,1)
    io.out.toAiq0.create_data(i).EXPT_VLD     := aiq0_create_data(i).EXPT(0)

    io.out.toAiq0.create_data(i).src_info(2).lsu_match := aiq0_create_data(i).src2_lsu_match
    io.out.toAiq0.create_data(i).src_info(2).src_data  := aiq0_create_data(i).src2_data.asUInt(8,0).asTypeOf(new srcData9)
    io.out.toAiq0.create_data(i).src_info(1).lsu_match := aiq0_create_data(i).src1_lsu_match
    io.out.toAiq0.create_data(i).src_info(1).src_data  := aiq0_create_data(i).src1_data
    io.out.toAiq0.create_data(i).src_info(0).lsu_match := aiq0_create_data(i).src0_lsu_match
    io.out.toAiq0.create_data(i).src_info(0).src_data  := aiq0_create_data(i).src0_data

    io.out.toAiq0.create_data(i).DST_VREG := aiq0_create_data(i).dst_vreg
    io.out.toAiq0.create_data(i).DST_PREG := aiq0_create_data(i).dst_preg
    io.out.toAiq0.create_data(i).DSTV_VLD := aiq0_create_data(i).dstv_vld
    io.out.toAiq0.create_data(i).DST_VLD  := aiq0_create_data(i).dst_vld
    io.out.toAiq0.create_data(i).src_vld  := aiq0_create_data(i).src_vld
    io.out.toAiq0.create_data(i).IID      := aiq0_create_iid(i)
    io.out.toAiq0.create_data(i).OPCODE   := aiq0_create_data(i).opcode
  }

  io.out.toAiq0.bypass_data.VL           := aiq0_create_data(0).VL
  io.out.toAiq0.bypass_data.LCH_PREG     := aiq0_create_data(0).LCH_PREG
  io.out.toAiq0.bypass_data.SPECIAL      := aiq0_create_data(0).SPECIAL
  io.out.toAiq0.bypass_data.VSEW         := aiq0_create_data(0).VSEW
  io.out.toAiq0.bypass_data.VLMUL        := aiq0_create_data(0).VLMUL
  io.out.toAiq0.bypass_data.LCH_RDY_SDIQ := aiq0_create_lch_rdy_sdiq(0)
  io.out.toAiq0.bypass_data.LCH_RDY_LSIQ := aiq0_create_lch_rdy_lsiq(0)
  io.out.toAiq0.bypass_data.LCH_RDY_BIQ  := aiq0_create_lch_rdy_biq(0)
  io.out.toAiq0.bypass_data.LCH_RDY_AIQ1 := aiq0_create_lch_rdy_aiq1(0)
  io.out.toAiq0.bypass_data.LCH_RDY_AIQ0 := aiq0_create_lch_rdy_aiq0(0)
  io.out.toAiq0.bypass_data.ALU_SHORT    := aiq0_create_data(0).ALU_SHORT
  io.out.toAiq0.bypass_data.PID          := aiq0_create_pid(0)
  io.out.toAiq0.bypass_data.PCFIFO       := aiq0_create_data(0).PCFIFO
  io.out.toAiq0.bypass_data.MTVR         := aiq0_create_data(0).MTVR
  io.out.toAiq0.bypass_data.DIV          := aiq0_create_data(0).DIV
  io.out.toAiq0.bypass_data.HIGH_HW_EXPT := aiq0_create_data(0).EXPT(6)
  io.out.toAiq0.bypass_data.EXPT_VEC     := aiq0_create_data(0).EXPT(5,1)
  io.out.toAiq0.bypass_data.EXPT_VLD     := aiq0_create_data(0).EXPT(0)

  io.out.toAiq0.bypass_data.src_info(2).lsu_match     := 0.U
  io.out.toAiq0.bypass_data.src_info(2).src_data.preg := aiq0_create_data(0).src2_data.preg
  io.out.toAiq0.bypass_data.src_info(2).src_data.wb   := aiq0_create_data(0).src2_data.wb
  io.out.toAiq0.bypass_data.src_info(2).src_data.rdy  := 0.U
  io.out.toAiq0.bypass_data.src_info(1).lsu_match     := 0.U
  io.out.toAiq0.bypass_data.src_info(1).src_data.preg := aiq0_create_data(0).src1_data.preg
  io.out.toAiq0.bypass_data.src_info(1).src_data.wb   := aiq0_create_data(0).src1_data.wb
  io.out.toAiq0.bypass_data.src_info(1).src_data.rdy  := 0.U
  io.out.toAiq0.bypass_data.src_info(0).lsu_match     := 0.U
  io.out.toAiq0.bypass_data.src_info(0).src_data.preg := aiq0_create_data(0).src0_data.preg
  io.out.toAiq0.bypass_data.src_info(0).src_data.wb   := aiq0_create_data(0).src0_data.wb
  io.out.toAiq0.bypass_data.src_info(0).src_data.rdy  := 0.U

  io.out.toAiq0.bypass_data.DST_VREG := aiq0_create_data(0).dst_vreg
  io.out.toAiq0.bypass_data.DST_PREG := aiq0_create_data(0).dst_preg
  io.out.toAiq0.bypass_data.DSTV_VLD := aiq0_create_data(0).dstv_vld
  io.out.toAiq0.bypass_data.DST_VLD  := aiq0_create_data(0).dst_vld
  io.out.toAiq0.bypass_data.src_vld  := aiq0_create_data(0).src_vld
  io.out.toAiq0.bypass_data.IID      := aiq0_create_iid(0)
  io.out.toAiq0.bypass_data.OPCODE   := aiq0_create_data(0).opcode

  io.out.toAiq0.src_rdy_for_bypass(0) := aiq0_create_data(0).src0_bp_rdy(1)
  io.out.toAiq0.src_rdy_for_bypass(1) := aiq0_create_data(0).src1_bp_rdy(1)
  io.out.toAiq0.src_rdy_for_bypass(2) := aiq0_create_data(0).src2_bp_rdy(1)
  io.out.toAiq0.create_div := aiq0_create_data(0).DIV

  //----------------------------------------------------------
  //                  Create Data for AIQ1
  //----------------------------------------------------------
  val aiq1_create_data         = Wire(Vec(2, new ISData))
  val aiq1_create_iid          = Wire(Vec(2, UInt(7.W)))
  val aiq1_create_pid          = Wire(Vec(2, UInt(5.W)))
  val aiq1_create_lch_rdy_aiq0 = Wire(Vec(2, Vec(8, UInt(3.W))))
  val aiq1_create_lch_rdy_aiq1 = Wire(Vec(2, Vec(8, UInt(3.W))))
  val aiq1_create_lch_rdy_biq  = Wire(Vec(2, Vec(12, UInt(2.W))))
  val aiq1_create_lch_rdy_lsiq = Wire(Vec(2, Vec(12, UInt(2.W))))
  val aiq1_create_lch_rdy_sdiq = Wire(Vec(2, Vec(12, Bool())))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(1)(i).bits === 0.U){
      aiq1_create_data(i) := inst_read_data(0)
      aiq1_create_iid(i) := inst_iid(0)
      aiq1_create_pid(i) := inst_pid(0)
      aiq1_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(0)
      aiq1_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(0)
      aiq1_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(0)
      aiq1_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(0)
      aiq1_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(0)
    }.elsewhen(dis_info.iq_create_sel(1)(i).bits === 1.U){
      aiq1_create_data(i) := inst_read_data(1)
      aiq1_create_iid(i) := inst_iid(1)
      aiq1_create_pid(i) := inst_pid(1)
      aiq1_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(1)
      aiq1_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(1)
      aiq1_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(1)
      aiq1_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(1)
      aiq1_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(1)
    }.elsewhen(dis_info.iq_create_sel(1)(i).bits === 2.U){
      aiq1_create_data(i) := inst_read_data(2)
      aiq1_create_iid(i) := inst_iid(2)
      aiq1_create_pid(i) := inst_pid(2)
      aiq1_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(2)
      aiq1_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(2)
      aiq1_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(2)
      aiq1_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(2)
      aiq1_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(2)
    }.elsewhen(dis_info.iq_create_sel(1)(i).bits === 3.U){
      aiq1_create_data(i) := inst_read_data(3)
      aiq1_create_iid(i) := inst_iid(3)
      aiq1_create_pid(i) := inst_pid(3)
      aiq1_create_lch_rdy_aiq0(i) := inst_lch_rdy_aiq0(3)
      aiq1_create_lch_rdy_aiq1(i) := inst_lch_rdy_aiq1(3)
      aiq1_create_lch_rdy_biq(i)  := inst_lch_rdy_biq(3)
      aiq1_create_lch_rdy_lsiq(i) := inst_lch_rdy_lsiq(3)
      aiq1_create_lch_rdy_sdiq(i) := inst_lch_rdy_sdiq(3)
    }.otherwise{
      aiq1_create_data(i) := 0.U.asTypeOf(new ISData)
      aiq1_create_iid(i) := 0.U
      aiq1_create_pid(i) := 0.U
      aiq1_create_lch_rdy_aiq0(i) := 0.U.asTypeOf(Vec(8, UInt(3.W)))
      aiq1_create_lch_rdy_aiq1(i) := 0.U.asTypeOf(Vec(8, UInt(3.W)))
      aiq1_create_lch_rdy_biq(i)  := 0.U.asTypeOf(Vec(12, UInt(2.W)))
      aiq1_create_lch_rdy_lsiq(i) := 0.U.asTypeOf(Vec(12, UInt(2.W)))
      aiq1_create_lch_rdy_sdiq(i) := 0.U.asTypeOf(Vec(12, Bool()))
    }
  }

  //----------------------------------------------------------
  //                Reorganize for AIQ1 create
  //----------------------------------------------------------
  for(i <- 0 until 2){
    io.out.toAiq1.create_data(i).VL           := aiq1_create_data(i).VL
    io.out.toAiq1.create_data(i).LCH_PREG     := aiq1_create_data(i).LCH_PREG
    io.out.toAiq1.create_data(i).VSEW         := aiq1_create_data(i).VSEW
    io.out.toAiq1.create_data(i).VLMUL        := aiq1_create_data(i).VLMUL
    io.out.toAiq1.create_data(i).LCH_RDY_SDIQ := aiq1_create_lch_rdy_sdiq(i)
    io.out.toAiq1.create_data(i).LCH_RDY_LSIQ := aiq1_create_lch_rdy_lsiq(i)
    io.out.toAiq1.create_data(i).LCH_RDY_BIQ  := aiq1_create_lch_rdy_biq(i)
    io.out.toAiq1.create_data(i).LCH_RDY_AIQ1 := aiq1_create_lch_rdy_aiq1(i)
    io.out.toAiq1.create_data(i).LCH_RDY_AIQ0 := aiq1_create_lch_rdy_aiq0(i)
    io.out.toAiq1.create_data(i).ALU_SHORT    := aiq1_create_data(i).ALU_SHORT
    io.out.toAiq1.create_data(i).MLA          := aiq1_create_data(i).MLA
    io.out.toAiq1.create_data(i).MTVR         := aiq1_create_data(i).MTVR

    io.out.toAiq1.create_data(i).SRC2_LSU_MATCH        := aiq1_create_data(i).src2_lsu_match
    io.out.toAiq1.create_data(i).SRC2_DATA             := aiq1_create_data(i).src2_data
    io.out.toAiq1.create_data(i).src_info(1).lsu_match := aiq1_create_data(i).src1_lsu_match
    io.out.toAiq1.create_data(i).src_info(1).src_data  := aiq1_create_data(i).src1_data
    io.out.toAiq1.create_data(i).src_info(0).lsu_match := aiq1_create_data(i).src0_lsu_match
    io.out.toAiq1.create_data(i).src_info(0).src_data  := aiq1_create_data(i).src0_data

    io.out.toAiq1.create_data(i).DST_VREG := aiq1_create_data(i).dst_vreg
    io.out.toAiq1.create_data(i).DST_PREG := aiq1_create_data(i).dst_preg
    io.out.toAiq1.create_data(i).DSTV_VLD := aiq1_create_data(i).dstv_vld
    io.out.toAiq1.create_data(i).DST_VLD  := aiq1_create_data(i).dst_vld
    io.out.toAiq1.create_data(i).src_vld  := aiq1_create_data(i).src_vld
    io.out.toAiq1.create_data(i).IID      := aiq1_create_iid(i)
    io.out.toAiq1.create_data(i).OPCODE   := aiq1_create_data(i).opcode
  }

  io.out.toAiq1.bypass_data.VL           := aiq1_create_data(0).VL
  io.out.toAiq1.bypass_data.LCH_PREG     := aiq1_create_data(0).LCH_PREG
  io.out.toAiq1.bypass_data.VSEW         := aiq1_create_data(0).VSEW
  io.out.toAiq1.bypass_data.VLMUL        := aiq1_create_data(0).VLMUL
  io.out.toAiq1.bypass_data.LCH_RDY_SDIQ := aiq1_create_lch_rdy_sdiq(0)
  io.out.toAiq1.bypass_data.LCH_RDY_LSIQ := aiq1_create_lch_rdy_lsiq(0)
  io.out.toAiq1.bypass_data.LCH_RDY_BIQ  := aiq1_create_lch_rdy_biq(0)
  io.out.toAiq1.bypass_data.LCH_RDY_AIQ1 := aiq1_create_lch_rdy_aiq1(0)
  io.out.toAiq1.bypass_data.LCH_RDY_AIQ0 := aiq1_create_lch_rdy_aiq0(0)
  io.out.toAiq1.bypass_data.ALU_SHORT    := aiq1_create_data(0).ALU_SHORT
  io.out.toAiq1.bypass_data.MLA          := aiq1_create_data(0).MLA
  io.out.toAiq1.bypass_data.MTVR         := aiq1_create_data(0).MTVR

  io.out.toAiq1.bypass_data.SRC2_LSU_MATCH            := 0.U
  io.out.toAiq1.bypass_data.SRC2_DATA.rdy             := 0.U
  io.out.toAiq1.bypass_data.SRC2_DATA.preg            := aiq1_create_data(0).src2_data.preg
  io.out.toAiq1.bypass_data.SRC2_DATA.wb              := aiq1_create_data(0).src2_data.wb
  io.out.toAiq1.bypass_data.SRC2_DATA.mla_rdy         := 0.U
  io.out.toAiq1.bypass_data.src_info(1).lsu_match     := 0.U
  io.out.toAiq1.bypass_data.src_info(1).src_data.preg := aiq1_create_data(0).src1_data.preg
  io.out.toAiq1.bypass_data.src_info(1).src_data.wb   := aiq1_create_data(0).src1_data.wb
  io.out.toAiq1.bypass_data.src_info(1).src_data.rdy  := 0.U
  io.out.toAiq1.bypass_data.src_info(0).lsu_match     := 0.U
  io.out.toAiq1.bypass_data.src_info(0).src_data.preg := aiq1_create_data(0).src0_data.preg
  io.out.toAiq1.bypass_data.src_info(0).src_data.wb   := aiq1_create_data(0).src0_data.wb
  io.out.toAiq1.bypass_data.src_info(0).src_data.rdy  := 0.U

  io.out.toAiq1.bypass_data.DST_VREG := aiq1_create_data(0).dst_vreg
  io.out.toAiq1.bypass_data.DST_PREG := aiq1_create_data(0).dst_preg
  io.out.toAiq1.bypass_data.DSTV_VLD := aiq1_create_data(0).dstv_vld
  io.out.toAiq1.bypass_data.DST_VLD  := aiq1_create_data(0).dst_vld
  io.out.toAiq1.bypass_data.src_vld  := aiq1_create_data(0).src_vld
  io.out.toAiq1.bypass_data.IID      := aiq1_create_iid(0)
  io.out.toAiq1.bypass_data.OPCODE   := aiq1_create_data(0).opcode

  io.out.toAiq1.src_rdy_for_bypass(0) := aiq1_create_data(0).src0_bp_rdy(1)
  io.out.toAiq1.src_rdy_for_bypass(1) := aiq1_create_data(0).src1_bp_rdy(1)
  io.out.toAiq1.src_rdy_for_bypass(2) := aiq1_create_data(0).src2_bp_rdy(1)
  io.out.toAiq1.create_alu := aiq1_create_data(0).ALU

  //----------------------------------------------------------
  //                  Create Data for BIQ
  //----------------------------------------------------------
  val biq_create_data = Wire(Vec(2, new ISData))
  val biq_create_iid  = Wire(Vec(2, UInt(7.W)))
  val biq_create_pid  = Wire(Vec(2, UInt(5.W)))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(2)(i).bits === 0.U){
      biq_create_data(i) := inst_read_data(0)
      biq_create_iid(i) := inst_iid(0)
      biq_create_pid(i) := inst_pid(0)
    }.elsewhen(dis_info.iq_create_sel(2)(i).bits === 1.U){
      biq_create_data(i) := inst_read_data(1)
      biq_create_iid(i) := inst_iid(1)
      biq_create_pid(i) := inst_pid(1)
    }.elsewhen(dis_info.iq_create_sel(2)(i).bits === 2.U){
      biq_create_data(i) := inst_read_data(2)
      biq_create_iid(i) := inst_iid(2)
      biq_create_pid(i) := inst_pid(2)
    }.elsewhen(dis_info.iq_create_sel(2)(i).bits === 3.U){
      biq_create_data(i) := inst_read_data(3)
      biq_create_iid(i) := inst_iid(3)
      biq_create_pid(i) := inst_pid(3)
    }.otherwise{
      biq_create_data(i) := 0.U.asTypeOf(new ISData)
      biq_create_iid(i) := 0.U
      biq_create_pid(i) := 0.U
    }
  }

  //----------------------------------------------------------
  //                Reorganize for BIQ create
  //----------------------------------------------------------
  for(i <- 0 until 2){
    io.out.toBiq.create_data(i).VL     := biq_create_data(i).VL
    io.out.toBiq.create_data(i).VSEW   := biq_create_data(i).VSEW
    io.out.toBiq.create_data(i).VLMUL  := biq_create_data(i).VLMUL
    io.out.toBiq.create_data(i).PCALL  := biq_create_data(i).PCALL
    io.out.toBiq.create_data(i).RTS    := biq_create_data(i).RTS
    io.out.toBiq.create_data(i).PID    := biq_create_pid(i)
    io.out.toBiq.create_data(i).LENGTH := biq_create_data(i).LENGTH

    io.out.toBiq.create_data(i).src_info(1).lsu_match := biq_create_data(i).src1_lsu_match
    io.out.toBiq.create_data(i).src_info(1).src_data  := biq_create_data(i).src1_data
    io.out.toBiq.create_data(i).src_info(0).lsu_match := biq_create_data(i).src0_lsu_match
    io.out.toBiq.create_data(i).src_info(0).src_data  := biq_create_data(i).src0_data

    io.out.toBiq.create_data(i).src_vld  := biq_create_data(i).src_vld.take(2)
    io.out.toBiq.create_data(i).IID      := biq_create_iid(i)
    io.out.toBiq.create_data(i).OPCODE   := biq_create_data(i).opcode
  }

  io.out.toBiq.bypass_data.VL     := biq_create_data(0).VL
  io.out.toBiq.bypass_data.VSEW   := biq_create_data(0).VSEW
  io.out.toBiq.bypass_data.VLMUL  := biq_create_data(0).VLMUL
  io.out.toBiq.bypass_data.PCALL  := biq_create_data(0).PCALL
  io.out.toBiq.bypass_data.RTS    := biq_create_data(0).RTS
  io.out.toBiq.bypass_data.PID    := biq_create_pid(0)
  io.out.toBiq.bypass_data.LENGTH := biq_create_data(0).LENGTH

  io.out.toBiq.bypass_data.src_info(1).lsu_match     := 0.U
  io.out.toBiq.bypass_data.src_info(1).src_data.preg := biq_create_data(0).src1_data.preg
  io.out.toBiq.bypass_data.src_info(1).src_data.wb   := biq_create_data(0).src1_data.wb
  io.out.toBiq.bypass_data.src_info(1).src_data.rdy  := 0.U
  io.out.toBiq.bypass_data.src_info(0).lsu_match     := 0.U
  io.out.toBiq.bypass_data.src_info(0).src_data.preg := biq_create_data(0).src0_data.preg
  io.out.toBiq.bypass_data.src_info(0).src_data.wb   := biq_create_data(0).src0_data.wb
  io.out.toBiq.bypass_data.src_info(0).src_data.rdy  := 0.U

  io.out.toBiq.bypass_data.src_vld  := biq_create_data(0).src_vld.take(2)
  io.out.toBiq.bypass_data.IID      := biq_create_iid(0)
  io.out.toBiq.bypass_data.OPCODE   := biq_create_data(0).opcode

  io.out.toBiq.src_rdy_for_bypass(0) := biq_create_data(0).src0_bp_rdy(1)
  io.out.toBiq.src_rdy_for_bypass(1) := biq_create_data(0).src1_bp_rdy(1)


  //----------------------------------------------------------
  //                  Create Data for LSIQ
  //----------------------------------------------------------
  val lsiq_create_data = Wire(Vec(2, new ISData))
  val lsiq_create_iid  = Wire(Vec(2, UInt(7.W)))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(3)(i).bits === 0.U){
      lsiq_create_data(i) := inst_read_data(0)
      lsiq_create_iid(i) := inst_iid(0)
    }.elsewhen(dis_info.iq_create_sel(3)(i).bits === 1.U){
      lsiq_create_data(i) := inst_read_data(1)
      lsiq_create_iid(i) := inst_iid(1)
    }.elsewhen(dis_info.iq_create_sel(3)(i).bits === 2.U){
      lsiq_create_data(i) := inst_read_data(2)
      lsiq_create_iid(i) := inst_iid(2)
    }.elsewhen(dis_info.iq_create_sel(3)(i).bits === 3.U){
      lsiq_create_data(i) := inst_read_data(3)
      lsiq_create_iid(i) := inst_iid(3)
    }.otherwise{
      lsiq_create_data(i) := 0.U.asTypeOf(new ISData)
      lsiq_create_iid(i) := 0.U
    }
  }
  //staddr indicate inst created into sdiq
  //some vector load use vmb entry id instead of sdiq entry id
  val is_lsiq_create_entry = Wire(Vec(2, UInt(12.W)))
  is_lsiq_create_entry(0) := Mux(lsiq_create_data(0).LOAD, Cat(0.U(4.W), io.in.fromLSU.vmb_create0_entry), io.in.iq_create_entry.sdiq_dp(0))
  is_lsiq_create_entry(1) := Mux(lsiq_create_data(1).LOAD, Cat(0.U(4.W), io.in.fromLSU.vmb_create1_entry), sdiq_vmb_create1_entry)

  sdiq_vmb_create1_dp_en := Mux(lsiq_create_data(0).STADDR, dis_info.iq_create_sel(4)(1).valid, dis_info.iq_create_sel(4)(0).valid)
  sdiq_vmb_create1_entry := Mux(lsiq_create_data(0).STADDR, io.in.iq_create_entry.sdiq_dp(1), io.in.iq_create_entry.sdiq_dp(0))

  //----------------------------------------------------------
  //                Reorganize for LSIQ create
  //----------------------------------------------------------
  val lsiq_create_sti_sel = Wire(Vec(2, Bool()))
  for(i <- 0 until 2){
    lsiq_create_sti_sel(i) := lsiq_create_data(i).STADDR && !lsiq_create_data(i).STR

    io.out.toLsiq.create_data(i).VL            := lsiq_create_data(i).VL
    io.out.toLsiq.create_data(i).VMB           := lsiq_create_data(i).VMB
    io.out.toLsiq.create_data(i).SPLIT_NUM     := lsiq_create_data(i).SPLIT_NUM
    io.out.toLsiq.create_data(i).VSEW          := lsiq_create_data(i).VSEW
    io.out.toLsiq.create_data(i).VLMUL         := lsiq_create_data(i).VLMUL
    io.out.toLsiq.create_data(i).BKPTB_DATA    := 0.U
    io.out.toLsiq.create_data(i).BKPTA_DATA    := 0.U
    io.out.toLsiq.create_data(i).AGEVEC_ALL    := 0.U
    io.out.toLsiq.create_data(i).ALREADY_DA    := 0.U
    io.out.toLsiq.create_data(i).UNALIGN_2ND   := 0.U
    io.out.toLsiq.create_data(i).SPEC_FAIL     := 0.U
    io.out.toLsiq.create_data(i).NO_SPEC_EXIST := io.in.lsiq_dp_no_spec_store_vld
    io.out.toLsiq.create_data(i).NO_SPEC       := lsiq_create_data(i).NO_SPEC
    io.out.toLsiq.create_data(i).SPLIT         := lsiq_create_data(i).SPLIT
    io.out.toLsiq.create_data(i).SDIQ_ENTRY    := is_lsiq_create_entry(i)
    io.out.toLsiq.create_data(i).STADDR        := lsiq_create_data(i).STADDR
    io.out.toLsiq.create_data(i).PC            := lsiq_create_data(i).LSU_PC
    io.out.toLsiq.create_data(i).BAR_TYPE      := lsiq_create_data(i).BAR_TYPE
    io.out.toLsiq.create_data(i).BAR           := lsiq_create_data(i).BAR
    io.out.toLsiq.create_data(i).STORE         := lsiq_create_data(i).STORE
    io.out.toLsiq.create_data(i).LOAD          := lsiq_create_data(i).LOAD

    io.out.toLsiq.create_data(i).SRCVM_LSU_MATCH := Mux(lsiq_create_data(i).srcv_vld(1), lsiq_create_data(i).srcv1_lsu_match, lsiq_create_data(i).srcvm_lsu_match)
    io.out.toLsiq.create_data(i).SRCVM_DATA      := Mux(lsiq_create_data(i).srcv_vld(1), lsiq_create_data(i).srcv1_data, lsiq_create_data(i).srcvm_data)
    io.out.toLsiq.create_data(i).src_info(1).lsu_match := Mux(lsiq_create_sti_sel(i), false.B, lsiq_create_data(i).src1_lsu_match)
    io.out.toLsiq.create_data(i).src_info(1).src_data  := Mux(lsiq_create_sti_sel(i), "b0000000_1_1".U.asTypeOf(new srcData9), lsiq_create_data(i).src1_data)
    io.out.toLsiq.create_data(i).src_info(0).lsu_match := lsiq_create_data(i).src0_lsu_match
    io.out.toLsiq.create_data(i).src_info(0).src_data  := lsiq_create_data(i).src0_data

    io.out.toLsiq.create_data(i).DST_VREG   := lsiq_create_data(i).dst_vreg
    io.out.toLsiq.create_data(i).DST_PREG   := lsiq_create_data(i).dst_preg
    io.out.toLsiq.create_data(i).DSTV_VLD   := lsiq_create_data(i).dstv_vld
    io.out.toLsiq.create_data(i).DST_VLD    := lsiq_create_data(i).dst_vld
    io.out.toLsiq.create_data(i).SRCVM_VLD  := Mux(lsiq_create_data(i).srcv_vld(1), true.B, lsiq_create_data(i).srcvm_vld)
    io.out.toLsiq.create_data(i).src_vld(1) := lsiq_create_data(i).src_vld(1) && !lsiq_create_sti_sel(i)
    io.out.toLsiq.create_data(i).src_vld(0) := lsiq_create_data(i).src_vld(0)
    io.out.toLsiq.create_data(i).IID        := lsiq_create_iid(i)
    io.out.toLsiq.create_data(i).OPCODE     := lsiq_create_data(i).opcode
  }

  io.out.toLsiq.bypass_data.VL            := lsiq_create_data(0).VL
  io.out.toLsiq.bypass_data.VMB           := lsiq_create_data(0).VMB
  io.out.toLsiq.bypass_data.SPLIT_NUM     := lsiq_create_data(0).SPLIT_NUM
  io.out.toLsiq.bypass_data.VSEW          := lsiq_create_data(0).VSEW
  io.out.toLsiq.bypass_data.VLMUL         := lsiq_create_data(0).VLMUL
  io.out.toLsiq.bypass_data.BKPTB_DATA    := 0.U
  io.out.toLsiq.bypass_data.BKPTA_DATA    := 0.U
  io.out.toLsiq.bypass_data.AGEVEC_ALL    := Cat(0.U(10.W), !io.in.lsiq_dp_create_bypass_oldest)
  io.out.toLsiq.bypass_data.ALREADY_DA    := 0.U
  io.out.toLsiq.bypass_data.UNALIGN_2ND   := 0.U
  io.out.toLsiq.bypass_data.SPEC_FAIL     := 0.U
  io.out.toLsiq.bypass_data.NO_SPEC_EXIST := io.in.lsiq_dp_no_spec_store_vld
  io.out.toLsiq.bypass_data.NO_SPEC       := lsiq_create_data(0).NO_SPEC
  io.out.toLsiq.bypass_data.SPLIT         := lsiq_create_data(0).SPLIT
  io.out.toLsiq.bypass_data.SDIQ_ENTRY    := is_lsiq_create_entry(0)
  io.out.toLsiq.bypass_data.STADDR        := lsiq_create_data(0).STADDR
  io.out.toLsiq.bypass_data.PC            := lsiq_create_data(0).LSU_PC
  io.out.toLsiq.bypass_data.BAR_TYPE      := lsiq_create_data(0).BAR_TYPE
  io.out.toLsiq.bypass_data.BAR           := lsiq_create_data(0).BAR
  io.out.toLsiq.bypass_data.STORE         := lsiq_create_data(0).STORE
  io.out.toLsiq.bypass_data.LOAD          := lsiq_create_data(0).LOAD

  io.out.toLsiq.bypass_data.SRCVM_LSU_MATCH := 0.U
  io.out.toLsiq.bypass_data.SRCVM_DATA.preg := Mux(lsiq_create_data(0).srcv_vld(1), lsiq_create_data(0).srcv1_data.preg, lsiq_create_data(0).srcvm_data.preg)
  io.out.toLsiq.bypass_data.SRCVM_DATA.wb   := Mux(lsiq_create_data(0).srcv_vld(1), lsiq_create_data(0).srcv1_data.wb, lsiq_create_data(0).srcvm_data.wb)
  io.out.toLsiq.bypass_data.SRCVM_DATA.rdy  := 0.U
  io.out.toLsiq.bypass_data.src_info(1).lsu_match     := 0.U
  io.out.toLsiq.bypass_data.src_info(1).src_data.preg := lsiq_create_data(0).src1_data.preg
  io.out.toLsiq.bypass_data.src_info(1).src_data.wb   := lsiq_create_data(0).src1_data.wb || lsiq_create_sti_sel(0)
  io.out.toLsiq.bypass_data.src_info(1).src_data.rdy  := 0.U
  io.out.toLsiq.bypass_data.src_info(0).lsu_match     := 0.U
  io.out.toLsiq.bypass_data.src_info(0).src_data.preg := lsiq_create_data(0).src0_data.preg
  io.out.toLsiq.bypass_data.src_info(0).src_data.wb   := lsiq_create_data(0).src0_data.wb
  io.out.toLsiq.bypass_data.src_info(0).src_data.rdy  := 0.U

  io.out.toLsiq.bypass_data.DST_VREG   := lsiq_create_data(0).dst_vreg
  io.out.toLsiq.bypass_data.DST_PREG   := lsiq_create_data(0).dst_preg
  io.out.toLsiq.bypass_data.DSTV_VLD   := lsiq_create_data(0).dstv_vld
  io.out.toLsiq.bypass_data.DST_VLD    := lsiq_create_data(0).dst_vld
  io.out.toLsiq.bypass_data.SRCVM_VLD  := Mux(lsiq_create_data(0).srcv_vld(1), true.B, lsiq_create_data(0).srcvm_vld)
  io.out.toLsiq.bypass_data.src_vld(1) := lsiq_create_data(0).src_vld(1) && !lsiq_create_sti_sel(0)
  io.out.toLsiq.bypass_data.src_vld(0) := lsiq_create_data(0).src_vld(0)
  io.out.toLsiq.bypass_data.IID        := lsiq_create_iid(0)
  io.out.toLsiq.bypass_data.OPCODE     := lsiq_create_data(0).opcode

  io.out.toLsiq.create0_src_rdy_for_bypass(0) := lsiq_create_data(0).src0_bp_rdy(1)
  io.out.toLsiq.create0_src_rdy_for_bypass(1) := lsiq_create_data(0).src1_bp_rdy(1) || lsiq_create_sti_sel(0)
  io.out.toLsiq.create0_srcvm_rdy_for_bypass  := Mux(lsiq_create_data(0).srcv_vld(1), lsiq_create_data(0).srcv1_bp_rdy(1), lsiq_create_data(0).srcvm_bp_rdy(1))

  for(i <- 0 until 2){
    io.out.toLsiq.create_load(i)    := lsiq_create_data(i).LOAD
    io.out.toLsiq.create_store(i)   := lsiq_create_data(i).STORE
    io.out.toLsiq.create_bar(i)     := lsiq_create_data(i).BAR
    io.out.toLsiq.create_no_spec(i) := lsiq_create_data(i).NO_SPEC
  }

  //----------------------------------------------------------
  //                  Create Data for SDIQ
  //----------------------------------------------------------
  val sdiq_create_data = Wire(Vec(2, new ISData))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(4)(i).bits === 0.U){
      sdiq_create_data(i) := inst_read_data(0)
    }.elsewhen(dis_info.iq_create_sel(4)(i).bits === 1.U){
      sdiq_create_data(i) := inst_read_data(1)
    }.elsewhen(dis_info.iq_create_sel(4)(i).bits === 2.U){
      sdiq_create_data(i) := inst_read_data(2)
    }.elsewhen(dis_info.iq_create_sel(4)(i).bits === 3.U){
      sdiq_create_data(i) := inst_read_data(3)
    }.otherwise{
      sdiq_create_data(i) := 0.U.asTypeOf(new ISData)
    }
  }

  //----------------------------------------------------------
  //                Reorganize for SDIQ create
  //----------------------------------------------------------
  //val sdiq_create_sti_sel = Wire(Vec(2, Bool()))
  for(i <- 0 until 2){
    dp_sdiq_create_sti_sel(i) := !sdiq_create_data(i).STR
    io.out.toAiq.sdiq_create_src_sel(i) := dp_sdiq_create_sti_sel(i)

    io.out.sdiq_create_data(i).LOAD            := sdiq_create_data(i).LOAD
    io.out.sdiq_create_data(i).STADDR1_IN_STQ  := 0.U
    io.out.sdiq_create_data(i).STADDR0_IN_STQ  := 0.U
    io.out.sdiq_create_data(i).STDATA1_VLD     := 0.U
    io.out.sdiq_create_data(i).UNALIGN         := 0.U
    io.out.sdiq_create_data(i).SRCV0_LSU_MATCH := sdiq_create_data(i).srcv2_lsu_match
    io.out.sdiq_create_data(i).SRCV0_DATA      := sdiq_create_data(i).srcv2_data.asUInt(8,0).asTypeOf(new srcData9)
    io.out.sdiq_create_data(i).SRC0_LSU_MATCH  := Mux(dp_sdiq_create_sti_sel(i), sdiq_create_data(i).src1_lsu_match, sdiq_create_data(i).src2_lsu_match)
    io.out.sdiq_create_data(i).SRC0_DATA       := Mux(dp_sdiq_create_sti_sel(i), sdiq_create_data(i).src1_data, sdiq_create_data(i).src2_data.asUInt(8,0).asTypeOf(new srcData9))
    io.out.sdiq_create_data(i).SRCV0_VLD       := sdiq_create_data(i).srcv_vld(2)
    io.out.sdiq_create_data(i).SRC0_VLD        := Mux(dp_sdiq_create_sti_sel(i), sdiq_create_data(i).src_vld(1), sdiq_create_data(i).src_vld(2))
  }

  //----------------------------------------------------------
  //                  Create Data for VIQ0
  //----------------------------------------------------------
  val viq0_create_data         = Wire(Vec(2, new ISData))
  val viq0_create_iid          = Wire(Vec(2, UInt(7.W)))
  val viq0_create_lch_rdy_viq0 = Wire(Vec(2, Vec(8, Bool())))
  val viq0_create_lch_rdy_viq1 = Wire(Vec(2, Vec(8, Bool())))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(5)(i).bits === 0.U){
      viq0_create_data(i)         := inst_read_data(0)
      viq0_create_iid(i)          := inst_iid(0)
      viq0_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(0)
      viq0_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(0)
    }.elsewhen(dis_info.iq_create_sel(5)(i).bits === 1.U){
      viq0_create_data(i)         := inst_read_data(1)
      viq0_create_iid(i)          := inst_iid(1)
      viq0_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(1)
      viq0_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(1)
    }.elsewhen(dis_info.iq_create_sel(5)(i).bits === 2.U){
      viq0_create_data(i)         := inst_read_data(2)
      viq0_create_iid(i)          := inst_iid(2)
      viq0_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(2)
      viq0_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(2)
    }.elsewhen(dis_info.iq_create_sel(5)(i).bits === 3.U){
      viq0_create_data(i)         := inst_read_data(3)
      viq0_create_iid(i)          := inst_iid(3)
      viq0_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(3)
      viq0_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(3)
    }.otherwise{
      viq0_create_data(i)         := 0.U.asTypeOf(new ISData)
      viq0_create_iid(i)          := 0.U
      viq0_create_lch_rdy_viq0(i) := WireInit(VecInit(Seq.fill(8)(false.B)))
      viq0_create_lch_rdy_viq1(i) := WireInit(VecInit(Seq.fill(8)(false.B)))
    }
  }

  //----------------------------------------------------------
  //                Reorganize for VIQ0 create
  //----------------------------------------------------------
  val out_viq0_create_data = WireInit(VecInit(Seq.fill(2)(0.U.asTypeOf(new VIQData))))

  for(i <- 0 until 2){
    out_viq0_create_data(i).VL           := viq0_create_data(i).VL
    out_viq0_create_data(i).VSEW         := viq0_create_data(i).VSEW
    out_viq0_create_data(i).VLMUL        := viq0_create_data(i).VLMUL
    out_viq0_create_data(i).VMUL         := viq0_create_data(i).VMUL
    //is_viq0_create_data(i).VMUL_UNSPLIT := viq0_create_data(i).VMUL_UNSPLIT
    out_viq0_create_data(i).VMLA_SHORT   := viq0_create_data(i).VMLA_SHORT
    out_viq0_create_data(i).VDIV         := viq0_create_data(i).VDIV
    out_viq0_create_data(i).LCH_RDY_VIQ1 := viq0_create_lch_rdy_viq1(i)
    out_viq0_create_data(i).LCH_RDY_VIQ0 := viq0_create_lch_rdy_viq0(i)
    out_viq0_create_data(i).VMLA_TYPE    := viq0_create_data(i).VMLA_TYPE
    out_viq0_create_data(i).SPLIT_NUM    := viq0_create_data(i).SPLIT_NUM
    out_viq0_create_data(i).SPLIT_LAST   := viq0_create_data(i).SPLIT_LAST
    out_viq0_create_data(i).MFVR         := viq0_create_data(i).MFVR
    out_viq0_create_data(i).VMLA         := viq0_create_data(i).VMLA

    out_viq0_create_data(i).SRCVM_LSU_MATCH := viq0_create_data(i).srcvm_lsu_match
    out_viq0_create_data(i).SRCVM_DATA      := viq0_create_data(i).srcvm_data
    out_viq0_create_data(i).SRCV2_LSU_MATCH := Mux(viq0_create_data(i).VIQ_SRCV12_SWITCH, viq0_create_data(i).srcv1_lsu_match, viq0_create_data(i).srcv2_lsu_match)
    out_viq0_create_data(i).SRCV2_DATA      := Mux(viq0_create_data(i).VIQ_SRCV12_SWITCH,
                                                Cat(0.U(1.W),viq0_create_data(i).srcv1_data.asUInt).asTypeOf(new srcData10),
                                                viq0_create_data(i).srcv2_data)
    out_viq0_create_data(i).srcv_info(1).lsu_match := Mux(viq0_create_data(i).VIQ_SRCV12_SWITCH, viq0_create_data(i).srcv2_lsu_match, viq0_create_data(i).srcv1_lsu_match)
    out_viq0_create_data(i).srcv_info(1).srcv_data := Mux(viq0_create_data(i).VIQ_SRCV12_SWITCH,
                                                        viq0_create_data(i).srcv2_data.asUInt(8,0).asTypeOf(new srcData9),
                                                        viq0_create_data(i).srcv1_data)
    out_viq0_create_data(i).srcv_info(0).lsu_match := viq0_create_data(i).srcv0_lsu_match
    out_viq0_create_data(i).srcv_info(0).srcv_data := viq0_create_data(i).srcv0_data

    out_viq0_create_data(i).DST_EREG     := viq0_create_data(i).dst_ereg
    out_viq0_create_data(i).DST_VREG     := viq0_create_data(i).dst_vreg
    out_viq0_create_data(i).DST_PREG     := viq0_create_data(i).dst_preg
    out_viq0_create_data(i).DSTE_VLD     := viq0_create_data(i).dste_vld
    out_viq0_create_data(i).DSTV_VLD     := viq0_create_data(i).dstv_vld
    out_viq0_create_data(i).DST_VLD      := viq0_create_data(i).dst_vld
    out_viq0_create_data(i).SRCVM_VLD    := viq0_create_data(i).srcvm_vld
    out_viq0_create_data(i).srcv_vld(2)  := Mux(viq0_create_data(i).VIQ_SRCV12_SWITCH, viq0_create_data(i).srcv_vld(1), viq0_create_data(i).srcv_vld(2))
    out_viq0_create_data(i).srcv_vld(1)  := Mux(viq0_create_data(i).VIQ_SRCV12_SWITCH, viq0_create_data(i).srcv_vld(2), viq0_create_data(i).srcv_vld(1))
    out_viq0_create_data(i).srcv_vld(0)  := viq0_create_data(i).srcv_vld(0)
    out_viq0_create_data(i).IID          := viq0_create_iid(i)
    out_viq0_create_data(i).OPCODE       := viq0_create_data(i).opcode
  }
  io.out.toViq0.create_data := out_viq0_create_data

  io.out.toViq0.bypass_data.VL           := viq0_create_data(0).VL
  io.out.toViq0.bypass_data.VSEW         := viq0_create_data(0).VSEW
  io.out.toViq0.bypass_data.VLMUL        := viq0_create_data(0).VLMUL
  io.out.toViq0.bypass_data.VMUL         := viq0_create_data(0).VMUL
  io.out.toViq0.bypass_data.VMUL_UNSPLIT := 0.U
  io.out.toViq0.bypass_data.VMLA_SHORT   := viq0_create_data(0).VMLA_SHORT
  io.out.toViq0.bypass_data.VDIV         := viq0_create_data(0).VDIV
  io.out.toViq0.bypass_data.LCH_RDY_VIQ1 := viq0_create_lch_rdy_viq1(0)
  io.out.toViq0.bypass_data.LCH_RDY_VIQ0 := viq0_create_lch_rdy_viq0(0)
  io.out.toViq0.bypass_data.VMLA_TYPE    := viq0_create_data(0).VMLA_TYPE
  io.out.toViq0.bypass_data.SPLIT_NUM    := viq0_create_data(0).SPLIT_NUM
  io.out.toViq0.bypass_data.SPLIT_LAST   := viq0_create_data(0).SPLIT_LAST
  io.out.toViq0.bypass_data.MFVR         := viq0_create_data(0).MFVR
  io.out.toViq0.bypass_data.VMLA         := viq0_create_data(0).VMLA

  io.out.toViq0.bypass_data.SRCVM_LSU_MATCH             := 0.U
  io.out.toViq0.bypass_data.SRCVM_DATA.preg             := viq0_create_data(0).srcvm_data.preg
  io.out.toViq0.bypass_data.SRCVM_DATA.wb               := viq0_create_data(0).srcvm_data.wb
  io.out.toViq0.bypass_data.SRCVM_DATA.rdy              := 0.U
  io.out.toViq0.bypass_data.SRCV2_LSU_MATCH             := 0.U
  io.out.toViq0.bypass_data.SRCV2_DATA.rdy              := 0.U
  io.out.toViq0.bypass_data.SRCV2_DATA.preg             := Mux(viq0_create_data(0).VIQ_SRCV12_SWITCH, viq0_create_data(0).srcv1_data.preg, viq0_create_data(0).srcv2_data.preg)
  io.out.toViq0.bypass_data.SRCV2_DATA.wb               := Mux(viq0_create_data(0).VIQ_SRCV12_SWITCH, viq0_create_data(0).srcv1_data.wb, viq0_create_data(0).srcv2_data.wb)
  io.out.toViq0.bypass_data.SRCV2_DATA.mla_rdy          := 0.U
  io.out.toViq0.bypass_data.srcv_info(1).lsu_match      := 0.U
  io.out.toViq0.bypass_data.srcv_info(1).srcv_data.preg := Mux(viq0_create_data(0).VIQ_SRCV12_SWITCH, viq0_create_data(0).srcv2_data.preg, viq0_create_data(0).srcv1_data.preg)
  io.out.toViq0.bypass_data.srcv_info(1).srcv_data.wb   := Mux(viq0_create_data(0).VIQ_SRCV12_SWITCH, viq0_create_data(0).srcv2_data.wb, viq0_create_data(0).srcv1_data.wb)
  io.out.toViq0.bypass_data.srcv_info(1).srcv_data.rdy  := 0.U
  io.out.toViq0.bypass_data.srcv_info(0).lsu_match      := 0.U
  io.out.toViq0.bypass_data.srcv_info(0).srcv_data.preg := viq0_create_data(0).srcv0_data.preg
  io.out.toViq0.bypass_data.srcv_info(0).srcv_data.wb   := viq0_create_data(0).srcv0_data.wb
  io.out.toViq0.bypass_data.srcv_info(0).srcv_data.rdy  := 0.U

  io.out.toViq0.bypass_data.DST_EREG     := viq0_create_data(0).dst_ereg
  io.out.toViq0.bypass_data.DST_VREG     := viq0_create_data(0).dst_vreg
  io.out.toViq0.bypass_data.DST_PREG     := viq0_create_data(0).dst_preg
  io.out.toViq0.bypass_data.DSTE_VLD     := viq0_create_data(0).dste_vld
  io.out.toViq0.bypass_data.DSTV_VLD     := viq0_create_data(0).dstv_vld
  io.out.toViq0.bypass_data.DST_VLD      := viq0_create_data(0).dst_vld
  io.out.toViq0.bypass_data.SRCVM_VLD    := viq0_create_data(0).srcvm_vld
  io.out.toViq0.bypass_data.srcv_vld(2)  := Mux(viq0_create_data(0).VIQ_SRCV12_SWITCH, viq0_create_data(0).srcv_vld(1), viq0_create_data(0).srcv_vld(2))
  io.out.toViq0.bypass_data.srcv_vld(1)  := Mux(viq0_create_data(0).VIQ_SRCV12_SWITCH, viq0_create_data(0).srcv_vld(2), viq0_create_data(0).srcv_vld(1))
  io.out.toViq0.bypass_data.srcv_vld(0)  := viq0_create_data(0).srcv_vld(0)
  io.out.toViq0.bypass_data.IID          := viq0_create_iid(0)
  io.out.toViq0.bypass_data.OPCODE       := viq0_create_data(0).opcode

  io.out.toViq0.srcv_rdy_for_bypass(0) := viq0_create_data(0).srcv0_bp_rdy
  io.out.toViq0.srcv_rdy_for_bypass(1) := viq0_create_data(0).srcv1_bp_rdy
  io.out.toViq0.srcv_rdy_for_bypass(2) := viq0_create_data(0).srcv2_bp_rdy
  io.out.toViq0.srcvm_rdy_for_bypass   := viq0_create_data(0).srcvm_bp_rdy
  io.out.toViq0.create_vdiv            := viq0_create_data(0).VDIV

  //----------------------------------------------------------
  //                  Create Data for VIQ1
  //----------------------------------------------------------
  val viq1_create_data         = Wire(Vec(2, new ISData))
  val viq1_create_iid          = Wire(Vec(2, UInt(7.W)))
  val viq1_create_lch_rdy_viq0 = Wire(Vec(2, Vec(8, Bool())))
  val viq1_create_lch_rdy_viq1 = Wire(Vec(2, Vec(8, Bool())))

  for(i <- 0 until 2){
    when(dis_info.iq_create_sel(6)(i).bits === 0.U){
      viq1_create_data(i)         := inst_read_data(0)
      viq1_create_iid(i)          := inst_iid(0)
      viq1_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(0)
      viq1_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(0)
    }.elsewhen(dis_info.iq_create_sel(6)(i).bits === 1.U){
      viq1_create_data(i)         := inst_read_data(1)
      viq1_create_iid(i)          := inst_iid(1)
      viq1_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(1)
      viq1_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(1)
    }.elsewhen(dis_info.iq_create_sel(6)(i).bits === 2.U){
      viq1_create_data(i)         := inst_read_data(2)
      viq1_create_iid(i)          := inst_iid(2)
      viq1_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(2)
      viq1_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(2)
    }.elsewhen(dis_info.iq_create_sel(6)(i).bits === 3.U){
      viq1_create_data(i)         := inst_read_data(3)
      viq1_create_iid(i)          := inst_iid(3)
      viq1_create_lch_rdy_viq0(i) := inst_lch_rdy_viq0(3)
      viq1_create_lch_rdy_viq1(i) := inst_lch_rdy_viq1(3)
    }.otherwise{
      viq1_create_data(i)         := 0.U.asTypeOf(new ISData)
      viq1_create_iid(i)          := 0.U
      viq1_create_lch_rdy_viq0(i) := WireInit(VecInit(Seq.fill(8)(false.B)))
      viq1_create_lch_rdy_viq1(i) := WireInit(VecInit(Seq.fill(8)(false.B)))
    }
  }

  //----------------------------------------------------------
  //                Reorganize for VIQ1 create
  //----------------------------------------------------------
  val out_viq1_create_data = WireInit(VecInit(Seq.fill(2)(0.U.asTypeOf(new VIQData))))

  for(i <- 0 until 2){
    out_viq1_create_data(i).VL           := viq1_create_data(i).VL
    out_viq1_create_data(i).VSEW         := viq1_create_data(i).VSEW
    out_viq1_create_data(i).VLMUL        := viq1_create_data(i).VLMUL
    //out_viq1_create_data(i).VMUL         := viq0_create_data(i).VMUL
    out_viq1_create_data(i).VMUL_UNSPLIT := viq1_create_data(i).VMUL_UNSPLIT
    out_viq1_create_data(i).VMLA_SHORT   := viq1_create_data(i).VMLA_SHORT
    //out_viq1_create_data(i).VDIV         := viq0_create_data(i).VDIV
    out_viq1_create_data(i).LCH_RDY_VIQ1 := viq1_create_lch_rdy_viq1(i)
    out_viq1_create_data(i).LCH_RDY_VIQ0 := viq1_create_lch_rdy_viq0(i)
    out_viq1_create_data(i).VMLA_TYPE    := viq1_create_data(i).VMLA_TYPE
    out_viq1_create_data(i).SPLIT_NUM    := viq1_create_data(i).SPLIT_NUM
    out_viq1_create_data(i).SPLIT_LAST   := viq1_create_data(i).SPLIT_LAST
    out_viq1_create_data(i).MFVR         := viq1_create_data(i).MFVR
    out_viq1_create_data(i).VMLA         := viq1_create_data(i).VMLA

    out_viq1_create_data(i).SRCVM_LSU_MATCH := viq1_create_data(i).srcvm_lsu_match
    out_viq1_create_data(i).SRCVM_DATA      := viq1_create_data(i).srcvm_data
    out_viq1_create_data(i).SRCV2_LSU_MATCH := Mux(viq1_create_data(i).VIQ_SRCV12_SWITCH, viq1_create_data(i).srcv1_lsu_match, viq1_create_data(i).srcv2_lsu_match)
    out_viq1_create_data(i).SRCV2_DATA      := Mux(viq1_create_data(i).VIQ_SRCV12_SWITCH,
      Cat(0.U(1.W),viq1_create_data(i).srcv1_data.asUInt).asTypeOf(new srcData10),
      viq1_create_data(i).srcv2_data)
    out_viq1_create_data(i).srcv_info(1).lsu_match := Mux(viq1_create_data(i).VIQ_SRCV12_SWITCH, viq1_create_data(i).srcv2_lsu_match, viq1_create_data(i).srcv1_lsu_match)
    out_viq1_create_data(i).srcv_info(1).srcv_data := Mux(viq1_create_data(i).VIQ_SRCV12_SWITCH,
      viq1_create_data(i).srcv2_data.asUInt(8,0).asTypeOf(new srcData9),
      viq1_create_data(i).srcv1_data)
    out_viq1_create_data(i).srcv_info(0).lsu_match := viq1_create_data(i).srcv0_lsu_match
    out_viq1_create_data(i).srcv_info(0).srcv_data := viq1_create_data(i).srcv0_data

    out_viq1_create_data(i).DST_EREG     := viq1_create_data(i).dst_ereg
    out_viq1_create_data(i).DST_VREG     := viq1_create_data(i).dst_vreg
    out_viq1_create_data(i).DST_PREG     := viq1_create_data(i).dst_preg
    out_viq1_create_data(i).DSTE_VLD     := viq1_create_data(i).dste_vld
    out_viq1_create_data(i).DSTV_VLD     := viq1_create_data(i).dstv_vld
    out_viq1_create_data(i).DST_VLD      := viq1_create_data(i).dst_vld
    out_viq1_create_data(i).SRCVM_VLD    := viq1_create_data(i).srcvm_vld
    out_viq1_create_data(i).srcv_vld(2)  := Mux(viq1_create_data(i).VIQ_SRCV12_SWITCH, viq1_create_data(i).srcv_vld(1), viq1_create_data(i).srcv_vld(2))
    out_viq1_create_data(i).srcv_vld(1)  := Mux(viq1_create_data(i).VIQ_SRCV12_SWITCH, viq1_create_data(i).srcv_vld(2), viq1_create_data(i).srcv_vld(1))
    out_viq1_create_data(i).srcv_vld(0)  := viq1_create_data(i).srcv_vld(0)
    out_viq1_create_data(i).IID          := viq1_create_iid(i)
    out_viq1_create_data(i).OPCODE       := viq1_create_data(i).opcode
  }
  io.out.toViq1.create_data := out_viq1_create_data

  io.out.toViq1.bypass_data.VL           := viq1_create_data(0).VL
  io.out.toViq1.bypass_data.VSEW         := viq1_create_data(0).VSEW
  io.out.toViq1.bypass_data.VLMUL        := viq1_create_data(0).VLMUL
  io.out.toViq1.bypass_data.VMUL         := 0.U
  io.out.toViq1.bypass_data.VMUL_UNSPLIT := viq1_create_data(0).VMUL_UNSPLIT
  io.out.toViq1.bypass_data.VMLA_SHORT   := viq1_create_data(0).VMLA_SHORT
  io.out.toViq1.bypass_data.VDIV         := 0.U
  io.out.toViq1.bypass_data.LCH_RDY_VIQ1 := viq1_create_lch_rdy_viq1(0)
  io.out.toViq1.bypass_data.LCH_RDY_VIQ0 := viq1_create_lch_rdy_viq0(0)
  io.out.toViq1.bypass_data.VMLA_TYPE    := viq1_create_data(0).VMLA_TYPE
  io.out.toViq1.bypass_data.SPLIT_NUM    := viq1_create_data(0).SPLIT_NUM
  io.out.toViq1.bypass_data.SPLIT_LAST   := viq1_create_data(0).SPLIT_LAST
  io.out.toViq1.bypass_data.MFVR         := viq1_create_data(0).MFVR
  io.out.toViq1.bypass_data.VMLA         := viq1_create_data(0).VMLA

  io.out.toViq1.bypass_data.SRCVM_LSU_MATCH             := 0.U
  io.out.toViq1.bypass_data.SRCVM_DATA.preg             := viq1_create_data(0).srcvm_data.preg
  io.out.toViq1.bypass_data.SRCVM_DATA.wb               := viq1_create_data(0).srcvm_data.wb
  io.out.toViq1.bypass_data.SRCVM_DATA.rdy              := 0.U
  io.out.toViq1.bypass_data.SRCV2_LSU_MATCH             := 0.U
  io.out.toViq1.bypass_data.SRCV2_DATA.rdy              := 0.U
  io.out.toViq1.bypass_data.SRCV2_DATA.preg             := Mux(viq1_create_data(0).VIQ_SRCV12_SWITCH, viq1_create_data(0).srcv1_data.preg, viq1_create_data(0).srcv2_data.preg)
  io.out.toViq1.bypass_data.SRCV2_DATA.wb               := Mux(viq1_create_data(0).VIQ_SRCV12_SWITCH, viq1_create_data(0).srcv1_data.wb, viq1_create_data(0).srcv2_data.wb)
  io.out.toViq1.bypass_data.SRCV2_DATA.mla_rdy          := 0.U
  io.out.toViq1.bypass_data.srcv_info(1).lsu_match      := 0.U
  io.out.toViq1.bypass_data.srcv_info(1).srcv_data.preg := Mux(viq1_create_data(0).VIQ_SRCV12_SWITCH, viq1_create_data(0).srcv2_data.preg, viq1_create_data(0).srcv1_data.preg)
  io.out.toViq1.bypass_data.srcv_info(1).srcv_data.wb   := Mux(viq1_create_data(0).VIQ_SRCV12_SWITCH, viq1_create_data(0).srcv2_data.wb, viq1_create_data(0).srcv1_data.wb)
  io.out.toViq1.bypass_data.srcv_info(1).srcv_data.rdy  := 0.U
  io.out.toViq1.bypass_data.srcv_info(0).lsu_match      := 0.U
  io.out.toViq1.bypass_data.srcv_info(0).srcv_data.preg := viq1_create_data(0).srcv0_data.preg
  io.out.toViq1.bypass_data.srcv_info(0).srcv_data.wb   := viq1_create_data(0).srcv0_data.wb
  io.out.toViq1.bypass_data.srcv_info(0).srcv_data.rdy  := 0.U

  io.out.toViq1.bypass_data.DST_EREG     := viq1_create_data(0).dst_ereg
  io.out.toViq1.bypass_data.DST_VREG     := viq1_create_data(0).dst_vreg
  io.out.toViq1.bypass_data.DST_PREG     := viq1_create_data(0).dst_preg
  io.out.toViq1.bypass_data.DSTE_VLD     := viq1_create_data(0).dste_vld
  io.out.toViq1.bypass_data.DSTV_VLD     := viq1_create_data(0).dstv_vld
  io.out.toViq1.bypass_data.DST_VLD      := viq1_create_data(0).dst_vld
  io.out.toViq1.bypass_data.SRCVM_VLD    := viq1_create_data(0).srcvm_vld
  io.out.toViq1.bypass_data.srcv_vld(2)  := Mux(viq1_create_data(0).VIQ_SRCV12_SWITCH, viq1_create_data(0).srcv_vld(1), viq1_create_data(0).srcv_vld(2))
  io.out.toViq1.bypass_data.srcv_vld(1)  := Mux(viq1_create_data(0).VIQ_SRCV12_SWITCH, viq1_create_data(0).srcv_vld(2), viq1_create_data(0).srcv_vld(1))
  io.out.toViq1.bypass_data.srcv_vld(0)  := viq1_create_data(0).srcv_vld(0)
  io.out.toViq1.bypass_data.IID          := viq1_create_iid(0)
  io.out.toViq1.bypass_data.OPCODE       := viq1_create_data(0).opcode

  io.out.toViq1.srcv_rdy_for_bypass(0) := viq1_create_data(0).srcv0_bp_rdy
  io.out.toViq1.srcv_rdy_for_bypass(1) := viq1_create_data(0).srcv1_bp_rdy
  io.out.toViq1.srcv_rdy_for_bypass(2) := viq1_create_data(0).srcv2_bp_rdy
  io.out.toViq1.srcvm_rdy_for_bypass   := viq1_create_data(0).srcvm_bp_rdy

  //==========================================================
  //             Output for IR pre dispatch logic
  //==========================================================
  //if inst shift between IS registers, the pre dispatch information
  //should be re-calculaten by IR ctrl related logic.
  io.out.toIR.inst2_ctrl_info.VMB     := inst_read_data(2).VMB
  io.out.toIR.inst2_ctrl_info.PIPE7   := inst_read_data(2).PIPE6
  io.out.toIR.inst2_ctrl_info.PIPE6   := inst_read_data(2).PIPE6
  io.out.toIR.inst2_ctrl_info.PIPE67  := inst_read_data(2).PIPE67
  io.out.toIR.inst2_ctrl_info.SPECIAL := inst_read_data(2).SPECIAL
  io.out.toIR.inst2_ctrl_info.STADDR  := inst_read_data(2).STADDR
  io.out.toIR.inst2_ctrl_info.INTMASK := inst_read_data(2).INTMASK
  io.out.toIR.inst2_ctrl_info.SPLIT   := inst_read_data(2).SPLIT
  io.out.toIR.inst2_ctrl_info.LSU     := inst_read_data(2).LSU
  io.out.toIR.inst2_ctrl_info.BJU     := inst_read_data(2).BJU
  io.out.toIR.inst2_ctrl_info.DIV     := inst_read_data(2).DIV
  io.out.toIR.inst2_ctrl_info.MULT    := inst_read_data(2).MULT
  io.out.toIR.inst2_ctrl_info.ALU     := inst_read_data(2).ALU

  io.out.toIR.inst3_ctrl_info.VMB     := inst_read_data(3).VMB
  io.out.toIR.inst3_ctrl_info.PIPE7   := inst_read_data(3).PIPE6
  io.out.toIR.inst3_ctrl_info.PIPE6   := inst_read_data(3).PIPE6
  io.out.toIR.inst3_ctrl_info.PIPE67  := inst_read_data(3).PIPE67
  io.out.toIR.inst3_ctrl_info.SPECIAL := inst_read_data(3).SPECIAL
  io.out.toIR.inst3_ctrl_info.STADDR  := inst_read_data(3).STADDR
  io.out.toIR.inst3_ctrl_info.INTMASK := inst_read_data(3).INTMASK
  io.out.toIR.inst3_ctrl_info.SPLIT   := inst_read_data(3).SPLIT
  io.out.toIR.inst3_ctrl_info.LSU     := inst_read_data(3).LSU
  io.out.toIR.inst3_ctrl_info.BJU     := inst_read_data(3).BJU
  io.out.toIR.inst3_ctrl_info.DIV     := inst_read_data(3).DIV
  io.out.toIR.inst3_ctrl_info.MULT    := inst_read_data(3).MULT
  io.out.toIR.inst3_ctrl_info.ALU     := inst_read_data(3).ALU

  //==========================================================
  //               PID assign signal for PCFIFO
  //==========================================================
  val pcfifo_inst_vld = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    pcfifo_inst_vld(i) := dis_info.inst_vld(i) && !is_dis_stall && inst_pcfifo(i)
  }

  io.out.toIU.pcfifo_inst_vld := pcfifo_inst_vld.asUInt.orR && !is_dis_stall
  io.out.toIU.pcfifo_inst_num := PopCount(pcfifo_inst_vld)

  //==========================================================
  //               Barrier inst valid signal
  //==========================================================
  //ignore stall signals and type stall
  val bar_inst_vld = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    bar_inst_vld(i) := instVld(i) && inst_read_data(i).BAR
  }
  io.out.toLsiq.bar_inst_vld := bar_inst_vld.asUInt.orR

  //==========================================================
  //               Issue Queue empty signal
  //==========================================================
  val iq_empty = Wire(Vec(8, Bool()))
  for(i <- 0 until 8){
    iq_empty(i) := io.in.iq_cnt_info(i).empty
  }
  io.out.toHad.iq_empty := iq_empty.asUInt.orR

  //==========================================================
  //                   Issue Stall signal
  //==========================================================
  val iq_full_updt_clk_en = Wire(Vec(8, Bool()))
  for(i <- 0 until 8){
    iq_full_updt_clk_en(i) := io.in.iq_cnt_info(i).full_updt_clk_en
  }
  val queue_full_clk_en = io.in.ir_pipedown.gateclk || instVld(0) || iq_full_updt_clk_en.asUInt.orR

  //----------------------------------------------------------
  //                Issue Queue Full Prepare
  //----------------------------------------------------------
  val dis_iq_create_en_updt = Wire(Vec(7, Vec(2, Bool())))
  val iq_full_updt = Wire(Vec(7, Bool()))
  for(i <- 0 until 7){
    for(j <- 0 until 2){
      when(!is_dis_stall){
        dis_iq_create_en_updt(i)(j) := io.in.pre_dispatch.iq_create_sel(i)(j).valid
      }.otherwise{
        dis_iq_create_en_updt(i)(j) := dis_info.iq_create_sel(i)(j).valid
      }
    }
    iq_full_updt(i) := dis_iq_create_en_updt(i)(0) && io.in.iq_cnt_info(i).full_updt ||
                       dis_iq_create_en_updt(i)(1) && io.in.iq_cnt_info(i).left_1_updt
  }

  //----------------------------------------------------------
  //                  LSU VMB Full Prepare
  //----------------------------------------------------------
  val dis_vmb_create_en_updt = Wire(Vec(2, Bool()))
  for(i <- 0 until 2){
    when(!is_dis_stall){
      dis_vmb_create_en_updt(i) := io.in.pre_dispatch.iq_create_sel(7)(i).valid
    }.otherwise{
      dis_vmb_create_en_updt(i) := dis_info.iq_create_sel(7)(i).valid
    }
  }
  val vmb_full_updt = io.in.iq_cnt_info(7).full_updt && dis_vmb_create_en_updt.asUInt.orR ||
                      io.in.iq_cnt_info(7).full_updt && dis_vmb_create_en_updt.asUInt.andR

  //----------------------------------------------------------
  //                      Queue Full
  //----------------------------------------------------------
  when(io.in.fromRTU.flush_fe || io.in.fromRTU.flush_is || io.in.fromRTU.yy_xx_flush){
    iq_full  := false.B
    vmb_full := false.B
  }.otherwise{
    iq_full  := iq_full_updt.asUInt.orR
    vmb_full := vmb_full_updt
  }

  io.out.toTop.iq_full  := iq_full
  io.out.toTop.vmb_full := vmb_full

  //==========================================================
  //                  Dispatch Stall signal
  //==========================================================
  //dispatch stall is when there is valid dispatch instruction and:
  //1.RTU rob full (less than max create inst num entry valid)
  val rob_full = dis_info.inst_vld(0) && io.in.fromRTU.rob_full
  //2.Issue Queue full
  is_dis_stall := rob_full || iq_full || vmb_full

  //==========================================================
  //                    IS stall signals
  //==========================================================
  //if cannot dispatch all is pipeline inst, stall ir stage
  io.out.toIR.stall := is_dis_stall || is_dis_type_stall
}