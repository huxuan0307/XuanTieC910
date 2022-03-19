package Core.IDU
import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.AddrConfig._
import Core.IntConfig._
import Core.ExceptionConfig._
import Core.PipelineConfig.NumPipeline
import Core.VectorUnitConfig._
import Core.FuTypeConfig._

class DebugInfo extends Bundle{
  val notdefine   = UInt(4.W)
  val bkpta_inst  = Bool()
  val split_short = Bool()
  val fence       = Bool()
  val expt_vld    = Bool()
  val inst        = UInt(32.W)
}

class DepInfo extends Bundle{
  val inst23_srcv1_mask = Bool()
  val inst12_srcv1_mask = Bool()
  val inst01_srcv1_mask = Bool()
  val inst03_vreg_mask  = Bool()
  val inst02_vreg_mask  = Bool()
  val inst13_vreg_mask  = Bool()
  val inst23_vreg_mask  = Bool()
  val inst12_vreg_mask  = Bool()
  val inst01_vreg_mask  = Bool()
  val inst13_preg_mask  = Bool()
  val inst02_preg_mask  = Bool()
  val inst23_src1_mask  = Bool()
  val inst23_src0_mask  = Bool()
  val inst12_src1_mask  = Bool()
  val inst12_src0_mask  = Bool()
  val inst01_src1_mask  = Bool()
  val inst01_src0_mask  = Bool()
}

class IDData extends  Bundle {
  val vl_pred      = Bool()//72
  val vl           = UInt(8.W)
  val pc           = UInt(15.W)
  val vsew         = UInt(3.W)
  val vlmul        = UInt(2.W)
  val no_spec      = Bool()//43
  val bkptb_inst   = Bool()
  val bkpta_inst   = Bool()
  val split_short  = Bool()
  val fence        = Bool()
  val split_long   = Bool()
  val high_hw_expt = Bool()//37
  val expt_vec     = UInt(4.W)
  val expt_vld     = Bool()//32
  val opcode       = UInt(32.W)//31
}

class IRData extends Bundle {
  val vl_pred    = Bool()//177
  val vl         = UInt(8.W)
  val vmb        = Bool()// 168
  val pc         = UInt(15.W)
  val vsew       = UInt(3.W)
  val vlmul      = UInt(2.W)
  val fmla       = Bool()//147
  val split_num  = UInt(7.W)
  val no_spec    = Bool()//139
  val mla        = Bool()//138
  val dst_x0     = Bool()//137
  val illegal    = Bool()//136
  val split_last = Bool()//135
  val vmla       = Bool()//134
  val idd_plus   = UInt(4.W)
  val bkptb_inst = Bool()//129
  val bkpta_inst = Bool()//128
  val fmov       = Bool()//127
  val mov        = Bool()//126
  val expt       = UInt(7.W)
  val length     = Bool()//118
  val intmask    = Bool()//117
  val split      = Bool()//116
  val inst_type  = UInt(10.W)

  val dstv_reg   = UInt(6.W)//105
  val dstv_vld   = Bool()//99
  val srcvm_vld  = Bool()//98
  val srcv2_vld  = Bool()//97
  val srcv1_reg  = UInt(6.W)
  val srcv1_vld  = Bool()//90
  val srcv0_reg  = UInt(6.W)
  val srcv0_vld  = Bool()//83
  val dste_vld   = Bool()//82
  val dstf_reg   = UInt(6.W)
  val dstf_vld   = Bool()//75
  val srcf2_reg  = UInt(6.W)
  val srcf2_vld  = Bool()//68
  val srcf1_reg  = UInt(6.W)
  val srcf1_vld  = Bool()//61
  val srcf0_reg  = UInt(6.W)
  val srcf0_vld  = Bool()//54
  val dst_reg    = UInt(6.W)
  val dst_vld    = Bool()//47
  val src2_vld   = Bool()//46
  val src1_reg   = UInt(6.W)
  val src1_vld   = Bool()//39
  val src0_reg   = UInt(6.W)
  val src0_vld   = Bool()//32
  val opcode     = UInt(32.W)
}

class IDStageInput extends Bundle{
  val fromCp0 = new Bundle{
    val icgEn = Bool()
    val yyClkEn  = Bool()

    val cskyee = Bool()
    val frm    = UInt(3.W)
    val fs     = UInt(2.W)
    val vill   = Bool()
    val vs     = UInt(2.W)
    val vstart = UInt(7.W)
    val zeroDelayMoveDisable = Bool()
    val yyHyper = Bool()
  }
  val fromIR = new Bundle{
    val irStageStall = Bool()
    val irStall      = Bool()
  }
  val fromFence = new Bundle{
    val idStall = Bool()
    val instVld = Vec(3, Bool())

    val instData = Vec(3, new IRData)
  }
  val fromHad = new Bundle{
    val debugIdInstEn = Bool()
  }
  val fromHpcp = new Bundle{
    val cntEn = Bool()
  }
  val fromIFU = new Bundle{
    val instVld = Vec(3,Bool())
    val pipedownGateclk = Bool()

    val instData = Vec(3, new IDData)
  }
  val fromIU = new Bundle{
    val yyxxCancel = Bool()
  }
  val fromPad = new Bundle{
    val yyIcgScanEn = Bool()
  }
  val fromRTU = new Bundle{
    val flushFe = Bool()
  }
}

class IDStageOutput extends Bundle{
  val toFence = new Bundle{
    val instVld = Bool()
    val stall   = Bool()

    val bkptb_inst   = Bool()
    val bkpta_inst   = Bool()
    val fence_type   = UInt(3.W)
    val inst         = UInt(32.W)
    val pc           = UInt(15.W)
    val vl           = UInt(8.W)
    val vl_pred      = Bool()
    val vlmul        = UInt(2.W)
    val vsew         = UInt(3.W)
  }
  val pipedown = new Bundle{
    val gateclk = Bool()
    val instVld = Vec(4,Bool())

    val depInfo = new DepInfo
    val instData = Vec(4, new IRData)
  }
  val toTop = new Bundle{
    val instVld = Vec(3,Bool())
  }
  val toHad = new Bundle{
    val instVld = Vec(3,Bool())
    val pipeStall = Bool()

    val instInfo = Vec(3, new DebugInfo)
  }
  val toHpcp = new Bundle{
    val backendStall = Bool()
  }
  val toIFU = new Bundle{
    val bypassStall = Bool()
    val stall = Bool()
  }
}

class IDStageIO extends Bundle{
  val in  = Input(new IDStageInput)
  val out = Output(new IDStageOutput)
}

class IDStage extends Module{
  val io = IO(new IDStageIO)

  //Reg
  val instVld  = RegInit(VecInit(Seq.fill(3)(false.B)))
  val instData = RegInit(VecInit(Seq.fill(3)(0.U.asTypeOf(new IDData))))
  //Wire
  val id_pipedown_inst_num  = Wire(Vec(3, Bool()))
  val id_pipedown_stall = Wire(Bool())
  val ctrl_id_stall = Wire(Bool())

  //==========================================================
  //                ID/IR pipeline registers
  //==========================================================
  //----------------------------------------------------------
  //            ID Pipedown Instruction Selection
  //----------------------------------------------------------
  val ib_pipedown_inst_vld = Wire(Vec(3, Bool()))
  ib_pipedown_inst_vld(0) :=
    id_pipedown_inst_num(0) && instVld(1) ||
    id_pipedown_inst_num(1) && instVld(2) ||
    id_pipedown_inst_num(2) && io.in.fromIFU.instVld(0)
  ib_pipedown_inst_vld(1) :=
    id_pipedown_inst_num(0) && instVld(2) ||
    id_pipedown_inst_num(1) && false.B ||
    id_pipedown_inst_num(2) && io.in.fromIFU.instVld(1)
  ib_pipedown_inst_vld(2) :=
    id_pipedown_inst_num(0) && false.B ||
    id_pipedown_inst_num(1) && false.B ||
    id_pipedown_inst_num(2) && io.in.fromIFU.instVld(2)

  val id_inst_data = Wire(Vec(3, new IDData))
  id_inst_data(0) := PriorityMux(Seq(
    id_pipedown_inst_num(0) -> instData(1),
    id_pipedown_inst_num(1) -> instData(2),
    id_pipedown_inst_num(2) -> io.in.fromIFU.instData(0)
  ))
  id_inst_data(1) := PriorityMux(Seq(
    id_pipedown_inst_num(0) -> instData(2),
    id_pipedown_inst_num(1) -> io.in.fromIFU.instData(0),
    id_pipedown_inst_num(2) -> io.in.fromIFU.instData(1)
  ))
  id_inst_data(2) := PriorityMux(Seq(
    id_pipedown_inst_num(0) -> io.in.fromIFU.instData(0),
    id_pipedown_inst_num(1) -> io.in.fromIFU.instData(1),
    id_pipedown_inst_num(2) -> io.in.fromIFU.instData(2)
  ))

  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  val id_inst_clk_en = io.in.fromIFU.pipedownGateclk || instVld.asUInt.orR

  //----------------------------------------------------------
  //                ID/IR pipeline registers
  //----------------------------------------------------------
  when(io.in.fromRTU.flushFe || io.in.fromIU.yyxxCancel){
    instVld  := WireInit(VecInit(Seq.fill(3)(false.B)))
  }.elsewhen(!id_pipedown_stall){
    instVld  := ib_pipedown_inst_vld
    instData := id_inst_data
  }
  io.out.pipedown.gateclk := instVld(0)
  io.out.toTop.instVld := instVld

  //TODO: Debug
  val debug_id_pipedown3 = RegInit(false.B)
  //----------------------------------------------------------
  //               ID inst opcode for debug
  //----------------------------------------------------------
  //----------------------------------------------------------
  //                ID inst info for debug
  //----------------------------------------------------------
  val debugInfo = RegInit(VecInit(Seq.fill(3)(0.U.asTypeOf(new DebugInfo))))
  when(debug_id_pipedown3){
    for(i <- 0 until 3){
      debugInfo(i).inst        := instData(i).opcode
      debugInfo(i).expt_vld    := instData(i).expt_vld
      debugInfo(i).fence       := instData(i).fence
      debugInfo(i).split_short := instData(i).split_short
      debugInfo(i).bkpta_inst  := instData(i).bkpta_inst
      debugInfo(i).notdefine   := 0.U(4.W)
    }
  }
  io.out.toHad.instInfo := debugInfo
  //----------------------------------------------------------
  //               Pipeline register implement
  //----------------------------------------------------------
  val debug_id_inst_vld = instVld(0) && (io.in.fromHad.debugIdInstEn || io.in.fromHpcp.cntEn)
  when(debug_id_inst_vld){
    debug_id_pipedown3 := !ctrl_id_stall
  }.otherwise{
    debug_id_pipedown3 := false.B
  }
  io.out.toHpcp.backendStall := !debug_id_pipedown3

  val debugInstVld = RegInit(VecInit(Seq.fill(3)(false.B)))
  when(debug_id_pipedown3){
    debugInstVld := instVld
  }
  io.out.toHad.instVld := debugInstVld
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  val dp_debug_id_inst_clk_en   = debug_id_pipedown3
  val ctrl_debug_id_inst_clk_en = debug_id_inst_vld || debug_id_pipedown3 || debug_id_inst_vld(0)
  //TODO: Debug

  //----------------------------------------------------------
  //               Type signals for Control Path
  //----------------------------------------------------------
  val inst_normal      = Wire(Vec(3, Bool()))
  val inst_fence       = Wire(Vec(3, Bool()))
  val inst_split_short = Wire(Vec(3, Bool()))
  val inst_split_long  = Wire(Vec(3, Bool()))
  for(i <- 0 until 3){
    inst_normal(i)   := instVld(i) && !instData(i).fence && !instData(i).split_short && !instData(i).split_long
    inst_fence(i)    := instVld(i) && instData(i).fence
    inst_split_short(i) := instVld(i) && instData(i).split_short
    inst_split_long(i)  := instVld(i) && instData(i).split_long
  }

  //----------------------------------------------------------
  //                 Fence Instruction Valid
  //----------------------------------------------------------
  //fence valid only when ID inst0 is valid fence inst
  io.out.toFence.instVld := inst_fence(0);

  //----------------------------------------------------------
  //              Long Split Instruction Valid
  //----------------------------------------------------------
  //long split valid only when ID inst0 is valid long split inst
  val split_long_vld = inst_split_long(0)

  //==========================================================
  //                    Normal Data Path
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of ID Decoder
  //----------------------------------------------------------
  val normal_decode = Seq.fill(3)(Module(new ct_idu_id_decd))
  val normal_decode_data = WireInit(VecInit(Seq.fill(3)(0.U.asTypeOf(new IRData))))
  for(i <- 0 until 3){
    normal_decode(i).io.cp0_idu_cskyee := io.in.fromCp0.cskyee
    normal_decode(i).io.cp0_idu_frm    := io.in.fromCp0.frm
    normal_decode(i).io.cp0_idu_fs     := io.in.fromCp0.fs
    normal_decode(i).io.cp0_idu_vill   := io.in.fromCp0.vill
    normal_decode(i).io.cp0_idu_vs     := io.in.fromCp0.vs
    normal_decode(i).io.cp0_idu_vstart := io.in.fromCp0.vstart
    normal_decode(i).io.cp0_idu_zero_delay_move_disable := io.in.fromCp0.zeroDelayMoveDisable
    normal_decode(i).io.cp0_yy_hyper   := io.in.fromCp0.yyHyper
    normal_decode(i).io.x_inst         := instData(i).opcode
    normal_decode(i).io.x_vl           := instData(i).vl
    normal_decode(i).io.x_vlmul        := instData(i).vlmul
    normal_decode(i).io.x_vsew         := instData(i).vsew

    normal_decode_data(i).vl_pred    := instData(i).vl_pred
    normal_decode_data(i).vl         := instData(i).vl
    normal_decode_data(i).vmb        := normal_decode(i).io.x_vmb
    normal_decode_data(i).pc         := instData(i).pc
    normal_decode_data(i).vsew       := instData(i).vsew
    normal_decode_data(i).vlmul      := instData(i).vlmul
    normal_decode_data(i).fmla       := normal_decode(i).io.x_fmla
    //normal_decode_data(i).split_num  :=
    normal_decode_data(i).no_spec    := instData(i).no_spec
    normal_decode_data(i).mla        := normal_decode(i).io.x_mla
    normal_decode_data(i).src2_vld   := normal_decode(i).io.x_src2_vld
    normal_decode_data(i).dst_x0     := normal_decode(i).io.x_dst_x0
    //normal_decode_data(i).illegal    := normal_decode(i).io.x_illegal
    //normal_decode_data(i).split_last := normal_decode(i).io.x_
    normal_decode_data(i).vmla       := normal_decode(i).io.x_vmla
    //normal_decode_data(i).idd_plus   := normal_decode(i).io.x_
    normal_decode_data(i).bkptb_inst := instData(i).bkptb_inst
    normal_decode_data(i).bkpta_inst := instData(i).bkpta_inst
    normal_decode_data(i).fmov       := normal_decode(i).io.x_fmov
    normal_decode_data(i).mov        := normal_decode(i).io.x_mov
    //normal_decode_data(i).expt       := normal_decode(i).io.x_
    normal_decode_data(i).length     := normal_decode(i).io.x_length
    //normal_decode_data(i).intmask    := normal_decode(i).io.x_
    //normal_decode_data(i).split      := normal_decode(i).io.x_
    normal_decode_data(i).inst_type  := normal_decode(i).io.x_inst_type

    normal_decode_data(i).dstv_reg   := Cat(0.U(1.W), normal_decode(i).io.x_dstv_reg)
    normal_decode_data(i).dstv_vld   := normal_decode(i).io.x_dstv_vld
    normal_decode_data(i).srcvm_vld  := normal_decode(i).io.x_srcvm_vld
    normal_decode_data(i).srcv2_vld  := normal_decode(i).io.x_srcv2_vld
    normal_decode_data(i).srcv1_reg  := Cat(0.U(1.W), normal_decode(i).io.x_srcv1_reg)
    normal_decode_data(i).srcv1_vld  := normal_decode(i).io.x_srcv1_vld
    normal_decode_data(i).srcv0_reg  := Cat(0.U(1.W), normal_decode(i).io.x_srcv0_reg)
    normal_decode_data(i).srcv0_vld  := normal_decode(i).io.x_srcv0_vld
    normal_decode_data(i).dste_vld   := normal_decode(i).io.x_dste_vld
    normal_decode_data(i).dstf_reg   := Cat(0.U(1.W), normal_decode(i).io.x_dstf_reg)
    normal_decode_data(i).dstf_vld   := normal_decode(i).io.x_dstf_vld
    normal_decode_data(i).srcf2_reg  := Cat(0.U(1.W), normal_decode(i).io.x_srcf2_reg)
    normal_decode_data(i).srcf2_vld  := normal_decode(i).io.x_srcf2_vld
    normal_decode_data(i).srcf1_reg  := Cat(0.U(1.W), normal_decode(i).io.x_srcf1_reg)
    normal_decode_data(i).srcf1_vld  := normal_decode(i).io.x_srcf1_vld
    normal_decode_data(i).srcf0_reg  := Cat(0.U(1.W), normal_decode(i).io.x_srcf0_reg)
    normal_decode_data(i).srcf0_vld  := normal_decode(i).io.x_srcf0_vld
    normal_decode_data(i).dst_reg    := Cat(0.U(1.W), normal_decode(i).io.x_dst_reg)
    normal_decode_data(i).dst_vld    := normal_decode(i).io.x_dst_vld
    normal_decode_data(i).src1_reg   := Cat(0.U(1.W), normal_decode(i).io.x_src1_reg)
    normal_decode_data(i).src1_vld   := normal_decode(i).io.x_src1_vld
    normal_decode_data(i).src0_reg   := Cat(0.U(1.W), normal_decode(i).io.x_src0_reg)
    normal_decode_data(i).src0_vld   := normal_decode(i).io.x_src0_vld
    normal_decode_data(i).opcode     := instData(i).opcode
  }
  io.out.toFence.fence_type := normal_decode(0).io.x_fence_type
  //----------------------------------------------------------
  //            normal expt inst expt data select
  //----------------------------------------------------------
  //ifu expt inst, illegal and bkpt treat as normal inst
  //add control path for power optimization
  val inst_expt_vld = Wire(Vec(3, Bool()))
  val inst_high_hw_expt = Wire(Vec(3, Bool()))
  val inst_expt_vec = Wire(Vec(3, UInt(5.W)))
  for(i <- 0 until 3){
    inst_expt_vld(i)     := instVld(i) && (instData(i).expt_vld || normal_decode(i).io.x_illegal)
    inst_high_hw_expt(i) := instData(i).expt_vld && instData(i).high_hw_expt
    inst_expt_vec(i)     := Mux(instData(i).expt_vld, Cat(0.U(1.W), instData(i).expt_vec), 2.U(5.W))
  }

  //----------------------------------------------------------
  //            Rename ID stage expt inst data
  //----------------------------------------------------------
  val expt_inst_data = WireInit(VecInit(Seq.fill(3)(0.U.asTypeOf(new IRData))))
  for(i <- 0 until 3){
    expt_inst_data(i).vl_pred    := instData(i).vl_pred
    expt_inst_data(i).vl         := instData(i).vl
    expt_inst_data(i).vsew       := instData(i).vsew
    expt_inst_data(i).vlmul      := instData(i).vlmul
    expt_inst_data(i).illegal    := normal_decode(i).io.x_illegal
    expt_inst_data(i).bkptb_inst := instData(i).bkptb_inst
    expt_inst_data(i).bkpta_inst := instData(i).bkpta_inst
    expt_inst_data(i).expt       := Cat(inst_high_hw_expt(i), inst_expt_vec(i), inst_expt_vld(i))
    //expt_inst_data(i).expt(5,1)  := inst_expt_vec(i)
    //expt_inst_data(i).expt(0)    := inst_expt_vld(i)
    expt_inst_data(i).length     := normal_decode(i).io.x_length
    expt_inst_data(i).inst_type  := SPECIAL.U
    expt_inst_data(i).opcode     := instData(i).opcode
  }

  //----------------------------------------------------------
  //               Normal inst data selection
  //----------------------------------------------------------
  val normal_inst_data = Wire(Vec(3, new IRData))
  for(i <- 0 until 3){
    normal_inst_data(i) := Mux(inst_expt_vld(i), expt_inst_data(i), normal_decode_data(i))
  }

  //==========================================================
  //                    Split Data Path
  //==========================================================
  //----------------------------------------------------------
  //                Instance of long split
  //----------------------------------------------------------
  // &Force ("nonport","id_inst1_split_long_type"); @624
  // &Force ("nonport","id_inst2_split_long_type"); @625
  //long split inst is always from id inst0
  //exception information come from ID decoded info
  // &Instance("ct_idu_id_split_long", "x_ct_idu_id_split_long"); @628
  val split_long_decode = Module(new ct_idu_id_split_long)
  split_long_decode.io.cp0_idu_icg_en              := io.in.fromCp0.icgEn
  split_long_decode.io.cp0_idu_vstart              := io.in.fromCp0.vstart
  split_long_decode.io.cp0_yy_clk_en               := io.in.fromCp0.yyClkEn
  split_long_decode.io.cpurst_b                    := !reset.asBool
  split_long_decode.io.ctrl_split_long_id_inst_vld := split_long_vld
  split_long_decode.io.ctrl_split_long_id_stall    := io.in.fromIR.irStall
  split_long_decode.io.dp_split_long_bkpta_inst    := instData(0).bkpta_inst
  split_long_decode.io.dp_split_long_bkptb_inst    := instData(0).bkptb_inst
  split_long_decode.io.dp_split_long_inst          := instData(0).opcode
  split_long_decode.io.dp_split_long_no_spec       := instData(0).no_spec
  split_long_decode.io.dp_split_long_pc            := instData(0).pc
  split_long_decode.io.dp_split_long_type          := normal_decode(0).io.x_split_long_type
  split_long_decode.io.dp_split_long_vl            := instData(0).vl
  split_long_decode.io.dp_split_long_vl_pred       := instData(0).vl_pred
  split_long_decode.io.dp_split_long_vlmul         := instData(0).vlmul
  split_long_decode.io.dp_split_long_vsew          := instData(0).vsew
  split_long_decode.io.forever_cpuclk              := clock.asBool
  split_long_decode.io.iu_yy_xx_cancel             := io.in.fromIU.yyxxCancel
  split_long_decode.io.pad_yy_icg_scan_en          := io.in.fromPad.yyIcgScanEn
  split_long_decode.io.rtu_idu_flush_fe            := io.in.fromRTU.flushFe

  val split_long_inst_data = Wire(Vec(4, new IRData))
  val split_long_dep_info  = Wire(new DepInfo)
  when(!inst_expt_vld(0)){
    split_long_inst_data(0) := split_long_decode.io.split_long_dp_inst0_data.asTypeOf(new IRData)
    split_long_inst_data(1) := split_long_decode.io.split_long_dp_inst1_data.asTypeOf(new IRData)
    split_long_inst_data(2) := split_long_decode.io.split_long_dp_inst2_data.asTypeOf(new IRData)
    split_long_inst_data(3) := split_long_decode.io.split_long_dp_inst3_data.asTypeOf(new IRData)
    split_long_dep_info     := split_long_decode.io.split_long_dp_dep_info.asTypeOf(new DepInfo)
  }.otherwise{
    split_long_inst_data(0) := expt_inst_data(0)
    split_long_inst_data(1) := expt_inst_data(0)
    split_long_inst_data(2) := expt_inst_data(0)
    split_long_inst_data(3) := expt_inst_data(0)
    split_long_dep_info     := 0.U.asTypeOf(new DepInfo)
  }

  val split_long_ctrl_id_stall = split_long_decode.io.split_long_ctrl_id_stall
  val split_long_ctrl_inst_vld = split_long_decode.io.split_long_ctrl_inst_vld

  //----------------------------------------------------------
  //                Instance of short split
  //----------------------------------------------------------
  //exception information come from ID decoded info
  val split_short_decode = Seq.fill(3)(Module(new ct_idu_id_split_short))
  for(i <- 0 until 3){
    split_short_decode(i).io.dp_split_short_bkpta_inst := instData(i).bkpta_inst
    split_short_decode(i).io.dp_split_short_bkptb_inst := instData(i).bkptb_inst
    split_short_decode(i).io.dp_split_short_inst       := instData(i).opcode
    split_short_decode(i).io.dp_split_short_no_spec    := instData(i).no_spec
    split_short_decode(i).io.dp_split_short_pc         := instData(i).pc
    split_short_decode(i).io.dp_split_short_type       := normal_decode(i).io.x_split_short_type
    split_short_decode(i).io.dp_split_short_vl         := instData(i).vl
    split_short_decode(i).io.dp_split_short_vl_pred    := instData(i).vl_pred
    split_short_decode(i).io.dp_split_short_vlmul      := instData(i).vlmul
    split_short_decode(i).io.dp_split_short_vsew       := instData(i).vsew
  }

  val split_short_inst_data = Wire(Vec(3, Vec(2, new IRData)))
  val split_short_dep_info  = Wire(Vec(3,UInt(4.W)))
  for(i <- 0 until 3){
    when(!inst_expt_vld(i)){
      split_short_inst_data(i)(0) := split_short_decode(i).io.split_short_dp_inst0_data.asTypeOf(new IRData)
      split_short_inst_data(i)(1) := split_short_decode(i).io.split_short_dp_inst1_data.asTypeOf(new IRData)
      split_short_dep_info(i)     := split_short_decode(i).io.split_short_dp_dep_info
    }.otherwise{
      split_short_inst_data(i)(0) := expt_inst_data(i)
      split_short_inst_data(i)(1) := expt_inst_data(i)
      split_short_dep_info(i)     := 0.U(4.W)
    }
  }

  //==========================================================
  //                    Fence Data Path
  //==========================================================
  //----------------------------------------------------------
  //              Select ID instruction for fence
  //----------------------------------------------------------
  io.out.toFence.inst       := instData(0).opcode
  io.out.toFence.bkpta_inst := instData(0).bkpta_inst
  io.out.toFence.bkptb_inst := instData(0).bkptb_inst
  io.out.toFence.vlmul      := instData(0).vlmul
  io.out.toFence.vsew       := instData(0).vsew
  io.out.toFence.vl         := instData(0).vl
  io.out.toFence.vl_pred    := instData(0).vl_pred
  io.out.toFence.pc         := instData(0).pc

  //==========================================================
  //                 Pipedown control signals
  //==========================================================
  val id_pipedown_inst_vld = Wire(Vec(4, Bool()))
  //----------------------------------------------------------
  //            IR Pipedown Instruction 0 valid
  //----------------------------------------------------------
  id_pipedown_inst_vld(0) :=
    inst_normal(0)      && instVld(0) ||
    inst_split_short(0) && true.B ||
    inst_split_long(0)  && split_long_ctrl_inst_vld(0) ||
    inst_fence(0)       && io.in.fromFence.instVld(0)
  //----------------------------------------------------------
  //            IR Pipedown Instruction 1 valid
  //----------------------------------------------------------
  id_pipedown_inst_vld(1) := PriorityMux(Seq(
    //consider id inst0 type
    inst_fence(0)       -> io.in.fromFence.instVld(1),
    inst_split_short(0) -> true.B,
    inst_split_long(0)  -> split_long_ctrl_inst_vld(1),
    //consider id inst1 type
    inst_fence(1)       -> false.B,
    inst_split_long(1)  -> false.B,
    inst_split_short(1) -> true.B,
    true.B              -> (instVld(1) && !id_pipedown_inst_num(0))
  ))
  //----------------------------------------------------------
  //            IR Pipedown Instruction 2 valid
  //----------------------------------------------------------
  id_pipedown_inst_vld(2) := PriorityMux(Seq(
    //consider id inst0 type
    inst_fence(0)       -> io.in.fromFence.instVld(2),
    inst_split_long(0)  -> split_long_ctrl_inst_vld(2),
    (inst_split_short(0) && inst_split_short(1)) -> true.B,
    inst_split_short(0) -> (instVld(1) && !id_pipedown_inst_num(0)),
    //consider id inst1 type
    inst_fence(1)       -> false.B,
    inst_split_long(1)  -> false.B,
    inst_split_short(1) -> true.B,
    //consider id inst2 type
    inst_fence(2)       -> false.B,
    inst_split_long(2)  -> false.B,
    inst_split_short(2) -> true.B,
    true.B              -> (instVld(2) && !id_pipedown_inst_num(0) && !id_pipedown_inst_num(1))
  ))
  //----------------------------------------------------------
  //            IR Pipedown Instruction 3 valid
  //----------------------------------------------------------
  id_pipedown_inst_vld(3) := PriorityMux(Seq(
    //consider id inst0 type
    inst_fence(0)       -> false.B,
    inst_split_long(0)  -> split_long_ctrl_inst_vld(3),
    (inst_split_short(0) && inst_split_short(1)) -> true.B,
    inst_split_short(0) -> (instVld(2) && !id_pipedown_inst_num(0) && !id_pipedown_inst_num(1)),
    //consider id inst1 type
    inst_fence(1)       -> false.B,
    inst_split_long(1)  -> false.B,
    inst_split_short(1) -> (instVld(2) && !id_pipedown_inst_num(0) && !id_pipedown_inst_num(1)),
    //consider id inst2 type
    true.B              -> (inst_split_short(2) && !id_pipedown_inst_num(0) && !id_pipedown_inst_num(1))
  ))

  io.out.pipedown.instVld := id_pipedown_inst_vld

  //==========================================================
  //             ID pipedown inst data selection
  //==========================================================
  val id_pipedown_inst_data = Wire(Vec(4, new IRData))
  //----------------------------------------------------------
  //                  Select inst 0 data
  //----------------------------------------------------------
  id_pipedown_inst_data(0) := PriorityMux(Seq(
    instData(0).split_short -> split_short_inst_data(0)(0),
    instData(0).split_long  -> split_long_inst_data(0),
    instData(0).fence       -> io.in.fromFence.instData(0),
    true.B                  -> normal_inst_data(0)
  ))
  //----------------------------------------------------------
  //                  Select inst 1 data
  //----------------------------------------------------------
  id_pipedown_inst_data(1) := PriorityMux(Seq(
    instData(0).fence       -> io.in.fromFence.instData(1),
    instData(0).split_short -> split_short_inst_data(0)(1),
    instData(0).split_long  -> split_long_inst_data(1),
    instData(1).split_short -> split_short_inst_data(1)(0),
    true.B                  -> normal_inst_data(1)
  ))
  //----------------------------------------------------------
  //                  Select inst 2 data
  //----------------------------------------------------------
  id_pipedown_inst_data(2) := PriorityMux(Seq(
    instData(0).fence       -> io.in.fromFence.instData(2),
    instData(0).split_long  -> split_long_inst_data(2),
    (instData(0).split_short && instData(1).split_short) -> split_short_inst_data(1)(0),
    instData(0).split_short -> normal_inst_data(1),
    instData(1).split_short -> split_short_inst_data(1)(1),
    instData(2).split_short -> split_short_inst_data(2)(0),
    true.B                  -> normal_inst_data(2)
  ))
  //----------------------------------------------------------
  //                  Select inst 3 data
  //----------------------------------------------------------
  id_pipedown_inst_data(3) := PriorityMux(Seq(
    instData(0).split_long  -> split_long_inst_data(3),
    (instData(0).split_short && instData(1).split_short) -> split_short_inst_data(1)(1),
    instData(0).split_short -> normal_inst_data(2),
    instData(1).split_short -> normal_inst_data(2),
    //instData(2).split_short -> split_short_inst_data(2)(1),
    true.B                  -> split_short_inst_data(2)(1)
  ))

  io.out.pipedown.instData := id_pipedown_inst_data

  //----------------------------------------------------------
  //                   Select dep info
  //----------------------------------------------------------
  val inst01_split_short_dep_info = WireInit(0.U.asTypeOf(new DepInfo))
  val inst_split_short_dep_info   = WireInit(VecInit(Seq.fill(3)(0.U.asTypeOf(new DepInfo))))

  inst01_split_short_dep_info.inst01_src0_mask  := split_short_dep_info(0)(0)
  inst01_split_short_dep_info.inst01_src1_mask  := split_short_dep_info(0)(1)
  inst01_split_short_dep_info.inst01_vreg_mask  := split_short_dep_info(0)(2)
  inst01_split_short_dep_info.inst01_srcv1_mask := split_short_dep_info(0)(3)
  inst01_split_short_dep_info.inst23_src0_mask  := split_short_dep_info(1)(0)
  inst01_split_short_dep_info.inst23_src1_mask  := split_short_dep_info(1)(1)
  inst01_split_short_dep_info.inst23_vreg_mask  := split_short_dep_info(1)(2)
  inst01_split_short_dep_info.inst23_srcv1_mask := split_short_dep_info(1)(3)

  inst_split_short_dep_info(0).inst01_src0_mask  := split_short_dep_info(0)(0)
  inst_split_short_dep_info(0).inst01_src1_mask  := split_short_dep_info(0)(1)
  inst_split_short_dep_info(0).inst01_vreg_mask  := split_short_dep_info(0)(2)
  inst_split_short_dep_info(0).inst01_srcv1_mask := split_short_dep_info(0)(3)

  inst_split_short_dep_info(1).inst12_src0_mask  := split_short_dep_info(1)(0)
  inst_split_short_dep_info(1).inst12_src1_mask  := split_short_dep_info(1)(1)
  inst_split_short_dep_info(1).inst12_vreg_mask  := split_short_dep_info(1)(2)
  inst_split_short_dep_info(1).inst12_srcv1_mask := split_short_dep_info(1)(3)

  inst_split_short_dep_info(2).inst23_src0_mask  := split_short_dep_info(2)(0)
  inst_split_short_dep_info(2).inst23_src1_mask  := split_short_dep_info(2)(1)
  inst_split_short_dep_info(2).inst23_vreg_mask  := split_short_dep_info(2)(2)
  inst_split_short_dep_info(2).inst23_srcv1_mask := split_short_dep_info(2)(3)

  io.out.pipedown.depInfo := PriorityMux(Seq(
    inst_split_long(0)  -> split_long_dep_info,
    (inst_split_short(0) && inst_split_short(1)) -> inst01_split_short_dep_info,
    inst_split_short(0) -> inst_split_short_dep_info(0),
    inst_split_short(1) -> inst_split_short_dep_info(1),
    inst_split_short(2) -> inst_split_short_dep_info(2),
    true.B              -> 0.U.asTypeOf(new DepInfo)
  ))

  //==========================================================
  //               Pipedown Instruction Type
  //==========================================================
  //short split: split to 2 inst
  //long  split: (may) split to more than 2 inst
  val id_1_fence_inst = inst_fence.asUInt.orR
  val id_1_split_long_inst = inst_split_long.asUInt.orR
  //----------------------------------------------------------
  //                    Pipedown 1 Inst
  //----------------------------------------------------------
  //pipedown 1 inst when:
  //   inst0 can pipedown (not a stall long split, fence stall deal in id stall)
  //   while inst1 cannot pipedown with inst0
  id_pipedown_inst_num(0) :=
    (inst_normal(0) || inst_split_short(0)) && (inst_fence(1) || inst_split_long(1)) ||
    (inst_split_long(0) && !split_long_ctrl_id_stall) || (inst_fence(0) && !io.in.fromFence.idStall)
  //----------------------------------------------------------
  //                    Pipedown 2 Inst
  //----------------------------------------------------------
  //pipedown 2 inst when:
  //   inst0 and inst1 are normal or split short
  //   while inst2 cannot pipedown with inst0 and inst1
  id_pipedown_inst_num(1) :=
    (inst_normal(0) || inst_split_short(0)) && (inst_normal(1) || inst_split_short(1)) &&
    (inst_normal(2) && inst_split_short(0) && inst_split_short(1) ||
    inst_split_short(2) && (inst_split_short(0) || inst_split_short(1)) ||
    inst_split_long(2) || inst_fence(2))
  //----------------------------------------------------------
  //                    Pipedown 3 Inst
  //----------------------------------------------------------
  //pipedown 3 inst when:
  //1. no fence inst and no long split inst and no 2 or more short split inst
  //   (except inst0/1 are short split insts and inst 2 is normal inst)
  id_pipedown_inst_num(2) := !id_1_fence_inst && !id_1_split_long_inst &&
    !(inst_split_short(2) && (inst_split_short(0) || inst_split_short(1))) &&
    !(instVld(2) && (inst_split_short(0) && inst_split_short(1)))

  val ctrl_id_pipedown_3_inst_for_bypass = id_pipedown_inst_num(2)
  //==========================================================
  //                     ID stage stall
  //==========================================================
  //----------------------------------------------------------
  //                  ID stage Stall Source
  //----------------------------------------------------------
  //if ir stage stall
  val ctrl_split_long_id_stall = io.in.fromIR.irStall
  io.out.toFence.stall        := io.in.fromIR.irStall
  //split long stall
  val ctrl_id_split_long_stall = inst_split_long(0) && split_long_ctrl_id_stall

  //----------------------------------------------------------
  //                     ID stage Stall
  //----------------------------------------------------------
  // id stall for IFU
  ctrl_id_stall := instVld(0) && (io.in.fromIR.irStall || !id_pipedown_inst_num(2))
  //bypass id stall for IFU bypass
  val ctrl_id_bypass_stall = instVld(0) && (io.in.fromIR.irStall || !ctrl_id_pipedown_3_inst_for_bypass)
  //pipedown stall for ID inst valid and data path
  id_pipedown_stall := instVld(0) && (io.in.fromIR.irStall || io.in.fromFence.idStall || ctrl_id_split_long_stall)
  //----------------------------------------------------------
  //                  Output stall signals
  //----------------------------------------------------------
  io.out.toIFU.stall       := ctrl_id_stall
  io.out.toIFU.bypassStall := ctrl_id_bypass_stall
  io.out.toHad.pipeStall   := instVld(0) && io.in.fromFence.idStall || io.in.fromIR.irStall
}