package Core.IDU
import chisel3._
import chisel3.util._

class hpcp_type extends Bundle{
  val alu   = Bool()
  val lsu   = Bool()
  val vec   = Bool()
  val csr   = Bool()
  val ecall = Bool()
  val sync  = Bool()
  val fp    = Bool()
}

class ISCtrl extends Bundle{
  val VMB     = Bool()//12
  val PIPE7   = Bool()//11
  val PIPE6   = Bool()//10
  val PIPE67  = Bool()//9
  val SPECIAL = Bool()//8
  val STADDR  = Bool()//7
  val INTMASK = Bool()//6
  val SPLIT   = Bool()//5
  val LSU     = Bool()//4
  val BJU     = Bool()//3
  val DIV     = Bool()//2
  val MULT    = Bool()//1
  val ALU     = Bool()//0
}

class ISData extends Bundle{
  val VL_PRED = Bool() //270
  val VL = UInt(6.W) //269
  val LCH_PREG = Bool() //261
  val VAMO = Bool() //260
  val UNIT_STRIDE = Bool() //259
  val VMB = Bool() //258
  val DSTV_IMP = Bool() //257
  val VIQ_SRCV12_SWITCH = Bool() //256
  val VSETVL = Bool() //255
  val VSETVLI = Bool() //254
  val VSEW = UInt(3.W) //253
  val VLMUL = UInt(2.W) //250
  val VMUL = Bool() //248
  val VMUL_UNSPLIT = Bool() //247
  val VMLA_SHORT = Bool() //246
  val VMLA_TYPE = UInt(3.W) //245
  val SPLIT_NUM = UInt(7.W) //242
  val NO_SPEC = Bool() //235
  val ALU_SHORT = Bool() //234
  val MLA = Bool() //233
  val STR = Bool() //232
  val SPLIT_LAST = Bool() //231
  val MFVR = Bool() //230
  val MTVR = Bool() //229
  val VMLA = Bool() //228
  val VDIV = Bool() //227
  val PIPE7 = Bool() //226
  val PIPE6 = Bool() //225
  val PIPE67 = Bool() //224
  val IID_PLUS = UInt(4.W) //223
  val BKPTB_INST = Bool() //219
  val BKPTA_INST = Bool() //218
  val EXPT = UInt(7.W) //217
  val RTS = Bool() //210
  val SPECIAL = Bool() //209
  val LSU = Bool() //208
  val DIV = Bool() //207
  val MULT = Bool() //206
  val INTMASK = Bool() //205
  val SPLIT = Bool() //204
  val LENGTH = Bool() //203
  val PCFIFO = Bool() //202
  val PCALL = Bool() //201
  val BJU = Bool() //200
  val LSU_PC = UInt(15.W) //199
  val BAR_TYPE = UInt(4.W) //184
  val BAR = Bool() //180
  val STADDR = Bool() //179
  val STORE = Bool() //178
  val LOAD = Bool() //177
  val ALU = Bool() //176

  val dst_rel_ereg = UInt(5.W)//175
  val dst_ereg = UInt(5.W)
  val dst_rel_vreg = UInt(7.W)
  val dst_vreg = UInt(7.W)
  val dstv_reg = UInt(5.W)
  val srcvm_lsu_match = Bool()
  val srcvm_bp_rdy = UInt(2.W)
  val srcvm_data = new srcData9
  val srcv2_lsu_match = Bool()
  val srcv2_bp_rdy = UInt(2.W)
  val srcv2_data = new srcData10
  val srcv1_lsu_match = Bool()
  val srcv1_bp_rdy = UInt(2.W)
  val srcv1_data = new srcData9
  val srcv0_lsu_match = Bool()
  val srcv0_bp_rdy = UInt(2.W)
  val srcv0_data = new srcData9
  val dste_vld = Bool()
  val dstv_vld = Bool()
  val srcvm_vld = Bool()
  val srcv_vld = Vec(3, Bool())

  val dst_rel_preg = UInt(7.W)
  val dst_preg = UInt(7.W)
  val dst_reg = UInt(5.W)
  val src2_lsu_match = Bool()
  val src2_bp_rdy = UInt(2.W)
  val src2_data = new srcData10
  val src1_lsu_match = Bool()
  val src1_bp_rdy = UInt(2.W)
  val src1_data = new srcData9
  val src0_lsu_match = Bool()
  val src0_bp_rdy = UInt(2.W)
  val src0_data = new srcData9
  val dst_vld = Bool()
  val src_vld = Vec(3, Bool())

  val opcode = UInt(32.W)
}


class IRStageInput extends Bundle{
  val fromCp0 = new Bundle {
    val icgEn = Bool()
    val dlbDisable = Bool()
    val yyClkEn = Bool()
    val robFoldDisable = Bool()
  }
  val id_pipedown = new Bundle{
    val gateclk = Bool()
    val instVld = Vec(4,Bool())

    val depInfo = new DepInfo
    val instData = Vec(4, new IRData)
  }
  val fromRTU = new Bundle{
    val flush = new IduFromRtuFlushBundle
    val flush_stall = Bool()
    val srt_en = Bool()

    val ereg_vld = Vec(4, Bool())
    val freg_vld = Vec(4, Bool())
    val preg_vld = Vec(4, Bool())
    val vreg_vld = Vec(4, Bool())
    val ereg = Vec(4, UInt(5.W))
    val freg = Vec(4, UInt(6.W))
    val preg = Vec(4, UInt(7.W))
    val vreg = Vec(4, UInt(6.W))
  }
  val fromIS = new Bundle{
    val dis_type_stall = Bool()
    val inst2_vld = Bool()
    val inst2_ctrl_info = new ISCtrl
    val inst3_vld = Bool()
    val inst3_ctrl_info = new ISCtrl
    val stall     = Bool()
    val inst0_sel = UInt(2.W)
    val inst_sel  = UInt(3.W)
  }
  val fromIU = new Bundle{
    val mispred_stall = Bool()
    val yyxxCancel = Bool()
  }
  val fromPad = new Bundle{
    val yyIcgScanEn = Bool()
  }
  val fromHpcp = new Bundle{
    val cntEn = Bool()
  }
  val aiq_entry_cnt_updt = Vec(2, Flipped(Valid(UInt(4.W))))
  val viq_entry_cnt_updt = Vec(2, Flipped(Valid(UInt(4.W))))
  //rename tables
  val rt_resp  = new RT_Resp
  val frt_resp = new FRT_Resp
  val vrt_resp = new VRT_Resp
}

class IRStageOutput extends Bundle{
  //aiq0 aiq1 biq lsiq sdiq viq0 viq1 vmb
  val pre_dispatch = new IR_preDispatch

  val rt_req  = new RT_Req
  val frt_req = new FRT_Req
  val vrt_req = new VRT_Req

  val ir_pipedown = new Bundle{
    val pipedown = Bool()
    val gateclk = Bool()
    val instVld = Vec(4,Bool())

    val inst_src_match = Vec(6,new ir_srcMatch)
    val instData = Vec(4, new ISData)
  }
  val toFence = new Bundle{
    val ir_pipe_empty = Bool()
  }
  val toTop = new Bundle{
    val inst_vld = Vec(4, Bool())
    val ereg_not_vld = Bool()
    val freg_not_vld = Bool()
    val preg_not_vld = Bool()
    val vreg_not_vld = Bool()

    val mispred_stall = Bool()
  }
  val toHpcp = new Bundle{
    val inst_vld  = Vec(4, new Bool())
    val inst_type = Vec(4, new hpcp_type)
  }
  val toRTU = new Bundle{
    val ereg_alloc_vld = Vec(4, Bool())
    val ereg_alloc_gateclk_vld = Bool()

    val freg_alloc_vld = Vec(4, Bool())
    val freg_alloc_gateclk_vld = Bool()

    val preg_alloc_vld = Vec(4, Bool())
    val preg_alloc_gateclk_vld = Bool()

    val vreg_alloc_vld = Vec(4, Bool())
    val vreg_alloc_gateclk_vld = Bool()
  }

  val ir_stage_stall = Bool()
  val ir_stall = Bool()
  val ir_type_stall_inst2_vld = Bool()
  val ir_type_stall_inst3_vld = Bool()
  val lsiq_ir_bar_inst_vld = Bool()
}

class IRStageIO extends Bundle{
  val in  = Input(new IRStageInput)
  val out = Output(new IRStageOutput)
}

class IRStage extends Module {
  val io = IO(new IRStageIO)

  //Reg
  val instVld = RegInit(VecInit(Seq.fill(4)(false.B)))
  val instData = RegInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new IRData))))
  val depInfo = RegInit(0.U.asTypeOf(new DepInfo))

  val hpcp_inst_vld_ff = RegInit(VecInit(Seq.fill(4)(false.B)))
  val hpcp_inst_type   = RegInit(VecInit(Seq.fill(4)(0.U.asTypeOf(new hpcp_type))))

  val aiq_dlb_en = RegInit(false.B)
  val viq_dlb_en = RegInit(false.B)
  //Wire
  val ir_stall = Wire(Bool())
  val ir_stage_stall = Wire(Bool())
  val pre_dis_type_stall_pipedown2 = Wire(Bool())
  val pre_dis_pipedown2 = Wire(Bool())
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val ir_inst_clk_en = io.in.id_pipedown.gateclk || instVld.asUInt.orR

  //==========================================================
  //                 IR pipeline registers
  //==========================================================
  //----------------------------------------------------------
  //               Pipeline register implement
  //----------------------------------------------------------
  when(io.in.fromRTU.flush.fe || io.in.fromIU.yyxxCancel) {
    instVld := WireInit(VecInit(Seq.fill(4)(false.B)))
  }.elsewhen(!ir_stall) {
    instVld := io.in.id_pipedown.instVld
    instData := io.in.id_pipedown.instData
    depInfo := io.in.id_pipedown.depInfo
  }

  //for is type stall, ignore type stall
  io.out.ir_type_stall_inst2_vld := instVld(2)
  io.out.ir_type_stall_inst3_vld := instVld(3)

  io.out.toTop.inst_vld := instVld

  //TODO: whether need to implement
  //==========================================================
  //                Prepare IR control data
  //==========================================================
  //TODO: whether need to implement

  //==========================================================
  //               Assign ptag, creg and lsu pc
  //==========================================================
  val inst_dst_preg = Wire(Vec(4, UInt(7.W)))
  val pipedown_inst_dst_preg = Wire(Vec(4, UInt(7.W)))
  for (i <- 0 until 4) {
    inst_dst_preg(i) := Mux(instData(i).dst_x0, 0.U(7.W), io.in.fromRTU.preg(i))
    pipedown_inst_dst_preg(i) := Mux(instData(i).dst_vld, inst_dst_preg(i), 0.U(7.W))
  }
  val inst_dst_vreg = io.in.fromRTU.vreg
  val inst_dst_freg = io.in.fromRTU.freg
  val inst_dst_ereg = io.in.fromRTU.ereg

  //==========================================================
  //                Prepare Rename Table data
  //==========================================================
  //----------------------------------------------------------
  //            Rename Table inst valid signals
  //----------------------------------------------------------
  //rename table inst can only be from IR pipeline inst,
  //their inst valid signals are from ir inst valid,
  //and consider is stall signal (including dispatch stall
  //and type stall). the is stall logic is in rt module.
  //----------------------------------------------------------
  //                 Dependency Information
  //----------------------------------------------------------
  io.out.rt_req.depInfo := depInfo
  io.out.frt_req.depInfo := depInfo

  val inst_dstv_imp = Wire(Vec(4, Bool()))
  for (i <- 0 until 4) {
    inst_dstv_imp(i) := instData(i).dstf_reg(5) || instData(i).dstv_reg(5)
  }

  //----------------------------------------------------------
  //                       RT signals
  //----------------------------------------------------------
  for (i <- 0 until 4) {
    io.out.rt_req.instInfo(i).valid := instVld(i)

    io.out.rt_req.instInfo(i).bits.dst_vld := instData(i).dst_vld
    io.out.rt_req.instInfo(i).bits.dst_reg := instData(i).dst_reg
    io.out.rt_req.instInfo(i).bits.dst_preg := inst_dst_preg(i)
    io.out.rt_req.instInfo(i).bits.src_vld(0) := instData(i).src0_vld
    io.out.rt_req.instInfo(i).bits.src_reg(0) := instData(i).src0_reg
    io.out.rt_req.instInfo(i).bits.src_vld(1) := instData(i).src1_vld
    io.out.rt_req.instInfo(i).bits.src_reg(1) := instData(i).src1_reg
    io.out.rt_req.instInfo(i).bits.src_vld(2) := instData(i).src2_vld
    io.out.rt_req.instInfo(i).bits.mla := instData(i).mla
    io.out.rt_req.instInfo(i).bits.mov := instData(i).mov
  }
  io.out.rt_req.instInfo(3).bits.mov := false.B

  //----------------------------------------------------------
  //                      FRT signals
  //----------------------------------------------------------
  for (i <- 0 until 4) {
    io.out.frt_req.instInfo(i).valid := instVld(i)

    io.out.frt_req.instInfo(i).bits.dstf_vld := instData(i).dstf_vld
    io.out.frt_req.instInfo(i).bits.dstf_reg := instData(i).dstf_reg
    io.out.frt_req.instInfo(i).bits.dst_freg := inst_dst_freg(i)
    io.out.frt_req.instInfo(i).bits.dste_vld := instData(i).dste_vld
    io.out.frt_req.instInfo(i).bits.dst_ereg := inst_dst_ereg(i)
    io.out.frt_req.instInfo(i).bits.srcf_vld(0) := instData(i).srcf0_vld
    io.out.frt_req.instInfo(i).bits.srcf_reg(0) := instData(i).srcf0_reg
    io.out.frt_req.instInfo(i).bits.srcf_vld(1) := instData(i).srcf1_vld
    io.out.frt_req.instInfo(i).bits.srcf_reg(1) := instData(i).srcf1_reg
    io.out.frt_req.instInfo(i).bits.srcf_vld(2) := instData(i).srcf2_vld
    io.out.frt_req.instInfo(i).bits.srcf_reg(2) := instData(i).srcf2_reg
    io.out.frt_req.instInfo(i).bits.fmla := instData(i).fmla
    io.out.frt_req.instInfo(i).bits.fmov := instData(i).fmov
  }
  io.out.frt_req.instInfo(3).bits.fmov := false.B

  //----------------------------------------------------------
  //                      VRT signals
  //----------------------------------------------------------
  for (i <- 0 until 4) {
    io.out.vrt_req.instInfo(i).dstv_vld := instData(i).dstv_vld
    io.out.vrt_req.instInfo(i).dstv_reg := instData(i).dstv_reg
    io.out.vrt_req.instInfo(i).dst_vreg := inst_dst_vreg(i)
    io.out.vrt_req.instInfo(i).srcv_vld(0) := instData(i).srcv0_vld
    io.out.vrt_req.instInfo(i).srcv_reg(0) := instData(i).srcv0_reg
    io.out.vrt_req.instInfo(i).srcv_vld(1) := instData(i).srcv1_vld
    io.out.vrt_req.instInfo(i).srcv_reg(1) := instData(i).srcv1_reg
    io.out.vrt_req.instInfo(i).srcv_vld(2) := instData(i).srcv2_vld
    io.out.vrt_req.instInfo(i).srcvm_vld := instData(i).srcvm_vld
    io.out.vrt_req.instInfo(i).vmla := instData(i).vmla
  }

  //==========================================================
  //                    FRT / VRT MUX
  //==========================================================
  val rt_inst_srcv0_vld = Wire(Vec(4, Bool()))
  val rt_inst_srcv1_vld = Wire(Vec(4, Bool()))
  val rt_inst_srcv2_vld = Wire(Vec(4, Bool()))
  val rt_inst_srcvm_vld = Wire(Vec(4, Bool()))
  val rt_inst_dstv_vld = Wire(Vec(4, Bool()))
  val rt_inst_dste_vld = Wire(Vec(4, Bool()))
  val rt_inst_vmla = Wire(Vec(4, Bool()))

  val rt_inst_srcv0_data = Wire(Vec(4, new srcData9))
  val rt_inst_srcv1_data = Wire(Vec(4, new srcData9))
  val rt_inst_srcv2_data = Wire(Vec(4, new srcData10))
  val rt_inst_srcvm_data = Wire(Vec(4, new srcData9))

  val rt_inst_dstv_reg = Wire(Vec(4, UInt(5.W)))
  val rt_inst_dst_vreg = Wire(Vec(4, UInt(7.W)))
  val rt_inst_rel_vreg = Wire(Vec(4, UInt(7.W)))
  val rt_inst_dst_ereg = Wire(Vec(4, UInt(5.W)))
  val rt_inst_rel_ereg = Wire(Vec(4, UInt(5.W)))

  for (i <- 0 until 4) {
    rt_inst_srcv0_vld(i) := instData(i).srcf0_vld || instData(i).srcv0_vld
    rt_inst_srcv1_vld(i) := instData(i).srcf1_vld || instData(i).srcv1_vld
    rt_inst_srcv2_vld(i) := instData(i).srcf2_vld || instData(i).srcv2_vld
    rt_inst_srcvm_vld(i) := instData(i).srcvm_vld
    rt_inst_dstv_vld(i) := instData(i).dstf_vld || instData(i).dstv_vld
    rt_inst_dste_vld(i) := instData(i).dste_vld
    rt_inst_vmla(i) := instData(i).fmla || instData(i).vmla
    rt_inst_srcv0_data(i) := Mux(instData(i).srcv0_vld, io.in.vrt_resp.instInfo(i).srcv0_data, io.in.frt_resp.instInfo(i).srcf0_data)
    rt_inst_srcv1_data(i) := Mux(instData(i).srcv1_vld, io.in.vrt_resp.instInfo(i).srcv1_data, io.in.frt_resp.instInfo(i).srcf1_data)
    rt_inst_srcv2_data(i) := Mux(instData(i).srcv2_vld, io.in.vrt_resp.instInfo(i).srcv2_data, io.in.frt_resp.instInfo(i).srcf2_data)
    rt_inst_srcvm_data(i) := io.in.vrt_resp.instInfo(i).srcvm_data
    rt_inst_dstv_reg(i) := Mux(instData(i).dstv_vld, instData(i).dstv_reg, instData(i).dstf_reg)
    rt_inst_dst_vreg(i) := Mux(instData(i).dstv_vld, Cat(1.U(1.W), inst_dst_vreg(i)), Cat(0.U(1.W), inst_dst_freg(i)))
    rt_inst_rel_vreg(i) := Mux(instData(i).dstv_vld, io.in.vrt_resp.instInfo(i).rel_vreg, io.in.frt_resp.instInfo(i).rel_freg)
    rt_inst_dst_ereg(i) := Mux(instData(i).dste_vld, inst_dst_ereg(i), 0.U(5.W))
    rt_inst_rel_ereg(i) := io.in.frt_resp.instInfo(i).rel_ereg
  }

  //----------------------------------------------------------
  //                      source match
  //----------------------------------------------------------
  for (i <- 0 until 6) {
    io.out.ir_pipedown.inst_src_match(i).src0 := io.in.rt_resp.srcMatch(i).src0
    io.out.ir_pipedown.inst_src_match(i).src1 := io.in.rt_resp.srcMatch(i).src1
    io.out.ir_pipedown.inst_src_match(i).src2 := io.in.rt_resp.srcMatch(i).src2
    io.out.ir_pipedown.inst_src_match(i).srcv2 := io.in.frt_resp.srcf2Match(i) || io.in.vrt_resp.srcv2Match(i)
  }

  //==========================================================
  //                   Instance IR Decoder
  //==========================================================
  val ir_decode = Seq.fill(4)(Module(new ct_idu_ir_decd))
  for (i <- 0 until 4) {
    ir_decode(i).io.x_illegal := instData(i).illegal
    ir_decode(i).io.x_opcode := instData(i).opcode
    ir_decode(i).io.x_type_alu := instData(i).inst_type(0)
    ir_decode(i).io.x_type_staddr := instData(i).inst_type(5)
    ir_decode(i).io.x_type_vload := instData(i).vmb
    ir_decode(i).io.x_vsew := instData(i).vsew
  }

  //==========================================================
  //                 Rename for IS data path
  //==========================================================
  //load inst will create
  //except lrw and split load (last split will create)
  //pop inst do not create lsfifo

  //==========================================================
  //               Prepare RF Launch Signal
  //==========================================================
  //timing optimization:
  //prepare rf launch signal at ir
  val lch_preg = Wire(Vec(4, Bool()))
  for (i <- 0 until 4) {
    lch_preg(i) := instData(i).dst_vld && (instData(i).inst_type(0) || instData(i).inst_type(9))
  }

  //==========================================================
  //                 Rename for IS data path
  //==========================================================
  //----------------------------------------------------------
  //                   Data path rename
  //----------------------------------------------------------
  for (i <- 0 until 4) {
    io.out.ir_pipedown.instData(i).VL_PRED := instData(i).vl_pred
    io.out.ir_pipedown.instData(i).VL := instData(i).vl
    io.out.ir_pipedown.instData(i).LCH_PREG := lch_preg(i)
    io.out.ir_pipedown.instData(i).VAMO := ir_decode(i).io.x_vamo
    io.out.ir_pipedown.instData(i).UNIT_STRIDE := ir_decode(i).io.x_unit_stride
    io.out.ir_pipedown.instData(i).VMB := instData(i).vmb
    io.out.ir_pipedown.instData(i).DSTV_IMP := inst_dstv_imp(i)
    io.out.ir_pipedown.instData(i).VIQ_SRCV12_SWITCH := ir_decode(i).io.x_viq_srcv12_switch
    io.out.ir_pipedown.instData(i).VSETVL := ir_decode(i).io.x_vsetvl
    io.out.ir_pipedown.instData(i).VSETVLI := ir_decode(i).io.x_vsetvli
    io.out.ir_pipedown.instData(i).VSEW := instData(i).vsew
    io.out.ir_pipedown.instData(i).VLMUL := instData(i).vlmul
    io.out.ir_pipedown.instData(i).VMUL := ir_decode(i).io.x_vmul
    io.out.ir_pipedown.instData(i).VMUL_UNSPLIT := ir_decode(i).io.x_vmul_unsplit
    io.out.ir_pipedown.instData(i).VMLA_SHORT := ir_decode(i).io.x_vmla_short
    io.out.ir_pipedown.instData(i).VMLA_TYPE := ir_decode(i).io.x_vmla_type
    io.out.ir_pipedown.instData(i).SPLIT_NUM := instData(i).split_num
    io.out.ir_pipedown.instData(i).NO_SPEC := instData(i).no_spec
    io.out.ir_pipedown.instData(i).ALU_SHORT := ir_decode(i).io.x_alu_short
    io.out.ir_pipedown.instData(i).MLA := instData(i).mla
    io.out.ir_pipedown.instData(i).STR := ir_decode(i).io.x_str
    io.out.ir_pipedown.instData(i).SPLIT_LAST := instData(i).split_last
    io.out.ir_pipedown.instData(i).MFVR := ir_decode(i).io.x_mfvr
    io.out.ir_pipedown.instData(i).MTVR := ir_decode(i).io.x_mtvr
    io.out.ir_pipedown.instData(i).VMLA := rt_inst_vmla(i)
    io.out.ir_pipedown.instData(i).VDIV := ir_decode(i).io.x_vdiv
    io.out.ir_pipedown.instData(i).PIPE7 := instData(i).inst_type(8) //pipe7
    io.out.ir_pipedown.instData(i).PIPE6 := instData(i).inst_type(7) //pipe6
    io.out.ir_pipedown.instData(i).PIPE67 := instData(i).inst_type(6) //pipe67
    io.out.ir_pipedown.instData(i).IID_PLUS := instData(i).iid_plus
    io.out.ir_pipedown.instData(i).BKPTB_INST := instData(i).bkptb_inst
    io.out.ir_pipedown.instData(i).BKPTA_INST := instData(i).bkpta_inst
    io.out.ir_pipedown.instData(i).EXPT := instData(i).expt
    io.out.ir_pipedown.instData(i).RTS := ir_decode(i).io.x_rts
    io.out.ir_pipedown.instData(i).SPECIAL := instData(i).inst_type(9) //special
    io.out.ir_pipedown.instData(i).LSU := instData(i).inst_type(4) //lsu
    io.out.ir_pipedown.instData(i).DIV := instData(i).inst_type(3) //div
    io.out.ir_pipedown.instData(i).MULT := instData(i).inst_type(2) //mult
    io.out.ir_pipedown.instData(i).INTMASK := instData(i).intmask
    io.out.ir_pipedown.instData(i).SPLIT := instData(i).split
    io.out.ir_pipedown.instData(i).LENGTH := instData(i).length
    io.out.ir_pipedown.instData(i).PCFIFO := ir_decode(i).io.x_pcfifo
    io.out.ir_pipedown.instData(i).PCALL := ir_decode(i).io.x_pcall
    io.out.ir_pipedown.instData(i).BJU := instData(i).inst_type(1) //bju
    io.out.ir_pipedown.instData(i).LSU_PC := instData(i).pc
    io.out.ir_pipedown.instData(i).BAR_TYPE := ir_decode(i).io.x_bar_type
    io.out.ir_pipedown.instData(i).BAR := ir_decode(i).io.x_bar
    io.out.ir_pipedown.instData(i).STADDR := instData(i).inst_type(5) //staddr
    io.out.ir_pipedown.instData(i).STORE := ir_decode(i).io.x_store
    io.out.ir_pipedown.instData(i).LOAD := ir_decode(i).io.x_load
    io.out.ir_pipedown.instData(i).ALU := instData(i).inst_type(0) //alu

    io.out.ir_pipedown.instData(i).dst_rel_ereg := rt_inst_rel_ereg(i)
    io.out.ir_pipedown.instData(i).dst_ereg := rt_inst_dst_ereg(i)
    io.out.ir_pipedown.instData(i).dst_rel_vreg := rt_inst_rel_vreg(i)
    io.out.ir_pipedown.instData(i).dst_vreg := rt_inst_dst_vreg(i)
    io.out.ir_pipedown.instData(i).dstv_reg := rt_inst_dstv_reg(i)
    io.out.ir_pipedown.instData(i).srcvm_lsu_match := 0.U(1.W) //lsu match for issue
    io.out.ir_pipedown.instData(i).srcvm_bp_rdy := 0.U(2.W) //rdy for issue/bypass
    io.out.ir_pipedown.instData(i).srcvm_data := rt_inst_srcvm_data(i)
    io.out.ir_pipedown.instData(i).srcv2_lsu_match := 0.U(1.W)
    io.out.ir_pipedown.instData(i).srcv2_bp_rdy := 0.U(2.W)
    io.out.ir_pipedown.instData(i).srcv2_data := rt_inst_srcv2_data(i)
    io.out.ir_pipedown.instData(i).srcv1_lsu_match := 0.U(1.W)
    io.out.ir_pipedown.instData(i).srcv1_bp_rdy := 0.U(2.W)
    io.out.ir_pipedown.instData(i).srcv1_data := rt_inst_srcv1_data(i)
    io.out.ir_pipedown.instData(i).srcv0_lsu_match := 0.U(1.W)
    io.out.ir_pipedown.instData(i).srcv0_bp_rdy := 0.U(2.W)
    io.out.ir_pipedown.instData(i).srcv0_data := rt_inst_srcv0_data(i)
    io.out.ir_pipedown.instData(i).dste_vld := rt_inst_dste_vld(i)
    io.out.ir_pipedown.instData(i).dstv_vld := rt_inst_dstv_vld(i)
    io.out.ir_pipedown.instData(i).srcvm_vld := rt_inst_srcvm_vld(i)
    io.out.ir_pipedown.instData(i).srcv_vld(2) := rt_inst_srcv2_vld(i)
    io.out.ir_pipedown.instData(i).srcv_vld(1) := rt_inst_srcv1_vld(i)
    io.out.ir_pipedown.instData(i).srcv_vld(0) := rt_inst_srcv0_vld(i)

    io.out.ir_pipedown.instData(i).dst_rel_preg := io.in.rt_resp.instInfo(i).rel_preg
    io.out.ir_pipedown.instData(i).dst_preg := pipedown_inst_dst_preg(i)
    io.out.ir_pipedown.instData(i).dst_reg := instData(i).dst_reg(4, 0)
    io.out.ir_pipedown.instData(i).src2_lsu_match := 0.U(1.W)
    io.out.ir_pipedown.instData(i).src2_bp_rdy := 0.U(2.W)
    io.out.ir_pipedown.instData(i).src2_data := io.in.rt_resp.instInfo(i).src2_data
    io.out.ir_pipedown.instData(i).src1_lsu_match := 0.U(1.W)
    io.out.ir_pipedown.instData(i).src1_bp_rdy := 0.U(2.W)
    io.out.ir_pipedown.instData(i).src1_data := io.in.rt_resp.instInfo(i).src1_data
    io.out.ir_pipedown.instData(i).src0_lsu_match := 0.U(1.W)
    io.out.ir_pipedown.instData(i).src0_bp_rdy := 0.U(2.W)
    io.out.ir_pipedown.instData(i).src0_data := io.in.rt_resp.instInfo(i).src0_data
    io.out.ir_pipedown.instData(i).dst_vld := instData(i).dst_vld
    io.out.ir_pipedown.instData(i).src_vld(2) := instData(i).src2_vld
    io.out.ir_pipedown.instData(i).src_vld(1) := instData(i).src1_vld
    io.out.ir_pipedown.instData(i).src_vld(0) := instData(i).src0_vld

    io.out.ir_pipedown.instData(i).opcode := instData(i).opcode
  }

  //TODO: ir_ctrl
  //----------------------------------------------------------
  //             IR pipedown inst valid signals
  //----------------------------------------------------------
  //to reduce dispatch stall timing, IR and IS stage use delicate
  //stall and inst valid design:
  //there are two IS stall sources: dispatch stall and type stall
  //1.if there is dispatch stall, IS stage contrl registers does
  //  not receive new inst predispatch information
  //2.if there is type stall, is_dis_inst_sel will select IS stage
  //  inst valid and datapath
  //there is one IR stall source: creg stall
  //3.if ther is creg stall, cannot pipe IR inst to IS stage.
  //  however creg stall does not affect IS stage shift inst
  //so pipedown inst valid can ignore both dispatch stall and type
  //stall, but should consider creg stall.
  val ir_pipedown_stall = ir_stage_stall || io.in.fromIS.dis_type_stall
  val ir_pipedown_inst_vld = Wire(Vec(4, Bool()))

  ir_pipedown_inst_vld(0) :=
    io.in.fromIS.inst0_sel(0) && io.in.fromIS.inst2_vld ||
    io.in.fromIS.inst0_sel(1) && instVld(0) && !ir_pipedown_stall

  ir_pipedown_inst_vld(1) :=
    io.in.fromIS.inst_sel(0) && io.in.fromIS.inst3_vld ||
    io.in.fromIS.inst_sel(1) && instVld(0) && !ir_pipedown_stall ||
    io.in.fromIS.inst_sel(2) && instVld(1) && !ir_pipedown_stall

  ir_pipedown_inst_vld(2) :=
    io.in.fromIS.inst_sel(0) && instVld(0) && !ir_pipedown_stall ||
    io.in.fromIS.inst_sel(1) && instVld(1) && !ir_pipedown_stall ||
    io.in.fromIS.inst_sel(2) && instVld(2) && !ir_pipedown_stall

  ir_pipedown_inst_vld(3) :=
    io.in.fromIS.inst_sel(0) && instVld(1) && !ir_pipedown_stall ||
    io.in.fromIS.inst_sel(1) && instVld(2) && !ir_pipedown_stall ||
    io.in.fromIS.inst_sel(2) && instVld(3) && !ir_pipedown_stall

  //----------------------------------------------------------
  //            Fence pipeline emtry signal
  //----------------------------------------------------------
  io.out.toFence.ir_pipe_empty := !instVld(0)

  //----------------------------------------------------------
  //               Barrier inst valid signal
  //----------------------------------------------------------
  //ignore stall signals
  io.out.lsiq_ir_bar_inst_vld :=
    instVld(0) && ir_decode(0).io.x_bar ||
    instVld(1) && ir_decode(1).io.x_bar ||
    instVld(2) && ir_decode(2).io.x_bar ||
    instVld(3) && ir_decode(3).io.x_bar

  //----------------------------------------------------------
  //            Rename pipedown inst valid signals
  //----------------------------------------------------------
  io.out.ir_pipedown.instVld := ir_pipedown_inst_vld

  //==========================================================
  //                IR stage pipedown signals
  //==========================================================
  //pipedown signal is used to update IS stage data path
  io.out.ir_pipedown.pipedown := ir_pipedown_inst_vld.asUInt.orR
  io.out.ir_pipedown.gateclk := instVld.asUInt.orR || io.in.fromIS.inst2_vld || io.in.fromIS.inst3_vld

  //==========================================================
  //                  IR stage stall signals
  //==========================================================
  val ir_preg_stall_vec = Wire(Vec(4, Bool()))
  val ir_vreg_stall_vec = Wire(Vec(4, Bool()))
  val ir_freg_stall_vec = Wire(Vec(4, Bool()))
  val ir_ereg_stall_vec = Wire(Vec(4, Bool()))

  for(i <- 0 until 4){
    ir_preg_stall_vec(i) := instVld(i) && instData(i).dst_vld  && !io.in.fromRTU.preg_vld(i)
    ir_vreg_stall_vec(i) := instVld(i) && instData(i).dstv_vld && !io.in.fromRTU.vreg_vld(i)
    ir_freg_stall_vec(i) := instVld(i) && instData(i).dstf_vld && !io.in.fromRTU.freg_vld(i)
    ir_ereg_stall_vec(i) := instVld(i) && instData(i).dste_vld && !io.in.fromRTU.ereg_vld(i)
  }

  val ir_preg_stall = ir_preg_stall_vec.asUInt.orR
  val ir_vreg_stall = ir_vreg_stall_vec.asUInt.orR
  val ir_freg_stall = ir_freg_stall_vec.asUInt.orR
  val ir_ereg_stall = ir_ereg_stall_vec.asUInt.orR

  io.out.toTop.preg_not_vld := !io.in.fromRTU.preg_vld.asUInt.andR
  io.out.toTop.vreg_not_vld := !io.in.fromRTU.vreg_vld.asUInt.andR
  io.out.toTop.freg_not_vld := !io.in.fromRTU.freg_vld.asUInt.andR
  io.out.toTop.ereg_not_vld := !io.in.fromRTU.ereg_vld.asUInt.andR

  ir_stage_stall :=
    ir_preg_stall || ir_vreg_stall || ir_freg_stall || ir_ereg_stall ||
    io.in.fromRTU.flush_stall || io.in.fromIU.mispred_stall
  io.out.ir_stage_stall := ir_stage_stall

  io.out.toTop.mispred_stall := io.in.fromIU.mispred_stall
  //ir stall include preg stall and is stall
  ir_stall := instVld(0) && (io.in.fromIS.stall || ir_stage_stall)
  io.out.ir_stall := ir_stall
  //==========================================================
  //  Allocate signal for Ptag pool / PST preg / LSU pcfifo
  //==========================================================
  val preg_alloc_gateclk_vec = Wire(Vec(4, Bool()))
  val vreg_alloc_gateclk_vec = Wire(Vec(4, Bool()))
  val freg_alloc_gateclk_vec = Wire(Vec(4, Bool()))
  val ereg_alloc_gateclk_vec = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    io.out.toRTU.preg_alloc_vld(i) := instVld(i) && !ir_stall && instData(i).dst_vld  && !instData(i).dst_x0
    io.out.toRTU.vreg_alloc_vld(i) := instVld(i) && !ir_stall && instData(i).dstv_vld
    io.out.toRTU.freg_alloc_vld(i) := instVld(i) && !ir_stall && instData(i).dstf_vld
    io.out.toRTU.ereg_alloc_vld(i) := instVld(i) && !ir_stall && instData(i).dste_vld
    preg_alloc_gateclk_vec(i) := instVld(i) && instData(i).dst_vld
    vreg_alloc_gateclk_vec(i) := instVld(i) && instData(i).dstv_vld
    freg_alloc_gateclk_vec(i) := instVld(i) && instData(i).dstf_vld
    ereg_alloc_gateclk_vec(i) := instVld(i) && instData(i).dste_vld
  }
  io.out.toRTU.preg_alloc_gateclk_vld := preg_alloc_gateclk_vec.asUInt.orR
  io.out.toRTU.vreg_alloc_gateclk_vld := vreg_alloc_gateclk_vec.asUInt.orR
  io.out.toRTU.freg_alloc_gateclk_vld := freg_alloc_gateclk_vec.asUInt.orR
  io.out.toRTU.ereg_alloc_gateclk_vld := ereg_alloc_gateclk_vec.asUInt.orR

  //==========================================================
  //                 Prepare Dispatch Signals
  //==========================================================
  //==========================================================
  //                Prepare IR control data
  //==========================================================
  val inst_ctrl_info = Wire(Vec(4, new ISCtrl))
  val hpcp_type = Wire(Vec(4, new hpcp_type))
  for(i <- 0 until 4){
    inst_ctrl_info(i).VMB     := instData(i).vmb
    inst_ctrl_info(i).PIPE7   := instData(i).inst_type(8)
    inst_ctrl_info(i).PIPE6   := instData(i).inst_type(7)
    inst_ctrl_info(i).PIPE67  := instData(i).inst_type(6)
    inst_ctrl_info(i).SPECIAL := instData(i).inst_type(9)
    inst_ctrl_info(i).STADDR  := instData(i).inst_type(5)
    inst_ctrl_info(i).INTMASK := instData(i).intmask
    inst_ctrl_info(i).SPLIT   := instData(i).split
    inst_ctrl_info(i).LSU     := instData(i).inst_type(4)
    inst_ctrl_info(i).BJU     := instData(i).inst_type(1)
    inst_ctrl_info(i).DIV     := instData(i).inst_type(3)
    inst_ctrl_info(i).MULT    := instData(i).inst_type(2)
    inst_ctrl_info(i).ALU     := instData(i).inst_type(0)

    hpcp_type(i).alu   := instData(i).inst_type(0) || instData(i).inst_type(2) || instData(i).inst_type(3)
    hpcp_type(i).lsu   := instData(i).inst_type(4)
    hpcp_type(i).vec   := ir_decode(i).io.x_vec
    hpcp_type(i).csr   := ir_decode(i).io.x_csr
    hpcp_type(i).ecall := ir_decode(i).io.x_ecall
    hpcp_type(i).sync  := ir_decode(i).io.x_sync
    hpcp_type(i).fp    := ir_decode(i).io.x_fp
  }

  //----------------------------------------------------------
  //            MUX between IR inst and IS shift inst
  //----------------------------------------------------------
  val ir_pipedown_inst_ctrl_info = Wire(Vec(4, new ISCtrl))

  ir_pipedown_inst_ctrl_info(0) := MuxLookup(io.in.fromIS.inst0_sel, 0.U.asTypeOf(new ISCtrl), Seq(
    "b01".U -> io.in.fromIS.inst2_ctrl_info,
    "b10".U -> inst_ctrl_info(0)
  ))

  ir_pipedown_inst_ctrl_info(1) := MuxLookup(io.in.fromIS.inst_sel, 0.U.asTypeOf(new ISCtrl), Seq(
    "b001".U -> io.in.fromIS.inst3_ctrl_info,
    "b010".U -> inst_ctrl_info(0),
    "b100".U -> inst_ctrl_info(1)
  ))
  //if sel is inst2, is inst2 should not be valid
  ir_pipedown_inst_ctrl_info(2) := MuxLookup(io.in.fromIS.inst_sel, 0.U.asTypeOf(new ISCtrl), Seq(
    "b001".U -> inst_ctrl_info(0),
    "b010".U -> inst_ctrl_info(1),
    "b100".U -> inst_ctrl_info(2)
  ))
  //if sel is inst3, is inst3 should not be valid
  ir_pipedown_inst_ctrl_info(3) := MuxLookup(io.in.fromIS.inst_sel, 0.U.asTypeOf(new ISCtrl), Seq(
    "b001".U -> inst_ctrl_info(1),
    "b010".U -> inst_ctrl_info(2),
    "b100".U -> inst_ctrl_info(3)
  ))

  //TODO: rename for pre dispatch
  val inst_aiq0          = Wire(Vec(4, Bool()))
  val inst_aiq1_bef_dlb  = Wire(Vec(4, Bool()))
  val inst_aiq01_bef_dlb = Wire(Vec(4, Bool()))
  val inst_viq0          = Wire(Vec(4, Bool()))
  val inst_viq1_bef_dlb  = Wire(Vec(4, Bool()))
  val inst_viq01_bef_dlb = Wire(Vec(4, Bool()))

  val inst_special = Wire(Vec(4, Bool()))
  val inst_split   = Wire(Vec(4, Bool()))
  val inst_intmask = Wire(Vec(4, Bool()))

  for(i <- 0 until 4){
    inst_aiq0(i)          := ir_pipedown_inst_ctrl_info(i).DIV || ir_pipedown_inst_ctrl_info(i).SPECIAL
    inst_aiq1_bef_dlb(i)  := ir_pipedown_inst_ctrl_info(i).MULT
    inst_aiq01_bef_dlb(i) := ir_pipedown_inst_ctrl_info(i).ALU
    inst_viq0(i)          := ir_pipedown_inst_ctrl_info(i).PIPE6
    inst_viq1_bef_dlb(i)  := ir_pipedown_inst_ctrl_info(i).PIPE7
    inst_viq01_bef_dlb(i) := ir_pipedown_inst_ctrl_info(i).PIPE67

    inst_special(i) := ir_pipedown_inst_ctrl_info(i).SPECIAL
    inst_split(i)   := ir_pipedown_inst_ctrl_info(i).SPLIT
    inst_intmask(i) := ir_pipedown_inst_ctrl_info(i).INTMASK
  }


  //----------------------------------------------------------
  //             Dynamic Load Balance (DLB) of AIQ
  //----------------------------------------------------------
  val dlb_clk_en = io.in.aiq_entry_cnt_updt(0).valid || io.in.aiq_entry_cnt_updt(1).valid ||
    io.in.viq_entry_cnt_updt(0).valid || io.in.viq_entry_cnt_updt(1).valid || viq_dlb_en

  val aiq_entry_cnt_diff = io.in.aiq_entry_cnt_updt(0).bits - io.in.aiq_entry_cnt_updt(1).bits
  //TODO: figure out the meaning of aiq_entry_cnt_diff_8
  val aiq_entry_cnt_diff_8 = io.in.aiq_entry_cnt_updt(0).bits === 8.U && io.in.aiq_entry_cnt_updt(1).bits === 0.U
  val aiq_entry_cnt_diff_7_2 = !aiq_entry_cnt_diff(3) && aiq_entry_cnt_diff(2,1).orR

  val aiq_dlb_updt_vld = !io.in.fromCp0.dlbDisable && (aiq_entry_cnt_diff_8 || aiq_entry_cnt_diff_7_2)

  when(io.in.fromRTU.flush.fe || io.in.fromRTU.flush.is || io.in.fromRTU.flush.be){
    aiq_dlb_en := false.B
  }.elsewhen(io.in.aiq_entry_cnt_updt(0).valid || io.in.aiq_entry_cnt_updt(1).valid){
    aiq_dlb_en := aiq_dlb_updt_vld
  }

  //----------------------------------------------------------
  //             Dynamic Load Balance (DLB) of VIQ
  //----------------------------------------------------------
 val viq_entry_cnt_diff = io.in.viq_entry_cnt_updt(0).bits - io.in.viq_entry_cnt_updt(1).bits
  //TODO: figure out the meaning of viq_entry_cnt_diff_8
  val viq_entry_cnt_diff_8 = io.in.viq_entry_cnt_updt(0).bits === 8.U && io.in.viq_entry_cnt_updt(1).bits === 0.U
  val viq_entry_cnt_diff_7_2 = !viq_entry_cnt_diff(3) && viq_entry_cnt_diff(2,1).orR

  val viq_dlb_updt_vld = !io.in.fromCp0.dlbDisable && (viq_entry_cnt_diff_8 || viq_entry_cnt_diff_7_2)

  when(io.in.fromRTU.flush.fe || io.in.fromRTU.flush.is || io.in.fromRTU.flush.be){
    viq_dlb_en := false.B
  }.elsewhen(io.in.viq_entry_cnt_updt(0).valid || io.in.viq_entry_cnt_updt(1).valid){
    viq_dlb_en := viq_dlb_updt_vld
  }

  //----------------------------------------------------------
  //                   DLB change inst type
  //----------------------------------------------------------
  //when dynamic load balance enable, change inst type from aiq01 to aiq1
  //if inst0 and inst1 is both changed, do not change inst2 and inst3
  val inst_aiq1  = Wire(Vec(4, Bool()))
  val inst_aiq01 = Wire(Vec(4, Bool()))

  inst_aiq1(0)  := inst_aiq1_bef_dlb(0) || inst_aiq01_bef_dlb(0) && aiq_dlb_en
  inst_aiq01(0) := inst_aiq01_bef_dlb(0) && !aiq_dlb_en
  inst_aiq1(1)  := inst_aiq1_bef_dlb(1) || inst_aiq01_bef_dlb(1) && aiq_dlb_en
  inst_aiq01(1) := inst_aiq01_bef_dlb(1) && !aiq_dlb_en

  inst_aiq1(2)  := inst_aiq1_bef_dlb(2) || inst_aiq01_bef_dlb(2) && !(inst_aiq01_bef_dlb(0) && inst_aiq01_bef_dlb(1)) && aiq_dlb_en
  inst_aiq01(2) := inst_aiq01_bef_dlb(2) && (inst_aiq01_bef_dlb(0) && inst_aiq01_bef_dlb(1) || !aiq_dlb_en)
  inst_aiq1(3)  := inst_aiq1_bef_dlb(3) || inst_aiq01_bef_dlb(3) && !(inst_aiq01_bef_dlb(0) && inst_aiq01_bef_dlb(1)) && aiq_dlb_en
  inst_aiq01(3) := inst_aiq01_bef_dlb(3) && (inst_aiq01_bef_dlb(0) && inst_aiq01_bef_dlb(1) || !aiq_dlb_en)

  //when dynamic load balance enable, change inst type from viq01 to viq1
  //if inst0 and inst1 is both changed, do not change inst2 and inst3
  val inst_viq1  = Wire(Vec(4, Bool()))
  val inst_viq01 = Wire(Vec(4, Bool()))

  inst_viq1(0)  := inst_viq1_bef_dlb(0) || inst_viq01_bef_dlb(0) && viq_dlb_en
  inst_viq01(0) := inst_viq01_bef_dlb(0) && !viq_dlb_en
  inst_viq1(1)  := inst_viq1_bef_dlb(1) || inst_viq01_bef_dlb(1) && viq_dlb_en
  inst_viq01(1) := inst_viq01_bef_dlb(1) && !viq_dlb_en

  inst_viq1(2)  := inst_viq1_bef_dlb(2) || inst_viq01_bef_dlb(2) && !(inst_viq01_bef_dlb(0) && inst_viq01_bef_dlb(1)) && viq_dlb_en
  inst_viq01(2) := inst_viq01_bef_dlb(2) && (inst_viq01_bef_dlb(0) && inst_viq01_bef_dlb(1) || !viq_dlb_en)
  inst_viq1(3)  := inst_viq1_bef_dlb(3) || inst_viq01_bef_dlb(3) && !(inst_viq01_bef_dlb(0) && inst_viq01_bef_dlb(1)) && viq_dlb_en
  inst_viq01(3) := inst_viq01_bef_dlb(3) && (inst_viq01_bef_dlb(0) && inst_viq01_bef_dlb(1) || !viq_dlb_en)

  //----------------------------------------------------------
  //                  prepare IS inst valid
  //----------------------------------------------------------
  //prepare is inst valid
  val pre_dis_inst_vld = Wire(Vec(4, Bool()))
  pre_dis_inst_vld(0) := ir_pipedown_inst_vld(0)
  pre_dis_inst_vld(1) := ir_pipedown_inst_vld(1)
  pre_dis_inst_vld(2) := ir_pipedown_inst_vld(2) && !pre_dis_type_stall_pipedown2
  pre_dis_inst_vld(3) := ir_pipedown_inst_vld(3) && !pre_dis_type_stall_pipedown2
  io.out.pre_dispatch.inst_vld := pre_dis_inst_vld
  //----------------------------------------------------------
  //             prepare IS dispatch type stall
  //----------------------------------------------------------
  //TODO: figure out why not include aiq01 viq01
  val inst_aiq0_vld = Wire(Vec(4, Bool()))
  val inst_aiq1_vld = Wire(Vec(4, Bool()))
  val inst_biq_vld  = Wire(Vec(4, Bool()))
  val inst_lsiq_vld = Wire(Vec(4, Bool()))
  val inst_viq0_vld = Wire(Vec(4, Bool()))
  val inst_viq1_vld = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    inst_aiq0_vld(i) := ir_pipedown_inst_vld(i) && inst_aiq0(i)
    inst_aiq1_vld(i) := ir_pipedown_inst_vld(i) && inst_aiq1(i)
    inst_biq_vld(i)  := ir_pipedown_inst_vld(i) && ir_pipedown_inst_ctrl_info(i).BJU
    inst_lsiq_vld(i) := ir_pipedown_inst_vld(i) && ir_pipedown_inst_ctrl_info(i).LSU
    inst_viq0_vld(i) := ir_pipedown_inst_vld(i) && inst_viq0(i)
    inst_viq1_vld(i) := ir_pipedown_inst_vld(i) && inst_viq1(i)
  }

  pre_dis_type_stall_pipedown2 :=
    PopCount(inst_aiq0_vld) > 2.U ||
    PopCount(inst_aiq1_vld) > 2.U ||
    PopCount(inst_biq_vld)  > 2.U ||
    PopCount(inst_lsiq_vld) > 2.U ||
    PopCount(inst_viq0_vld) > 2.U ||
    PopCount(inst_viq1_vld) > 2.U

  pre_dis_pipedown2 := ir_pipedown_inst_vld(2) && pre_dis_type_stall_pipedown2
  io.out.pre_dispatch.pipedown2 := pre_dis_pipedown2
  //----------------------------------------------------------
  //           prepare dispatch IQ create signals
  //----------------------------------------------------------
  val pre_dis_inst_aiq0  = Wire(Vec(4, Bool()))
  val pre_dis_inst_aiq1  = Wire(Vec(4, Bool()))
  val pre_dis_inst_aiq01 = Wire(Vec(4, Bool()))
  val pre_dis_inst_biq   = Wire(Vec(4, Bool()))
  val pre_dis_inst_lsiq  = Wire(Vec(4, Bool()))
  val pre_dis_inst_sdiq  = Wire(Vec(4, Bool()))
  val pre_dis_inst_viq0  = Wire(Vec(4, Bool()))
  val pre_dis_inst_viq1  = Wire(Vec(4, Bool()))
  val pre_dis_inst_viq01 = Wire(Vec(4, Bool()))
  val pre_dis_inst_vmb   = Wire(Vec(4, Bool()))

  for(i <- 0 until 4){
    pre_dis_inst_aiq0(i)  := pre_dis_inst_vld(i) && inst_aiq0(i)
    pre_dis_inst_aiq1(i)  := pre_dis_inst_vld(i) && inst_aiq1(i)
    pre_dis_inst_aiq01(i) := pre_dis_inst_vld(i) && inst_aiq01(i)
    pre_dis_inst_biq(i)   := pre_dis_inst_vld(i) && ir_pipedown_inst_ctrl_info(i).BJU
    pre_dis_inst_lsiq(i)  := pre_dis_inst_vld(i) && ir_pipedown_inst_ctrl_info(i).LSU
    pre_dis_inst_sdiq(i)  := pre_dis_inst_vld(i) && ir_pipedown_inst_ctrl_info(i).STADDR
    pre_dis_inst_viq0(i)  := pre_dis_inst_vld(i) && inst_viq0(i)
    pre_dis_inst_viq1(i)  := pre_dis_inst_vld(i) && inst_viq1(i)
    pre_dis_inst_viq01(i) := pre_dis_inst_vld(i) && inst_viq01(i)
    pre_dis_inst_vmb(i)   := pre_dis_inst_vld(i) && ir_pipedown_inst_ctrl_info(i).VMB
  }

  //----------------------------------------------------------
  //           AIQ0 and AIQ1 create enable prepare
  //----------------------------------------------------------
  val pre_dis_al_1_aiq0_inst = pre_dis_inst_aiq0.asUInt.orR
  val pre_dis_al_2_aiq0_inst = PopCount(pre_dis_inst_aiq0) > 1.U
  val pre_dis_al_1_aiq1_inst = pre_dis_inst_aiq1.asUInt.orR
  val pre_dis_al_2_aiq1_inst = PopCount(pre_dis_inst_aiq1) > 1.U
  val pre_dis_al_1_aiq01_inst = pre_dis_inst_aiq01.asUInt.orR
  val pre_dis_al_2_aiq01_inst = PopCount(pre_dis_inst_aiq01) > 1.U
  val pre_dis_al_3_aiq01_inst = PopCount(pre_dis_inst_aiq01) > 2.U
  val pre_dis_al_4_aiq01_inst = pre_dis_inst_aiq01.asUInt.andR
  //----------------------------------------------------------
  //               AIQ0 and AIQ1 create enable
  //----------------------------------------------------------
  //aiq01 inst create use priority: aiq0_c0 > aiq1_c0 > aiq0_c1 > aiq1_c1
  val pre_dis_aiq0_create0_en = pre_dis_al_1_aiq0_inst || pre_dis_al_1_aiq01_inst
  val pre_dis_aiq0_create1_en =
    pre_dis_al_2_aiq0_inst ||
    pre_dis_al_1_aiq01_inst && (pre_dis_al_1_aiq0_inst && pre_dis_al_1_aiq1_inst) ||
    pre_dis_al_2_aiq01_inst && (pre_dis_al_1_aiq0_inst || pre_dis_al_1_aiq1_inst) ||
    pre_dis_al_3_aiq01_inst

  val pre_dis_aiq1_create0_en =
    pre_dis_al_1_aiq1_inst ||
    pre_dis_al_1_aiq0_inst && pre_dis_al_1_aiq01_inst ||
    pre_dis_al_2_aiq01_inst

  val pre_dis_aiq1_create1_en =
    pre_dis_al_2_aiq1_inst ||
    pre_dis_al_1_aiq01_inst && (pre_dis_al_2_aiq0_inst && pre_dis_al_1_aiq1_inst) ||
    pre_dis_al_2_aiq01_inst && (pre_dis_al_2_aiq0_inst || pre_dis_al_1_aiq0_inst && pre_dis_al_1_aiq1_inst) ||
    pre_dis_al_3_aiq01_inst && (pre_dis_al_1_aiq0_inst || pre_dis_al_1_aiq1_inst) ||
    pre_dis_al_4_aiq01_inst

  //----------------------------------------------------------
  //                  AIQ0 create 0 select
  //----------------------------------------------------------
  val pre_dis_aiq0_create0_sel_inst0 = pre_dis_inst_aiq0(0) || pre_dis_inst_aiq01(0) && !pre_dis_inst_aiq0.asUInt.orR
  val pre_dis_aiq0_create0_sel_inst1 = pre_dis_inst_aiq0(1) || pre_dis_inst_aiq01(1) && !(pre_dis_inst_aiq0(2) || pre_dis_inst_aiq0(3))
  val pre_dis_aiq0_create0_sel_inst2 = pre_dis_inst_aiq0(2) || pre_dis_inst_aiq01(2) && !pre_dis_inst_aiq0(3)

  val pre_dis_aiq0_create0_sel = Wire(UInt(2.W))
  when(pre_dis_aiq0_create0_sel_inst0){
    pre_dis_aiq0_create0_sel := 0.U
  }.elsewhen(pre_dis_aiq0_create0_sel_inst1){
    pre_dis_aiq0_create0_sel := 1.U
  }.elsewhen(pre_dis_aiq0_create0_sel_inst2){
    pre_dis_aiq0_create0_sel := 2.U
  }.otherwise{
    pre_dis_aiq0_create0_sel := 3.U
  }

  //----------------------------------------------------------
  //                  AIQ0 create 1 select
  //----------------------------------------------------------
  val pre_dis_inst123_aiq0 = pre_dis_inst_aiq0.asUInt.orR && !pre_dis_inst_aiq0(0)
  val pre_dis_inst123_aiq1 = pre_dis_inst_aiq1.asUInt.orR && !pre_dis_inst_aiq1(0)
  val pre_dis_inst023_aiq0 = pre_dis_inst_aiq0.asUInt.orR && !pre_dis_inst_aiq0(1)
  val pre_dis_inst023_aiq1 = pre_dis_inst_aiq1.asUInt.orR && !pre_dis_inst_aiq1(1)
  val pre_dis_inst013_aiq0 = pre_dis_inst_aiq0.asUInt.orR && !pre_dis_inst_aiq0(2)
  val pre_dis_inst013_aiq1 = pre_dis_inst_aiq1.asUInt.orR && !pre_dis_inst_aiq1(2)

  val pre_dis_inst123_2_aiq0 = PopCount(VecInit(Seq(pre_dis_inst_aiq0(1), pre_dis_inst_aiq0(2), pre_dis_inst_aiq0(3)))) > 1.U
  val pre_dis_inst023_2_aiq0 = PopCount(VecInit(Seq(pre_dis_inst_aiq0(0), pre_dis_inst_aiq0(2), pre_dis_inst_aiq0(3)))) > 1.U
  val pre_dis_inst013_2_aiq0 = PopCount(VecInit(Seq(pre_dis_inst_aiq0(0), pre_dis_inst_aiq0(1), pre_dis_inst_aiq0(3)))) > 1.U

  val pre_dis_aiq0_create1_sel_inst0 = pre_dis_inst_aiq01(0) &&
    pre_dis_inst123_aiq0 && pre_dis_inst123_aiq1 && !pre_dis_inst123_2_aiq0
  val pre_dis_aiq0_create1_sel_inst1 =
    pre_dis_inst_aiq0(1) && pre_dis_inst_aiq0(0) ||
    pre_dis_inst_aiq01(1) && !pre_dis_inst023_2_aiq0 &&
    (pre_dis_inst023_aiq0 && pre_dis_inst023_aiq1 ||
     pre_dis_inst023_aiq0 && pre_dis_inst_aiq01(0) ||
     pre_dis_inst023_aiq1 && pre_dis_inst_aiq01(0))
  val pre_dis_aiq0_create1_sel_inst2 =
    pre_dis_inst_aiq0(2) && (pre_dis_inst_aiq0(0) || pre_dis_inst_aiq0(1)) ||
    pre_dis_inst_aiq01(2) && !pre_dis_inst013_2_aiq0 &&
      (pre_dis_inst013_aiq0 && pre_dis_inst013_aiq1 ||
        (pre_dis_inst013_aiq0 || pre_dis_inst013_aiq1) && (pre_dis_inst_aiq01(0) || pre_dis_inst_aiq01(1)) ||
        (pre_dis_inst_aiq01(0) && pre_dis_inst_aiq01(1)))

  val pre_dis_aiq0_create1_sel = Wire(UInt(2.W))
  when(pre_dis_aiq0_create1_sel_inst0){
    pre_dis_aiq0_create1_sel := 0.U
  }.elsewhen(pre_dis_aiq0_create1_sel_inst1){
    pre_dis_aiq0_create1_sel := 1.U
  }.elsewhen(pre_dis_aiq0_create1_sel_inst2){
    pre_dis_aiq0_create1_sel := 2.U
  }.otherwise{
    pre_dis_aiq0_create1_sel := 3.U
  }

  //----------------------------------------------------------
  //                  AIQ1 create 0 select
  //----------------------------------------------------------
  val pre_dis_aiq1_create0_sel_inst0 = pre_dis_inst_aiq1(0) ||
    pre_dis_inst_aiq01(0) && pre_dis_inst123_aiq0 && !pre_dis_inst123_aiq1
  val pre_dis_aiq1_create0_sel_inst1 = pre_dis_inst_aiq1(1) ||
    pre_dis_inst_aiq01(1) && (pre_dis_inst023_aiq0 || pre_dis_inst_aiq01(0)) &&
      !(pre_dis_inst_aiq1(2) || pre_dis_inst_aiq1(3))
  val pre_dis_aiq1_create0_sel_inst2 = pre_dis_inst_aiq1(2) ||
    pre_dis_inst_aiq01(2) && (pre_dis_inst013_aiq0 || pre_dis_inst_aiq01(0) || pre_dis_inst_aiq01(1)) &&
      !pre_dis_inst_aiq1(3)

  val pre_dis_aiq1_create0_sel = Wire(UInt(2.W))
  when(pre_dis_aiq1_create0_sel_inst0){
    pre_dis_aiq1_create0_sel := 0.U
  }.elsewhen(pre_dis_aiq1_create0_sel_inst1){
    pre_dis_aiq1_create0_sel := 1.U
  }.elsewhen(pre_dis_aiq1_create0_sel_inst2){
    pre_dis_aiq1_create0_sel := 2.U
  }.otherwise{
    pre_dis_aiq1_create0_sel := 3.U
  }

  //----------------------------------------------------------
  //                  AIQ1 create 1 select
  //----------------------------------------------------------
  val pre_dis_inst23_aiq0 = pre_dis_inst_aiq0(2) || pre_dis_inst_aiq0(3)
  val pre_dis_inst23_aiq1 = pre_dis_inst_aiq1(2) || pre_dis_inst_aiq1(3)
  val pre_dis_inst_013_2_aiq1 = PopCount(VecInit(Seq(pre_dis_inst_aiq1(0), pre_dis_inst_aiq1(1), pre_dis_inst_aiq1(3)))) > 1.U

  val pre_dis_aiq1_create1_sel_inst0 = pre_dis_inst_aiq01(0) && pre_dis_inst123_2_aiq0 && pre_dis_inst123_aiq1
  val pre_dis_aiq1_create1_sel_inst1 = pre_dis_inst_aiq1(1) && pre_dis_inst_aiq1(0) ||
    pre_dis_inst_aiq01(1) && (pre_dis_inst023_2_aiq0 && pre_dis_inst023_aiq1 ||
      pre_dis_inst_aiq01(0) && pre_dis_inst23_aiq0 && pre_dis_inst23_aiq1 ||
      pre_dis_inst_aiq01(0) && pre_dis_inst_aiq0(2) && pre_dis_inst_aiq0(3))

  val pre_dis_aiq1_create1_sel_inst2 = pre_dis_inst_aiq1(2) && (pre_dis_inst_aiq1(0) || pre_dis_inst_aiq1(1)) ||
    pre_dis_inst_aiq01(2) && (pre_dis_inst_aiq0(3) || pre_dis_inst_aiq1(3)) &&
      (pre_dis_inst_aiq0(0) || pre_dis_inst_aiq1(0) || pre_dis_inst_aiq01(0)) &&
      (pre_dis_inst_aiq0(1) || pre_dis_inst_aiq1(1) || pre_dis_inst_aiq01(1)) &&
      !pre_dis_inst_013_2_aiq1

  val pre_dis_aiq1_create1_sel = Wire(UInt(2.W))
  when(pre_dis_aiq1_create1_sel_inst0){
    pre_dis_aiq1_create1_sel := 0.U
  }.elsewhen(pre_dis_aiq1_create1_sel_inst1){
    pre_dis_aiq1_create1_sel := 1.U
  }.elsewhen(pre_dis_aiq1_create1_sel_inst2){
    pre_dis_aiq1_create1_sel := 2.U
  }.otherwise{
    pre_dis_aiq1_create1_sel := 3.U
  }

  //out put
  io.out.pre_dispatch.iq_create_sel(0)(0).valid := pre_dis_aiq0_create0_en
  io.out.pre_dispatch.iq_create_sel(0)(1).valid := pre_dis_aiq0_create1_en
  io.out.pre_dispatch.iq_create_sel(1)(0).valid := pre_dis_aiq1_create0_en
  io.out.pre_dispatch.iq_create_sel(1)(1).valid := pre_dis_aiq1_create1_en

  io.out.pre_dispatch.iq_create_sel(0)(0).bits := pre_dis_aiq0_create0_sel
  io.out.pre_dispatch.iq_create_sel(0)(1).bits := pre_dis_aiq0_create1_sel
  io.out.pre_dispatch.iq_create_sel(1)(0).bits := pre_dis_aiq1_create0_sel
  io.out.pre_dispatch.iq_create_sel(1)(1).bits := pre_dis_aiq1_create1_sel

  //----------------------------------------------------------
  //               BIQ create enable and Select
  //----------------------------------------------------------
  val pre_dis_biq_create0_en = pre_dis_inst_biq.asUInt.orR
  val pre_dis_biq_create0_sel = Wire(UInt(2.W))
  when(pre_dis_inst_biq(0)){
    pre_dis_biq_create0_sel := 0.U
  }.elsewhen(pre_dis_inst_biq(1)){
    pre_dis_biq_create0_sel := 1.U
  }.elsewhen(pre_dis_inst_biq(2)){
    pre_dis_biq_create0_sel := 2.U
  }.otherwise{
    pre_dis_biq_create0_sel := 3.U
  }

  val pre_dis_biq_create1_en = PopCount(pre_dis_inst_biq) > 1.U
  val pre_dis_biq_create1_sel_inst1 = pre_dis_inst_biq(0) && pre_dis_inst_biq(1)
  val pre_dis_biq_create1_sel_inst2 = pre_dis_inst_biq(2) && (pre_dis_inst_biq(0) || pre_dis_inst_biq(1))

  val pre_dis_biq_create1_sel = Wire(UInt(2.W))
  when(pre_dis_biq_create1_sel_inst1) {
    pre_dis_biq_create1_sel := 1.U
  }.elsewhen(pre_dis_biq_create1_sel_inst2){
    pre_dis_biq_create1_sel := 2.U
  }.otherwise{
    pre_dis_biq_create1_sel := 3.U
  }

  io.out.pre_dispatch.iq_create_sel(2)(0).valid := pre_dis_biq_create0_en
  io.out.pre_dispatch.iq_create_sel(2)(1).valid := pre_dis_biq_create1_en
  io.out.pre_dispatch.iq_create_sel(2)(0).bits := pre_dis_biq_create0_sel
  io.out.pre_dispatch.iq_create_sel(2)(1).bits := pre_dis_biq_create1_sel

  //----------------------------------------------------------
  //              LSIQ create enable and Select
  //----------------------------------------------------------
  val vmb_create = Wire(Vec(2, Valid(UInt(2.W))))
  val pre_dis_lsiq_create0_en = pre_dis_inst_lsiq.asUInt.orR
  val pre_dis_lsiq_create0_sel = Wire(UInt(2.W))
  when(pre_dis_inst_lsiq(0)){
    pre_dis_lsiq_create0_sel := 0.U
    vmb_create(0).bits := 0.U
    vmb_create(0).valid := pre_dis_inst_vmb(0)
  }.elsewhen(pre_dis_inst_lsiq(1)){
    pre_dis_lsiq_create0_sel := 1.U
    vmb_create(0).bits := 1.U
    vmb_create(0).valid := pre_dis_inst_vmb(1)
  }.elsewhen(pre_dis_inst_lsiq(2)){
    pre_dis_lsiq_create0_sel := 2.U
    vmb_create(0).bits := 2.U
    vmb_create(0).valid := pre_dis_inst_vmb(2)
  }.otherwise{
    pre_dis_lsiq_create0_sel := 3.U
    vmb_create(0).bits := 3.U
    vmb_create(0).valid := pre_dis_inst_vmb(3) && pre_dis_inst_lsiq(3)
  }

  val pre_dis_lsiq_create1_en = PopCount(pre_dis_inst_lsiq) > 1.U
  val pre_dis_lsiq_create1_sel_inst1 = pre_dis_inst_lsiq(0) && pre_dis_inst_lsiq(1)
  val pre_dis_lsiq_create1_sel_inst2 = pre_dis_inst_lsiq(2) && (pre_dis_inst_lsiq(0) || pre_dis_inst_lsiq(1))

  val pre_dis_lsiq_create1_sel = Wire(UInt(2.W))
  when(pre_dis_lsiq_create1_sel_inst1) {
    pre_dis_lsiq_create1_sel := 1.U
    vmb_create(1).bits := 1.U
    vmb_create(1).valid := pre_dis_inst_vmb(1)
  }.elsewhen(pre_dis_lsiq_create1_sel_inst2){
    pre_dis_lsiq_create1_sel := 2.U
    vmb_create(1).bits := 2.U
    vmb_create(1).valid := pre_dis_inst_vmb(2)
  }.otherwise{
    pre_dis_lsiq_create1_sel := 3.U
    vmb_create(1).bits := 3.U
    vmb_create(1).valid := pre_dis_inst_vmb(3) && pre_dis_inst_lsiq(3) && (pre_dis_inst_lsiq(0) || pre_dis_inst_lsiq(1) || pre_dis_inst_lsiq(2))
  }

  io.out.pre_dispatch.iq_create_sel(3)(0).valid := pre_dis_lsiq_create0_en
  io.out.pre_dispatch.iq_create_sel(3)(1).valid := pre_dis_lsiq_create1_en
  io.out.pre_dispatch.iq_create_sel(3)(0).bits := pre_dis_lsiq_create0_sel
  io.out.pre_dispatch.iq_create_sel(3)(1).bits := pre_dis_lsiq_create1_sel
  io.out.pre_dispatch.iq_create_sel(7) := vmb_create

  //----------------------------------------------------------
  //              SDIQ create enable and Select
  //----------------------------------------------------------
  val pre_dis_sdiq_create0_en = pre_dis_inst_sdiq.asUInt.orR
  val pre_dis_sdiq_create0_sel = Wire(UInt(2.W))
  when(pre_dis_inst_sdiq(0)){
    pre_dis_sdiq_create0_sel := 0.U
  }.elsewhen(pre_dis_inst_sdiq(1)){
    pre_dis_sdiq_create0_sel := 1.U
  }.elsewhen(pre_dis_inst_sdiq(2)){
    pre_dis_sdiq_create0_sel := 2.U
  }.otherwise{
    pre_dis_sdiq_create0_sel := 3.U
  }

  val pre_dis_sdiq_create1_en = PopCount(pre_dis_inst_sdiq) > 1.U
  val pre_dis_sdiq_create1_sel_inst1 = pre_dis_inst_sdiq(0) && pre_dis_inst_sdiq(1)
  val pre_dis_sdiq_create1_sel_inst2 = pre_dis_inst_sdiq(2) && (pre_dis_inst_sdiq(0) || pre_dis_inst_sdiq(1))

  val pre_dis_sdiq_create1_sel = Wire(UInt(2.W))
  when(pre_dis_sdiq_create1_sel_inst1) {
    pre_dis_sdiq_create1_sel := 1.U
  }.elsewhen(pre_dis_sdiq_create1_sel_inst2){
    pre_dis_sdiq_create1_sel := 2.U
  }.otherwise{
    pre_dis_sdiq_create1_sel := 3.U
  }

  io.out.pre_dispatch.iq_create_sel(4)(0).valid := pre_dis_sdiq_create0_en
  io.out.pre_dispatch.iq_create_sel(4)(1).valid := pre_dis_sdiq_create1_en
  io.out.pre_dispatch.iq_create_sel(4)(0).bits := pre_dis_sdiq_create0_sel
  io.out.pre_dispatch.iq_create_sel(4)(1).bits := pre_dis_sdiq_create1_sel

  //----------------------------------------------------------
  //           VIQ0 and VIQ1 create enable prepare
  //----------------------------------------------------------
  val pre_dis_al_1_viq0_inst = pre_dis_inst_viq0.asUInt.orR
  val pre_dis_al_2_viq0_inst = PopCount(pre_dis_inst_viq0) > 1.U
  val pre_dis_al_1_viq1_inst = pre_dis_inst_viq1.asUInt.orR
  val pre_dis_al_2_viq1_inst = PopCount(pre_dis_inst_viq1) > 1.U
  val pre_dis_al_1_viq01_inst = pre_dis_inst_viq01.asUInt.orR
  val pre_dis_al_2_viq01_inst = PopCount(pre_dis_inst_viq01) > 1.U
  val pre_dis_al_3_viq01_inst = PopCount(pre_dis_inst_viq01) > 2.U
  val pre_dis_al_4_viq01_inst = pre_dis_inst_viq01.asUInt.andR
  //----------------------------------------------------------
  //               VIQ0 and VIQ1 create enable
  //----------------------------------------------------------
  //viq01 inst create use priority: viq0_c0 > viq1_c0 > viq0_c1 > viq1_c1
  val pre_dis_viq0_create0_en = pre_dis_al_1_viq0_inst || pre_dis_al_1_viq01_inst
  val pre_dis_viq0_create1_en =
    pre_dis_al_2_viq0_inst ||
      pre_dis_al_1_viq01_inst && (pre_dis_al_1_viq0_inst && pre_dis_al_1_viq1_inst) ||
      pre_dis_al_2_viq01_inst && (pre_dis_al_1_viq0_inst || pre_dis_al_1_viq1_inst) ||
      pre_dis_al_3_viq01_inst

  val pre_dis_viq1_create0_en =
    pre_dis_al_1_viq1_inst ||
      pre_dis_al_1_viq0_inst && pre_dis_al_1_viq01_inst ||
      pre_dis_al_2_viq01_inst

  val pre_dis_viq1_create1_en =
    pre_dis_al_2_viq1_inst ||
      pre_dis_al_1_viq01_inst && (pre_dis_al_2_viq0_inst && pre_dis_al_1_viq1_inst) ||
      pre_dis_al_2_viq01_inst && (pre_dis_al_2_viq0_inst || pre_dis_al_1_viq0_inst && pre_dis_al_1_viq1_inst) ||
      pre_dis_al_3_viq01_inst && (pre_dis_al_1_viq0_inst || pre_dis_al_1_viq1_inst) ||
      pre_dis_al_4_viq01_inst

  //----------------------------------------------------------
  //                  VIQ0 create 0 select
  //----------------------------------------------------------
  val pre_dis_viq0_create0_sel_inst0 = pre_dis_inst_viq0(0) || pre_dis_inst_viq01(0) && !pre_dis_inst_viq0.asUInt.orR
  val pre_dis_viq0_create0_sel_inst1 = pre_dis_inst_viq0(1) || pre_dis_inst_viq01(1) && !(pre_dis_inst_viq0(2) || pre_dis_inst_viq0(3))
  val pre_dis_viq0_create0_sel_inst2 = pre_dis_inst_viq0(2) || pre_dis_inst_viq01(2) && !pre_dis_inst_viq0(3)

  val pre_dis_viq0_create0_sel = Wire(UInt(2.W))
  when(pre_dis_viq0_create0_sel_inst0){
    pre_dis_viq0_create0_sel := 0.U
  }.elsewhen(pre_dis_viq0_create0_sel_inst1){
    pre_dis_viq0_create0_sel := 1.U
  }.elsewhen(pre_dis_viq0_create0_sel_inst2){
    pre_dis_viq0_create0_sel := 2.U
  }.otherwise{
    pre_dis_viq0_create0_sel := 3.U
  }

  //----------------------------------------------------------
  //                  VIQ0 create 1 select
  //----------------------------------------------------------
  val pre_dis_inst123_viq0 = pre_dis_inst_viq0.asUInt.orR && !pre_dis_inst_viq0(0)
  val pre_dis_inst123_viq1 = pre_dis_inst_viq1.asUInt.orR && !pre_dis_inst_viq1(0)
  val pre_dis_inst023_viq0 = pre_dis_inst_viq0.asUInt.orR && !pre_dis_inst_viq0(1)
  val pre_dis_inst023_viq1 = pre_dis_inst_viq1.asUInt.orR && !pre_dis_inst_viq1(1)
  val pre_dis_inst013_viq0 = pre_dis_inst_viq0.asUInt.orR && !pre_dis_inst_viq0(2)
  val pre_dis_inst013_viq1 = pre_dis_inst_viq1.asUInt.orR && !pre_dis_inst_viq1(2)

  val pre_dis_inst123_2_viq0 = PopCount(VecInit(Seq(pre_dis_inst_viq0(1), pre_dis_inst_viq0(2), pre_dis_inst_viq0(3)))) > 1.U
  val pre_dis_inst023_2_viq0 = PopCount(VecInit(Seq(pre_dis_inst_viq0(0), pre_dis_inst_viq0(2), pre_dis_inst_viq0(3)))) > 1.U
  val pre_dis_inst013_2_viq0 = PopCount(VecInit(Seq(pre_dis_inst_viq0(0), pre_dis_inst_viq0(1), pre_dis_inst_viq0(3)))) > 1.U

  val pre_dis_viq0_create1_sel_inst0 = pre_dis_inst_viq01(0) &&
    pre_dis_inst123_viq0 && pre_dis_inst123_viq1 && !pre_dis_inst123_2_viq0
  val pre_dis_viq0_create1_sel_inst1 =
    pre_dis_inst_viq0(1) && pre_dis_inst_viq0(0) ||
      pre_dis_inst_viq01(1) && !pre_dis_inst023_2_viq0 &&
        (pre_dis_inst023_viq0 && pre_dis_inst023_viq1 ||
          pre_dis_inst023_viq0 && pre_dis_inst_viq01(0) ||
          pre_dis_inst023_viq1 && pre_dis_inst_viq01(0))
  val pre_dis_viq0_create1_sel_inst2 =
    pre_dis_inst_viq0(2) && (pre_dis_inst_viq0(0) || pre_dis_inst_viq0(1)) ||
      pre_dis_inst_viq01(2) && !pre_dis_inst013_2_viq0 &&
        (pre_dis_inst013_viq0 && pre_dis_inst013_viq1 ||
          (pre_dis_inst013_viq0 || pre_dis_inst013_viq1) && (pre_dis_inst_viq01(0) || pre_dis_inst_viq01(1)) ||
          (pre_dis_inst_viq01(0) && pre_dis_inst_viq01(1)))

  val pre_dis_viq0_create1_sel = Wire(UInt(2.W))
  when(pre_dis_viq0_create1_sel_inst0){
    pre_dis_viq0_create1_sel := 0.U
  }.elsewhen(pre_dis_viq0_create1_sel_inst1){
    pre_dis_viq0_create1_sel := 1.U
  }.elsewhen(pre_dis_viq0_create1_sel_inst2){
    pre_dis_viq0_create1_sel := 2.U
  }.otherwise{
    pre_dis_viq0_create1_sel := 3.U
  }

  //----------------------------------------------------------
  //                  VIQ1 create 0 select
  //----------------------------------------------------------
  val pre_dis_viq1_create0_sel_inst0 = pre_dis_inst_viq1(0) ||
    pre_dis_inst_viq01(0) && pre_dis_inst123_viq0 && !pre_dis_inst123_viq1
  val pre_dis_viq1_create0_sel_inst1 = pre_dis_inst_aiq1(1) ||
    pre_dis_inst_viq01(1) && (pre_dis_inst023_viq0 || pre_dis_inst_viq01(0)) &&
      !(pre_dis_inst_viq1(2) || pre_dis_inst_viq1(3))
  val pre_dis_viq1_create0_sel_inst2 = pre_dis_inst_viq1(2) ||
    pre_dis_inst_viq01(2) && (pre_dis_inst013_viq0 || pre_dis_inst_viq01(0) || pre_dis_inst_viq01(1)) &&
      !pre_dis_inst_viq1(3)

  val pre_dis_viq1_create0_sel = Wire(UInt(2.W))
  when(pre_dis_viq1_create0_sel_inst0){
    pre_dis_viq1_create0_sel := 0.U
  }.elsewhen(pre_dis_viq1_create0_sel_inst1){
    pre_dis_viq1_create0_sel := 1.U
  }.elsewhen(pre_dis_viq1_create0_sel_inst2){
    pre_dis_viq1_create0_sel := 2.U
  }.otherwise{
    pre_dis_viq1_create0_sel := 3.U
  }

  //----------------------------------------------------------
  //                  VIQ1 create 1 select
  //----------------------------------------------------------
  val pre_dis_inst23_viq0 = pre_dis_inst_viq0(2) || pre_dis_inst_viq0(3)
  val pre_dis_inst23_viq1 = pre_dis_inst_viq1(2) || pre_dis_inst_viq1(3)
  val pre_dis_inst_013_2_viq1 = PopCount(VecInit(Seq(pre_dis_inst_viq1(0), pre_dis_inst_viq1(1), pre_dis_inst_viq1(3)))) > 1.U

  val pre_dis_viq1_create1_sel_inst0 = pre_dis_inst_viq01(0) && pre_dis_inst123_2_viq0 && pre_dis_inst123_viq1
  val pre_dis_viq1_create1_sel_inst1 = pre_dis_inst_viq1(1) && pre_dis_inst_viq1(0) ||
    pre_dis_inst_viq01(1) && (pre_dis_inst023_2_viq0 && pre_dis_inst023_viq1 ||
      pre_dis_inst_viq01(0) && pre_dis_inst23_viq0 && pre_dis_inst23_viq1 ||
      pre_dis_inst_viq01(0) && pre_dis_inst_viq0(2) && pre_dis_inst_viq0(3))

  val pre_dis_viq1_create1_sel_inst2 = pre_dis_inst_viq1(2) && (pre_dis_inst_viq1(0) || pre_dis_inst_viq1(1)) ||
    pre_dis_inst_viq01(2) && (pre_dis_inst_viq0(3) || pre_dis_inst_viq1(3)) &&
      (pre_dis_inst_viq0(0) || pre_dis_inst_viq1(0) || pre_dis_inst_viq01(0)) &&
      (pre_dis_inst_viq0(1) || pre_dis_inst_viq1(1) || pre_dis_inst_viq01(1)) &&
      !pre_dis_inst_013_2_viq1

  val pre_dis_viq1_create1_sel = Wire(UInt(2.W))
  when(pre_dis_viq1_create1_sel_inst0){
    pre_dis_viq1_create1_sel := 0.U
  }.elsewhen(pre_dis_viq1_create1_sel_inst1){
    pre_dis_viq1_create1_sel := 1.U
  }.elsewhen(pre_dis_viq1_create1_sel_inst2){
    pre_dis_viq1_create1_sel := 2.U
  }.otherwise{
    pre_dis_viq1_create1_sel := 3.U
  }

  io.out.pre_dispatch.iq_create_sel(5)(0).valid := pre_dis_viq0_create0_en
  io.out.pre_dispatch.iq_create_sel(5)(1).valid := pre_dis_viq0_create1_en
  io.out.pre_dispatch.iq_create_sel(6)(0).valid := pre_dis_viq1_create0_en
  io.out.pre_dispatch.iq_create_sel(6)(1).valid := pre_dis_viq1_create1_en

  io.out.pre_dispatch.iq_create_sel(5)(0).bits := pre_dis_viq0_create0_sel
  io.out.pre_dispatch.iq_create_sel(5)(1).bits := pre_dis_viq0_create1_sel
  io.out.pre_dispatch.iq_create_sel(6)(0).bits := pre_dis_viq1_create0_sel
  io.out.pre_dispatch.iq_create_sel(6)(1).bits := pre_dis_viq1_create1_sel

  //----------------------------------------------------------
  //         prepare dispatch ROB/PST create signals
  //----------------------------------------------------------
  //inst can be fold if:
  //1.inst is aiq01 or aiq1 or aiq0 (expt inst will be fence of type aiq0)
  //  or inst is vdsp
  //  or inst is vfpu
  //2.inst is not split
  //3.inst is not intmask
  val ir_inst_fold = Wire(Vec(4, Bool()))
  val pre_dis_inst_fold = Wire(Vec(4, Bool()))
  for(i <- 0 until 4){
    ir_inst_fold(i) := (inst_aiq0(i) || inst_aiq1(i) || inst_aiq01(i)) && !inst_special(i) || (inst_viq0(i) || inst_viq1(i) || inst_viq01(i))
    pre_dis_inst_fold(i) := pre_dis_inst_vld(i) && ir_inst_fold(i) &&
      !inst_split(i) && !inst_intmask(i) && !io.in.fromRTU.srt_en && !io.in.fromCp0.robFoldDisable && false.B // Todo: enable
  }

  val pre_dis_inst01_fold  = pre_dis_inst_fold(0) && pre_dis_inst_fold(1) && !pre_dis_inst_fold(2)
  val pre_dis_inst12_fold  = !pre_dis_inst_fold(0) && pre_dis_inst_fold(1) && pre_dis_inst_fold(2) && !pre_dis_inst_fold(3)
  val pre_dis_inst23_fold  = !pre_dis_inst_fold(1) && pre_dis_inst_fold(2) && pre_dis_inst_fold(3)
  val pre_dis_inst012_fold = pre_dis_inst_fold(0) && pre_dis_inst_fold(1) && pre_dis_inst_fold(2)
  val pre_dis_inst123_fold = !pre_dis_inst_fold(0) && pre_dis_inst_fold(1) && pre_dis_inst_fold(2) && pre_dis_inst_fold(3)

  //rob create0 en
  //create0 en always from dis inst0 vld
  //rob create1 en
  when(pre_dis_inst012_fold){
    io.out.pre_dispatch.rob_create.en1 := pre_dis_inst_vld(3)
  }.elsewhen(pre_dis_inst01_fold){
    io.out.pre_dispatch.rob_create.en1 := pre_dis_inst_vld(2)
  }.otherwise{
    io.out.pre_dispatch.rob_create.en1 := pre_dis_inst_vld(1)
  }

  //rob create2 en
  io.out.pre_dispatch.rob_create.en2 := !pre_dis_inst012_fold && !pre_dis_inst123_fold &&
    Mux(pre_dis_inst01_fold || pre_dis_inst12_fold, pre_dis_inst_vld(3), pre_dis_inst_vld(2))

  //rob create3 en
  io.out.pre_dispatch.rob_create.en3 := pre_dis_inst_vld(3) &&
    !pre_dis_inst012_fold && !pre_dis_inst123_fold &&
    !pre_dis_inst01_fold && !pre_dis_inst12_fold && !pre_dis_inst23_fold

  //rob create0 select:
  when(pre_dis_inst012_fold){
    io.out.pre_dispatch.rob_create.sel0 := 2.U//select inst0, inst1 and inst2
  }.elsewhen(pre_dis_inst01_fold){
    io.out.pre_dispatch.rob_create.sel0 := 1.U//select inst0 and inst1
  }.otherwise{
    io.out.pre_dispatch.rob_create.sel0 := 0.U//else 2'd0: select inst0
  }

  //rob create1 select
  when(pre_dis_inst123_fold){
    io.out.pre_dispatch.rob_create.sel1 := 4.U//select inst1, inst2 and inst3
  }.elsewhen(pre_dis_inst012_fold){
    io.out.pre_dispatch.rob_create.sel1 := 3.U//select inst3
  }.elsewhen(pre_dis_inst01_fold){
    io.out.pre_dispatch.rob_create.sel1 := 2.U//select inst2
  }.elsewhen(pre_dis_inst12_fold){
    io.out.pre_dispatch.rob_create.sel1 := 1.U//select inst1 and inst2
  }.otherwise{
    io.out.pre_dispatch.rob_create.sel1 := 0.U//else 2'd0: select inst1
  }

  //rob create2 select
  when(pre_dis_inst01_fold){
    io.out.pre_dispatch.rob_create.sel2 := 3.U//select inst3
  }.elsewhen(pre_dis_inst12_fold){
    io.out.pre_dispatch.rob_create.sel2 := 3.U//select inst3
  }.elsewhen(pre_dis_inst23_fold) {
    io.out.pre_dispatch.rob_create.sel2 := 2.U//select inst2 and inst3
  }.otherwise{
    io.out.pre_dispatch.rob_create.sel2 := 0.U//else 2'd0: select inst2
  }

  //rob create3 select
  //always select inst3

  //pst create inst0 iid
  //always select rtu inst0 iid

  //pst create inst1 iid
  //select rtu inst0 iid if fold with inst0
  io.out.pre_dispatch.pst_create_iid_sel(0) := pre_dis_inst01_fold || pre_dis_inst012_fold

  //pst create inst2 iid
  val pst_create2_iid_sel = Wire(Vec(3, Bool()))
  //select inst0
  pst_create2_iid_sel(0) := pre_dis_inst012_fold
  //select inst1
  pst_create2_iid_sel(1) := pre_dis_inst01_fold || pre_dis_inst12_fold || pre_dis_inst123_fold
  //select inst2
  pst_create2_iid_sel(2) := !(pre_dis_inst012_fold || pre_dis_inst01_fold || pre_dis_inst12_fold || pre_dis_inst123_fold)

  io.out.pre_dispatch.pst_create_iid_sel(1) := pst_create2_iid_sel.asUInt

  //pst create inst3 iid
  val pst_create3_iid_sel = Wire(Vec(3, Bool()))
  //select inst1
  pst_create3_iid_sel(0) := pre_dis_inst012_fold || pre_dis_inst123_fold
  //select inst2
  pst_create3_iid_sel(1) := pre_dis_inst01_fold || pre_dis_inst12_fold || pre_dis_inst23_fold
  //select inst3
  pst_create3_iid_sel(2) := !(pre_dis_inst012_fold || pre_dis_inst123_fold ||
    pre_dis_inst01_fold || pre_dis_inst12_fold || pre_dis_inst23_fold)

  io.out.pre_dispatch.pst_create_iid_sel(2) := pst_create3_iid_sel.asUInt

  //==========================================================
  //                   Performance Monitor
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  val hpcp_clk_en = io.in.fromHpcp.cntEn && instVld.asUInt.orR && hpcp_inst_vld_ff.asUInt.orR

  //----------------------------------------------------------
  //                    RF inst valid
  //----------------------------------------------------------
  val hpcp_inst_vld = ir_stall
  //----------------------------------------------------------
  //              RF stage performance monitor
  //----------------------------------------------------------
  when(io.in.fromHpcp.cntEn && hpcp_inst_vld){
    hpcp_inst_vld_ff := instVld
    hpcp_inst_type := hpcp_type
  }.otherwise{
    hpcp_inst_vld_ff := WireInit(VecInit(Seq.fill(4)(false.B)))
  }

  io.out.toHpcp.inst_type := hpcp_inst_type
  io.out.toHpcp.inst_vld  := hpcp_inst_vld_ff
}
