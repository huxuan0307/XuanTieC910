package Core.IDU.IS

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._
import Core.IntConfig._
import Core.ExceptionConfig._

class AiqEntryData extends Bundle with AiqConfig {
  val vl : UInt = UInt(VlmaxBits.W)
  val launchPreg : Bool = Bool()
  val special : Bool = Bool()
  val vsew : UInt = UInt(VsewBits.W)
  val vlmul : UInt = UInt(VlmaxBits.W)
  // Todo: Add other iq bypass
  val aluShort : Bool = Bool()
  // Todo: imm
  val pid : UInt = UInt(5.W)
  val pcFifo : Bool = Bool()
  val mtvr : Bool = Bool()
  val div : Bool = Bool()
  val highHwExcept : Bool = Bool()
  val exceptVec = ValidIO(UInt(ExceptionVecWidth.W))
  val srcVec : Vec[DepRegEntryData] = Vec(NumSrcArith, new DepRegEntryData)
  // Todo: imm
  val dstVreg : UInt = UInt(7.W)
  val dstPreg : UInt = UInt(NumPhysicRegsBits.W)
  val dstVValid : Bool = Bool()
  val dstValid : Bool = Bool()
  val srcValid : Vec[Bool] = Vec(NumSrcArith, Bool())
  val iid : UInt = UInt(InstructionIdWidth.W)
  // Replace opcode with inst
  val inst : UInt = UInt(InstBits.W)
}

trait EntryHasDiv {
  val divBusy : Bool = Bool()
}

class AiqEntryInput extends Bundle
  with AiqConfig
  with DepRegEntryConfig
  with EntryHasDiv {
  val fromCp0 = new IqEntryFromCp0
  val fwdValid : Vec[FwdValidBundle] = Vec(NumSrcArith, new FwdValidBundle)
  /**
   * Include alu0, alu1, mul, div, load, vfpu0, vfpu1 <br>
   * alu0 : ctrl_xx_rf_pipe0_preg_lch_vld_dupx <br>
   *        dp_xx_rf_pipe0_dst_preg_dupx <br>
   * alu1 : ctrl_xx_rf_pipe1_preg_lch_vld_dupx <br>
   *        dp_xx_rf_pipe1_dst_preg_dupx <br>
   * mult : iu_idu_ex2_pipe1_mult_inst_vld_dupx <br>
   *        iu_idu_ex2_pipe1_preg_dupx <br>
   * div  : iu_idu_div_inst_vld <br>
   *        iu_idu_div_preg_dupx <br>
   * load : lsu_idu_dc_pipe3_load_inst_vld_dupx <br>
   *        lsu_idu_dc_pipe3_preg_dupx <br>
   * vfpu0: vfpu_idu_ex1_pipe6_mfvr_inst_vld_dupx <br>
   *        vfpu_idu_ex1_pipe6_preg_dupx <br>
   * vfpu1: vfpu_idu_ex1_pipe7_mfvr_inst_vld_dupx <br>
   *        vfpu_idu_ex1_pipe7_preg_dupx <br>
   */
  val fuDstPreg : Vec[ValidIO[UInt]] = Vec(NumFuHasDstReg, ValidIO(UInt(NumPhysicRegsBits.W)))

  /**
   * Include pipe0,1,3 wb <br>
   * pipe0wb : iu_idu_ex2_pipe0_wb_preg_vld_dupx
   * pipe1wb : iu_idu_ex2_pipe1_wb_preg_vld_dupx
   * pipe3wb : lsu_idu_wb_pipe3_wb_preg_vld_dupx
   */
  val wbPreg : Vec[ValidIO[UInt]] = Vec(WbNum, ValidIO(UInt(NumPhysicRegsBits.W)))

  /**
   * LSU reg Bypass
   * lsu_idu_ag_pipe3_load_inst_vld
   * lsu_idu_ag_pipe3_preg_dupx
   */
  val loadPreg = ValidIO(UInt(NumPhysicRegsBits.W))

  val rfPopValid : Bool = Bool()
  val rfReadyClr : Vec[Bool] = Vec(NumSrcArith, Bool())
  val stall : Bool = Bool()
  val fromRtu = new EntryFromRtu
  val create = new Bundle {
    val ageVec : Vec[Bool] = Vec(NumAiqEntry - 1, Bool())
    val data = new AiqEntryData
    val dpEn : Bool = Bool()
    val en : Bool = Bool()
    val freeze : Bool = Bool()
    val gateClkEn : Bool = Bool()
  }
  val freezeClr : Bool = Bool()
  val issueEn : Bool = Bool()
  val popCurEntry : Bool = Bool()
  val popOtherEntry : Vec[Bool] = Vec(this.NumAiqEntry - 1, Bool())
}

class AiqEntryOutput extends Bundle with AiqConfig {
  val ageVec : Vec[Bool] = Vec(this.NumAiqEntry - 1, Bool())
  val ready : Bool = Bool()
  val readData = new AiqEntryData
  val valid : Bool = Bool()
  val validWithoutFreeze : Bool = Bool()
}

class AiqEntryIO extends Bundle {
  val in : AiqEntryInput = Flipped(Output(new AiqEntryInput))
  val out : AiqEntryOutput = Output(new AiqEntryOutput)
}

class AiqEntry extends Module with AiqConfig {
  val io : AiqEntryIO = IO(new AiqEntryIO)

  private val create = io.in.create
  private val rtu = io.in.fromRtu

  private val srcEntry = Seq.fill(NumSrcArith)(Module(new DepRegEntry))

  // Todo: check if need extra dataInit
  private val data = RegInit(0.U.asTypeOf(Output(new AiqEntryData)))
  private val valid = RegInit(false.B)
  private val ageVec = RegInit(VecInit(Seq.fill(this.NumAiqEntry - 1)(false.B)))
  private val freeze = RegInit(false.B)

  // Todo: gated clk

  when(rtu.flush.fe || rtu.flush.is) {
    valid := false.B
  }.elsewhen(io.in.create.en) {
    valid := true.B
  }.elsewhen(io.in.rfPopValid && io.in.popCurEntry) {
    valid := false.B
  }
  when(create.en) {
    freeze := create.freeze
  }.elsewhen(io.in.issueEn) {
    freeze := true.B
  }.elsewhen(io.in.freezeClr) {
    freeze := false.B
  }
  when(create.en) {
    ageVec := create.ageVec
  }.elsewhen(io.in.rfPopValid) {
    ageVec := VecInit((ageVec.asUInt & ~io.in.popOtherEntry.asUInt).asBools)
  }

  when(create.dpEn) {
    data.dstPreg        := create.data.dstPreg
    data.dstVreg        := create.data.dstVreg
    data.pid            := create.data.pid
    data.exceptVec      := create.data.exceptVec
    data.highHwExcept   := create.data.highHwExcept
    data.inst         := create.data.inst
    data.iid            := create.data.iid
    data.srcValid       := create.data.srcValid
    data.dstValid       := create.data.dstValid
    data.dstVValid      := create.data.dstVValid
    data.div            := create.data.div
    data.mtvr           := create.data.mtvr
    data.pcFifo         := create.data.pcFifo
    data.aluShort       := create.data.aluShort
    data.vlmul          := create.data.vlmul
    data.vsew           := create.data.vsew
    data.vl             := create.data.vl
    data.special        := create.data.special
    data.launchPreg        := create.data.launchPreg
  }

  io.out.valid := valid
  io.out.validWithoutFreeze := valid && !freeze
  io.out.ageVec := ageVec
  io.out.readData.dstPreg       := data.dstPreg
  io.out.readData.dstVreg       := data.dstVreg
  io.out.readData.pid           := data.pid
  io.out.readData.exceptVec     := data.exceptVec
  io.out.readData.highHwExcept  := data.highHwExcept
  io.out.readData.inst        := data.inst
  io.out.readData.iid           := data.iid
  io.out.readData.srcValid      := data.srcValid
  io.out.readData.dstValid      := data.dstValid
  io.out.readData.dstVValid     := data.dstVValid
  io.out.readData.div           := data.div
  io.out.readData.mtvr          := data.mtvr
  io.out.readData.pcFifo        := data.pcFifo
  io.out.readData.aluShort      := data.aluShort
  io.out.readData.vlmul         := data.vlmul
  io.out.readData.vsew          := data.vsew
  io.out.readData.vl            := data.vl
  io.out.readData.special       := data.special
  io.out.readData.launchPreg       := data.launchPreg

  //==========================================================
  //              Source Dependency Information
  //==========================================================

  private val createSrcDataVec = Wire(Vec(NumSrcArith, new DepRegEntryCreateData))
  createSrcDataVec.zipWithIndex.foreach{
    case (data, i) =>
      data := io.in.create.data.srcVec(i)
  }
  private val createSrcGateClkVec = Wire(Vec(NumSrcArith, Bool()))
  createSrcGateClkVec.zipWithIndex.foreach{
    case (bool, i) => bool := io.in.create.gateClkEn && io.in.create.data.srcValid(i)
  }
  private val srcReadyClearVec = Wire(Vec(NumSrcArith, Bool()))
  (0 until NumSrcArith).foreach{
    case idx =>
      srcReadyClearVec(idx) := io.in.freezeClr && io.in.rfReadyClr(idx)
  }
  private val readSrcDataVec = Wire(Vec(NumSrcArith, new DepRegEntryReadData))

  srcEntry.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fwdValid := io.in.fwdValid(i)
      in.fromCp0  := io.in.fromCp0
      in.fuDstPreg:= io.in.fuDstPreg
      in.wbPreg   := io.in.wbPreg
      in.loadPreg := io.in.loadPreg
      in.flush    := io.in.fromRtu.flush
      in.createData := createSrcDataVec(i)
      in.gateClkIdxWen := createSrcGateClkVec(i)
      in.gateClkWen := create.gateClkEn
      in.readyClear := srcReadyClearVec(i)
      in.wen        := create.dpEn
      readSrcDataVec(i) := out.readData
  }

  io.out.readData.srcVec.zipWithIndex.foreach{
    case (data, i) =>
      data.wb := readSrcDataVec(i).wb
      data.preg := readSrcDataVec(i).preg
      data.ready := false.B
      data.lsuMatch := false.B
  }

  //==========================================================
  //                  Entry Ready Signal
  //==========================================================
  io.out.ready := valid &&
    !freeze &&
    !io.in.stall &&
    !(io.out.readData.div && io.in.divBusy) &&
    readSrcDataVec.map(_.readyForIssue).reduce(_&&_)

  // Todo: add fwd and bypass data
}


