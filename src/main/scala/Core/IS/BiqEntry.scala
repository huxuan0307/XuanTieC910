package Core.IS

import Core.ExceptionConfig._
import Core.IntConfig._
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

trait BiqConfig {
  def NumSrcBr = 2
  def NumBiqEntry = 12
}

object BiqConfig extends BiqConfig

class EntryFromRtu extends Bundle {
  val flush = new Bundle {
    val fe : Bool = Bool()
    val is : Bool = Bool()
  }
}

class BiqEntryData extends Bundle with BiqConfig {
  val vl        : UInt = UInt(VlmaxBits.W)
  val vsew      : UInt = UInt(VsewBits.W)
  val vlmul     : UInt = UInt(VlmaxBits.W)
  val pcall     : Bool = Bool()
  val rts       : Bool = Bool()
  // Todo: imm
  val pid       : UInt = UInt(5.W)
  val length    : Bool = Bool()
  val srcVec    : Vec[DepRegEntryData] = Vec(NumSrcBr, new DepRegEntryData)
  val srcValid  : Vec[Bool] = Vec(NumSrcBr, Bool())
  val iid       : UInt = UInt(InstructionIdWidth.W)
  val opcode    : UInt = UInt(OpcodeBits.W)
}

class BiqEntryInput extends Bundle
  with BiqConfig
  with DepRegEntryConfig {
  val fromCp0 = new IqEntryFromCp0
  val fwdValid : Vec[FwdValidBundle] = Vec(NumSrcBr, new FwdValidBundle)
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
  val rfReadyClr : Vec[Bool] = Vec(NumSrcBr, Bool())
  val fromRtu = new EntryFromRtu
  val create = new Bundle {
    val ageVec    : Vec[Bool] = Vec(NumBiqEntry - 1, Bool())
    val data      : BiqEntryData = new BiqEntryData
    val dpEn      : Bool = Bool()
    val en        : Bool = Bool()
    val freeze    : Bool = Bool()
    val gateClkEn : Bool = Bool()
  }
  val freezeClr     : Bool = Bool()
  val issueEn       : Bool = Bool()
  val popCurEntry   : Bool = Bool()
  val popOtherEntry : Vec[Bool] = Vec(this.NumBiqEntry - 1, Bool())
}

class BiqEntryOutput extends Bundle with BiqConfig {
  val ageVec    : Vec[Bool] = Vec(this.NumBiqEntry - 1, Bool())
  val ready     : Bool = Bool()
  val readData  : BiqEntryData = new BiqEntryData
  val valid     : Bool = Bool()
  val validWithoutFreeze : Bool = Bool()
}

class BiqEntryIO extends Bundle with BiqConfig {
  val in  : BiqEntryInput = Flipped(Output(new BiqEntryInput))
  val out : BiqEntryOutput = Output(new BiqEntryOutput)
}

class BiqEntry extends Module with BiqConfig {
  val io : BiqEntryIO = IO(new BiqEntryIO)

  private val create = io.in.create
  private val rtu = io.in.fromRtu
  private val srcEntry : Seq[DepRegEntry] = Seq.fill(NumSrcBr)(Module(new DepRegEntry))

  // Todo: check if need extra dataInit
  private val data = RegInit(0.U.asTypeOf(Output(new BiqEntryData)))
  private val valid = RegInit(false.B)
  private val ageVec = RegInit(VecInit(Seq.fill(this.NumBiqEntry - 1)(false.B)))
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

  //==========================================================
  //                 Instruction Information
  //==========================================================
  when(create.dpEn) {
    data.opcode     := create.data.opcode
    data.iid        := create.data.iid
    data.srcValid   := create.data.srcValid
    data.length     := create.data.length
    data.rts        := create.data.rts
    data.pcall      := create.data.pcall
    data.pid        := create.data.pid
    data.vlmul      := create.data.vlmul
    data.vsew       := create.data.vsew
    data.vl         := create.data.vl
  }

  io.out.valid                := valid
  io.out.validWithoutFreeze   := valid && !freeze
  io.out.ageVec               := ageVec
  io.out.readData.opcode      := data.opcode
  io.out.readData.iid         := data.iid
  io.out.readData.srcValid    := data.srcValid
  io.out.readData.length      := data.length
  io.out.readData.rts         := data.rts
  io.out.readData.pcall       := data.pcall
  io.out.readData.pid         := data.pid
  io.out.readData.vlmul       := data.vlmul
  io.out.readData.vsew        := data.vsew
  io.out.readData.vl          := data.vl

  //==========================================================
  //              Source Dependency Information
  //==========================================================

  private val createSrcDataVec = Wire(Vec(NumSrcBr, new DepRegEntryCreateData))
  createSrcDataVec.zipWithIndex.foreach{
    case (data, i) =>
      data := io.in.create.data.srcVec(i)
  }
  private val createSrcGateClkVec = Wire(Vec(NumSrcBr, Bool()))
  createSrcGateClkVec.zipWithIndex.foreach{
    case (bool, i) => bool := io.in.create.gateClkEn && io.in.create.data.srcValid(i)
  }
  private val srcReadyClearVec = Wire(Vec(NumSrcBr, Bool()))
  (0 until NumSrcBr).foreach{
    case idx =>
      srcReadyClearVec(idx) := io.in.freezeClr && io.in.rfReadyClr(idx)
  }
  private val readSrcDataVec = Wire(Vec(NumSrcBr, new DepRegEntryReadData))

  srcEntry.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fwdValid   := io.in.fwdValid(i)
      in.fromCp0    := io.in.fromCp0
      in.fuDstPreg  := io.in.fuDstPreg
      in.wbPreg     := io.in.wbPreg
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
      data.wb       := readSrcDataVec(i).wb
      data.preg     := readSrcDataVec(i).preg
      data.ready    := false.B
      data.lsuMatch := false.B
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
    readSrcDataVec.map(_.readyForIssue).reduce(_&&_)

  // Todo: add fwd and bypass data

}
