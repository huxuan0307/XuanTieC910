package Core.IS

import Core.ExceptionConfig._
import Core.IntConfig._
import Core.RTU.RobBreakpointDataBundle
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

trait LsiqConfig {
  def NumSrcLs = 3 // src0/1,src_vm
  def NumSrcLsX = 2
  def NumSrcLsV = 1
  def NumLsiqEntry = 12
  def NumLsiqCreatePort = 2
}

object LsiqConfig extends LsiqConfig

class BarType extends Bundle {
  val aftStore = Bool()
  val aftLoad = Bool()
  val befStore = Bool()
  val befLoad = Bool()
}

class OtherBundle extends Bundle with LsiqConfig with SdiqConfig {
  val aftLoadVec : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
  val aftStoreVec : Vec[Bool] = Vec(NumSdiqEntry - 1, Bool())
  val barVec : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
  val freeze : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
  val load : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
  val noSpec : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
  val rawReady : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
  val store : Vec[Bool] = Vec(NumSdiqEntry - 1, Bool())
}

class BreakpointDataBundle extends RobBreakpointDataBundle
class BreakpointDataSetBundle extends BreakpointDataBundle

class LsiqEntryData extends Bundle with LsiqConfig with SdiqConfig{
  val vmb : Bool = Bool()
  // Todo: imm
  val splitNum : UInt = UInt(7.W)
  val vl        : UInt = UInt(VlmaxBits.W)
  val vsew      : UInt = UInt(VsewBits.W)
  val vlmul     : UInt = UInt(VlmaxBits.W)
  val noSpecExist : Bool = Bool()
  val noSpec : Bool = Bool()
  val split : Bool = Bool()
  // Todo: imm
  val pc : UInt = UInt(15.W)
  val stAddr : Bool = Bool()
  // barrier for Freeze Clear Signals
  val bar : Bool = Bool()
  val store : Bool = Bool()
  val load : Bool = Bool()
  val dstVValid : Bool = Bool()
  val dstValid : Bool = Bool()

  val srcValid  : Vec[Bool] = Vec(NumSrcLsX, Bool())
  val srcVmValid : Bool = Bool()
  val iid       : UInt = UInt(InstructionIdWidth.W)
  val opcode    : UInt = UInt(OpcodeBits.W)

  // Todo: create DepVregEntry
  val srcVm = new DepRegEntryData
  val srcVec : Vec[DepRegEntryData] = Vec(this.NumSrcLsX, new DepRegEntryData)
  val dstPreg : UInt = UInt(NumPhysicRegsBits.W)
  val dstVreg : UInt = UInt(VregNumBits.W)

  val breakpointData = new BreakpointDataBundle
  val ageVecAll : Vec[Bool] = Vec(this.NumLsiqEntry - 1, Bool())
  /**
   * already data accessed
   */
  val alreadyDa : Bool = Bool()
  // Todo: figure out
  val unalign2nd : Bool = Bool()
  // Todo: figure out
  val specFail : Bool = Bool()

  val sdEntry : Vec[Bool] = Vec(NumSdiqEntry, Bool())
  // Todo: figure out
  val barType = new BarType
}

class LsiqFromPad extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

class IqEntryFwdBundle(NumSrc: Int) extends Bundle {
  val alu = Vec(NumSrc, Vec(NumAlu, Bool()))
  val load = Vec(NumLu, Bool())
}

class LsiqEntryInput extends Bundle with LsiqConfig with DepRegEntryConfig {
  val fromCp0 = new IqEntryFromCp0
  /**
   * Include load fwd, alu0 reg fwd, alu1 reg fwd
   * alu0: x_alu0_reg_fwd_vld
   * alu1: x_alu1_reg_fwd_vld
   * load: lsu_idu_dc_pipe3_load_fwd_inst_vld_dupx
   */
  val fwdValid = new IqEntryFwdBundle(this.NumSrcLsX)
  // Todo: vload fwd
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

  val popValid : Bool = Bool()
  val readyClr : Vec[Bool] = Vec(NumSrcLs, Bool())
  val fromRtu = new EntryFromRtu
  val create = new Bundle {
    val ageVec : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
    // Todo: figure out
    val ageVecAll : Vec[Bool] = Vec(NumLsiqEntry - 1, Bool())
    val data = new LsiqEntryData
    val dpEn : Bool = Bool()
    val en : Bool = Bool()
    val freeze : Bool = Bool()
    val gateClkEn : Bool = Bool()
  }
  val freezeClr : Bool = Bool()
  val issueEn : Bool = Bool()
  val popCurEntry : Bool = Bool()
  val popOtherEntry : Vec[Bool] = Vec(this.NumLsiqEntry - 1, Bool())

  // clk
  val lqFullClk : Clock = Clock()
  val rbFullClk : Clock = Clock()
  val sqFullClk : Clock = Clock()
  val tlbBusyClk : Clock = Clock()
  val waitFenceClk : Clock = Clock()
  val waitOldClk : Clock = Clock()

  // Todo: figure out
  val lsiqBarMode : Bool = Bool()
  val fromLsu = new Bundle {
    val lqNotFull : Bool = Bool()
    val rbNotFull : Bool = Bool()
    val sqNotFull : Bool = Bool()
    val noFence : Bool = Bool()
  }

  val fromPad = new LsiqFromPad
  val alreadyDaSet : Bool = Bool()

  val breakpointDataSet = new BreakpointDataSetBundle
  // force set signal
  val lqFullSet : Bool = Bool()
  val rbFullSet : Bool = Bool()
  val specFailSet : Bool = Bool()
  val sqFullSet : Bool = Bool()
  val tlbBusySet : Bool = Bool()
  val tlbWakeUp : Bool = Bool()

  val other = new OtherBundle

  val unalignGateClkEn : Bool = Bool()
  // force set signal
  val unalign2ndSet : Bool = Bool()
  val waitFenceSet : Bool = Bool()
  val waitOldSet : Bool = Bool()
}

class LsiqEntryOutput extends Bundle with LsiqConfig {
  val ready : Bool = Bool()
  val readData = new LsiqEntryData
  val valid : Bool = Bool()
  val validWithFreeze : Bool = Bool()

  val bar : Bool = Bool()
  val barType = new BarType

  val freeze : Bool = Bool()
  val freezeValid : Bool = Bool()
  val load : Bool = Bool()
  val store : Bool = Bool()
  val lqFull : Bool = Bool()
  val rbFull : Bool = Bool()
  val sqFull : Bool = Bool()
  val noSpec : Bool = Bool()
  val rawReady : Bool = Bool()
  val tlbBusy : Bool = Bool()
  val waitFence : Bool = Bool()
  val waitOld : Bool = Bool()
}

class LsiqEntryIO extends Bundle {
  val in : LsiqEntryInput = Flipped(Output(new LsiqEntryInput))
  val out : LsiqEntryOutput = Output(new LsiqEntryOutput)
}

class LsiqEntry extends Module with LsiqConfig {
  val io : LsiqEntryIO = IO(new LsiqEntryIO)

  private val create = io.in.create
  private val rtu = io.in.fromRtu
  private val lsu = io.in.fromLsu
  private val other = io.in.other

  private val srcEntry = Seq.fill(NumSrcLsX)(Module(new DepRegEntry))

  /**
   * Regs
   */

  private val data = RegInit(0.U.asTypeOf(Output(new LsiqEntryData)))
  private val valid = RegInit(false.B)
  private val ageVec = RegInit(VecInit(Seq.fill(this.NumLsiqEntry - 1)(false.B)))
  private val freeze = RegInit(false.B)
  private val tlbBusy = RegInit(false.B)
  private val waitFence = RegInit(false.B)
  private val waitOld = RegInit(false.B)
  private val sqFull = RegInit(false.B)
  private val lqFull = RegInit(false.B)
  private val rbFull = RegInit(false.B)
  private val noSpecCheck = RegInit(false.B)
  // bar: Barrier
  private val barCheck = RegInit(false.B)

  /**
   * wires
   */
  // Todo: figure out wake up postfix
  private val tlbBusyWakeUp = Wire(Bool())
  private val waitFenceWakeUp = Wire(Bool())
  private val waitOldWakeUp = Wire(Bool())
  private val sqFullWakeUp = Wire(Bool())
  private val lqFullWakeUp = Wire(Bool())
  private val rbFullWakeUp = Wire(Bool())
  private val noSpecCheckWakeUp = Wire(Bool())
  private val barCheckWakeUp = Wire(Bool())
  private val srcReadyClearVec = io.in.readyClr

  // Todo: gated clk

  //==========================================================
  //                      Entry Valid
  //==========================================================

  when(rtu.flush.fe || rtu.flush.is) {
    valid := false.B
  }.elsewhen(create.en) {
    valid := true.B
  }.elsewhen(io.in.popValid && io.in.popCurEntry) {
    valid := false.B
  }

  //==========================================================
  //                        Freeze
  //==========================================================
  //inst cannot bypass freeze bar

  // Todo: check if can simplify it
  // Todo: figure out
  private val lsuFreezeClear =
    (barCheck || noSpecCheck || lqFull || sqFull || rbFull ||
      waitOld || waitFence || tlbBusy) &&
      (!barCheck    || barCheck     && barCheckWakeUp)    &&
      (!noSpecCheck || noSpecCheck  && noSpecCheckWakeUp) &&
      (!lqFull      || lqFull       && lqFullWakeUp)      &&
      (!sqFull      || sqFull       && sqFullWakeUp)      &&
      (!rbFull      || rbFull       && rbFullWakeUp)      &&
      (!waitOld     || waitOld      && waitOldWakeUp)     &&
      (!waitFence   || waitFence    && waitFenceWakeUp)   &&
      (!tlbBusy     || tlbBusy      && tlbBusyWakeUp)

  //issue en has higher priority because bar check
  //is still 1 when issue en, frz should be set
  //when issue en and frz clr in this case
  //bar check will be 0 after issue en
  when(create.en) {
    freeze := create.freeze
  }.elsewhen(io.in.issueEn) {
    freeze := true.B
  }.elsewhen(io.in.freezeClr || lsuFreezeClear) {
    freeze := false.B
  }

  //==========================================================
  //                       Age Vector
  //==========================================================
  //agevec of same type (store and bar share same type),
  //used for issue

  when(create.en) {
    ageVec := create.ageVec
    data.ageVecAll := create.ageVecAll
  }.elsewhen(io.in.popValid) {
    ageVec := VecInit((ageVec.asUInt & ~io.in.popOtherEntry.asUInt).asBools)
    data.ageVecAll := VecInit((data.ageVecAll.asUInt & ~io.in.popOtherEntry.asUInt).asBools)
  }

  //==========================================================
  //                 Instruction Information
  //==========================================================

  when(create.dpEn) {
    data.dstPreg := create.data.dstPreg
    data.dstVreg := create.data.dstVreg
    data.barType := create.data.barType
    data.sdEntry := create.data.sdEntry
  }

  when(create.dpEn) {
    data.opcode   := create.data.opcode
    data.iid      := create.data.iid
    data.srcValid := create.data.srcValid
    data.dstValid := create.data.dstValid
    data.dstVValid:= create.data.dstVValid
    data.load     := create.data.load
    data.store    := create.data.store
    data.bar      := create.data.bar
    data.stAddr   := create.data.stAddr
    data.pc       := create.data.pc
    data.split    := create.data.split
    data.noSpec   := create.data.noSpec
    data.noSpecExist := create.data.noSpecExist
    data.vlmul    := create.data.vlmul
    data.vsew     := create.data.vsew
    data.vl       := create.data.vl
    data.splitNum := create.data.splitNum
    data.vmb      := create.data.vmb
  }

  //==========================================================
  //              Barrier Freeze Clear Signals
  //==========================================================
  when(create.en) {
    barCheck := io.in.lsiqBarMode
  }.elsewhen(barCheckWakeUp) {
    barCheck := false.B
  }

  private val befLoad = data.barType.befLoad
  private val befStore = data.barType.befStore

  barCheckWakeUp :=
    //1.  lsiq exit or no bar mode
    !io.in.lsiqBarMode ||
    //2.  if older entry is bar with after load (no matter raw ready), not clear bar frz
    data.load && (!(other.barVec.asUInt & data.ageVecAll.asUInt & other.aftLoadVec.asUInt).orR) ||
    //3.  if older entry is bar with after store (no matter raw ready), not clear bar frz
    data.store && (!(other.barVec.asUInt & data.ageVecAll.asUInt & other.aftStoreVec.asUInt).orR) ||
    //4.1 if older entry is load and cur entry is bar with before load, not ready
    data.bar && ((!(other.load.asUInt & data.ageVecAll.asUInt).orR) && befLoad || !befLoad) &&
    //4.2 if older entry is store and cur entry is bar with before store, not ready
                ((!(other.store.asUInt & data.ageVecAll.asUInt).orR) && befStore || !befStore) &&
    //4.3 if older entry is bar, not ready
                (!(other.barVec.asUInt & data.ageVecAll.asUInt).orR)

  //==========================================================
  //              No Speculation Signals
  //==========================================================
  when(create.en) {
    noSpecCheck := create.data.noSpec && create.data.load
  }.elsewhen(noSpecCheckWakeUp) {
    noSpecCheck := false.B
  }

  noSpecCheckWakeUp := data.load && !(
    other.store.asUInt & other.noSpec.asUInt &
    data.ageVecAll.asUInt & ~other.freeze.asUInt
    ).orR

  //==========================================================
  //                LSU Freeze Clear Signals
  //==========================================================
  //if all bits of age vec is 0, this entry is the oldest entry
  private val oldest = !data.ageVecAll.asUInt.orR
  when(rtu.flush.fe || rtu.flush.is) {
    lqFull := false.B
  }.elsewhen(io.in.lqFullSet && valid) {
    lqFull := true.B
  }.elsewhen(lqFullWakeUp) {
    lqFull := false.B
  }
  lqFullWakeUp := lsu.lqNotFull || oldest

  when(rtu.flush.fe || rtu.flush.is) {
    sqFull := false.B
  }.elsewhen(io.in.sqFullSet && valid) {
    sqFull := true.B
  }.elsewhen(sqFullWakeUp) {
    sqFull := false.B
  }
  sqFullWakeUp := lsu.sqNotFull || oldest

  when(rtu.flush.fe || rtu.flush.is) {
    rbFull := false.B
  }.elsewhen(io.in.rbFullSet && valid) {
    rbFull := true.B
  }.elsewhen(rbFullWakeUp) {
    rbFull := false.B
  }
  rbFullWakeUp := lsu.rbNotFull || oldest

  when(rtu.flush.fe || rtu.flush.is) {
    waitOld := false.B
  }.elsewhen(io.in.waitOldSet && oldest) {
    waitOld := true.B
  }.elsewhen(waitOldWakeUp) {
    waitOld := false.B
  }
  waitOldWakeUp := oldest

  when(rtu.flush.fe || rtu.flush.is) {
    waitFence := false.B
  }.elsewhen(io.in.waitFenceSet && valid) {
    waitFence := true.B
  }.elsewhen(waitFenceWakeUp) {
    waitFence := false.B
  }
  waitFenceWakeUp := lsu.noFence

  when(rtu.flush.fe || rtu.flush.is) {
    tlbBusy := false.B
  }.elsewhen(io.in.tlbBusySet && valid) {
    tlbBusy := true.B
  }.elsewhen(tlbBusyWakeUp) {
    tlbBusy := false.B
  }
  tlbBusyWakeUp := io.in.tlbWakeUp

  //==========================================================
  //                    LSU Pass Signals
  //==========================================================
  when(create.en) {
    data.alreadyDa := false.B
  }.elsewhen(io.in.unalign2ndSet) {
    data.alreadyDa := false.B
  }.elsewhen(io.in.alreadyDaSet) {
    data.alreadyDa := true.B
  }

  when(create.en) {
    data.unalign2nd := false.B
  }.elsewhen(io.in.unalign2ndSet) {
    data.unalign2nd := true.B
  }

  when(create.en) {
    data.breakpointData := 0.U.asTypeOf(chiselTypeOf(data.breakpointData))
  }.otherwise {
    when(io.in.breakpointDataSet.a) {
      data.breakpointData.a := true.B
    }
    when(io.in.breakpointDataSet.b) {
      data.breakpointData.b := true.B
    }
  }

  //==========================================================
  //              Source Dependency Information
  //==========================================================
  private val createSrcGateClkVec = Wire(Vec(this.NumSrcLsX, Bool()))
  createSrcGateClkVec.zipWithIndex.foreach {
    case (gateClkEn, i) => gateClkEn := create.gateClkEn && create.data.srcValid(i)
  }
  private val createSrcDataVec = Wire(Vec(NumSrcLsX, new DepRegEntryCreateData))
  createSrcDataVec.zip(0 until this.NumSrcLsX).foreach{
    case (data, i) =>
      data := create.data.srcVec(i)
  }
  private val readSrcDataVec = Wire(Vec(NumSrcLsX, new DepRegEntryReadData))

  srcEntry.zipWithIndex.foreach {
    case (entry, i) =>
      val in  = entry.io.in
      val out = entry.io.out
      in.fwdValid.alu := io.in.fwdValid.alu(i)
      in.fwdValid.load := io.in.fwdValid.load
      in.fromCp0  := io.in.fromCp0
      in.fuDstPreg:= io.in.fuDstPreg
      in.wbPreg   := io.in.wbPreg
      in.loadPreg := io.in.loadPreg
      in.flush    := rtu.flush
      in.createData := createSrcDataVec(i)
      in.gateClkIdxWen := createSrcGateClkVec(i)
      in.gateClkWen := create.gateClkEn
      in.readyClear := srcReadyClearVec(i)
      in.wen        := create.dpEn
      readSrcDataVec(i) := out.readData
  }

  // Todo: src entry for vm
//  private val srcEntryVm

  io.out.readData.ageVecAll := data.ageVecAll
  io.out.readData.opcode    := data.opcode
  io.out.readData.iid       := data.iid
  io.out.readData.srcValid  := data.srcValid
  io.out.readData.dstValid  := data.dstValid
  io.out.readData.dstVValid := data.dstVValid
  io.out.readData.dstPreg   := data.dstPreg
  io.out.readData.dstVreg   := data.dstPreg
  io.out.readData.load      := data.load
  io.out.readData.store     := data.store
  io.out.readData.bar       := data.bar
  io.out.readData.barType   := data.barType
  io.out.readData.pc        := data.pc
  io.out.readData.stAddr    := data.stAddr
  io.out.readData.sdEntry   := data.sdEntry
  io.out.readData.split     := data.split
  io.out.readData.noSpec    := data.noSpec
  io.out.readData.noSpecExist := data.noSpecExist
  io.out.readData.vlmul     := data.vlmul
  io.out.readData.vsew      := data.vsew
  io.out.readData.vl        := data.vl
  io.out.readData.splitNum  := data.splitNum
  io.out.readData.vmb       := data.vmb

  io.out.readData.alreadyDa := data.alreadyDa
  io.out.readData.unalign2nd := data.unalign2nd
  io.out.readData.specFail  := data.specFail
  io.out.readData.breakpointData := data.breakpointData
  io.out.readData.srcVec.zipWithIndex.foreach{
    case (data, i) =>
      data.wb := readSrcDataVec(i).wb
      data.preg := readSrcDataVec(i).preg
      data.ready := false.B
      data.lsuMatch := false.B
  }
  io.out.readData.srcVm := DontCare
  io.out.readData.srcVmValid := DontCare

  io.out.load     := data.load
  io.out.store    := data.store
  io.out.bar      := data.bar
  io.out.barType  := data.barType
  io.out.noSpec   := data.noSpec

  io.out.freeze   := freeze
  io.out.freezeValid := freeze && valid
  io.out.lqFull   := lqFull
  io.out.rbFull   := rbFull
  io.out.sqFull   := sqFull
  io.out.tlbBusy  := tlbBusy

  io.out.valid    := valid
  // Todo: rename maybe validWithOutFreeze?
  io.out.validWithFreeze := valid && (!freeze || data.bar)
  io.out.waitFence:= waitFence
  io.out.waitOld  := waitOld

  //==========================================================
  //                  Entry Ready Signal
  //==========================================================
  //------------------------raw ready-------------------------

  // Todo: add vm ready for issue
  io.out.rawReady := valid &&
    !freeze &&
    readSrcDataVec.map(_.readyForIssue).reduce(_&&_)

  //----------------------older ready-------------------------
  //if older entry of same type raw ready, mask cur entry ready
  private val olderEntryReadyMask = (ageVec.asUInt & other.rawReady.asUInt).orR

  //----------------------final ready-------------------------
  //if older entry is ready, mask current entry ready
  io.out.ready := io.out.rawReady && !olderEntryReadyMask
}
