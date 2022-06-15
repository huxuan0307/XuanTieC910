package Core.IDU.IS

import Core.PipelineConfig._
import Utils.Bits.DropBit
import chisel3._
import chisel3.util._

class LsiqCtrlInput extends Bundle with LsiqConfig {
  val createDpEnVec : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
  val createEnVec : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
  val createGateClkVec : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
  val fromIr = new Bundle {
    val barInstValid : Bool = Bool()
  }
  val fromIs = new Bundle {
    val barInstValid : Bool = Bool()
  }
  val fromRf = new Bundle {
    // Todo: imm
    val launchFailValid : Vec[Bool] = Vec(NumLsuPipe, Bool())
  }
}

class LsiqDataInput extends Bundle with LsiqConfig {
  val bypassData : LsiqEntryData = new LsiqEntryData
  val createData : Vec[LsiqEntryData] = Vec(NumLsiqCreatePort, new LsiqEntryData)
  val create = new Bundle {
    val bar : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
    val load : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
    val store : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
    val noSpec : Vec[Bool] = Vec(NumLsiqCreatePort, Bool())
    /**
     * src0, src1, src_vm
     */
    val srcReadyForBypass : Vec[Bool] = Vec(NumSrcLs, Bool())
  }
  val fromRf = new Bundle {
    val launchEntry : Vec[Vec[Bool]] = Vec(NumLsuPipe, Vec(NumLsiqEntry, Bool()))
    val readyClear : Vec[Vec[Bool]] = Vec(NumLsuPipe, Vec(NumSrcLs, Bool()))
  }
}

class LsiqFromLsuBundle extends Bundle with LsiqConfig {
  val alreadyDaVec : Vec[Bool] = Vec(NumLsiqEntry, Bool())
  val breakpointDataVec : Vec[BreakpointDataBundle] = Vec(NumLsiqEntry, new BreakpointDataBundle)
  val lqFullVec : Vec[Bool] = Vec(NumLsiqEntry, Bool())
  val lqNotFull : Bool = Bool()
  val rbFullVec : Vec[Bool] = Vec(NumLsiqEntry, Bool())
  val rbNotFull : Bool = Bool()
  val sqFullVec : Vec[Bool] = Vec(NumLsiqEntry, Bool())
  val sqNotFull : Bool = Bool()
  // Todo: imm
  val popValidVec : Vec[Bool] = Vec(2, Bool())
  val popEntryVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val popValid : Bool = Bool()

  val noFence : Bool = Bool()
  val secd : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val specFail : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val tlbBusyVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val tlbWakeUpVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val waitFenceVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val waitOldVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val wakeUpVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())

  val lqFullGateClkEn : Bool = Bool()
  val rbFullGateClkEn : Bool = Bool()
  val sqFullGateClkEn : Bool = Bool()
  val tlbBusyGateClkEn : Bool = Bool()
  val unalignGateClkEnVec : Vec[Bool] = Vec(this.NumLsiqEntry, Bool())
  val waitFenceGateClkEn : Bool = Bool()
  val waitOldGateClkEn : Bool = Bool()
}

class LsiqInput
  extends IqInput(LsiqConfig.NumLsiqEntry, LsiqConfig.NumSrcLsX)
  with LsiqConfig {
  val ctrl = new LsiqCtrlInput
  val data = new LsiqDataInput
  val fromLsu = new LsiqFromLsuBundle
}

class LsiqCtrlOutput extends Bundle with LsiqConfig {
  val empty : Bool = Bool()
  val full : Bool = Bool()
  val fullUpdate : Bool = Bool()
  val fullUpdateClkEn : Bool = Bool()
  val oneLeftUpdate : Bool = Bool()
}

class LsiqDataOutput extends Bundle with LsiqConfig {
  val createBypassOldest : Bool = Bool()
  val noSpecStoreValid : Bool = Bool()
  val issueEntryVec : Vec[Vec[Bool]] = Vec(NumLsuPipe, Vec(this.NumLsiqEntry, Bool()))
  val issueReadData : Vec[LsiqEntryData] = Vec(NumLsuPipe, new LsiqEntryData)
}

class LsiqOutput extends Bundle with LsiqConfig {
  val entryEnqOHVec : Vec[UInt] = Vec(NumLsiqCreatePort, UInt(this.NumLsiqEntry.W))
  val ctrl = new LsiqCtrlOutput
  val data = new LsiqDataOutput
  val toTop = new Bundle {
    val freezeEntryValid : Bool = Bool()
    val entryCnt : UInt = UInt(log2Up(NumLsiqEntry).W)
  }
  val gateClkIssueEn : Bool = Bool()
  val issueEnVec : Vec[Bool] = Vec(NumLsuPipe, Bool())
}

class LsiqIO extends Bundle {
  val in : LsiqInput = Flipped(Output(new LsiqInput))
  val out : LsiqOutput = Output(new LsiqOutput)
}

class Lsiq extends Module with LsiqConfig {
  val io : LsiqIO = IO(new LsiqIO)

  private val cp0 = io.in.fromCp0
  private val rtu = io.in.fromRtu
  private val ctrlLsiq = io.in.ctrl
  private val dataLsiq = io.in.data
  private val lsu = io.in.fromLsu

  private val entries = Seq.fill(this.NumLsiqEntry)(Module(new LsiqEntry))

  /**
   * Regs
   */
  private val barMode = RegInit(false.B)
  private val entryCnt = RegInit(0.U(log2Up(this.NumLsiqEntry).W))

  /**
   * Wires
   */
  private val entryCreateEnVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryCreateDpEnVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryCreateGateClkVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryCreateAgeVec = Wire(Vec(this.NumLsiqEntry, Vec(this.NumLsiqEntry - 1, Bool())))
  private val entryCreateAgeVecAll = Wire(Vec(this.NumLsiqEntry, Vec(this.NumLsiqEntry - 1, Bool())))
  private val entryCreateDataVec = Wire(Vec(this.NumLsiqEntry, new LsiqEntryData))
  private val entryCreateFreezeVec = Wire(Vec(this.NumLsiqEntry, Bool()))

  private val entryPortCreateAgeVec = Wire(Vec(this.NumLsiqCreatePort, Vec(this.NumLsiqEntry, Bool())))
  private val entryPortCreateAgeVecAll = Wire(Vec(this.NumLsiqCreatePort, Vec(this.NumLsiqEntry, Bool())))


  private val entryOutLqFullVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutSqFullVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutRbFullVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutTlbBusyVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutWaitOldVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutWaitFenceVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutValidVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutBarVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutFreezeVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutFreezeValidVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutLoadVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutStoreVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutNoSpecVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutReadyVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutReadDataVec = Wire(Vec(this.NumLsiqEntry, new LsiqEntryData))
  private val entryOutRawVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryOutBarTypeVec = Wire(Vec(this.NumLsiqEntry, new BarType))

  private val entryValidWithOutFreezeVec = Wire(Vec(this.NumLsiqEntry, Bool()))

  private val entryOtherVec = Wire(Vec(this.NumLsiqEntry, new OtherBundle))

  // find two empty entry
  private val enq0OH : Vec[Bool] = VecInit(PriorityEncoderOH(entryOutValidVec.map(!_)))
  private val enq1OH : Vec[Bool] = VecInit(PriorityEncoderOH(entryOutValidVec.reverse.map(!_)).reverse)
  io.out.entryEnqOHVec(0) := enq0OH.asUInt
  io.out.entryEnqOHVec(1) := enq1OH.asUInt
  //==========================================================
  //                LSU Restart gateclk
  //==========================================================

  private val lqFullClkEn = lsu.lqFullGateClkEn || entryOutLqFullVec.reduce(_||_)
  private val sqFullClkEn = lsu.sqFullGateClkEn || entryOutSqFullVec.reduce(_||_)
  private val rbFullClkEn = lsu.rbFullGateClkEn || entryOutRbFullVec.reduce(_||_)
  private val tlbBusyClkEn = lsu.tlbBusyGateClkEn || entryOutTlbBusyVec.reduce(_||_)
  private val waitOldClkEn = lsu.waitOldGateClkEn || entryOutWaitOldVec.reduce(_||_)
  private val waitFenceClkEn = lsu.waitFenceGateClkEn || entryOutWaitFenceVec.reduce(_||_)
  // Todo: gated clk

  //==========================================================
  //            LSU Issue Queue Barrier Mode
  //==========================================================

  private val barClkEn = barMode ||
    ctrlLsiq.fromIr.barInstValid || ctrlLsiq.fromIs.barInstValid

  //when ir or is has bar inst, enable barrier mode
  //when lsiq has no bar inst, disable barrier mode
  private val lsiqNoBarInst = !(entryOutValidVec.asUInt & entryOutBarVec.asUInt).orR

  //in barrier mode:
  //1.disable lsiq bypass
  //2.all lsiq inst create into lsiq with frz set
  //3.check older inst barrier state, if it is not barriered by older inst, clear frz
  when(rtu.flush.fe || rtu.flush.is) {
    barMode := false.B
  }.elsewhen(ctrlLsiq.fromIr.barInstValid || ctrlLsiq.fromIs.barInstValid) {
    barMode := true.B
  }.elsewhen(lsiqNoBarInst) {
    barMode := false.B
  }

  //==========================================================
  //            LSU Issue Queue Create Control
  //==========================================================
  private val cntClkEn = (entryCnt =/= 0.U) || ctrlLsiq.createGateClkVec.reduce(_||_)
  // Todo: gated clk

  io.out.ctrl.fullUpdateClkEn := cntClkEn

  //--------------------lsiq entry counter--------------------
  //if create, add entry counter
  private val entryCntCreate = ctrlLsiq.createEnVec.count(b => b)
  //if pop, sub entry counter
  private val entryCntPop = lsu.popValidVec.count(b => b)
  private val entryCntUpdateValid = ctrlLsiq.createEnVec(0) || lsu.popValid
  private val entryCntUpdate = entryCnt + entryCntCreate - entryCntPop

  when(rtu.flush.fe || rtu.flush.is || rtu.flush.be) {
    entryCnt := 0.U
  }.elsewhen(entryCntUpdateValid) {
    entryCnt := entryCntUpdate
  }

  io.out.ctrl.full := entryCnt === this.NumLsiqEntry.U
  io.out.toTop.entryCnt := entryCnt

  //--------------------lsiq entry full-----------------------
  // Num of entry of create cannot be more than available entry
  io.out.ctrl.fullUpdate :=
    entryCnt === (this.NumLsiqEntry - 2).U && entryCntCreate === 2.U && entryCntPop === 0.U ||
      entryCnt === (this.NumLsiqEntry - 1).U && entryCntCreate === 1.U && entryCntPop === 0.U ||
      entryCnt === this.NumLsiqEntry.U && entryCntCreate === 0.U && entryCntPop === 0.U
  io.out.ctrl.oneLeftUpdate :=
    entryCnt === (this.NumLsiqEntry - 3).U && entryCntCreate === 2.U && entryCntPop === 0.U ||
      entryCnt === (this.NumLsiqEntry - 2).U && entryCntCreate === 1.U && entryCntPop === 0.U ||
      entryCnt === (this.NumLsiqEntry - 2).U && entryCntCreate === 2.U && entryCntPop === 1.U ||
      entryCnt === (this.NumLsiqEntry - 1).U && entryCntCreate === 1.U && entryCntPop === 1.U ||
      entryCnt === (this.NumLsiqEntry - 1).U && entryCntCreate === 0.U && entryCntPop === 0.U ||
      entryCnt === (this.NumLsiqEntry - 0).U && entryCntCreate === 0.U && entryCntPop === 1.U

  //---------------------create bypass------------------------
  //if create instruction is ready, it may bypass from issue queue
  //only create 0 can bypass
  private val create0ReadyBypass = ctrlLsiq.createEnVec(0) &&
    !cp0.iqBypassDisable &&
    !dataLsiq.create.bar(0) &&
    !(dataLsiq.create.noSpec(0)&&dataLsiq.create.load(0)) &&
    dataLsiq.create.srcReadyForBypass.reduce(_&&_)

  //data path bypass signal, with timing optimized
  private val create0ReadyBypassDpEn = ctrlLsiq.createDpEnVec(0) &&
    !cp0.iqBypassDisable &&
    !dataLsiq.create.bar(0) &&
    !(dataLsiq.create.noSpec(0)&&dataLsiq.create.load(0)) &&
    dataLsiq.create.srcReadyForBypass.reduce(_&&_)

  //data path bypass signal, with timing optimized
  private val create0ReadyBypassGateClk = ctrlLsiq.createGateClkVec(0) &&
    dataLsiq.create.srcReadyForBypass.reduce(_&&_)

  //inst cannot bypass freeze bar
  private val createBypassEmpty = !entryValidWithOutFreezeVec.reduce(_||_)
  io.out.toTop.freezeEntryValid := !entryOutFreezeValidVec.reduce(_||_)
  io.out.ctrl.empty := !entryOutValidVec.reduce(_||_)

  // Todo: figure out
  private val bypassEn = createBypassEmpty && !barMode && create0ReadyBypass

  entryCreateEnVec.zipWithIndex.foreach {
    case (entryCreateEn, i) =>
      entryCreateEn := ctrlLsiq.createEnVec(0) && enq0OH(i) ||
        ctrlLsiq.createEnVec(1) && enq1OH(i)
  }

  private val bypassDpEn = createBypassEmpty && !barMode && create0ReadyBypassDpEn
  private val bypassGateClk = createBypassEmpty && !barMode && create0ReadyBypassGateClk

  //issue queue entry create data path control
  entryCreateDpEnVec.zipWithIndex.foreach {
    case (entryCreateDpEn, i) =>
      entryCreateDpEn := ctrlLsiq.createDpEnVec(0) && enq0OH(i) ||
        ctrlLsiq.createDpEnVec(1) && enq1OH(i)
  }

  //issue queue entry create gateclk control
  //ignore bypass signal for timing optimization
  entryCreateGateClkVec.zipWithIndex.foreach {
    case (entryCreateGateClk, i) =>
      entryCreateGateClk := ctrlLsiq.createGateClkVec(0) && enq0OH(i) ||
        ctrlLsiq.createDpEnVec(1) && enq1OH(i)
  }

  //-------------------agevec of same type--------------------
  //create 0 age vectors:
  //1.existed entries of same type (bar and store shares same type)
  // if one entry already has occupied by load/store inst, then set age true
  entryPortCreateAgeVec(0).zipWithIndex.foreach {
    case (age, i) =>
      age := entryOutValidVec(i) && (
        dataLsiq.createData(0).load && entryOutLoadVec(i) || (
          (dataLsiq.createData(0).store || dataLsiq.create.bar(0)) &&
            (entryOutStoreVec(i) || entryOutBarVec(i))
        )
      ) && !lsu.popEntryVec(i)
  }

  //create 1 age vectors:
  //1.existed entries of same type
  //2.create 0 entry of same type
  // treat store and bar as the same
  entryPortCreateAgeVec(1).zipWithIndex.foreach {
    case (age, i) =>
      age := entryOutValidVec(i) && (
        dataLsiq.createData(1).load && entryOutLoadVec(i) || (
          (dataLsiq.createData(1).store || dataLsiq.create.bar(1)) &&
            (entryOutStoreVec(i) || entryOutBarVec(i))
        )
      ) && !lsu.popEntryVec(i) ||
        enq0OH(i) && (
          dataLsiq.create.load(0) && dataLsiq.create.load(1) ||
            (dataLsiq.create.store(0) || dataLsiq.create.bar(0)) &&
              (dataLsiq.create.store(1) || dataLsiq.create.bar(1))
        )
  }

  //-------------------agevec of all types--------------------
  //create 0 age vectors:
  //1.existed entries of all types
  entryPortCreateAgeVecAll(0).zipWithIndex.foreach {
    case (age, i) => age := entryOutValidVec(i) && !lsu.popEntryVec(i)
  }

  //oldest for bypass inst
  //bypass oldest ignore pop vld for timing optimization, if bypass
  //inst is wrongly recognized as not oldest, it will replay later
  io.out.data.createBypassOldest := !entryOutValidVec.reduce(_||_)

  //create 1 age vectors:
  //1.existed entries of all types
  //2.create 0 entry
  entryPortCreateAgeVecAll(1).zipWithIndex.foreach {
    case (age, i) => age := entryOutValidVec(i) && !lsu.popEntryVec(i) || enq0OH(i)
  }

  //-------------------no spec store valid--------------------
  io.out.data.noSpecStoreValid := entryOutValidVec.zipWithIndex.map {
    case (valid, i) =>
      valid && !entryOutFreezeVec(i) && entryOutStoreVec(i) && entryOutNoSpecVec(i)
  }.reduce(_||_)

  //-------------------create frz signal----------------------
  private val entryPortCreateFreezeVec = Wire(Vec(this.NumLsiqCreatePort, Bool()))
  entryPortCreateFreezeVec(0) := barMode || bypassDpEn ||
    dataLsiq.create.noSpec(0) && dataLsiq.create.load(0)
  entryPortCreateFreezeVec(1) := barMode ||
    dataLsiq.create.noSpec(1) && dataLsiq.create.load(1)

  //-----------------create select signals--------------------
  //create 0/1 select:
  //entry 0~5 use ~lsiq_entry_create0_in for better timing
  //entry 6~11 use lsiq_entry_create1_in for better timing
  //lsiq_entry_create0/1_in cannot be both 1,
  //if both 0, do not create
  private val entryCreateSel = Wire(Vec(this.NumLsiqEntry, Bool()))
  for (i <- 0 until this.NumLsiqEntry) {
    entryCreateSel(i) := ctrlLsiq.createDpEnVec(1) && enq1OH(i)
  }
  // Todo: timing optimization
  //  for (i <- 0 until NumLsiqEntry / 2) {
  //    entryCreateSel(i) := ~(ctrlBiq.createDpEn(0) && enq0OH(i))
  //  }
  //  for (i <- NumLsiqEntry / 2 until NumLsiqEntry {
  //    entryCreateSel(i) := ctrlBiq.createDpEn(1) && enq1OH(i))
  //  }

  //----------------entry0 flop create signals----------------

  for (i <- 0 until this.NumLsiqEntry) {
    when(!entryCreateSel(i)) {
      entryCreateFreezeVec(i) := entryPortCreateFreezeVec(0)
      entryCreateAgeVec(i)    := VecInit(DropBit(entryPortCreateAgeVec(0), i).asBools)
      entryCreateAgeVecAll(i) := VecInit(DropBit(entryPortCreateAgeVecAll(0), i).asBools)
      entryCreateDataVec(i)   := dataLsiq.createData(0)
    }.otherwise {
      entryCreateFreezeVec(i) := entryPortCreateFreezeVec(1)
      entryCreateAgeVec(i)    := VecInit(DropBit(entryPortCreateAgeVec(1), i).asBools)
      entryCreateAgeVecAll(i) := VecInit(DropBit(entryPortCreateAgeVecAll(1), i).asBools)
      entryCreateDataVec(i)   := dataLsiq.createData(1)
    }
  }

  //==========================================================
  //             LSU Issue Queue Issue Control
  //==========================================================

  //------------prepare type signals for each entry-----------
  entryOtherVec.zipWithIndex.foreach {
    case (other, i) =>
      other.load := VecInit(DropBit(entryOutLoadVec, i).asBools)
      other.store := VecInit(DropBit(entryOutStoreVec, i).asBools)
      other.barVec := VecInit(DropBit(entryOutBarVec, i).asBools)
      other.rawReady := VecInit(DropBit(entryOutRawVec, i).asBools)
      other.aftLoadVec := VecInit(DropBit(entryOutLoadVec, i).asBools)
      other.aftStoreVec := VecInit(DropBit(entryOutStoreVec, i).asBools)
      other.noSpec := VecInit(DropBit(entryOutNoSpecVec, i).asBools)
      other.freeze := VecInit(DropBit(entryOutFreezeVec, i).asBools)
  }

  //-----------entry issue enable signals for entries---------
  //issue signals for entries ignore inst type
  private val entryIssueEnVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  entryIssueEnVec := entryOutReadyVec

  //---------------entry issue signals for rf pipes-----------
  private val loadEntryReadyVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val storeEntryReadyVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  //load entry ready <=> pipe3 entry ready
  loadEntryReadyVec.zipWithIndex.foreach {
    case (loadEntryReady, i) =>
      loadEntryReady := entryOutReadyVec(i) && entryOutLoadVec(i)
  }
  //store entry ready <=> pipe4 entry ready
  storeEntryReadyVec.zipWithIndex.foreach {
    case (storeEntryReady, i) =>
      storeEntryReady := entryOutReadyVec(i) && (entryOutStoreVec(i) || entryOutBarVec(i))
  }

  //bypass enable
  private val loadBypassEn = bypassEn && dataLsiq.create.load(0)
  private val storeBypassEn = bypassEn && dataLsiq.create.store(0)

  //issue enable signals:
  //if bypass or ready entry exists
  io.out.issueEnVec(0) := loadBypassEn || loadEntryReadyVec.reduce(_||_)
  io.out.issueEnVec(1) := storeBypassEn || storeEntryReadyVec.reduce(_||_)

  //gate clock issue enable with timing optimization
  io.out.gateClkIssueEn := bypassGateClk || entryValidWithOutFreezeVec.reduce(_||_)

  //issue enable for rf pipes:
  //consider inst type
  //-----------------issue entry indication--------------------
  io.out.data.issueEntryVec(0) := Mux(
    createBypassEmpty,
    enq0OH,
    loadEntryReadyVec
  )
  io.out.data.issueEntryVec(1) := Mux(
    createBypassEmpty,
    enq1OH,
    storeEntryReadyVec
  )

  //-----------------issue data path selection----------------
  //issue data path will select oldest ready entry in issue queue
  //if no instruction valid, the data path will always select bypass
  //data path

  private val loadEntryReadData = entryOutReadDataVec(OHToUInt(loadEntryReadyVec))
  private val storeEntryReadData = entryOutReadDataVec(OHToUInt(storeEntryReadyVec))

  //if no entry valid, select bypass path
  io.out.data.issueReadData(0) := Mux(
    createBypassEmpty,
    dataLsiq.bypassData,
    loadEntryReadData
  )
  io.out.data.issueReadData(1) := Mux(
    createBypassEmpty,
    dataLsiq.bypassData,
    storeEntryReadData
  )

  //==========================================================
  //            LSU Issue Queue Launch Control
  //==========================================================

  //-------------------entry pop enable signals---------------
  private val entryPopCurEntryVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val entryPopOtherEntryVec = Wire(Vec(this.NumLsiqEntry, Vec(this.NumLsiqEntry - 1, Bool())))
  entryPopCurEntryVec := lsu.popEntryVec
  entryPopOtherEntryVec.zipWithIndex.foreach {
    case (popOther, i) => popOther := VecInit(DropBit(lsu.popEntryVec, i).asBools)
  }

  //-------------------entry spec fail signals---------------
  //clear freeze and source rdy when launch fail
  private val entryFreezeClearVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val loadEntryFreezeClearVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  private val storeEntryFreezeClearVec = Wire(Vec(this.NumLsiqEntry, Bool()))
  // load pipe
  loadEntryFreezeClearVec.zipWithIndex.foreach {
    case (freezeClear, i) =>
      freezeClear := ctrlLsiq.fromRf.launchFailValid(0) && dataLsiq.fromRf.launchEntry(0)(i)
  }
  // store pipe
  storeEntryFreezeClearVec.zipWithIndex.foreach {
    case (freezeClear, i) =>
      freezeClear := ctrlLsiq.fromRf.launchFailValid(1) && dataLsiq.fromRf.launchEntry(1)(i)
  }
  entryFreezeClearVec.zipWithIndex.foreach {
    case (freezeClear, i) =>
      freezeClear := loadEntryFreezeClearVec(i) || storeEntryFreezeClearVec(i) || lsu.wakeUpVec(i)
    // Todo: figure out wakeup from lsu
  }

  private val entryReadyClearVec = Wire(Vec(this.NumLsiqEntry, Vec(this.NumSrcLs, Bool())))

  entryReadyClearVec.zipWithIndex.foreach {
    case (readyClrVec, i) =>
      readyClrVec := VecInit((
        Fill(this.NumSrcLs,loadEntryFreezeClearVec(i)) & dataLsiq.fromRf.readyClear(0).asUInt |
          Fill(this.NumSrcLs,storeEntryFreezeClearVec(i)) & dataLsiq.fromRf.readyClear(1).asUInt
      ).asBools)
  }

  entries.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fromCp0 := io.in.fromCp0
      in.fwdValid.alu(0) := io.in.fwd.aluValid(i)(0)
      in.fwdValid.alu(1) := io.in.fwd.aluValid(i)(1)
      in.fwdValid.load(0) := io.in.fwd.loadValid

      in.fuDstPreg := io.in.fuDstPreg
      in.wbPreg := io.in.wbPreg
      in.loadPreg := io.in.loadPreg
      in.popValid := lsu.popValid
      in.readyClr := entryReadyClearVec(i)
      in.fromRtu.flush := io.in.fromRtu.flush
      in.create.ageVec := entryCreateAgeVec(i)
      in.create.ageVecAll := entryCreateAgeVecAll(i)
      in.create.data := entryCreateDataVec(i)
      in.create.dpEn := entryCreateDpEnVec(i)
      in.create.en := entryCreateEnVec(i)
      in.create.freeze := entryCreateFreezeVec(i)
      in.create.gateClkEn := entryCreateGateClkVec(i)
      in.freezeClr := entryFreezeClearVec(i)
      in.issueEn := entryIssueEnVec(i)
      in.popCurEntry := entryPopCurEntryVec(i)
      in.popOtherEntry := entryPopOtherEntryVec(i)
      in.lqFullClk := clock // Todo: gated clk
      in.rbFullClk := clock // Todo: gated clk
      in.sqFullClk := clock // Todo: gated clk
      in.tlbBusyClk := clock // Todo: gated clk
      in.waitFenceClk := clock // Todo: gated clk
      in.waitOldClk := clock // Todo: gated clk
      in.lsiqBarMode := barMode
      in.fromLsu.lqNotFull := lsu.lqNotFull
      in.fromLsu.rbNotFull := lsu.rbNotFull
      in.fromLsu.sqNotFull := lsu.sqNotFull
      in.fromLsu.noFence := lsu.noFence
      in.fromPad := io.in.fromPad
      in.alreadyDaSet := lsu.alreadyDaVec(i)
      in.breakpointDataSet := lsu.breakpointDataVec(i) // Todo: same type
      in.lqFullSet := lsu.lqFullVec(i)
      in.rbFullSet := lsu.rbFullVec(i)
      in.sqFullSet := lsu.sqFullVec(i)
      in.specFailSet := lsu.specFail(i)
      in.tlbBusySet := lsu.tlbBusyVec(i)
      in.tlbWakeUp := lsu.tlbWakeUpVec(i)
      in.other := entryOtherVec(i)
      in.unalignGateClkEn := lsu.unalignGateClkEnVec(i)
      in.unalign2ndSet := lsu.secd(i)
      in.waitFenceSet := lsu.waitFenceVec(i)
      in.waitOldSet := lsu.waitOldVec(i)

      entryOutReadyVec(i) := out.ready
      entryOutReadDataVec(i) := out.readData
      entryOutValidVec(i) := out.valid
      entryValidWithOutFreezeVec(i) := out.validWithFreeze //Todo:check
      entryOutBarVec(i) := out.bar
      entryOutBarTypeVec(i) := out.barType
      entryOutFreezeVec(i) := out.freeze
      entryOutFreezeValidVec(i) := out.freezeValid
      entryOutLoadVec(i) := out.load
      entryOutStoreVec(i) := out.store
      entryOutLqFullVec(i) := out.lqFull
      entryOutRbFullVec(i) := out.rbFull
      entryOutSqFullVec(i) := out.sqFull
      entryOutNoSpecVec(i) := out.noSpec
      entryOutRawVec(i) := out.rawReady
      entryOutTlbBusyVec(i) := out.tlbBusy
      entryOutWaitFenceVec(i) := out.waitFence
      entryOutWaitOldVec(i) := out.waitOld
  }
}
