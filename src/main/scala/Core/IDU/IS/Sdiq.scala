package Core.IDU.IS

import Core.IntConfig.NumPhysicRegs
import Core.PipelineConfig.NumLsuPipe
import Utils.Bits.DropBit
import chisel3._
import chisel3.util._

class SdiqCtrlInput extends Bundle with SdiqConfig {
  val createDpEnVec     : Vec[Bool] = Vec(NumSdiqCreatePort, Bool())
  val createEnVec       : Vec[Bool] = Vec(NumSdiqCreatePort, Bool())
  val createGateClkVec  : Vec[Bool] = Vec(NumSdiqCreatePort, Bool())
  val fromRf = new Bundle {
    val stAddrReadySet  : Bool = Bool()
    val launchFailValid : Bool = Bool()
  }
}

class SdiqDataInput extends Bundle with SdiqConfig {
  val createData: Vec[SdiqEntryData] = Vec(NumSdiqCreatePort, new SdiqEntryData)
  val fromRf = new Bundle {
    val readyClear      : Vec[Bool] = Vec(NumSrcSd, Bool())
    val launchEntry     : Vec[Bool] = Vec(NumSdiqEntry, Bool())
    val sdiqEntry       : Vec[Bool] = Vec(NumSdiqEntry, Bool())
    val stAddr1Valid    : Bool = Bool()
    val stAddrReadyClear: Bool = Bool()
    val stData1Valid    : Bool = Bool()
  }
}

class SdiqFromLsuBundle extends Bundle with SdiqConfig {
  val dc = new Bundle {
    val sdiqEntryVec  : Vec[Bool] = Vec(NumSdiqEntry, Bool())
    val stAddr1Valid  : Bool = Bool() // Todo: Figure out difference with stAddrValid
    val stAddrUnalign : Bool = Bool()
    val stAddrValid   : Bool = Bool()
  }
  val ex1 = new Bundle {
    val sdiqEntryVec  : Vec[Bool] = Vec(NumSdiqEntry, Bool())
    val freezeClear   : Bool = Bool()
    val popValid      : Bool = Bool()
  }
}

class SdiqInput
  extends IqInput(SdiqConfig.NumSdiqEntry, SdiqConfig.NumSrcSdX)
  with SdiqConfig {
  val ctrl = new SdiqCtrlInput
  val data = new SdiqDataInput
  val fromLsu = new SdiqFromLsuBundle
}

class SdiqCtrlOutput extends Bundle with SdiqConfig {
  val empty           : Bool = Bool()
  val full            : Bool = Bool()
  val fullUpdate      : Bool = Bool()
  val fullUpdateClkEn : Bool = Bool()
  val oneLeftUpdate   : Bool = Bool()
}

class SdiqDataOutput extends Bundle with SdiqConfig {
  val createEntry : Vec[UInt] = Vec(NumSdiqCreatePort, UInt(this.NumSdiqEntry.W))
  val issueEntry  : UInt = UInt(this.NumSdiqEntry.W)
  val issueReadData = new SdiqEntryData
}

class SdiqOutput extends Bundle with SdiqConfig {
  val ctrl = new SdiqCtrlOutput
  val data = new SdiqDataOutput
  val toTop = new Bundle {
    val entryCnt : UInt = UInt(log2Up(NumSdiqEntry).W)
  }
  val gateClkIssueEn : Bool = Bool()
  val issueEn: Bool = Bool()
  val toRtu = new Bundle {
    val pregDeallocMask: Vec[Bool] = Vec(NumPhysicRegs, Bool())
  }
}

class SdiqIO extends Bundle {
  val in  : SdiqInput   = Flipped(Output(new SdiqInput))
  val out : SdiqOutput  = Output(new SdiqOutput)
}

class Sdiq extends Module with SdiqConfig {
  val io: SdiqIO = IO(new SdiqIO)

  private val cp0 = io.in.fromCp0
  private val rtu = io.in.fromRtu
  private val ctrl = io.in.ctrl
  private val data = io.in.data
  private val lsu = io.in.fromLsu

  /**
   * Regs
   */
  private val numEntry = RegInit(0.U(log2Up(this.NumSdiqEntry).W))

  /**
   * Wires
   */
  private val entryCreateEnVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryCreateDpEnVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryCreateGateClkVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryCreateAgeVec = Wire(Vec(this.NumSdiqEntry, Vec(this.NumSdiqEntry - 1, Bool())))
  private val entryCreateDataVec = Wire(Vec(this.NumSdiqEntry, new SdiqEntryData))

  private val entryPortCreateAgeVec = Wire(Vec(this.NumSdiqCreatePort, Vec(this.NumSdiqEntry, Bool())))

  private val entryOutValidVec = Wire(Vec(this.NumSdiqEntry, Bool()))

  //==========================================================
  //            LSU Issue Queue Create Control
  //==========================================================
  //-------------------gated cell instance--------------------
  private val cntClkEn = (numEntry =/= 0.U) || ctrl.createGateClkVec.reduce(_ || _)
  // Todo: gated clk

  io.out.ctrl.fullUpdateClkEn := cntClkEn

  //--------------------sdiq entry counter--------------------
  private val numEntryCreate = PopCount(ctrl.createEnVec)
  private val numEntryPop = lsu.ex1.popValid // Only pop 1 entry per period
  private val numEntryUpdateValid = ctrl.createEnVec(0) || lsu.ex1.popValid
  private val numEntryUpdate = numEntry + numEntryCreate - numEntryPop

  //implement entry counter
  when (rtu.flush.be) {
    numEntry := 0.U
  }.elsewhen (numEntryUpdateValid) {
    numEntry := numEntryUpdate
  }

  io.out.ctrl.full := numEntry === this.NumSdiqEntry.U
  io.out.toTop.entryCnt := numEntry

  private val fullUpdate =
    (numEntry === (this.NumSdiqEntry - 2).U) && numEntryCreate === 2.U && numEntryPop === 0.U ||
    (numEntry === (this.NumSdiqEntry - 1).U) && numEntryCreate === 1.U && numEntryPop === 0.U ||
    (numEntry === (this.NumSdiqEntry - 0).U) && numEntryCreate === 0.U && numEntryPop === 0.U

  private val oneLeftUpdate =
    (numEntry === (this.NumSdiqEntry - 3).U) && numEntryCreate === 2.U && numEntryPop === 0.U ||
    (numEntry === (this.NumSdiqEntry - 2).U) && numEntryCreate === 1.U && numEntryPop === 0.U ||
    (numEntry === (this.NumSdiqEntry - 2).U) && numEntryCreate === 2.U && numEntryPop === 1.U ||
    (numEntry === (this.NumSdiqEntry - 1).U) && numEntryCreate === 1.U && numEntryPop === 1.U ||
    (numEntry === (this.NumSdiqEntry - 1).U) && numEntryCreate === 0.U && numEntryPop === 0.U ||
    (numEntry === (this.NumSdiqEntry - 0).U) && numEntryCreate === 0.U && numEntryPop === 1.U

  io.out.ctrl.fullUpdate    := fullUpdate
  io.out.ctrl.oneLeftUpdate := oneLeftUpdate

  //-------------------sdiq create pointer--------------------
  //create0 priority is from entry 0 to 11
  private val entryCreate0OHVec : Vec[Bool] = VecInit(PriorityEncoderOH(entryOutValidVec.map(!_)))
  //create1 priority is from entry 11 to 0
  private val entryCreate1OHVec : Vec[Bool] = VecInit(PriorityEncoderOH(entryOutValidVec.reverse.map(!_)).reverse)

  io.out.data.createEntry(0) := entryCreate0OHVec.asUInt
  io.out.data.createEntry(1) := entryCreate1OHVec.asUInt

  io.out.ctrl.empty := !entryOutValidVec.reduce(_ || _)

  entryCreateEnVec := VecInit((
    Mux(ctrl.createEnVec(0), entryCreate0OHVec.asUInt, 0.U) |
    Mux(ctrl.createEnVec(1), entryCreate1OHVec.asUInt, 0.U)).asBools)

  //issue queue entry create data path control
  entryCreateDpEnVec := VecInit((
    Mux(ctrl.createDpEnVec(0), entryCreate0OHVec.asUInt, 0.U) |
    Mux(ctrl.createDpEnVec(1), entryCreate1OHVec.asUInt, 0.U)).asBools)

  //issue queue entry create gateclk control
  //ignore bypass signal for timing optimization
  entryCreateGateClkVec := VecInit((
    Mux(ctrl.createGateClkVec(0), entryCreate0OHVec.asUInt, 0.U) |
    Mux(ctrl.createGateClkVec(1), entryCreate1OHVec.asUInt, 0.U)).asBools)

  //-------------------prepare create signals-----------------
  //create 0 age vectors:
  //1.existed entries
  for (i <- 0 until this.NumSdiqEntry) {
    entryPortCreateAgeVec(0)(i) :=
      entryOutValidVec(i) && !(lsu.ex1.popValid && lsu.ex1.sdiqEntryVec(i))
  }

  //create 1 age vectors:
  //1.existed entries
  //2.create 0 entry
  for (i <- 0 until this.NumSdiqEntry) {
    entryPortCreateAgeVec(1)(i) :=
      (entryOutValidVec(i) && !(lsu.ex1.popValid && lsu.ex1.sdiqEntryVec(i))) ||
      entryCreate0OHVec(i)
  }

  //create 0/1 select:
  //entry 0~5 use ~sdiq_entry_create0_in for better timing
  //entry 6~11 use sdiq_entry_create1_in for better timing
  //sdiq_entry_create0/1_in cannot be both 1,
  //if both 0, do not create
  private val entryCreateSel = Wire(Vec(this.NumSdiqEntry, Bool()))
  for (i <- 0 until this.NumSdiqEntry / 2) {
    entryCreateSel(i) := ~(ctrl.createDpEnVec(0) && entryCreate0OHVec(i))
  }
  for (i <- this.NumSdiqEntry / 2 until this.NumSdiqEntry) {
    entryCreateSel(i) := ctrl.createDpEnVec(1) && entryCreate1OHVec(i)
  }

  //----------------entry flop create signals----------------

  for (i <- 0 until this.NumSdiqEntry) {
    when (!entryCreateSel(i)) {
      entryCreateAgeVec(i)  := VecInit(DropBit(entryPortCreateAgeVec(0), i).asBools)
      entryCreateDataVec(i) := data.createData(0)
    }.otherwise {
      entryCreateAgeVec(i)  := VecInit(DropBit(entryPortCreateAgeVec(1), i).asBools)
      entryCreateDataVec(i) := data.createData(1)
    }
  }

  //==========================================================
  //             LSU Issue Queue Issue Control
  //==========================================================
  //----------------Pipe0 Launch Ready Signals----------------

  //------------------issue prepare signals-------------------

  //----------------older entry ready signals-----------------
  private val entryOutAgeVec = Wire(Vec(this.NumSdiqEntry, Vec(this.NumSdiqEntry - 1, Bool())))
  private val entryOutReadyVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryOutValidWithFreezeVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryOutReadDataVec = Wire(Vec(this.NumSdiqEntry, new SdiqEntryData))
  private val entryOutPregOHVec = Wire(Vec(this.NumSdiqEntry, UInt(NumPhysicRegs.W)))

  private val olderEntryReadyVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  for (i <- 0 until this.NumSdiqEntry) {
    olderEntryReadyVec(i) := (entryOutAgeVec(i).asUInt & DropBit(entryOutReadyVec, i)).orR
  }

  //-----------entry issue enable signals for entries---------
  //not ready if older ready exists
  private val entryIssueEnVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  for (i <- 0 until this.NumSdiqEntry) {
    entryIssueEnVec(i) := entryOutReadyVec(i) && !olderEntryReadyVec(i)
  }

  //---------------entry issue signals for rf pipes-----------
  //issue enable signals:
  //if bypass or ready entry exists
  io.out.issueEn := entryOutReadyVec.reduce(_ || _)
  //gate clock issue enable with timing optimization
  io.out.gateClkIssueEn := entryOutValidWithFreezeVec.reduce(_ || _)

  //-----------------issue entry indiction--------------------
  io.out.data.issueEntry := entryIssueEnVec.asUInt

  //-----------------issue data path selection----------------
  //issue data path will select oldest ready entry in issue queue
  //if no instruction valid, the data path will always select bypass
  //data path

  private val entryReadData = entryOutReadDataVec(OHToUInt(entryIssueEnVec))

  io.out.data.issueReadData := entryReadData

  //==========================================================
  //            LSU Issue Queue Launch Control
  //==========================================================
  //-------------------entry pop enable signals---------------
  //pop when rf launch pass
  private val entryPopCurEntryVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryPopOtherEntryVec = Wire(Vec(this.NumSdiqEntry, Vec(this.NumSdiqEntry - 1, Bool())))
  entryPopCurEntryVec := lsu.ex1.sdiqEntryVec
  entryPopOtherEntryVec.zipWithIndex.foreach {
    case (popOther, i) => popOther := VecInit(DropBit(lsu.ex1.sdiqEntryVec, i).asBools)
  }

  //-------------------entry spec fail signals---------------
  //clear freeze and source rdy when launch fail
  private val entryFreezeClearVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  private val entryEx1FreezeClearVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  for (i <- 0 until this.NumSdiqEntry) {
    entryFreezeClearVec(i) := ctrl.fromRf.launchFailValid && data.fromRf.launchEntry(i)
  }
  for (i <- 0 until this.NumSdiqEntry) {
    entryEx1FreezeClearVec(i) := lsu.ex1.freezeClear && lsu.ex1.sdiqEntryVec(i)
  }

  //----------------rf stage ready set signals---------------
  //store address inst 0/1 set its staddr0/1_rdy at pipe4 RF stage
  private val entryStAddrReadySetVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  for (i <- 0 until this.NumSdiqEntry) {
    entryStAddrReadySetVec(i) := ctrl.fromRf.stAddrReadySet && data.fromRf.launchEntry(i)
  }

  //------------dc stage store queue create signals----------
  //store address inst 0/1 set its staddr0/1_stq_create at pipe4 DC stage
  private val entryStAddrStqCreateVec = Wire(Vec(this.NumSdiqEntry, Bool()))
  for (i <- 0 until this.NumSdiqEntry) {
    entryStAddrStqCreateVec(i) := lsu.dc.stAddrValid && lsu.dc.sdiqEntryVec(i)
  }
  //==========================================================
  //            LSU Issue Queue Create Control
  //==========================================================
  //-------------------gated cell instance--------------------
  private val srcRegMaskUpdateValidFF = RegInit(false.B)
  private val srcRegMaskUpdateValid = WireInit(false.B)

  private val srcMaskClkEn = srcRegMaskUpdateValidFF || srcRegMaskUpdateValid
  //  Todo: gated clk for src mask


  //--------------------sdiq src preg mask--------------------
  when (rtu.flush.be) {
    srcRegMaskUpdateValidFF := false.B
  }.elsewhen(srcRegMaskUpdateValid) {
    srcRegMaskUpdateValidFF := true.B
  }.otherwise {
    srcRegMaskUpdateValidFF := false.B
  }

  private val src0PregDeallocMaskUpdate = Wire(UInt(NumPhysicRegs.W))
  src0PregDeallocMaskUpdate := entryOutPregOHVec.reduce(_|_)
  private val src0PregDeallocMask = RegInit(0.U(NumPhysicRegs.W))

  when (rtu.flush.be) {
    src0PregDeallocMask := 0.U.asTypeOf(chiselTypeOf(src0PregDeallocMask))
  }.elsewhen(srcRegMaskUpdateValidFF) {
    src0PregDeallocMask := src0PregDeallocMaskUpdate
  }

  io.out.toRtu.pregDeallocMask := VecInit(src0PregDeallocMask.asBools)
  // Todo: freg, vreg

  //==========================================================
  //             LSU Issue Queue Entry Instance
  //==========================================================
  private val entries = Seq.fill(this.NumSdiqEntry)(Module(new SdiqEntry))
  entries.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fromCp0 := io.in.fromCp0
      in.fwdValid(0).alu := io.in.fwd.aluValid(i)(0)  // ith entry, 0th src
      in.fwdValid(0).load(0):= io.in.fwd.loadValid

      in.fuDstPreg := io.in.fuDstPreg // all fu dst
      in.wbPreg := io.in.wbPreg       // all wb
      in.loadPreg := io.in.loadPreg
      in.popValid := lsu.ex1.popValid
      in.readyClr := data.fromRf.readyClear // ready clear from rf, but in other iq it from pipe
      in.fromRf.stAddr1Valid := data.fromRf.stAddr1Valid
      in.fromRf.stAddrReadyClear := data.fromRf.stAddrReadyClear
      in.fromRf.stData1Valid := data.fromRf.stData1Valid
      in.fromRf.freezeClr := entryFreezeClearVec(i) // ct_idu_is_sdiq.v:1394
      in.issueEn := entryIssueEnVec(i)
      in.popCurEntry := entryPopCurEntryVec(i)
      in.popOtherEntry := entryPopOtherEntryVec(i)
      in.fromLsu.stAddr1Valid := lsu.dc.stAddr1Valid
      in.fromLsu.stAddrUnalign:= lsu.dc.stAddrUnalign
      in.fromRtu.flush := rtu.flush
      in.fromPad := io.in.fromPad

      in.create.ageVec  := entryCreateAgeVec(i)
      in.create.data    := entryCreateDataVec(i)
      in.create.dpEn    := entryCreateDpEnVec(i)
      in.create.en      := entryCreateEnVec(i)
      in.create.gateClkEn := entryCreateGateClkVec(i)
      in.vFwdValid  := DontCare
      in.vWbPreg    := DontCare
      in.vLoadPreg  := DontCare
      in.vFuDstPreg := DontCare

      in.stAddrReadySet := entryStAddrReadySetVec(i)
      in.stAddrStqCreate := entryStAddrStqCreateVec(i)
      in.freezeClrEx1 := entryEx1FreezeClearVec(i)


      entryOutAgeVec(i) := out.ageVec
      entryOutReadyVec(i) := out.ready
      entryOutReadDataVec(i) := out.readData
      entryOutValidVec(i) := out.valid
      entryOutValidWithFreezeVec(i) := out.validWithoutFreeze // no problem
      entryOutPregOHVec(i) := out.src0PregOH
  }

}


