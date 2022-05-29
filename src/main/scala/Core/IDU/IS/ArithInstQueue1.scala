package Core.IDU.IS

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._
import Core.IntConfig._
import Utils.Bits._


class ArithInstQueue1 extends Module with AiqConfig {
  val io : ArithInstQueueIO = IO(new ArithInstQueueIO)

  private val entryValidVec : Vec[Bool] = Wire(Vec(NumAiqEntry, Bool()))
  private val entries = Seq.fill(NumAiqEntry)(Module(new AiqEntry))

  // find two empty entry
  private val enq0OH : Vec[Bool] = VecInit(PriorityEncoderOH(entryValidVec.map(!_)))
  private val enq1OH : Vec[Bool] = VecInit(PriorityEncoderOH(entryValidVec.reverse.map(!_)))
  io.out.entryEnqOHVec(0) := enq0OH.asUInt
  io.out.entryEnqOHVec(1) := enq1OH.asUInt

  private val rtu = io.in.fromRtu
  private val iu  = io.in.fromIu
  private val lsu = io.in.fromLsu

  private val ctrlAiq0  = io.in.ctrl.createVec(0)
  private val ctrlAiq1  = io.in.ctrl.createVec(1)
  private val ctrlBiq   = io.in.ctrl.createVec(2)
  private val ctrlLsiq  = io.in.ctrl.createVec(3)
  private val ctrlSdiq  = io.in.ctrl.createVec(4)
  private val dataAiq1  = io.in.data

  /**
   * Regs
   */

  // Todo: imm
  private val entryCnt = RegInit(0.U(4.W))

  /**
   * Wires
   */

  private val entryCreateEnVec = Wire(Vec(NumAiqEntry, Bool()))
  private val entryCreateDpEnVec = Wire(Vec(NumAiqEntry, Bool()))
  private val entryCreateGateClkVec = Wire(Vec(NumAiqEntry, Bool()))
  private val entryCreateAgeVec = Wire(Vec(NumAiqEntry, Vec(NumAiqEntry-1, Bool())))
  private val entryCreateDataVec = Wire(Vec(NumAiqEntry, Output(new AiqEntryData)))
  private val entryCreateFrzVec = Wire(Vec(NumAiqEntry, Bool()))

  //==========================================================
  //            ALU Issue Queue Create Control
  //==========================================================
  //----------------------------------------------------------
  //                   gated clk for cnt
  //----------------------------------------------------------
  private val cntClkEn = entryCnt =/= 0.U ||
    ctrlAiq1.createGateClkEn(0) || ctrlAiq1.createGateClkEn(1)
  // Todo: gated clk for cnt clk

  io.out.ctrl.fullUpdateClkEn := cntClkEn

  //----------------------------------------------------------
  //                   aiq0 entry counter
  //----------------------------------------------------------
  private val entryCntCreate = ctrlAiq1.createEn.count(b => b)
  private val entryCntPop = ctrlAiq1.rfPopValid.asUInt
  private val entryCntUpdateValid = ctrlAiq1.createEn(0) || ctrlAiq1.rfPopValid
  private val entryCntUpdate = entryCnt + entryCntCreate - entryCntPop

  //after flush fe/is, the rf may wrongly pop before rtu_yy_xx_flush // Todo: figure out
  //need flush also when rtu_yy_xx_flush
  when (rtu.flushFe || rtu.flushIs || rtu.yyXXFlush) {
    entryCnt := 0.U
  }.elsewhen(entryCntUpdateValid) {
    entryCnt := entryCntUpdate
  }

  private val full = entryCnt === entries.length.U
  private val empty = !entryValidVec.asUInt.orR
  io.out.ctrl.full := full
  io.out.ctrl.empty := empty

  //---------aiq0 entry counter update value for dlb---------
  // Todo: figure out
  io.out.ctrl.entryCntUpdate.valid := ctrlAiq1.createEn(0) || ctrlAiq1.rfPopDlbValid
  io.out.ctrl.entryCntUpdate.bits  := entryCnt + entryCntCreate - ctrlAiq1.rfPopDlbValid.asUInt

  //--------------------aiq0 entry full-----------------------
  // Todo: check
  private val fullUpdate = entryCntUpdate === entries.length.U
  private val oneLeftUpdate = entryCntUpdate === (entries.length - 1).U
  io.out.ctrl.fullUpdate := fullUpdate
  io.out.ctrl.oneLeftUpdate := oneLeftUpdate

  //---------------------create bypass------------------------
  /**
   *  if create instruction is ready, it may bypass from issue queue
   *  only create 0 can bypass
   */

  // Todo: figure out if this create way is used by split instruction

  private val createReadyBypassMask = !io.in.fromCp0.iqBypassDisable &&
    !ctrlAiq1.stall &&
    !(dataAiq1.createDiv && iu.div.busy) &&
    dataAiq1.srcReadyForBypass.asUInt.andR

  private val create0ReadyBypass = ctrlAiq1.createEn(0) && createReadyBypassMask
  private val create0ReadyBypassDp = ctrlAiq1.createDpEn(0) && createReadyBypassMask
  private val create0ReadyBypassGateClk = ctrlAiq1.createGateClkEn(0) && dataAiq1.srcReadyForBypass.asUInt.andR

  private val entryValidWithFrzVec = Wire(Vec(NumAiqEntry, Bool()))
  private val createBypassEmpty = !entryValidWithFrzVec.asUInt.orR

  // Todo: figure out
  private val bypassEn = createBypassEmpty && create0ReadyBypass
  private val bypassDpEn = createBypassEmpty && create0ReadyBypassDp
  private val bypassGateClkEn = createBypassEmpty && create0ReadyBypassGateClk

  entryCreateEnVec := VecInit(
    ((Fill(NumAiqEntry, ctrlAiq1.createEn(0)) & enq0OH.asUInt)
      | (Fill(NumAiqEntry, ctrlAiq1.createEn(1)) & enq1OH.asUInt)).asBools
  )

  entryCreateDpEnVec := VecInit(
    ((Fill(NumAiqEntry, ctrlAiq1.createDpEn(0)) & enq0OH.asUInt)
      | (Fill(NumAiqEntry, ctrlAiq1.createDpEn(1)) & enq1OH.asUInt)).asBools
  )

  entryCreateGateClkVec := VecInit(
    ((Fill(NumAiqEntry, ctrlAiq1.createGateClkEn(0)) & enq0OH.asUInt)
      | (Fill(NumAiqEntry, ctrlAiq1.createGateClkEn(1)) & enq1OH.asUInt)).asBools
  )

  //aiq create entry should consider pop signal and create0
  private val entryCreatePortAgeVec = Wire(Vec(NumAiqCreatePort, Vec(NumAiqEntry, Bool())))
  entryCreatePortAgeVec(0) := VecInit((entryValidVec.asUInt &
    ~(Fill(NumAiqEntry, ctrlAiq1.rfPopValid) & dataAiq1.rfLaunchEntry.asUInt)).asBools)
  entryCreatePortAgeVec(1) := VecInit((entryValidVec.asUInt &
    ~(Fill(NumAiqEntry, ctrlAiq1.rfPopValid) & dataAiq1.rfLaunchEntry.asUInt) |
    enq0OH.asUInt).asBools)

  //create 0/1 select:
  //entry 0~3 use ~aiq0_entry_create0_in for better timing
  //entry 4~7 use aiq0_entry_create1_in for better timing
  //aiq0_entry_create0/1_in cannot be both 1,
  //if both 0, do not create

  // entryCreateSel(i) === 1.U means entryCreate(1) will be used to create entry
  // Todo: timing optimization
  private val entryCreateSel : UInt = Fill(this.NumAiqEntry, ctrlAiq1.createEn(1)) &
    enq1OH.asUInt

  for (i <- 0 until this.NumAiqEntry) {
    when(entryCreateSel(i) === 0.U) {
      entryCreateFrzVec(i) := bypassDpEn
      entryCreateAgeVec(i) := VecInit(DropBit(entryCreatePortAgeVec(0).asUInt, i).asBools)
      entryCreateDataVec(i) := dataAiq1.createData(0)
    }.otherwise {
      entryCreateFrzVec(i) := 0.U
      entryCreateAgeVec(i) := VecInit(DropBit(entryCreatePortAgeVec(1).asUInt, i).asBools)
      entryCreateDataVec(i) := dataAiq1.createData(1)
    }
  }

  //==========================================================
  //             ALU Issue Queue 0 Issue Control
  //==========================================================
  //----------------Pipe0 Launch Ready Signals----------------

  private val entryAluRegFwdValid = Wire(Vec(NumAlu, Vec(this.NumAiqEntry, UInt(NumSrcArith.W))))

  entryAluRegFwdValid := ctrlAiq1.rfAluRegFwdValid

  //-------------------issue enable signals-------------------
  private val entryReadyVec = Wire(Vec(this.NumAiqEntry, Bool()))

  //if there is any entry ready or bypass enable, issue enable
  io.out.xxIssueEn := bypassEn || entryReadyVec.asUInt.orR

  io.out.xxGateClkIssueEn := bypassGateClkEn || entryValidWithFrzVec.asUInt.orR

  // find older ready entry
  private val entryAgeVec = Wire(Vec(this.NumAiqEntry, UInt((this.NumAiqEntry - 1).W)))
  private val olderEntryReadyVec = Wire(Vec(this.NumAiqEntry, Bool()))
  for (i <- 0 until this.NumAiqEntry) {
    olderEntryReadyVec(i) := entryAgeVec(i) & DropBit(entryReadyVec.asUInt, i)
  }

  //------------------entry issue enable signals--------------
  private val entryIssueEnVec = Wire(Vec(this.NumAiqEntry, Bool()))
  entryIssueEnVec := VecInit((entryReadyVec.asUInt & ~olderEntryReadyVec.asUInt).asBools)

  //-----------------issue entry indiction--------------------
  // Todo: figure out
  io.out.data.issueEntryVec := Mux(
    createBypassEmpty,
    enq0OH,
    entryIssueEnVec
  )

  //-----------------issue data path selection----------------
  //issue data path will select oldest ready entry in issue queue
  //if no instruction valid, the data path will always select bypass
  //data path
  private val entryReadData = Wire(Output(new AiqEntryData))
  private val entryReadDataVec = Wire(Vec(this.NumAiqEntry, Output(new AiqEntryData)))
  entryReadData := entryReadDataVec(OHToUInt(entryIssueEnVec))

  //if no entry valid, select bypass path
  // Todo: figure out why select bypass data when createBypassEmpty
  io.out.data.issueData := Mux(
    createBypassEmpty,
    dataAiq1.bypassData,
    entryReadData
  )

  //==========================================================
  //            ALU Issue Queue 0 Launch Control
  //==========================================================

  //-------------------entry pop enable signals---------------
  //pop when rf launch pass
  private val entryPopCurEntry = Wire(Vec(this.NumAiqEntry, Bool()))
  private val entryPopOtherEntry = Wire(Vec(this.NumAiqEntry, Vec(this.NumAiqEntry - 1, Bool())))
  entryPopCurEntry := dataAiq1.rfLaunchEntry
  entryPopOtherEntry.zipWithIndex.foreach {
    case (value, i) => value := VecInit(DropBit(dataAiq1.rfLaunchEntry.asUInt, i).asBools)
  }

  //-------------------entry frz clr signals-----------------
  //clear freeze and source rdy when launch fail
  private val entryFreezeClearVec = Wire(Vec(this.NumAiqEntry, Bool()))
  entryFreezeClearVec := Mux(ctrlAiq1.rfLaunchFailValid,
    dataAiq1.rfLaunchEntry,
    0.U.asTypeOf(chiselTypeOf(entryFreezeClearVec))
  )

  //==========================================================
  //             ALU Issue Queue 0 Entry Instance
  //==========================================================

  entries.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fromCp0  := io.in.fromCp0
      in.fwdValid := VecInit(
        entryAluRegFwdValid(0)(i),
        entryAluRegFwdValid(1)(i),
        lsu.load.dcFwdInstValid
      ).asTypeOf(chiselTypeOf(in.fwdValid))
      in.fuDstPreg  := io.in.fuResultDstPreg
      in.wbPreg     := io.in.wbPreg
      in.loadPreg   := io.in.loadPreg
      in.rfPopValid := ctrlAiq1.rfPopValid
      in.rfReadyClr := dataAiq1.rfReadyClr
      in.stall      := ctrlAiq1.stall
      in.fromRtu.flush.is   := rtu.flushIs
      in.fromRtu.flush.fe   := rtu.flushFe
      in.create.ageVec    := entryCreateAgeVec(i)
      in.create.data      := entryCreateDataVec(i)
      in.create.dpEn      := entryCreateDpEnVec(i)
      in.create.en        := entryCreateEnVec(i)
      in.create.freeze    := entryCreateFrzVec(i)
      in.create.gateClkEn := entryCreateGateClkVec(i)
      in.freezeClr        := entryFreezeClearVec(i)
      in.issueEn          := entryIssueEnVec(i)
      in.popCurEntry      := entryPopCurEntry(i)
      in.popOtherEntry    := entryPopOtherEntry(i)
      in.divBusy          := iu.div.busy
      entryAgeVec(i)      := out.ageVec.asUInt
      entryReadyVec(i)    := out.ready
      entryReadDataVec(i) := out.readData
      entryValidVec(i)    := out.valid
      entryValidWithFrzVec(i) := out.validWithoutFreeze
  }


}
