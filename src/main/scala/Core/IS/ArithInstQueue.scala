package Core.IS

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._
import Core.IntConfig._

trait AiqConfig {
  def NumAiqEntry = 8
  def NumBiqEntry = 12 // Todo: move
  def NumFwdPath = 2
  def NumSrc = 3
  def NumAiqCreatePort = 2
}

class InstQueFromCp0 extends Bundle {
  val icgEn           : Bool = Bool()
  val iqBypassDisable : Bool = Bool()
  val yyClkEn         : Bool = Bool()
}

class InstQueCtrlInput(numEntry: Int) extends Bundle with AiqConfig{
  val createDpEn : Vec[Bool] = Vec(NumAiqCreatePort, Bool())
  val createEn : Vec[Bool] = Vec(NumAiqCreatePort, Bool())
  val createGateClkEn : Vec[Bool] = Vec(NumAiqCreatePort, Bool())
  val createSel : UInt = UInt(NumAiqCreatePort.W)
  // Todo: figure out
  val rfLaunchFailValid : Bool = Bool()
  val rfAluRegFwdValid : Vec[Vec[UInt]] = Vec(NumAlu, Vec(numEntry, UInt(NumSrc.W)))
  val rfPopDlbValid : Bool = Bool()
  val rfPopValid : Bool = Bool()
  val stall : Bool = Bool()
}

class InstQueDataInput(numEntry: Int) extends Bundle with AiqConfig {
  val bypassData : AiqEntryData = Output(new AiqEntryData)
  val createData : Vec[AiqEntryData] = Vec(NumAiqCreatePort, new AiqEntryData)
  val createDiv : Bool = Bool()
  val srcReadyForBypass : Vec[Bool] = Vec(NumSrc, Bool())
  val rfLaunchEntry : Vec[Bool] = Vec(numEntry, Bool())
  val rfReadyClr : Vec[Bool] = Vec(NumSrc, Bool())
  // dispatch num: 4, src num: 3
  val dispatchInstSrcPregVec : Vec[Vec[UInt]] = Vec(NumCreateEntry, Vec(NumSrc, UInt(NumPhysicRegsBits.W)))
  val sdiqCreateSrcSelVec : Vec[Bool] = Vec(NumAiqCreatePort, Bool())
  // Todo: more bundles
}

class InstQueCtrlOutput extends Bundle with AiqConfig {
  // Todo: figure out
  val oneLeftUpdate : Bool = Bool()
  val empty : Bool = Bool()
  val full : Bool = Bool()
  val fullUpdate : Bool = Bool()
  val fullUpdateClkEn : Bool = Bool()
  // Todo: check width
  val entryCntUpdate = ValidIO(UInt(log2Up(NumAiqEntry).W))
}

class InstQueDataOutput extends Bundle with AiqConfig {
  val issueEntryVec : Vec[Bool] = Output(Vec(NumAiqEntry, Bool()))
  val issueData : AiqEntryData = Output(new AiqEntryData)
}

class ArithInstQueueInput extends Bundle with AiqConfig with DepRegEntryConfig {
  val aiqEntryCreateVec = Vec(2, UInt(NumAiqEntry.W))
  val biqEntryCreateVec = Vec(2, UInt(NumBiqEntry.W))
  val fromCp0 = new InstQueFromCp0
  val ctrl = new Bundle {
    // 0, 1: arith, 2: biq, 3: lsiq, 4: sdiq
    val createVec : Vec[InstQueCtrlInput] = Vec(5, new InstQueCtrlInput(NumAiqEntry))
    // Todo: figure out
    val xxRfPipe0PregLaunchValidDupx : Bool = Bool()
    val xxRfPipe1PregLaunchValidDupx : Bool = Bool()
  }
  val data : InstQueDataInput = Flipped(Output(new InstQueDataInput(NumAiqEntry)))
  val fromRtu = new Bundle {
    val flushFe : Bool = Bool()
    val flushIs : Bool = Bool()
    val yyXXFlush : Bool = Bool()
  }
  val fromIu = new Bundle {
    val div = new Bundle {
      val busy : Bool = Bool()
    }
  }
  val fromLsu = new Bundle {
    val load = new Bundle {
      val dcFwdInstValid : Bool = Bool()
    }
  }

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
  val fuResultDstPreg : Vec[ValidIO[UInt]] = Vec(NumFuHasDstReg, ValidIO(UInt(NumPhysicRegsBits.W)))
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
  val loadPreg = ValidIO(UInt(NumPhysicRegsBits.W)) // Todo: Rename
}

class ArithInstQueueOutput extends Bundle with AiqConfig {
  val entryEnqOHVec : Vec[UInt] = Vec(NumAiqCreatePort, UInt(NumAiqEntry.W))
  val ctrl = new InstQueCtrlOutput
  val data = new InstQueDataOutput
  val xxGateClkIssueEn : Bool = Bool()
  val xxIssueEn : Bool = Bool()
}

class ArithInstQueueIO extends Bundle {
  val in : ArithInstQueueInput = Flipped(Output(new ArithInstQueueInput))
  val out : ArithInstQueueOutput = Output(new ArithInstQueueOutput)
}

class ArithInstQueue extends Module with AiqConfig {
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
  private val dataAiq0  = io.in.data

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
    ctrlAiq0.createGateClkEn(0) || ctrlAiq0.createGateClkEn(1)
  // Todo: gated clk for cnt clk

  io.out.ctrl.fullUpdateClkEn := cntClkEn

  //----------------------------------------------------------
  //                   aiq0 entry counter
  //----------------------------------------------------------
  private val entryCntCreate = ctrlAiq0.createEn.count(b => b)
  private val entryCntPop = ctrlAiq0.rfPopValid.asUInt
  private val entryCntUpdateValid = ctrlAiq0.createEn(0) || ctrlAiq0.rfPopValid
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
  io.out.ctrl.entryCntUpdate.valid := ctrlAiq0.createEn(0) || ctrlAiq0.rfPopDlbValid
  io.out.ctrl.entryCntUpdate.bits  := entryCnt + entryCntCreate - ctrlAiq0.rfPopDlbValid.asUInt

  //--------------------aiq0 entry full-----------------------
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
    !ctrlAiq0.stall &&
    !(dataAiq0.createDiv && iu.div.busy) &&
    dataAiq0.srcReadyForBypass.asUInt.andR

  private val create0ReadyBypass = ctrlAiq0.createEn(0) && createReadyBypassMask
  private val create0ReadyBypassDp = ctrlAiq0.createDpEn(0) && createReadyBypassMask
  private val create0ReadyBypassGateClk = ctrlAiq0.createGateClkEn(0) && dataAiq0.srcReadyForBypass.asUInt.andR

  private val entryValidWithFrzVec = Wire(Vec(NumAiqEntry, Bool()))
  private val createBypassEmpty = !entryValidWithFrzVec.asUInt.orR

  // Todo: figure out
  private val bypassEn = createBypassEmpty && create0ReadyBypass
  private val bypassDpEn = createBypassEmpty && create0ReadyBypassDp
  private val bypassGateClkEn = createBypassEmpty && create0ReadyBypassGateClk

  entryCreateEnVec := VecInit(
    ((Fill(NumAiqEntry, ctrlAiq0.createEn(0)) & enq0OH.asUInt)
      | (Fill(NumAiqEntry, ctrlAiq0.createEn(1)) & enq1OH.asUInt)).asBools
  )

  entryCreateDpEnVec := VecInit(
    ((Fill(NumAiqEntry, ctrlAiq0.createDpEn(0)) & enq0OH.asUInt)
      | (Fill(NumAiqEntry, ctrlAiq0.createDpEn(1)) & enq1OH.asUInt)).asBools
  )

  entryCreateGateClkVec := VecInit(
    ((Fill(NumAiqEntry, ctrlAiq0.createGateClkEn(0)) & enq0OH.asUInt)
      | (Fill(NumAiqEntry, ctrlAiq0.createGateClkEn(1)) & enq1OH.asUInt)).asBools
  )

  //aiq create entry should consider pop signal and create0
  private val entryCreatePortAgeVec = Wire(Vec(NumAiqCreatePort, Vec(NumAiqEntry, Bool())))
  entryCreatePortAgeVec(0) := VecInit((entryValidVec.asUInt &
    ~(Fill(NumAiqEntry, ctrlAiq0.rfPopValid) & dataAiq0.rfLaunchEntry.asUInt)).asBools)
  entryCreatePortAgeVec(1) := VecInit((entryValidVec.asUInt &
    ~(Fill(NumAiqEntry, ctrlAiq0.rfPopValid) & dataAiq0.rfLaunchEntry.asUInt) |
    enq0OH.asUInt).asBools)

  //create 0/1 select:
  //entry 0~3 use ~aiq0_entry_create0_in for better timing
  //entry 4~7 use aiq0_entry_create1_in for better timing
  //aiq0_entry_create0/1_in cannot be both 1,
  //if both 0, do not create

  // entryCreateSel(i) === 1.U means entryCreate(1) will be used to create entry
  // Todo: timing optimization
  private val entryCreateSel : UInt = Fill(this.NumAiqEntry, ctrlAiq0.createEn(1)) &
    enq1OH.asUInt

  def DropBit(src: UInt, idx: Int): UInt = {
    val width = src.getWidth
    if (idx == 0)
      src(width-1, 1)
    else if(idx == width - 1)
      src(width-2, 0)
    else if(idx > 0 && idx < width - 1)
      Cat(src(width-1, idx+1), src(idx-1, 0))
    else {
      assert(cond = false, "idx out of width of src")
      0.U
    }
  }

  for (i <- 0 until this.NumAiqEntry) {
    when(entryCreateSel(i) === 0.U) {
      entryCreateFrzVec(i) := bypassDpEn
      entryCreateAgeVec(i) := VecInit(DropBit(entryCreatePortAgeVec(0).asUInt, i).asBools)
      entryCreateDataVec(i) := dataAiq0.createData(0)
    }.otherwise {
      entryCreateFrzVec(i) := 0.U
      entryCreateAgeVec(i) := VecInit(DropBit(entryCreatePortAgeVec(1).asUInt, i).asBools)
      entryCreateDataVec(i) := dataAiq0.createData(1)
    }
  }

  //==========================================================
  //             ALU Issue Queue 0 Issue Control
  //==========================================================
  //----------------Pipe0 Launch Ready Signals----------------

  private val entryAluRegFwdValid = Wire(Vec(NumAlu, Vec(this.NumAiqEntry, UInt(NumSrc.W))))

  entryAluRegFwdValid := ctrlAiq0.rfAluRegFwdValid
//  for (idxAlu <- 0 to NumAlu) {
//    for (idxEntry <- 0 to this.NumAiqEntry) {
//      entryAluRegFwdValid(idxAlu)(idxEntry) := ctrlAiq0.rfAluRegFwdValid(idxAlu)(idxEntry)
//    }
//  }

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

  // Todo: typo
  //-----------------issue entry indiction--------------------
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
    dataAiq0.bypassData,
    entryReadData
  )

  //==========================================================
  //            ALU Issue Queue 0 Launch Control
  //==========================================================

  //-------------------entry pop enable signals---------------
  //pop when rf launch pass
  private val entryPopCurEntry = Wire(Vec(this.NumAiqEntry, Bool()))
  private val entryPopOtherEntry = Wire(Vec(this.NumAiqEntry, Vec(this.NumAiqEntry - 1, Bool())))
  entryPopCurEntry := dataAiq0.rfLaunchEntry
  entryPopOtherEntry.zipWithIndex.foreach {
    case (value, i) => value := VecInit(DropBit(dataAiq0.rfLaunchEntry.asUInt, i).asBools)
  }

  //-------------------entry frz clr signals-----------------
  //clear freeze and source rdy when launch fail
  private val entryFreezeClearVec = Wire(Vec(this.NumAiqEntry, Bool()))
  entryFreezeClearVec := Mux(ctrlAiq0.rfLaunchFailValid,
    dataAiq0.rfLaunchEntry,
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
      in.rfPopValid := ctrlAiq0.rfPopValid
      in.rfReadyClr := dataAiq0.rfReadyClr
      in.stall      := ctrlAiq0.stall
      in.flush.is   := rtu.flushIs
      in.flush.fe   := rtu.flushFe
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
