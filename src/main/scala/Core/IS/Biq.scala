package Core.IS

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._
import Core.IntConfig._
import Utils.Bits.DropBit

class InstQueFromPad extends Bundle {
  val yyIcgScanEn : Bool = Bool()
}

class InstQueFromRtu extends Bundle {
  val flushFe : Bool = Bool()
  val flushIs : Bool = Bool()
  val yyXXFlush : Bool = Bool()
}

class BiqCtrlInput extends Bundle
  with BiqConfig
  with DepRegEntryConfig {
  val createDpEn : Vec[Bool] = Vec(NumBiqCreatePort, Bool())
  val createEn : Vec[Bool] = Vec(NumBiqCreatePort, Bool())
  val gateClkEn : Vec[Bool] = Vec(NumBiqCreatePort, Bool())
  val rfLaunchFailValid : Bool = Bool()
  val rfAluRegFwdValid : Vec[Vec[UInt]] = Vec(NumAlu, Vec(NumBiqEntry, UInt(NumSrcBr.W)))
  val rfPopValid : Bool = Bool()
}

class BiqDataInput extends Bundle
  with BiqConfig
  with DepRegEntryConfig {
  val bypassData : BiqEntryData = Output(new BiqEntryData)
  val createData : Vec[BiqEntryData] = Vec(NumBiqCreatePort, new BiqEntryData)
  val srcReadyForBypass : Vec[Bool] = Vec(NumSrcBr, Bool())
  val rfLaunchEntry : Vec[Bool] = Vec(NumBiqEntry, Bool())
  val rfReadyClr : Vec[Bool] = Vec(NumSrcBr, Bool())
}

class BiqInput extends Bundle
  with BiqConfig
  with DepRegEntryConfig {
  val fromCp0 = new IqFromCp0
  val ctrl = new BiqCtrlInput
  val data = new BiqDataInput
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

  val fromIu = new Bundle {
    val div = new Bundle {
      val instValid : Bool = Bool()
      val preg : UInt = UInt(NumPhysicRegsBits.W)
    }
  }
  val fromLsu = new Bundle {
    val load = new Bundle {
      val dcFwdInstValid : Bool = Bool()
    }
  }
  val fromPad = new InstQueFromPad
  val fromRtu = new InstQueFromRtu
}

class BiqCtrlOutput extends Bundle with BiqConfig {
  val oneLeftUpdate : Bool = Bool()
  val empty : Bool = Bool()
  val full : Bool = Bool()
  val fullUpdate : Bool = Bool()
  val fullUpdateClkEn : Bool = Bool()
}

class BiqDataOutput extends Bundle with BiqConfig {
  val issueEntryVec : Vec[Bool] = Output(Vec(NumBiqEntry, Bool()))
  val issueReadData : BiqEntryData = Output(new BiqEntryData)
}

class BiqOutput extends Bundle with BiqConfig {
  val entryEnqOHVec : Vec[UInt] = Vec(NumBiqCreatePort, UInt(NumBiqEntry.W))
  val ctrl = new BiqCtrlOutput
  val data = new BiqDataOutput
  val toTop = new Bundle {
    val entryCnt : UInt = UInt(log2Up(NumBiqEntry).W)
  }
  val xxGateClkIssueEn : Bool = Bool()
  val xxIssueEn : Bool = Bool()
}

class BiqIO extends Bundle with BiqConfig {
  val in : BiqInput = Flipped(Output(new BiqInput))
  val out : BiqOutput = Output(new BiqOutput)
}

class Biq extends Module with BiqConfig {
  val io : BiqIO = IO(new BiqIO)

  private val cp0 = io.in.fromCp0
  private val lsu = io.in.fromLsu
  private val rtu = io.in.fromRtu
  private val ctrlBiq = io.in.ctrl
  private val dataBiq = io.in.data

  private val entryValidVec = Wire(Vec(this.NumBiqEntry, Bool()))
  private val entries = Seq.fill(this.NumBiqEntry)(Module(new BiqEntry))

  // find two empty entry
  private val enq0OH : Vec[Bool] = VecInit(PriorityEncoderOH(entryValidVec.map(!_)))
  private val enq1OH : Vec[Bool] = VecInit(PriorityEncoderOH(entryValidVec.reverse.map(!_)))
  io.out.entryEnqOHVec(0) := enq0OH.asUInt
  io.out.entryEnqOHVec(1) := enq1OH.asUInt


  /**
   * Regs
   */
  private val entryCnt = RegInit(0.U(log2Up(this.NumBiqEntry).W))

  /**
   * Wires
   */
  private val entryCreateEnVec = Wire(Vec(NumBiqEntry, Bool()))
  private val entryCreateDpEnVec = Wire(Vec(NumBiqEntry, Bool()))
  private val entryCreateGateClkVec = Wire(Vec(NumBiqEntry, Bool()))
  private val entryCreateAgeVec = Wire(Vec(this.NumBiqEntry, Vec(this.NumBiqEntry - 1, Bool())))
  private val entryCreateDataVec = Wire(Vec(this.NumBiqEntry, new BiqEntryData))
  private val entryCreateFreezeVec = Wire(Vec(this.NumBiqEntry, Bool()))

  //==========================================================
  //            Branch Issue Queue Create Control
  //==========================================================
  private val cntClkEn : Bool = (entryCnt =/= 0.U) || io.in.ctrl.gateClkEn.reduce(_||_)

  // Todo: gated clk

  io.out.ctrl.fullUpdateClkEn := cntClkEn
  //--------------------biq entry counter--------------------
  private val entryCntCreate  = ctrlBiq.createEn.count(b => b)
  private val entryCntPop     = ctrlBiq.rfPopValid.asUInt
  private val entryCntUpdateValid  = ctrlBiq.createEn(0) || ctrlBiq.rfPopValid
  private val entryCntUpdate  = entryCnt + entryCntCreate - entryCntPop

  when (rtu.flushFe || rtu.flushIs || rtu.yyXXFlush) {
    entryCnt := 0.U
  }.elsewhen(entryCntUpdateValid) {
    entryCnt := entryCntUpdate
  }

  private val full = entryCnt === this.NumBiqEntry.U
  private val empty = !entryValidVec.asUInt.orR
  io.out.ctrl.full  := full
  io.out.ctrl.empty := empty
  io.out.toTop.entryCnt := entryCnt

  //---------------------biq entry full-----------------------
  // Todo: check
  private val fullUpdate = entryCntUpdate === entries.length.U
  private val oneLeftUpdate = entryCntUpdate === (entries.length - 1).U
  io.out.ctrl.fullUpdate := fullUpdate
  io.out.ctrl.oneLeftUpdate := oneLeftUpdate

  //---------------------create bypass------------------------
  //if create instruction is ready, it may bypass from issue queue
  private val create0ReadyBypass = ctrlBiq.createEn(0) &&
    !cp0.iqBypassDisable && dataBiq.srcReadyForBypass.asUInt.andR
  private val create0ReadyBypassDp = ctrlBiq.createDpEn(0) &&
    !cp0.iqBypassDisable && dataBiq.srcReadyForBypass.asUInt.andR
  private val create0ReadyBypassGateClk = ctrlBiq.gateClkEn(0) && dataBiq.srcReadyForBypass.asUInt.andR

  private val entryValidWithoutFreezeVec = Wire(Vec(NumBiqEntry, Bool()))
  private val createBypassEmpty = !entryValidWithoutFreezeVec.asUInt.orR

  private val byPassEn = createBypassEmpty && create0ReadyBypass
  private val bypassDpEn = createBypassEmpty && create0ReadyBypassDp
  private val bypassGateClkEn = createBypassEmpty && create0ReadyBypassGateClk

  entryCreateEnVec := VecInit(
    ( (Fill(NumBiqEntry, ctrlBiq.createEn(0)) & enq0OH.asUInt)
    | (Fill(NumBiqEntry, ctrlBiq.createEn(1)) & enq1OH.asUInt)).asBools
  )

  //issue queue entry create data path control
  entryCreateDpEnVec := VecInit(
    ( (Fill(NumBiqEntry, ctrlBiq.createDpEn(0)) & enq0OH.asUInt)
    | (Fill(NumBiqEntry, ctrlBiq.createDpEn(1)) & enq1OH.asUInt)).asBools
  )

  entryCreateGateClkVec := VecInit(
    ( (Fill(NumBiqEntry, ctrlBiq.gateClkEn(0)) & enq0OH.asUInt)
    | (Fill(NumBiqEntry, ctrlBiq.gateClkEn(1)) & enq1OH.asUInt)).asBools
  )

  //biq create entry should consider pop signal and create0
  private val entryCreatePortAgeVec = Wire(Vec(NumBiqCreatePort, Vec(NumBiqEntry, Bool())))
  for (i <- 0 until NumBiqEntry) {
    entryCreatePortAgeVec(0)(i) := entryValidVec(i) &&
      !ctrlBiq.rfPopValid && dataBiq.rfLaunchEntry(i)
    entryCreatePortAgeVec(1)(i) := entryValidVec(i) &&
      !ctrlBiq.rfPopValid && dataBiq.rfLaunchEntry(i) || enq0OH(i)
  }

  //create 0/1 select:
  //entry 0~5 use ~biq_entry_create0_in for better timing
  //entry 6~12 use biq_entry_create1_in for better timing
  //biq_entry_create0/1_in cannot be both 1,
  //if both 0, do not create

  private val entryCreateSel = Wire(Vec(NumBiqEntry, Bool()))
  for (i <- 0 until NumBiqEntry) {
    entryCreateSel(i) := ctrlBiq.createDpEn(1) && enq1OH(i)
  }
  // Todo: timing optimization
  //  for (i <- 0 until NumBiqEntry / 2) {
  //    entryCreateSel(i) := ~(ctrlBiq.createDpEn(0) && enq0OH(i))
  //  }
  //  for (i <- NumBiqEntry / 2 until NumBiqEntry {
  //    entryCreateSel(i) := ctrlBiq.createDpEn(1) && enq1OH(i))
  //  }

  //----------------entries flop create signals----------------

  for (i <- 0 until this.NumBiqEntry) {
    when(entryCreateSel(i) === 0.U) {
      entryCreateFreezeVec(i) := bypassDpEn
      entryCreateAgeVec(i)    := VecInit(DropBit(entryCreatePortAgeVec(0).asUInt, i).asBools)
      entryCreateDataVec(i)   := dataBiq.createData(0)
    }.otherwise {
      entryCreateFreezeVec(i) := false.B
      entryCreateAgeVec(i)    := VecInit(DropBit(entryCreatePortAgeVec(1).asUInt, i).asBools)
      entryCreateDataVec(i)   := dataBiq.createData(1)
    }
  }

  //==========================================================
  //             Branch Issue Queue Issue Control
  //==========================================================
  //----------------Pipe0 Launch Ready Signals----------------

  private val entryAluRegFwdValid = Wire(Vec(NumAlu, Vec(this.NumBiqEntry, UInt(NumSrcBr.W))))
  entryAluRegFwdValid := ctrlBiq.rfAluRegFwdValid

  //-------------------issue enable signals-------------------
  private val entryOutReadyVec = Wire(Vec(this.NumBiqEntry, Bool()))

  // if there is any entry ready or bypass enable, issue enable
  io.out.xxIssueEn := byPassEn || entryOutReadyVec.reduce(_||_)

  // gate clock issue enable with timing optimization
  io.out.xxGateClkIssueEn := bypassGateClkEn || entryValidWithoutFreezeVec.reduce(_||_)

  private val entryOutAgeVec = Wire(Vec(this.NumBiqEntry, UInt((this.NumBiqEntry - 1).W)))
  private val olderEntryReadyVec = Wire(Vec(this.NumBiqEntry, Bool()))
  for (i <- 0 until this.NumBiqEntry) {
    olderEntryReadyVec(i) := (entryOutAgeVec(i) & DropBit(entryOutReadyVec, i)).orR
  }

  //------------------entry issue enable signals--------------
  //not ready if older ready exists
  private val entryIssueEnVec = Wire(Vec(this.NumBiqEntry, Bool()))
  for (i <- 0 until this.NumBiqEntry) {
    entryIssueEnVec(i) := entryOutReadyVec(i) && !olderEntryReadyVec(i)
  }

  //-----------------issue entry indication--------------------
  // Todo: figure out
  io.out.data.issueEntryVec := Mux(
    createBypassEmpty,
    enq0OH,
    entryIssueEnVec
  )

  //-----------------issue data path selection----------------
  private val entryOutReadDataVec = Wire(Vec(this.NumBiqEntry, Output(new BiqEntryData)))
  private val entryReadData = Wire(Output(new BiqEntryData))
  entryReadData := entryOutReadDataVec(OHToUInt(entryIssueEnVec))

  io.out.data.issueReadData := Mux(
    createBypassEmpty,
    dataBiq.bypassData,
    entryReadData
  )

  //==========================================================
  //            Branch Issue Queue Launch Control
  //==========================================================
  //-------------------entry pop enable signals---------------
  //pop when rf launch pass

  private val entryPopCurEntryVec = Wire(Vec(this.NumBiqEntry, Bool()))
  private val entryPopOtherEntryVec = Wire(Vec(this.NumBiqEntry, Vec(this.NumBiqEntry - 1, Bool())))
  entryPopCurEntryVec := dataBiq.rfLaunchEntry
  entryPopOtherEntryVec.zipWithIndex.foreach {
    case (popOther, i) => popOther := VecInit(DropBit(dataBiq.rfLaunchEntry, i).asBools)
  }

  //-------------------entry spec fail signals---------------
  //clear freeze and source rdy when launch fail
  private val entryFreezeClearVec = Wire(Vec(this.NumBiqEntry, Bool()))
  entryFreezeClearVec.zipWithIndex.foreach {
    case (freezeClear, i) => freezeClear := ctrlBiq.rfLaunchFailValid && dataBiq.rfLaunchEntry(i)
  }

  //==========================================================
  //             Branch Issue Queue Entry Instance
  //==========================================================

  entries.zipWithIndex.foreach {
    case (entry, i) =>
      val in = entry.io.in
      val out = entry.io.out
      in.fromCp0    := io.in.fromCp0
      in.fwdValid   := Cat(
        entryAluRegFwdValid(0)(i),
        entryAluRegFwdValid(1)(i),
        lsu.load.dcFwdInstValid
      ).asTypeOf(chiselTypeOf(in.fwdValid))
      in.fuDstPreg        := io.in.fuResultDstPreg
      in.wbPreg           := io.in.wbPreg
      in.loadPreg         := io.in.loadPreg
      in.fromRtu.flush.fe := io.in.fromRtu.flushFe
      in.fromRtu.flush.is := io.in.fromRtu.flushIs
      in.create.ageVec    := entryCreateAgeVec(i)
      in.create.data      := entryCreateDataVec(i)
      in.create.dpEn      := entryCreateDpEnVec(i)
      in.create.en        := entryCreateEnVec(i)
      in.create.freeze    := entryCreateFreezeVec(i)
      in.create.gateClkEn := entryCreateGateClkVec(i)
      in.rfPopValid       := io.in.ctrl.rfPopValid
      in.rfReadyClr       := io.in.data.rfReadyClr
      in.freezeClr        := entryFreezeClearVec(i)
      in.issueEn          := entryIssueEnVec(i)
      in.popCurEntry      := entryPopCurEntryVec(i)
      in.popOtherEntry    := entryPopOtherEntryVec(i)
      entryOutAgeVec(i)             := out.ageVec.asUInt
      entryOutReadyVec(i)           := out.ready
      entryOutReadDataVec(i)        := out.readData
      entryValidVec(i)              := out.valid
      entryValidWithoutFreezeVec(i) := out.validWithoutFreeze
  }

}
