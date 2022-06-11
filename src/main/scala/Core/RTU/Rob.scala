package Core.RTU

import Core.AddrConfig._
import Core.ExceptionConfig._
import Core.GlobalConfig.{DifftestEnable, NumFoldMax, RobFoldEnable}
import Core.IntConfig._
import Core.PipelineConfig.NumPipeline
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Utils.Bits.RingShiftLeft
import chisel3._
import chisel3.util._
import difftest.{DifftestInstrCommit, DifftestTrapEvent}


class RobInput extends Bundle {
  val fromCp0 = new RobFromCp0Bundle
  val fromHad = new RobFromHadBundle
  val fromHpcp = new RtuFromHpcpBundle
  val fromIdu = new Bundle() {
    val fenceIdle : Bool = Bool()
    val robCreate : Vec[RobCreateBundle] = Vec(NumCreateEntry, new RobCreateBundle)
  }
  val fromIfu = new RobFromIfu
  val fromIu = new Bundle() {
    val pcFifoPopDataVec : Vec[PcFifoData] = Vec(NumPopEntry, new PcFifoData)
    val pipe0 = new Bundle() {
      val iid : UInt = UInt(InstructionIdWidth.W)
      val cmplt : Bool = Bool()
      val abnormal : Bool = Bool()
      val breakPoint : Bool = Bool()
      val efPc = ValidIO(UInt(PcWidth.W))
      val exceptionVec = ValidIO(UInt(ExceptionVecWidth.W))
      val flush : Bool = Bool()
      val highHwException : Bool = Bool()
      val instMmuException : Bool = Bool()
      val mtval : UInt = UInt(MtvalWidth.W)
      val vsetvl : Bool = Bool()
      val vstart = ValidIO(UInt(VlmaxBits.W))
    }
    val pipe1 = new Bundle() {
      val iid : UInt = UInt(InstructionIdWidth.W)
      val cmplt : Bool = Bool()
    }
    val pipe2 = new Bundle() {
      val iid : UInt = UInt(InstructionIdWidth.W)
      val cmplt : Bool = Bool()
      val abnormal : Bool = Bool()
      val bhtMispred : Bool = Bool()
      val jmpMispred : Bool = Bool()
    }
  }
  val fromLsu = new Bundle() {
    val allCommitDataValid : Bool = Bool()
    val pipe3 : RobFromLsuPipeCommonBundle = new RobFromLsuPipeCommonBundle {}
    val pipe4 : RobFromLsuPipeCommonBundle = new RobFromLsuPipeCommonBundle {}
  }
  val fromPad = new RtuFromPadBundle
  val fromRetire = new RobFromRetire {

  }
  val fromRtu = new Bundle() {
    val yyXxFlush : Bool = Bool()
  }
  val fromVfpu = new RobFromVfpu
}

class RobOutput extends Bundle {
  class RobToPst extends Bundle {
    val iid : Vec[UInt] = Vec(NumRetireEntry, UInt(InstructionIdWidth.W))
    val iidUpdate : Vec[UInt] = Vec(NumRetireEntry, UInt(InstructionIdWidth.W))
    val gateClkValid : Vec[Bool] = Vec(NumRetireEntry, Bool())
  }
  val toPst = new RobToPst
  val toRetire = new RobToRetireBundle {}
  val toTop = new Bundle() {
    val commit0 : Bool = Bool()
    val commitStNoValid : Bool = Bool()
    val create0Iid : UInt = UInt(InstructionIdWidth.W)
    // Todo: imm
    val entryNum : UInt = UInt(NumRobEntry.W)
    val flushState : UInt = UInt(FlushState.width.W)
    val read0Iid : UInt = UInt(InstructionIdWidth.W)
    // Todo: imm
    val robCurPc : UInt = UInt(7.W)
    val robFull : Bool = Bool()
    val ssfState : UInt = UInt(SsfState.width.W)
  }
  val toCpu = new RobToCpu
  // Todo: figure out
  val toHad = new RobToHad
  // Todo: figure out
  val toHpcp = new RobToHpcp
  val toIdu = new Bundle() {
    val retireInterruptValid : Bool = Bool()
    val robEmpty : Bool = Bool()
    val robFull : Bool = Bool()
    // Todo: check
    val robInstIidVec : Vec[UInt] = Vec(NumCreateEntry, UInt(InstructionIdWidth.W))
  }
  val toIu = new RobToIu
  val toLsu = new Bundle() {
    // Todo: check
    val commitIidUpdateVal : Vec[UInt] = Vec(NumRetireEntry, UInt(InstructionIdWidth.W))
  }
  val toPad = new RobToPad
  val yyXx = new RobYyXx
}

class RobIO extends Bundle {
  val in : RobInput = Flipped(Output(new RobInput))
  val out : RobOutput = Output(new RobOutput)
}

class Rob extends Module {
  val io : RobIO = IO(new RobIO)

  /**
   * Regs
   */

  private val debugInfoFlushCurState = RegInit(FlushState.idle)
  private val debugInfoRobCommit0 = RegInit(false.B)
  private val debugInfoRobCommitStNoValid = RegInit(false.B)
  private val debugInfoRobCreate0Iid = RegInit(0.U(InstructionIdWidth.W))
  private val debugInfoRobRead0Iid = RegInit(0.U(InstructionIdWidth.W))

  private val debugInfoRobEntryNum = RegInit(0.U(NumRobEntry.W))
  private val debugInfoRobFull = RegInit(false.B)

  private val robCreatePtr = RegInit(1.U(RobPtrWidth.W))
  private val robCreateIidVec = RegInit(VecInit((0 until  NumCreateEntry).map(_.U(InstructionIdWidth.W))))

  private val robEntryNum = RegInit(0.U(NumRobEntryBits.W))
  private val robFull = RegInit(false.B)
  private val robPopIidVec = RegInit(VecInit((0 until  NumPopEntry).map(_.U(InstructionIdWidth.W))))

  private val robReadPtr = RegInit(1.U(NumRobEntry.W))
  private val robReadIidVec = RegInit(VecInit((0 until  RobReadPtrNum).map(_.U(InstructionIdWidth.W))))

  /**
   * Wires
   */

  private val robEntryNumAdd = Wire(UInt(3.W))
  private val robEntryNumSub = Wire(UInt(2.W))

  private val readEntryUpdateDataVec = Wire(Vec(3, new RobEntryData))
  private val robReadEntryUpdateIid = Wire(Vec(3, UInt(6.W)))
  // Todo: check imm
  private val robReadEntryDataVec = Wire(Vec(RobReadPtrNum, new RobEntryData))

  //==========================================================
  //                   ROB Entry Instance
  //==========================================================

  private val miscCmpltGateClkEn        = Wire(Bool())
  // Todo: imm
  private val entryCmpltValidVec        = Wire(Vec(NumRobEntry, UInt(7.W)))
  private val entryCmpltGateClkValidVec = Wire(Vec(NumRobEntry, Bool()))
  private val entryCreateDpEnVec        = Wire(Vec(NumRobEntry, Bool()))
  private val entryCreateEnVec          = Wire(Vec(NumRobEntry, Bool()))
  private val entryCreateGateClkEnVec   = Wire(Vec(NumRobEntry, Bool()))
  // Todo: imm
  private val entryCreateSelVec         = Wire(Vec(NumRobEntry, UInt(NumCreateEntry.W)))
  private val entryPopEnVec             = Wire(Vec(NumRobEntry, Bool()))
  private val entryReadDataVec          = Wire(Vec(NumRobEntry, new RobEntryData))
  for (i <- 0 until NumRobEntry) {
    entryCmpltGateClkValidVec(i) := entryCmpltValidVec(i).asUInt.orR
  }

  private val robEntryVec = Seq.fill(NumRobEntry)(Module(new RobEntry))
  for (i <- 0 until NumRobEntry) {
    val in = robEntryVec(i).io.in
    val out = robEntryVec(i).io.out
    in.fromCp0.icgEn := io.in.fromCp0.icgEn
    in.fromCp0.yyClkEn := io.in.fromCp0.yyClkEn
    in.fromIdu.createDataVec.zipWithIndex.foreach {
      case (data, i) => data := io.in.fromIdu.robCreate(i).data
    }
    in.fromLsu.miscCmpltGateClkEn := miscCmpltGateClkEn
    in.fromLsu.pipe3.wbNoSpec  := io.in.fromLsu.pipe3.noSpec
    in.fromLsu.pipe3.wbBreakpointData := io.in.fromLsu.pipe3.breakpointData
    in.fromLsu.pipe4.wbNoSpec  := io.in.fromLsu.pipe4.noSpec
    in.fromLsu.pipe4.wbBreakpointData := io.in.fromLsu.pipe4.breakpointData
    in.fromPad := io.in.fromPad
    in.fromRetire := io.in.fromRetire
    in.x.cmpltGateClkValid :=  entryCmpltGateClkValidVec(i)
    in.x.cmpltValidVec := VecInit(entryCmpltValidVec(i).asBools)
    in.x.createDpEn := entryCreateDpEnVec(i)
    in.x.createEn := entryCreateEnVec(i)
    in.x.createGateClkEn := entryCreateGateClkEnVec(i)
    in.x.createSel := VecInit(entryCreateSelVec(i).asBools)
    in.x.popEn := entryPopEnVec(i)
    entryReadDataVec(i) := out.readData
  }

  private val readEntryCreateDataVec = Wire(Vec(NumRobReadEntry, new RobEntryData))

  private val readEntryCmpltValidVec        = Wire(Vec(NumRobReadEntry, UInt(7.W)))
  private val readEntryCmpltGateClkValidVec = Wire(Vec(NumRobReadEntry, Bool()))
  private val readEntryCreateDpEnVec        = Wire(Vec(NumRobReadEntry, Bool()))
  private val readEntryCreateEnVec          = Wire(Vec(NumRobReadEntry, Bool()))
  private val readEntryCreateGateClkEnVec   = Wire(Vec(NumRobReadEntry, Bool()))
  private val readEntryCreateSelVec         = Wire(Vec(NumRobReadEntry, UInt(NumCreateEntry.W)))
  private val readEntryPopEnVec             = Wire(Vec(NumRobReadEntry, Bool()))
  private val readEntryReadDataVec          = Wire(Vec(NumRobReadEntry, new RobEntryData))

  private val robReadEntryVec = Seq.fill(NumRobReadEntry)(Module(new RobEntry))
  for (i <- 0 until NumRobReadEntry) {
    val in = robReadEntryVec(i).io.in
    val out = robReadEntryVec(i).io.out
    in.fromCp0 := io.in.fromCp0
    in.fromIdu.createDataVec(0) := readEntryCreateDataVec(i)
    // Todo: check idx upperbound
    for (j <- 1 until NumCreateEntry) {
      in.fromIdu.createDataVec(j) := io.in.fromIdu.robCreate(j).data
    }
    in.fromLsu.miscCmpltGateClkEn := miscCmpltGateClkEn
    // Todo: simplify wire connection
    in.fromLsu.pipe3.wbNoSpec           := io.in.fromLsu.pipe3.noSpec
    in.fromLsu.pipe3.wbBreakpointData   := io.in.fromLsu.pipe3.breakpointData
    in.fromLsu.pipe4.wbNoSpec           := io.in.fromLsu.pipe4.noSpec
    in.fromLsu.pipe4.wbBreakpointData   := io.in.fromLsu.pipe4.breakpointData
    in.fromPad                          := io.in.fromPad
    in.fromRetire.flush                 := io.in.fromRetire.flush
    in.fromRetire.flushGateClk          := io.in.fromRetire.flushGateClk
    in.x.cmpltGateClkValid              := readEntryCmpltGateClkValidVec(i)
    in.x.cmpltValidVec                  := VecInit(readEntryCmpltValidVec(i).asBools)
    in.x.createDpEn                     := readEntryCreateDpEnVec(i)
    in.x.createEn                       := readEntryCreateEnVec(i)
    in.x.createGateClkEn                := readEntryCreateGateClkEnVec(i)
    in.x.createSel                      := VecInit(readEntryCreateSelVec(i).asBools)
    in.x.popEn                          := readEntryPopEnVec(i)
    readEntryReadDataVec(i)             := out.readData
  }

  //==========================================================
  //                    ROB Create Control
  //==========================================================
  //----------------------------------------------------------
  //                  create enable signals
  //----------------------------------------------------------

  // Todo: imm
  private val robCreatePtrVec = Wire(Vec(NumCreateEntry, UInt(RobPtrWidth.W)))
  robCreatePtrVec(0) := robCreatePtr
  private val robCreateEn = Wire(UInt(RobPtrWidth.W))
  private val robCreateDpEn = Wire(UInt(RobPtrWidth.W))
  private val robCreateGateClkEn = Wire(UInt(RobPtrWidth.W))

  robCreateEn := io.in.fromIdu.robCreate.zip(robCreatePtrVec).map{
    case (robCreate, ptr) => Mux(robCreate.en, ptr, 0.U)
  }.reduce(_ | _)
  robCreateDpEn := io.in.fromIdu.robCreate.zip(robCreatePtrVec).map {
    case (robCreate, ptr) => Mux(robCreate.dpEn, ptr, 0.U)
  }.reduce(_ | _)
  robCreateGateClkEn := io.in.fromIdu.robCreate.zip(robCreatePtrVec).map {
    case (robCreate, ptr) => Mux(robCreate.gateClkEn, ptr, 0.U)
  }.reduce(_ | _)

  entryCreateEnVec := robCreateEn.asBools
  entryCreateDpEnVec := robCreateDpEn.asBools
  entryCreateGateClkEnVec := robCreateGateClkEn.asBools

  for (i <- 0 until RobPtrWidth) {
    entryCreateSelVec(i) := VecInit(robCreatePtrVec.map(ptr => ptr(i))).asUInt
  }

  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  // Todo: Instance of Gated Cell

  //----------------------------------------------------------
  //                    create pointers
  //----------------------------------------------------------
  //create to last entry is disable
  val robCreatePtrAdd = WireInit(VecInit(Seq.fill(RobPtrNum + 1)(false.B)))

  for (i <- 1 to RobPtrNum) {
    robCreatePtrAdd(i) := io.in.fromIdu.robCreate(i - 1).en
  }

  when(reset.asBool) {
    robCreatePtr := 1.U
    for (i <- 0 until RobPtrNum) {
      robCreateIidVec(i) := i.U
    }
  }.elsewhen(io.in.fromRetire.flush) {
    robCreatePtr := 1.U
    for (i <- 0 until RobPtrNum) {
      robCreateIidVec(i) := i.U
    }
  }.elsewhen(robCreatePtrAdd(4)) {
    // Todo: need function ring shift
    robCreatePtr := Utils.Bits.RingShiftLeft(robCreatePtr, 4)
    robCreateIidVec.foreach(iid => iid := iid + 4.U)
  }.elsewhen(robCreatePtrAdd(3)) {
    robCreatePtr := Utils.Bits.RingShiftLeft(robCreatePtr, 3)
    robCreateIidVec.foreach(iid => iid := iid + 3.U)
  }.elsewhen(robCreatePtrAdd(2)) {
    robCreatePtr := Utils.Bits.RingShiftLeft(robCreatePtr, 2)
    robCreateIidVec.foreach(iid => iid := iid + 2.U)
  }.elsewhen(robCreatePtrAdd(1)) {
    robCreatePtr := Utils.Bits.RingShiftLeft(robCreatePtr, 1)
    robCreateIidVec.foreach(iid => iid := iid + 1.U)
  }.otherwise {
    robCreatePtr := robCreatePtr
    robCreateIidVec := robCreateIidVec
  }

  robCreatePtrVec(1) := Cat(robCreatePtr(62,0), robCreatePtr(63,63))
  robCreatePtrVec(2) := Cat(robCreatePtr(61,0), robCreatePtr(63,62))
  robCreatePtrVec(3) := Cat(robCreatePtr(60,0), robCreatePtr(63,61))

  //create to last entry is disabled
  io.out.toIdu.robInstIidVec := robCreateIidVec

  //==========================================================
  //                        ROB full
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  // Todo: gated cell

  //----------------------------------------------------------
  //                  ROB valid entry counter
  //----------------------------------------------------------
  private val robPopPtrAdd = Wire(Vec(RobPtrNum, Bool()))

  robEntryNumAdd := OHToUInt(PriorityEncoderOH(robCreatePtrAdd.reverse).reverse)
  robEntryNumSub := OHToUInt(PriorityEncoderOH(robPopPtrAdd.reverse).reverse)

  // Create or pop at least 1 entry
  private val robEntryNumUpdateValid = robCreatePtrAdd(1) || robPopPtrAdd(1)
  private val robEntryNumUpdate = robEntryNum + robEntryNumAdd - robEntryNumSub

  when (io.in.fromRetire.flush) {
    robEntryNum := 0.U
  }.elsewhen(robEntryNumUpdateValid) {
    robEntryNum := robEntryNumUpdate
  }

  private val robEmpty = robEntryNum === 0.U
  private val rob1Entry = robEntryNum === 1.U
  private val rob2Entry = robEntryNum === 2.U
  io.out.toIdu.robEmpty := robEmpty && io.in.fromRetire.retireEmpty

  //----------------------------------------------------------
  //                     ROB Full signal
  //----------------------------------------------------------
  private val robFullUpdateVal = robEntryNumUpdate > (NumRobEntry - NumCreateEntry).U

  when(io.in.fromRetire.flush) {
    robFull := false.B
  }.elsewhen(robEntryNumUpdateValid) {
    robFull := robFullUpdateVal
  }

  io.out.toIdu.robFull := robFull

  //----------------------------------------------------------
  //                   read vld bit
  //----------------------------------------------------------
  private val entryValid = Wire(Vec(NumRobEntry, Bool()))
  entryValid := entryReadDataVec.map(_.ctrl.valid)

  // Todo: figure out why not use robEntryNum directly
  io.out.toHad.robEmpty := !entryValid.asUInt.orR

  //==========================================================
  //                    ROB Complete Port
  //==========================================================

  // Todo: imm: num of pipeline
  private val PipeIidLsbVec = Wire(Vec(7, UInt((InstructionIdWidth-1).W)))

  PipeIidLsbVec(0) := io.in.fromIu.pipe0.iid(5,0)
  PipeIidLsbVec(1) := io.in.fromIu.pipe1.iid(5,0)
  PipeIidLsbVec(2) := io.in.fromIu.pipe2.iid(5,0)
  PipeIidLsbVec(3) := io.in.fromLsu.pipe3.iid(5,0)
  PipeIidLsbVec(4) := io.in.fromLsu.pipe4.iid(5,0)
  PipeIidLsbVec(5) := io.in.fromVfpu.pipe6.iid(5,0)
  PipeIidLsbVec(6) := io.in.fromVfpu.pipe7.iid(5,0)

  // Todo: imm: num of pipeline
  val completeOHVec : Vec[UInt] = Wire(Vec(7, UInt(NumRobEntry.W)))


  completeOHVec(0) := UIntToOH(PipeIidLsbVec(0)) & Fill(NumRobEntry, io.in.fromIu.pipe0.cmplt)
  completeOHVec(1) := UIntToOH(PipeIidLsbVec(1)) & Fill(NumRobEntry, io.in.fromIu.pipe1.cmplt)
  completeOHVec(2) := UIntToOH(PipeIidLsbVec(2)) & Fill(NumRobEntry, io.in.fromIu.pipe2.cmplt)
  completeOHVec(3) := UIntToOH(PipeIidLsbVec(3)) & Fill(NumRobEntry, io.in.fromLsu.pipe3.cmplt)
  completeOHVec(4) := UIntToOH(PipeIidLsbVec(4)) & Fill(NumRobEntry, io.in.fromLsu.pipe4.cmplt)
  completeOHVec(5) := UIntToOH(PipeIidLsbVec(5)) & Fill(NumRobEntry, io.in.fromVfpu.pipe6.cmplt)
  completeOHVec(6) := UIntToOH(PipeIidLsbVec(6)) & Fill(NumRobEntry, io.in.fromVfpu.pipe7.cmplt)

  for (i <- 0 until NumRobEntry) {
    entryCmpltValidVec(i) := Cat(completeOHVec.map(completeOH => completeOH(i)).reverse)
  }

  for (i <- 0 until NumRobEntry) {
    entryCmpltGateClkValidVec(i) := entryCmpltValidVec(i).asUInt
  }

  //----------------------------------------------------------
  //                 lsu cmplt info gateclk en
  //----------------------------------------------------------
  miscCmpltGateClkEn :=
    io.in.fromLsu.pipe3.cmplt &&
      (io.in.fromLsu.pipe3.breakpointData.a || io.in.fromLsu.pipe3.breakpointData.b ||
      io.in.fromLsu.pipe3.noSpec.hit || io.in.fromLsu.pipe3.noSpec.miss || io.in.fromLsu.pipe3.noSpec.mispred) ||
    io.in.fromLsu.pipe4.cmplt &&
      (io.in.fromLsu.pipe4.breakpointData.a || io.in.fromLsu.pipe4.breakpointData.b ||
      io.in.fromLsu.pipe4.noSpec.hit || io.in.fromLsu.pipe4.noSpec.miss || io.in.fromLsu.pipe4.noSpec.mispred)

  //==========================================================
  //              Read Port for Retire Entry
  //==========================================================

  //----------------------------------------------------------
  //                    5 Read Ports
  //----------------------------------------------------------
  private val robReadPtrVec = Wire(Vec(RobReadPtrNum, UInt(RobPtrWidth.W)))

  for (i <- 0 until RobReadPtrNum) {
    when(robReadPtrVec(i).orR) {
      robReadEntryDataVec(i) := entryReadDataVec(OHToUInt(robReadPtrVec(i)))
    }.otherwise{
      robReadEntryDataVec(i) := 0.U.asTypeOf(chiselTypeOf(robReadEntryDataVec(i)))
    }
  }

  //==========================================================
  //              Read Port for read entry update
  //==========================================================
  //----------------------------------------------------------
  //              Read entry update data select
  //----------------------------------------------------------

  // Todo: init
  private val retireEntryUpdateValidVec = Wire(Vec(3, Bool()))
  when(retireEntryUpdateValidVec(2)) {
    readEntryUpdateDataVec(0) := robReadEntryDataVec(3)
    readEntryUpdateDataVec(1) := robReadEntryDataVec(4)
    readEntryUpdateDataVec(2) := robReadEntryDataVec(5)
    robReadEntryUpdateIid(0) := robReadIidVec(3)
    robReadEntryUpdateIid(1) := robReadIidVec(4)
    robReadEntryUpdateIid(2) := robReadIidVec(5)
  }.elsewhen(retireEntryUpdateValidVec(1)) {
    readEntryUpdateDataVec(0) := robReadEntryDataVec(2)
    readEntryUpdateDataVec(1) := robReadEntryDataVec(3)
    readEntryUpdateDataVec(2) := robReadEntryDataVec(4)
    robReadEntryUpdateIid(0) := robReadIidVec(2)
    robReadEntryUpdateIid(1) := robReadIidVec(3)
    robReadEntryUpdateIid(2) := robReadIidVec(4)
  }.elsewhen(retireEntryUpdateValidVec(0)) {
    readEntryUpdateDataVec(0) := robReadEntryDataVec(1)
    readEntryUpdateDataVec(1) := robReadEntryDataVec(2)
    readEntryUpdateDataVec(2) := robReadEntryDataVec(3)
    robReadEntryUpdateIid(0) := robReadIidVec(1)
    robReadEntryUpdateIid(1) := robReadIidVec(2)
    robReadEntryUpdateIid(2) := robReadIidVec(3)
  }.otherwise {
    readEntryUpdateDataVec(0) := robReadEntryDataVec(0)
    readEntryUpdateDataVec(1) := robReadEntryDataVec(1)
    readEntryUpdateDataVec(2) := robReadEntryDataVec(2)
    robReadEntryUpdateIid(0) := robReadIidVec(0)
    robReadEntryUpdateIid(1) := robReadIidVec(1)
    robReadEntryUpdateIid(2) := robReadIidVec(2)
  }

  //----------------------------------------------------------
  //             Read entry create data select
  //----------------------------------------------------------

  private val robCreateToReadEntryEnVec = Wire(Vec(NumRobReadEntry, Bool()))
  private val robCreateToReadEntryDpEnVec = Wire(Vec(NumRobReadEntry, Bool()))
  private val robCreateToReadEntryGateClkEn = Wire(Vec(NumRobReadEntry, Bool()))
  for (i <- 0 until NumRobReadEntry) {
    robCreateToReadEntryEnVec(i) := robReadEntryDataVec(i).ctrl.valid && ! readEntryReadDataVec(i).ctrl.valid
    robCreateToReadEntryDpEnVec(i) := robCreateToReadEntryEnVec(i)
  }
  robCreateToReadEntryGateClkEn(0) := !robEmpty &&
                                      !readEntryReadDataVec(0).ctrl.valid
  robCreateToReadEntryGateClkEn(1) := !robEmpty && !rob1Entry
                                      !readEntryReadDataVec(0).ctrl.valid
  robCreateToReadEntryGateClkEn(2) := !robEmpty && !rob1Entry && !rob2Entry
                                      !readEntryReadDataVec(0).ctrl.valid

  //mux between create and update
  readEntryCreateDataVec := readEntryUpdateDataVec

  //----------------------------------------------------------
  //                Read entry create enable
  //----------------------------------------------------------
  private val retireEntryUpdateGateClkValid = Wire(Bool())

  for (i <- 0 until NumRobReadEntry) {
    readEntryCreateEnVec(i) := retireEntryUpdateValidVec(0) || robCreateToReadEntryEnVec(i)
    readEntryCreateDpEnVec(i) := retireEntryUpdateValidVec(0) || robCreateToReadEntryDpEnVec(i)
    readEntryCreateGateClkEnVec(i) := retireEntryUpdateGateClkValid || robCreateToReadEntryGateClkEn(i)
    readEntryCreateSelVec(i) := "b0001".U
  }

  //----------------------------------------------------------
  //                Read entry complete port
  //----------------------------------------------------------

  private val pipeCompleteVec = Wire(Vec(NumPipeline, Bool()))
  private val pipeIidVec = Wire(Vec(NumPipeline, UInt(NumRobEntryBits.W)))
  pipeCompleteVec(0) := io.in.fromIu.pipe0.cmplt
  pipeCompleteVec(1) := io.in.fromIu.pipe1.cmplt
  pipeCompleteVec(2) := io.in.fromIu.pipe2.cmplt
  pipeCompleteVec(3) := io.in.fromLsu.pipe3.cmplt
  pipeCompleteVec(4) := io.in.fromLsu.pipe4.cmplt
  pipeCompleteVec(5) := io.in.fromVfpu.pipe6.cmplt
  pipeCompleteVec(6) := io.in.fromVfpu.pipe7.cmplt
  pipeIidVec(0) := io.in.fromIu.pipe0.iid(NumRobEntryBits - 1, 0)
  pipeIidVec(1) := io.in.fromIu.pipe1.iid(NumRobEntryBits - 1, 0)
  pipeIidVec(2) := io.in.fromIu.pipe2.iid(NumRobEntryBits - 1, 0)
  pipeIidVec(3) := io.in.fromLsu.pipe3.iid
  pipeIidVec(4) := io.in.fromLsu.pipe4.iid
  pipeIidVec(5) := io.in.fromVfpu.pipe6.iid
  pipeIidVec(6) := io.in.fromVfpu.pipe7.iid


  for (i <- 0 until NumRobReadEntry) {
    readEntryCmpltValidVec(i) := Cat(pipeCompleteVec.zip(pipeIidVec).map{
      case (cmplt, iid) => cmplt && iid === robReadEntryUpdateIid(i)
    })
  }
  private val readEntryCmpltGateClkValid = pipeCompleteVec.asUInt.orR
  readEntryCmpltGateClkValidVec.foreach(_ := readEntryCmpltGateClkValid)

  //----------------------------------------------------------
  //                   Read Entry Read
  //----------------------------------------------------------
  private val robReadData = readEntryReadDataVec

  //----------------------------------------------------------
  //                   Read Entry Pop
  //----------------------------------------------------------
  //Read entry never pop
  readEntryPopEnVec.foreach(_ := false.B)

  //==========================================================
  //                  Pop for Retire Entry
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  // Todo: gated cell

  //----------------------------------------------------------
  //                    read pointers
  //----------------------------------------------------------
  private val robReadPtrAddVec = Wire(Vec(4, Bool()))
  robReadPtrAddVec(0) := false.B
  robReadPtrAddVec(1) := retireEntryUpdateValidVec(0)
  robReadPtrAddVec(2) := retireEntryUpdateValidVec(1)
  robReadPtrAddVec(3) := retireEntryUpdateValidVec(2)

  // Todo: simplify
  when(io.in.fromRetire.flush) {
    robReadPtr := 1.U(NumRobEntry.W)
    for (i <- 0 until RobReadPtrNum) {
      robReadIidVec(i) := i.U
    }
  }.elsewhen(robReadPtrAddVec(3)) {
    robReadPtr := Utils.Bits.RingShiftLeft(robReadPtr, 3)
    for (i <- 0 until RobReadPtrNum) {
      robReadIidVec(i) := robReadIidVec(i) + 3.U
    }
  }.elsewhen(robReadPtrAddVec(2)) {
    robReadPtr := Utils.Bits.RingShiftLeft(robReadPtr, 2)
    for (i <- 0 until RobReadPtrNum) {
      robReadIidVec(i) := robReadIidVec(i) + 2.U
    }
  }.elsewhen(robReadPtrAddVec(1)) {
    robReadPtr := Utils.Bits.RingShiftLeft(robReadPtr, 1)
    for (i <- 0 until RobReadPtrNum) {
      robReadIidVec(i) := robReadIidVec(i) + 1.U
    }
  }.otherwise {
    robReadPtr := robReadPtr
    robReadIidVec := robReadIidVec
  }

  robReadPtrVec(0) := robReadPtr

  for (i <- 1 until RobReadPtrNum) {
    robReadPtrVec(i) := RingShiftLeft(robReadPtr, i)
  }

  // Todo: imm
  for (i <- 0 until 3) {
    io.out.toPst.iidUpdate(i) := robReadIidVec(i)
  }

  for (i <- 0 until NumCommitEntry) {
    io.out.toLsu.commitIidUpdateVal(i) := robReadIidVec(i)
  }

  //==========================================================
  //                    ROB pop control
  //==========================================================

  //----------------------------------------------------------
  //                      3 Pop Ports
  //----------------------------------------------------------
  private val robPopPtrVec = Wire(Vec(3, UInt(NumRobEntryBits.W)))

  entryPopEnVec := VecInit((
    (Fill(NumRobEntry, io.out.yyXx.retire(0)) & UIntToOH(robPopPtrVec(0))) |
    (Fill(NumRobEntry, io.out.yyXx.retire(1)) & UIntToOH(robPopPtrVec(1))) |
    (Fill(NumRobEntry, io.out.yyXx.retire(2)) & UIntToOH(robPopPtrVec(2)))
  ).asBools)

  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------

  // Todo: gated clk for pop ptr
  robPopPtrAdd(0) := 0.U
  robPopPtrAdd(1) := io.out.yyXx.retire(0)
  robPopPtrAdd(2) := io.out.yyXx.retire(1)
  robPopPtrAdd(3) := io.out.yyXx.retire(2)

  when(io.in.fromRetire.flush) {
    for (i <- 0 until NumPopEntry) {
      robPopIidVec(i) := i.U
    }
  }.elsewhen(robPopPtrAdd(3)) {
    for (i <- 0 until NumPopEntry) {
      robPopIidVec(i) := robPopIidVec(i) + 3.U
    }
  }.elsewhen(robPopPtrAdd(2)) {
    for (i <- 0 until NumPopEntry) {
      robPopIidVec(i) := robPopIidVec(i) + 2.U
    }
  }.elsewhen(robPopPtrAdd(1)) {
    for (i <- 0 until NumPopEntry) {
      robPopIidVec(i) := robPopIidVec(i) + 1.U
    }
  }.otherwise {
    for (i <- 0 until NumPopEntry) {
      robPopIidVec(i) := robPopIidVec(i)
    }
  }

  robPopPtrVec := robPopIidVec

  //==========================================================
  //                  ROB debug info signals
  //==========================================================
  //capture rob information when rob flush (jdb req)
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  private val debugInfoClkEn = io.in.fromRetire.flushGateClk
  // Todo: gated clk for debug info

  //----------------------------------------------------------
  //                   ROB debug info
  //----------------------------------------------------------
  private val robDebugCommit0 = Wire(Bool())
  when(io.in.fromRetire.flushGateClk) {
    debugInfoRobFull            := robFull
    debugInfoRobRead0Iid        := robReadIidVec(0)
    debugInfoRobCreate0Iid      := robCreateIidVec(0)
    debugInfoRobEntryNum        := robEntryNum
    debugInfoRobCommit0         := robDebugCommit0
    debugInfoRobCommitStNoValid := !io.in.fromLsu.allCommitDataValid
    debugInfoFlushCurState      := io.in.fromRetire.flushState
  }.otherwise {
    // maintain
  }

  io.out.toTop.robFull          := debugInfoRobFull
  io.out.toTop.read0Iid         := debugInfoRobRead0Iid
  io.out.toTop.create0Iid       := debugInfoRobCreate0Iid
  io.out.toTop.entryNum         := debugInfoRobEntryNum
  io.out.toTop.commit0          := debugInfoRobCommit0
  io.out.toTop.commitStNoValid  := debugInfoRobCommitStNoValid
  io.out.toTop.flushState    := debugInfoFlushCurState

  //==========================================================
  //                   Expt Entry Instance
  //==========================================================

  private val robExcept = Module(new RobExcept)
  private val exceptIn = robExcept.io.in
  private val exceptOut = robExcept.io.out

  private val exceptOutEntry = Wire(new RobExceptEntryBundle)
  private val retireExceptInst0Abnormal = Wire(Bool())
  private val retireExceptInst0Valid = Wire(Bool())
  private val robExceptInst0Iid = Wire(UInt(InstructionIdWidth.W))
  val robRetireInst0Split = WireInit(io.out.toRetire.instVec(0).split)

  exceptIn.fromCp0                    := io.in.fromCp0
  exceptIn.fromIu.pipe0.cmplt         := io.in.fromIu.pipe0.cmplt
  exceptIn.fromIu.pipe0.iid           := io.in.fromIu.pipe0.iid
  exceptIn.fromIu.pipe0.abnormal      := io.in.fromIu.pipe0.abnormal
  exceptIn.fromIu.pipe0.breakPoint    := io.in.fromIu.pipe0.breakPoint
  exceptIn.fromIu.pipe0.efPc          := io.in.fromIu.pipe0.efPc
  exceptIn.fromIu.pipe0.exceptVec     := io.in.fromIu.pipe0.exceptionVec
  exceptIn.fromIu.pipe0.flush         := io.in.fromIu.pipe0.flush
  exceptIn.fromIu.pipe0.highHwExcept  := io.in.fromIu.pipe0.highHwException
  exceptIn.fromIu.pipe0.instMmuExcept := io.in.fromIu.pipe0.instMmuException
  exceptIn.fromIu.pipe0.mtval         := io.in.fromIu.pipe0.mtval
  exceptIn.fromIu.pipe0.vsetvl        := io.in.fromIu.pipe0.vsetvl
  exceptIn.fromIu.pipe0.vstart        := io.in.fromIu.pipe0.vstart
  exceptIn.fromIu.pipe0.instMmuExcept := io.in.fromIu.pipe0.instMmuException
  exceptIn.fromIu.pipe2               := io.in.fromIu.pipe2
  exceptIn.fromLsu.pipe3              := io.in.fromLsu.pipe3
  exceptIn.fromLsu.pipe4              := io.in.fromLsu.pipe4
  exceptIn.fromPad                    := io.in.fromPad
  exceptIn.fromRetire.inst0Abnormal   := retireExceptInst0Abnormal
  exceptIn.fromRetire.inst0Valid      := retireExceptInst0Valid
  exceptIn.fromRetire.flush           := io.in.fromRetire.flush
  exceptIn.fromRob.inst0Iid           := robExceptInst0Iid
  exceptIn.fromRob.inst0Split         := robRetireInst0Split
  exceptIn.fromRtu.yyXxFlush          := io.in.fromRtu.yyXxFlush

  exceptOutEntry                            := exceptOut.except
  private val retireOutInstExtra = Wire(Output(new RobToRetireInstExtraBundle))
  io.out.toRetire.instExtra       := retireOutInstExtra
  io.out.toRetire.instExtra.bhtMispred      := exceptOut.toRetire.inst0.bhtMispred
  io.out.toRetire.instExtra.breakPoint      := exceptOut.toRetire.inst0.breakpoint
  io.out.toRetire.instExtra.efPcValid       := exceptOut.toRetire.inst0.efPcValid
  io.out.toRetire.instExtra.exceptionVec    := exceptOut.toRetire.inst0.exceptionVec
  io.out.toRetire.instExtra.highHwException := exceptOut.toRetire.inst0.highHwException
  io.out.toRetire.instExtra.instMmuException:= exceptOut.toRetire.inst0.instMmuException
  io.out.toRetire.instExtra.instFlush       := exceptOut.toRetire.inst0.instFlush
  io.out.toRetire.instExtra.jmpMispred      := exceptOut.toRetire.inst0.jmpMispred
  io.out.toRetire.instExtra.mtval           := exceptOut.toRetire.inst0.mtval
  io.out.toRetire.instExtra.specFail        := exceptOut.toRetire.inst0.specFail
  io.out.toRetire.instExtra.specFailNoSsf   := exceptOut.toRetire.inst0.specFailNoSsf
  io.out.toRetire.instExtra.specFailSsf     := exceptOut.toRetire.inst0.specFailNoSsf
  io.out.toRetire.instExtra.vsetvl          := exceptOut.toRetire.inst0.vsetvl
  io.out.toRetire.instExtra.vstart          := exceptOut.toRetire.inst0.vstart
  io.out.toRetire.splitSpecFailSrt      := exceptOut.toRetire.splitSpecFailSrt
  io.out.toRetire.ssfIid                := exceptOut.toRetire.ssfIid
  io.out.toTop.ssfState              := exceptOut.toTop.ssfStateCur

  //==========================================================
  //                  Retire Entry Instance
  //==========================================================
  private val robRetire = Module(new RobRetire)
  private val retireIn = robRetire.io.in
  private val retireOut = robRetire.io.out
  retireIn.fromCp0 := io.in.fromCp0
  retireIn.fromExptEntry := exceptOutEntry
  retireIn.fromHad := io.in.fromHad
  retireIn.fromHpcp := io.in.fromHpcp
  retireIn.fromIdu.fenceIdle := io.in.fromIdu.fenceIdle
  retireIn.fromIfu := io.in.fromIfu
  retireIn.fromIu.pcFifoPopDataVec  := io.in.fromIu.pcFifoPopDataVec
  retireIn.fromIu.pipe0.iid         := io.in.fromIu.pipe0.iid
  retireIn.fromIu.pipe0.cmplt       := io.in.fromIu.pipe0.cmplt
  retireIn.fromIu.pipe0.abnormal    := io.in.fromIu.pipe0.abnormal
  retireIn.fromIu.pipe0.efPc        := io.in.fromIu.pipe0.efPc
  retireIn.fromIu.pipe1.iid         := io.in.fromIu.pipe1.iid
  retireIn.fromIu.pipe1.cmplt       := io.in.fromIu.pipe1.cmplt
  retireIn.fromIu.pipe2.iid         := io.in.fromIu.pipe2.iid
  retireIn.fromIu.pipe2.cmplt       := io.in.fromIu.pipe2.cmplt
  retireIn.fromIu.pipe2.abnormal    := io.in.fromIu.pipe2.abnormal
  retireIn.fromLsu.pipe3.cmplt   := io.in.fromLsu.pipe3.cmplt
  retireIn.fromLsu.pipe3.iid     := io.in.fromLsu.pipe3.iid
  retireIn.fromLsu.pipe3.abnormal:= io.in.fromLsu.pipe3.abnormal
  retireIn.fromLsu.pipe3.breakpointData:= io.in.fromLsu.pipe3.breakpointData
  retireIn.fromLsu.pipe3.noSpec  := io.in.fromLsu.pipe3.noSpec
  retireIn.fromLsu.pipe4.cmplt   := io.in.fromLsu.pipe4.cmplt
  retireIn.fromLsu.pipe4.iid     := io.in.fromLsu.pipe4.iid
  retireIn.fromLsu.pipe4.abnormal:= io.in.fromLsu.pipe4.abnormal
  retireIn.fromLsu.pipe4.breakpointData:= io.in.fromLsu.pipe4.breakpointData
  retireIn.fromLsu.pipe4.noSpec  := io.in.fromLsu.pipe4.noSpec
  retireIn.fromPad                  := io.in.fromPad
  retireIn.fromRetire               := io.in.fromRetire
  for (i <- 0 until NumRobReadEntry) {
    retireIn.fromRobReadData(i)    := robReadData(i)
    retireIn.fromRobReadIid(i)     := robReadIidVec(i)
  }
  retireIn.fromVfpu.pipe6          := io.in.fromVfpu.pipe6
  retireIn.fromVfpu.pipe7          := io.in.fromVfpu.pipe7
  io.out.toRetire.instVec         := retireOut.toRetire.instVec
  retireOutInstExtra := retireOut.toRetire.instExtra

  retireEntryUpdateGateClkValid   := retireOut.retire.updateGateClkValid
  retireEntryUpdateValidVec := retireOut.retire.entryUpdateValidVec
  retireExceptInst0Valid := retireOut.retire.except.inst0Valid
  retireExceptInst0Abnormal := retireOut.retire.except.inst0Abnormal
  robDebugCommit0 := retireOut.toRob.debugCommit0
  robExceptInst0Iid := retireOut.toRob.exceptInst0Iid
  retireOut.toPst.zipWithIndex.foreach {
    case (data, i) =>
      io.out.toPst.iid(i) := data.iid
      io.out.toPst.gateClkValid(i) := data.gateClkValid
  }
  io.out.toRetire.commitValidVec := retireOut.toRetire.commitValidVec
  io.out.toRetire.ctcFlushSrtEn := retireOut.toRetire.ctcFlushSrtEn
  io.out.toRetire.instVec   := retireOut.toRetire.instVec
  io.out.toRetire.intSrtEn  := retireOut.toRetire.intSrtEn
  io.out.toRetire.robCurPc  := retireOut.toRetire.robCurPc
  io.out.toTop.robCurPc     := retireOut.toTop.robCurPc
  io.out.toCpu              := retireOut.toCpu
  io.out.toHad              := retireOut.toHad
  io.out.toHpcp             := retireOut.toHpcp
  io.out.toIdu.retireInterruptValid := retireOut.toIdu.retireInterruptValid
  io.out.toIu               := retireOut.toIu
  io.out.toPad              := retireOut.toPad
  io.out.yyXx               := retireOut.yyXx

  //==========================================================
  //          to Difftest
  //==========================================================
  private val NumFoldInstMax = if (RobFoldEnable) NumFoldMax else 1
  if (DifftestEnable) {
    for (i <- 0 until NumCommitEntry) {
      val commitValid = io.out.toRetire.instVec(i).valid
      val instNum = io.out.toRetire.instVec(i).num
      val pcVec = io.out.toRetire.instVec(i).debug.pc
      val instVec = io.out.toRetire.instVec(i).debug.inst
      val RVCVec = io.out.toRetire.instVec(i).debug.RVC
      val rfwenVec = io.out.toRetire.instVec(i).debug.rfwen
      val wpdestVec = io.out.toRetire.instVec(i).debug.wpdest
      val wdestVec = io.out.toRetire.instVec(i).debug.wdest
      for (j <- 0 until NumFoldInstMax) {
        val instrCommit = Module(new DifftestInstrCommit)
        instrCommit.io.clock   := clock
        instrCommit.io.coreid  := 0.U
        instrCommit.io.index   := (i * NumFoldInstMax + j).U
        instrCommit.io.valid   := RegNext(commitValid && (j.U < instNum))
        instrCommit.io.pc      := RegNext(Cat(pcVec(j), 0.U(1.W))(63, 0) | BigInt("80000000", 16).U)
        instrCommit.io.instr   := RegNext(instVec(j))
        instrCommit.io.special := RegNext(0.U)
        instrCommit.io.skip    := RegNext(false.B)
        instrCommit.io.isRVC   := RegNext(RVCVec(j))
        instrCommit.io.rfwen   := RegNext(rfwenVec(j))
        instrCommit.io.fpwen   := RegNext(false.B)
        instrCommit.io.wdest   := RegNext(wdestVec(j))
        instrCommit.io.wpdest  := RegNext(wpdestVec(j))
      }
    }
    val cycleCnt = RegInit(0.U(64.W))
    cycleCnt := cycleCnt + 1.U
    val instrCnt = RegInit(0.U(64.W))
    when(io.out.toRetire.commitValidVec(0) || io.out.toRetire.commitValidVec(1) || io.out.toRetire.commitValidVec(2)) {
      instrCnt := instrCnt + io.out.toRetire.commitValidVec(0).asUInt + io.out.toRetire.commitValidVec(1).asUInt + io.out.toRetire.commitValidVec(2).asUInt
    }

    val trapEvent = Module(new DifftestTrapEvent)
    trapEvent.io.clock     := clock
    trapEvent.io.coreid    := 0.U
    trapEvent.io.valid     := false.B
    trapEvent.io.code      := RegNext(0.U)
    trapEvent.io.pc        := RegNext(0.U)
    trapEvent.io.cycleCnt  := RegNext(cycleCnt)
    trapEvent.io.instrCnt  := RegNext(instrCnt)
    trapEvent.io.hasWFI    := RegNext(false.B)
  }
}
