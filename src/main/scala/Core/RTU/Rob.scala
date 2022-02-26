package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.AddrConfig._
import Core.IntConfig._
import Core.ExceptionConfig._
import Core.PipelineConfig.NumPipeline
import Core.VectorUnitConfig._
import Utils.RingShiftLeft



class RobInput extends Bundle {
  val fromCp0 = new RobFromCp0Bundle
  val fromHad = new RobFromHadBundle
  val fromHpcp = new RobFromHpcpBundle
  val fromIdu = new Bundle() {
    val fenceIdle = Bool()
    val robCreate = Vec(NumCreateEntry, new RobCreateBundle)
  }
  val fromIfu = new RobFromIfu
  val fromIu = new Bundle() {
    val pcFifoPopDataVec = Vec(3, UInt(48.W))
    val pipe0 = new Bundle() {
      val iid = UInt(InstructionIdWidth.W)
      val cmplt = Bool()
      val abnormal = Bool()
      val breakPoint = Bool()
      val efPc = ValidIO(UInt(PcWidth.W))
      val exceptionVec = ValidIO(UInt(ExceptionVecWidth.W))
      val flush = Bool()
      val highHwException = Bool()
      val instMmuException = Bool()
      val mtval = UInt(MtvalWidth.W)
      val vsetvl = Bool()
      val vstart = ValidIO(UInt(VlmaxBits.W))
    }
    val pipe1 = new Bundle() {
      val iid = UInt(InstructionIdWidth.W)
      val cmplt = Bool()
    }
    val pipe2 = new Bundle() {
      val iid = UInt(InstructionIdWidth.W)
      val cmplt = Bool()
      val abnormal = Bool()
      val bhtMispred = Bool()
      val jmpMispred = Bool()
    }
  }
  val fromLsu = new Bundle() {
    val allCommitDataValid = Bool()
    val pipe3 = new RobFromLsuPipeCommonBundle {
      val vsetvl = Bool()
    }
    val pipe4 = new RobFromLsuPipeCommonBundle {}
  }
  val fromPad = new RobFromPad
  val fromRetire = new RobFromRetire {
    // Todo: imm
    val flushCurState = UInt(5.W)
    // Todo: imm
    val retireEmpty = Bool()
  }
  val fromRtu = new Bundle() {
    val yyXxFlush = Bool()
  }
  val fromVfpu = new RobFromVfpu
}

class RobOutput extends Bundle {
  // Todo: imm
  val pstRetire = Vec(3, new Bundle() {
    val iid = UInt(InstructionIdWidth.W)
    val iidUpdateVal = UInt(InstructionIdWidth.W)
    val gateClkValid = Bool()
  })
  val toRetire = new RobToRetireBundle {
    val interrupt = Bool()
    val splitSpecFailSrt = Bool()
    val ssfIid = UInt(InstructionIdWidth.W)
  }
  val toTop = new Bundle() {
    val commit0 = Bool()
    val commitStNoValid = Bool()
    val create0Iid = UInt(InstructionIdWidth.W)
    // Todo: imm
    val entryNum = UInt(7.W)
    // Todo: imm
    val flushCurState = UInt(5.W)
    val read0Iid = UInt(InstructionIdWidth.W)
    // Todo: imm
    val robCurPc = UInt(7.W)
    val robFull = Bool()
    // Todo: imm
    val ssfStateCur = UInt(2.W)
  }
  val toCpu = new RobToCpu
  // Todo: figure out
  val toHad = new RobToHad
  // Todo: figure out
  val toHpcp = new RobToHpcp
  val toIdu = new Bundle() {
    val retireInterruptValid = Bool()
    val robEmpty = Bool()
    val robFull = Bool()
    // Todo: check
    val robInstIidVec = Vec(NumCreateEntry, UInt(InstructionIdWidth.W))
  }
  val toIu = new RobToIu
  val toLsu = new Bundle() {
    // Todo: check
    val commitIidUpdateVal = Vec(NumRetireEntry, UInt(InstructionIdWidth.W))
  }
  val toPad = new RobToPad
  val yyXx = new RobYyXx
}

class RobIO extends Bundle {
  val in = Input(new RobInput)
  val out = Output(new RobOutput)
}

class Rob extends Module {
  val io = IO(new RobIO)

  /**
   * Regs
   */

  // Todo: imm
  val debugInfoFlushCurState = RegInit(0.U(5.W))
  val debugInfoRobCommit0 = RegInit(false.B)
  val debugInfoRobCommitStNoValid = RegInit(false.B)
  val debugInfoRobCreate0Iid = RegInit(0.U(InstructionIdWidth.W))
  val debugInfoRobRead0Iid = RegInit(0.U(InstructionIdWidth.W))
  // Todo: imm
  val debugInfoRobEntryNum = RegInit(0.U(7.W))
  val debugInfoRobFull = RegInit(false.B)

  // Todo: imm
  val readEntryUpdateDataVec = RegInit(VecInit(Seq.fill(3)(0.U(40.W))))
  val robCreatePtr = RegInit(0.U(RobPtrWidth.W))
  val robCreateIidVec = RegInit(VecInit(Seq.fill(NumCreateEntry)(0.U(InstructionIdWidth.W))))
  // Todo: imm
  val robEntryNum = RegInit(0.U(7.W))
  val robEntryNumAdd = RegInit(0.U(3.W))
  val robEntryNumSub = RegInit(0.U(2.W))
  val robFull = RegInit(false.B)
  // Todo: imm
  val robPopIidVec = RegInit(VecInit(Seq.fill(3)(0.U(InstructionIdWidth.W))))
  // Todo: imm
  val robReadPtr = RegInit(0.U(RobPtrNum.W))
  // Todo: imm
  val robReadEntryDataVec = RegInit(VecInit(Seq.fill(6)(0.U.asTypeOf(new RobEntryData))))
  val robReadIidVec = RegInit(VecInit(Seq.fill(6)(0.U(InstructionIdWidth.W))))
  val robReadEntryUpdateIid = RegInit(VecInit(Seq.fill(3)(0.U(6.W))))

  //==========================================================
  //                   ROB Entry Instance
  //==========================================================

  val entryCmpltValidVec        = Wire(Vec(NumRobEntry, UInt(7.W)))
  val entryCmpltGateClkValidVec = Wire(Vec(NumRobEntry, Bool()))
  val entryCreateDpEnVec        = Wire(Vec(NumRobEntry, Bool()))
  val entryCreateEnVec          = Wire(Vec(NumRobEntry, Bool()))
  val entryCreateGateClkEnVec   = Wire(Vec(NumRobEntry, Bool()))
  // Todo: imm
  val entryCreateSelVec         = Wire(Vec(NumRobEntry, UInt(4.W)))
  val entryPopEnVec             = Wire(Vec(NumRobEntry, Bool()))
  val entryReadDateVec          = Wire(Vec(NumRobEntry, new RobEntryData))
  for (i <- 0 until NumRobEntry) {
    entryCmpltGateClkValidVec(i) := entryCmpltValidVec(i).asUInt.orR
  }

  val robEntryVec = Seq.fill(NumRobEntry)(Module(new RobEntry))
  for (i <- 0 until NumRobEntry) {
    val in = robEntryVec(i).io.in
    val out = robEntryVec(i).io.out
    in.fromCp0 := io.in.fromCp0
    in.fromIdu := io.in.fromIdu
    in.fromLsu := io.in.fromLsu
    in.fromPad := io.in.fromPad
    in.fromRetire := io.in.fromRetire
    in.x.cmpltGateClkValid :=  entryCmpltGateClkValidVec(i)
    in.x.cmpltValidVec := entryCmpltValidVec(i)
    in.x.createDpEn := entryCreateDpEnVec(i)
    in.x.createEn := entryCreateEnVec(i)
    in.x.createGateClkEn := entryCreateGateClkEnVec(i)
    in.x.createSel := entryCreateSelVec(i)
    in.x.popEn := entryPopEnVec(i)
    entryReadDateVec(i) := out.readData
  }

  val readEntryCreateDataVec = Wire(Vec(NumRobReadEntry, new RobEntryData))
  val miscCmpltGateClkEn = Wire(Bool())

  val readEntryCmpltValidVec        = Wire(Vec(NumRobReadEntry, UInt(7.W)))
  val readEntryCmpltGateClkValidVec = Wire(Vec(NumRobReadEntry, Bool()))
  val readEntryCreateDpEnVec        = Wire(Vec(NumRobReadEntry, Bool()))
  val readEntryCreateEnVec          = Wire(Vec(NumRobReadEntry, Bool()))
  val readEntryCreateGateClkEnVec   = Wire(Vec(NumRobReadEntry, Bool()))
  // Todo: imm
  val readEntryCreateSelVec         = Wire(Vec(NumRobReadEntry, UInt(4.W)))
  val readEntryPopEnVec             = Wire(Vec(NumRobReadEntry, Bool()))
  val readEntryReadDataVec          = Wire(Vec(NumRobReadEntry, new RobEntryData))


  val robReadEntryVec = Seq.fill(NumRobReadEntry)(Module(new RobEntry))
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
    in.fromLsu.pipe3.wbBreakpointAData  := io.in.fromLsu.pipe3.wb.breakPointAData
    in.fromLsu.pipe3.wbBreakpointBData  := io.in.fromLsu.pipe3.wb.breakPointBData
    in.fromLsu.pipe3.wbNoSpecHit        := io.in.fromLsu.pipe3.wb.noSpecHit
    in.fromLsu.pipe3.wbNoSpecMispred    := io.in.fromLsu.pipe3.wb.noSpecMispred
    in.fromLsu.pipe3.wbNoSpecMiss       := io.in.fromLsu.pipe3.wb.noSpecMiss
    in.fromLsu.pipe4.wbBreakpointAData  := io.in.fromLsu.pipe4.wb.breakPointAData
    in.fromLsu.pipe4.wbBreakpointBData  := io.in.fromLsu.pipe4.wb.breakPointBData
    in.fromLsu.pipe4.wbNoSpecHit        := io.in.fromLsu.pipe4.wb.noSpecHit
    in.fromLsu.pipe4.wbNoSpecMispred    := io.in.fromLsu.pipe4.wb.noSpecMispred
    in.fromLsu.pipe4.wbNoSpecMiss       := io.in.fromLsu.pipe4.wb.noSpecMiss
    in.fromPad                          := io.in.fromPad
    in.fromRetire.flush                 := io.in.fromRetire.flush
    in.fromRetire.flushGateClk          := io.in.fromRetire.flushGateClk
    in.x.cmpltGateClkValid              := readEntryCmpltGateClkValidVec(i)
    in.x.cmpltValidVec                  := readEntryCmpltValidVec(i)
    in.x.createDpEn                     := readEntryCreateDpEnVec(i)
    in.x.createEn                       := readEntryCreateEnVec(i)
    in.x.createGateClkEn                := readEntryCreateGateClkEnVec(i)
    in.x.createSel                      := readEntryCreateSelVec(i)
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
  val robCreatePtrVec = Wire(Vec(4, UInt(RobPtrNum.W)))
  robCreatePtrVec(0) := robCreatePtr
  val robCreateEn = Wire(UInt(RobPtrWidth.W))
  val robCreateDpEn = Wire(UInt(RobPtrWidth.W))
  val robCreateGateClkEn = Wire(UInt(RobPtrWidth.W))

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
    robCreatePtr := RingShiftLeft(robCreatePtr, 4)
    robCreateIidVec.foreach(iid => iid := iid + 4.U)
  }.elsewhen(robCreatePtrAdd(3)) {
    robCreatePtr := RingShiftLeft(robCreatePtr, 3)
    robCreateIidVec.foreach(iid => iid := iid + 3.U)
  }.elsewhen(robCreatePtrAdd(2)) {
    robCreatePtr := RingShiftLeft(robCreatePtr, 2)
    robCreateIidVec.foreach(iid => iid := iid + 2.U)
  }.elsewhen(robCreatePtrAdd(1)) {
    robCreatePtr := RingShiftLeft(robCreatePtr, 1)
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
  val robPopPtrAdd = Wire(Vec(RobPtrNum, Bool()))

  robEntryNumAdd := OHToUInt(PriorityEncoderOH(robCreatePtrAdd))
  robEntryNumSub := OHToUInt(PriorityEncoderOH(robPopPtrAdd.asUInt(RobPtrNum-1,0)))

  // Create or pop at least 1 entry
  val robEntryNumUpdateValid = robCreatePtrAdd(1) || robPopPtrAdd(1)
  val robEntryNumUpdate = robEntryNum + robEntryNumAdd - robEntryNumSub

  when (io.in.fromRetire.flush) {
    robEntryNum := 0.U
  }.elsewhen(robEntryNumUpdateValid) {
    robEntryNum := robEntryNumUpdate
  }

  val robEmpty = robEntryNum === 0.U
  val rob1Entry = robEntryNum === 1.U
  val rob2Entry = robEntryNum === 2.U
  io.out.toIdu.robEmpty := robEmpty && io.in.fromRetire.retireEmpty

  //----------------------------------------------------------
  //                     ROB Full signal
  //----------------------------------------------------------
  val robFullUpdateVal = robEntryNumUpdate > (NumRobEntry - NumCreateEntry).U

  when(io.in.fromRetire.flush) {
    robFull := false.B
  }.elsewhen(robEntryNumUpdateValid) {
    robFull := robFullUpdateVal
  }

  io.out.toIdu.robFull := robFull

  //----------------------------------------------------------
  //                   read vld bit
  //----------------------------------------------------------
  val entryValid = Wire(Vec(NumRobEntry, Bool()))
  entryValid := entryReadDateVec.map(_.ctrl.valid)

  // Todo: figure out why not use robEntryNum directly
  io.out.toHad.robEmpty := !entryValid.asUInt.orR

  //==========================================================
  //                    ROB Complete Port
  //==========================================================

  // Todo: imm: num of pipeline
  val PipeIidLsbVec = Wire(Vec(7, UInt((InstructionIdWidth-1).W)))

  PipeIidLsbVec(0) := io.in.fromIu.pipe0.iid(5,0)
  PipeIidLsbVec(1) := io.in.fromIu.pipe1.iid(5,0)
  PipeIidLsbVec(2) := io.in.fromIu.pipe2.iid(5,0)
  PipeIidLsbVec(3) := io.in.fromLsu.pipe3.wb.iid(5,0)
  PipeIidLsbVec(4) := io.in.fromLsu.pipe4.wb.iid(5,0)
  PipeIidLsbVec(5) := io.in.fromVfpu.pipe6.iid(5,0)
  PipeIidLsbVec(6) := io.in.fromVfpu.pipe7.iid(5,0)

  // Todo: imm: num of pipeline
  val completeOHVec : Vec[UInt] = Wire(Vec(7, UInt(NumRobEntry.W)))


  completeOHVec(0) := UIntToOH(PipeIidLsbVec(0)) & Fill(NumRobEntry, io.in.fromIu.pipe0.cmplt)
  completeOHVec(1) := UIntToOH(PipeIidLsbVec(1)) & Fill(NumRobEntry, io.in.fromIu.pipe1.cmplt)
  completeOHVec(2) := UIntToOH(PipeIidLsbVec(2)) & Fill(NumRobEntry, io.in.fromIu.pipe2.cmplt)
  completeOHVec(3) := UIntToOH(PipeIidLsbVec(3)) & Fill(NumRobEntry, io.in.fromLsu.pipe3.wb.cmplt)
  completeOHVec(4) := UIntToOH(PipeIidLsbVec(4)) & Fill(NumRobEntry, io.in.fromLsu.pipe4.wb.cmplt)
  completeOHVec(5) := UIntToOH(PipeIidLsbVec(5)) & Fill(NumRobEntry, io.in.fromVfpu.pipe6.cmplt)
  completeOHVec(6) := UIntToOH(PipeIidLsbVec(6)) & Fill(NumRobEntry, io.in.fromVfpu.pipe7.cmplt)

  for (i <- 0 until NumRobEntry) {
    entryCmpltValidVec(i) := Cat(completeOHVec.map(completeOH => completeOH(i)))
  }

  for (i <- 0 until NumRobEntry) {
    entryCmpltGateClkValidVec(i) := entryCmpltValidVec(i).asUInt
  }

  //----------------------------------------------------------
  //                 lsu cmplt info gateclk en
  //----------------------------------------------------------
  miscCmpltGateClkEn :=
    io.in.fromLsu.pipe3.wb.cmplt &&
      (io.in.fromLsu.pipe3.wb.breakPointAData || io.in.fromLsu.pipe3.wb.breakPointBData ||
      io.in.fromLsu.pipe3.wb.noSpecHit || io.in.fromLsu.pipe3.wb.noSpecMiss || io.in.fromLsu.pipe3.wb.noSpecMispred) ||
    io.in.fromLsu.pipe4.wb.cmplt &&
      (io.in.fromLsu.pipe4.wb.breakPointAData || io.in.fromLsu.pipe4.wb.breakPointBData ||
      io.in.fromLsu.pipe4.wb.noSpecHit || io.in.fromLsu.pipe4.wb.noSpecMiss || io.in.fromLsu.pipe4.wb.noSpecMispred)

  //==========================================================
  //              Read Port for Retire Entry
  //==========================================================

  //----------------------------------------------------------
  //                    5 Read Ports
  //----------------------------------------------------------
  val robReadPtrVec = Wire(Vec(RobReadPtrNum, UInt(NumRobEntry.W)))

  for (i <- 0 until NumRobReadEntry) {
    when(robReadPtrVec(i).orR) {
      robReadEntryDataVec(i) := entryReadDateVec(OHToUInt(robReadPtrVec(i)))
    }.otherwise{
      robReadEntryDataVec(i) := 0.U
    }
  }

  //==========================================================
  //              Read Port for read entry update
  //==========================================================
  //----------------------------------------------------------
  //              Read entry update data select
  //----------------------------------------------------------

  // Todo: init
  val retireEntryUpdateValidVec = Wire(Vec(3, Bool()))
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
  // Todo: imm
  val robCreateToReadEntryEnVec = Wire(Vec(NumRobReadEntry, Bool()))
  val robCreateToReadEntryDpEnVec = Wire(Vec(NumRobReadEntry, Bool()))
  val robCreateToReadEntryGateClkEn = Wire(Vec(NumRobReadEntry, Bool()))
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
  val retireEntryUpdateGateClkValid = Wire(Bool())

  for (i <- 0 until NumRobReadEntry) {
    readEntryCreateEnVec(i) := retireEntryUpdateValidVec(0) || robCreateToReadEntryEnVec(i)
    readEntryCreateDpEnVec(i) := retireEntryUpdateValidVec(0) || robCreateToReadEntryDpEnVec(i)
    readEntryCreateGateClkEnVec(i) := retireEntryUpdateGateClkValid || robCreateToReadEntryGateClkEn(i)
    readEntryCreateSelVec(i) := "b0001".U
  }

  //----------------------------------------------------------
  //                Read entry complete port
  //----------------------------------------------------------

  val pipeCompleteVec = Wire(Vec(NumPipeline, Bool()))
  val pipeIidVec = Wire(Vec(NumPipeline, UInt(NumRobEntryBits.W)))
  pipeCompleteVec(0) := io.in.fromIu.pipe0.cmplt
  pipeCompleteVec(1) := io.in.fromIu.pipe1.cmplt
  pipeCompleteVec(2) := io.in.fromIu.pipe2.cmplt
  pipeCompleteVec(3) := io.in.fromLsu.pipe3.wb.cmplt
  pipeCompleteVec(4) := io.in.fromLsu.pipe4.wb.cmplt
  pipeCompleteVec(5) := io.in.fromVfpu.pipe6.cmplt
  pipeCompleteVec(6) := io.in.fromVfpu.pipe7.cmplt
  pipeIidVec(0) := io.in.fromIu.pipe0.iid(NumRobEntryBits - 1, 0)
  pipeIidVec(1) := io.in.fromIu.pipe1.iid(NumRobEntryBits - 1, 0)
  pipeIidVec(2) := io.in.fromIu.pipe2.iid(NumRobEntryBits - 1, 0)
  pipeIidVec(3) := io.in.fromLsu.pipe3.wb.iid
  pipeIidVec(4) := io.in.fromLsu.pipe4.wb.iid
  pipeIidVec(5) := io.in.fromVfpu.pipe6.iid
  pipeIidVec(6) := io.in.fromVfpu.pipe7.iid


  for (i <- 0 until NumRobReadEntry) {
    readEntryCmpltValidVec(i) := Cat(pipeCompleteVec.zip(pipeIidVec).map{
      case (cmplt, iid) => cmplt && iid === robReadEntryUpdateIid(i)
    })
  }
  val readEntryCmpltGateClkValid = pipeCompleteVec.asUInt.orR
  readEntryCmpltGateClkValidVec.foreach(_ := readEntryCmpltGateClkValid)

  //----------------------------------------------------------
  //                   Read Entry Read
  //----------------------------------------------------------
  val robReadData = readEntryReadDataVec

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
  val robReadPtrAddVec = Wire(Vec(4, Bool()))
  robReadPtrAddVec(0) := false.B
  robReadPtrAddVec(1) := retireEntryUpdateValidVec(0)
  robReadPtrAddVec(2) := retireEntryUpdateValidVec(1)
  robReadPtrAddVec(3) := retireEntryUpdateValidVec(2)

  when(io.in.fromRetire.flush) {
    robReadPtr := 1.U(NumRobEntry.W)
    // Todo: imm
    for (i <- 0 until 6) {
      robReadIidVec(i) := i.U
    }
  }.elsewhen(robReadPtrAddVec(3)) {
    robReadPtr := RingShiftLeft(robReadPtr, 3)
    // Todo: imm
    for (i <- 0 until 6) {
      robReadIidVec(i) := robReadIidVec(i) + 3.U
    }
  }.elsewhen(robReadPtrAddVec(2)) {
    robReadPtr := RingShiftLeft(robReadPtr, 2)
    // Todo: imm
    for (i <- 0 until 6) {
      robReadIidVec(i) := robReadIidVec(i) + 2.U
    }
  }.elsewhen(robReadPtrAddVec(3)) {
    robReadPtr := RingShiftLeft(robReadPtr, 1)
    // Todo: imm
    for (i <- 0 until 6) {
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
    io.out.pstRetire(i).iidUpdateVal := robReadIidVec(i)
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
  val robPopPtrVec = Wire(Vec(3, UInt(NumRobEntryBits.W)))

  entryPopEnVec :=
    (Fill(NumRobEntry, io.out.yyXx.retire(0)) & UIntToOH(robPopPtrVec(0))) |
    (Fill(NumRobEntry, io.out.yyXx.retire(1)) & UIntToOH(robPopPtrVec(1))) |
    (Fill(NumRobEntry, io.out.yyXx.retire(2)) & UIntToOH(robPopPtrVec(2)))

  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------

  // Todo: gated clk for pop ptr

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
  val debugInfoClkEn = io.in.fromRetire.flushGateClk
  // Todo: gated clk for debug info

  //----------------------------------------------------------
  //                   ROB debug info
  //----------------------------------------------------------
  val robDebugCommit0 = Wire(Bool())
  when(io.in.fromRetire.flushGateClk) {
    debugInfoRobFull            := robFull
    debugInfoRobRead0Iid        := robReadIidVec(0)
    debugInfoRobCreate0Iid      := robCreateIidVec(0)
    debugInfoRobEntryNum        := robEntryNum
    debugInfoRobCommit0         := robDebugCommit0
    debugInfoRobCommitStNoValid := !io.in.fromLsu.allCommitDataValid
    debugInfoFlushCurState      := io.in.fromRetire.flushCurState
  }.otherwise {
    // maintain
  }

  io.out.toTop.robFull          := debugInfoRobFull
  io.out.toTop.read0Iid         := debugInfoRobRead0Iid
  io.out.toTop.create0Iid       := debugInfoRobCreate0Iid
  io.out.toTop.entryNum         := debugInfoRobEntryNum
  io.out.toTop.commit0          := debugInfoRobCommit0
  io.out.toTop.commitStNoValid  := debugInfoRobCommitStNoValid
  io.out.toTop.flushCurState    := debugInfoFlushCurState

  //==========================================================
  //                   Expt Entry Instance
  //==========================================================


  val robExcept = Module(new RobExcept)
  val exceptIn = robExcept.io.in
  val exceptOut = robExcept.io.out

  val exceptOutEntry = Wire(new RobExceptEntryBundle)
  val retireExceptInst0Abnormal = Wire(Bool())
  val retireExceptInst0Valid = Wire(Bool())
  val robRetireExceptInst0Iid = Wire(UInt(InstructionIdWidth.W))
  val robRetireInst0Split = WireInit(io.out.toRetire.instVec(0).split)

  exceptIn.fromCp0                  := io.in.fromCp0
  exceptIn.fromIu.pipe0             := io.in.fromIu.pipe0
  exceptIn.fromIu.pipe2             := io.in.fromIu.pipe2
  exceptIn.fromLsu.pipe3            := io.in.fromLsu.pipe3
  exceptIn.fromLsu.pipe4            := io.in.fromLsu.pipe4
  exceptIn.fromPad                  := io.in.fromPad
  exceptIn.fromRetire.inst0Abnormal := retireExceptInst0Abnormal
  exceptIn.fromRetire.inst0Valid    := retireExceptInst0Valid
  exceptIn.fromRetire.flush         := io.in.fromRetire.flush
  exceptIn.fromRob.inst0Iid         := robRetireExceptInst0Iid
  exceptIn.fromRob.inst0Split       := robRetireInst0Split
  exceptIn.fromRtu.yyXxFlush        := io.in.fromRtu.yyXxFlush

  exceptOutEntry                            := exceptOut.except
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
//  io.out.toRetire.instExtra.split           := exceptOut.toRetire.inst0.split
  io.out.toRetire.instExtra.vsetvl          := exceptOut.toRetire.inst0.vsetvl
  io.out.toRetire.instExtra.vstart          := exceptOut.toRetire.inst0.vstart
  io.out.toRetire.splitSpecFailSrt      := exceptOut.toRetire.splitSpecFailSrt
  io.out.toRetire.ssfIid                := exceptOut.toRetire.ssfIid
  io.out.toTop.ssfStateCur              := exceptOut.toTop.ssfStateCur

  //==========================================================
  //                  Retire Entry Instance
  //==========================================================
  val robRoute = Module(new RobRoute)
  val routeIn = robRoute.io.in
  val routeOut = robRoute.io.out
  routeIn.fromCp0 := io.in.fromCp0
  routeIn.fromExptEntry := exceptOutEntry
  routeIn.fromHad := io.in.fromHad
  routeIn.fromHpcp := io.in.fromHpcp
  routeIn.fromIdu.fenceIdle := io.in.fromIdu.fenceIdle
  routeIn.fromIfu := io.in.fromIfu
  routeIn.fromIu.pcFifoPopDataVec := io.in.fromIu.pcFifoPopDataVec
  routeIn.fromIu.pipe0.iid        := io.in.fromIu.pipe0.iid
  routeIn.fromIu.pipe0.cmplt      := io.in.fromIu.pipe0.cmplt
  routeIn.fromIu.pipe0.abnormal   := io.in.fromIu.pipe0.abnormal
  routeIn.fromIu.pipe0.efPc       := io.in.fromIu.pipe0.efPc
  routeIn.fromIu.pipe1.iid        := io.in.fromIu.pipe1.iid
  routeIn.fromIu.pipe1.cmplt      := io.in.fromIu.pipe1.cmplt
  routeIn.fromIu.pipe2.iid        := io.in.fromIu.pipe2.iid
  routeIn.fromIu.pipe2.cmplt      := io.in.fromIu.pipe2.cmplt
  routeIn.fromIu.pipe2.abnormal   := io.in.fromIu.pipe2.abnormal
  routeIn.fromLsu                 := io.in.fromLsu
  routeIn.fromPad                 := io.in.fromPad
  routeIn.fromRetire              := io.in.fromRetire
  for (i <- 0 until NumRobReadEntry) {
    routeIn.fromRobReadData(i)    := robReadData(i)
    routeIn.fromRobReadIid(i)     := robReadIidVec(i)
  }
  routeIn.fromVfpu.pipe6          := io.in.fromVfpu.pipe6
  routeIn.fromVfpu.pipe7          := io.in.fromVfpu.pipe7
  io.out.toRetire.instVec         := routeOut.toRetire.instVec
  io.out.toRetire.instExtra       := routeOut.toRetire.instExtra
  // Todo: RobRoute output

}
