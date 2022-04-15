package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.AddrConfig._
import Core.IntConfig._
import Core.ExceptionConfig._
import Core.PipelineConfig.NumPipeline
import Core.VectorUnitConfig._

class PcFifoData extends Bundle {
  val length      : Bool = Bool() // 47
  val bhtPred     : Bool = Bool() // 46 ifu forward
  val bhtMispred  : Bool = Bool() // 44
  val jmp         : Bool = Bool() // 43
  val pret        : Bool = Bool() // 42 jalr
  val pcall       : Bool = Bool() // 41 jalr
  val condBranch  : Bool = Bool() // 40
  val pcNext      : UInt = UInt(PcWidth.W) // 39:1
  val lsb         : Bool = Bool()
  def checkIdx : UInt = pcNext(10, 3)
}

class RobRetireFromLsuBundle extends Bundle {
  val iid       : UInt = UInt(InstructionIdWidth.W)
  val cmplt     : Bool = Bool()
  val abnormal  : Bool = Bool()
  val breakpointData = new RobBreakpointDataBundle
  val noSpec  = new RobNoSpecBundle
}

class RobRetireInput extends Bundle {
  val fromCp0       = new RobFromCp0Bundle
  val fromExptEntry = new RobExceptEntryBundle
  val fromHad       = new RobFromHadBundle
  val fromHpcp      = new RtuFromHpcpBundle
  val fromIdu       = new Bundle() {
    val fenceIdle   : Bool = Bool()
  }
  val fromIfu       = new RobFromIfu
  val fromIu        = new Bundle() {
    // Todo: imm
    val pcFifoPopDataVec : Vec[PcFifoData] = Vec(3, new PcFifoData)
    val pipe0       = new Bundle() {
      val iid       : UInt = UInt(InstructionIdWidth.W)
      val cmplt     : Bool = Bool()
      val abnormal  : Bool = Bool()
      val efPc      : ValidIO[UInt] = ValidIO(UInt(PcWidth.W))
    }
    val pipe1       = new Bundle() {
      val iid       : UInt = UInt(InstructionIdWidth.W)
      val cmplt     : Bool = Bool()
    }
    val pipe2       = new Bundle() {
      val iid       : UInt = UInt(InstructionIdWidth.W)
      val cmplt     : Bool = Bool()
      val abnormal  : Bool = Bool()
    }
  }
  val fromLsu       = new Bundle() {
    val pipe3       = new RobRetireFromLsuBundle
    val pipe4       = new RobRetireFromLsuBundle
  }
  val fromPad       = new RtuFromPadBundle
  val fromRetire    = new RobFromRetire {}
  val fromRobReadData : Vec[RobEntryData] = Vec(NumRobReadEntry, new RobEntryData)
  val fromRobReadIid  : Vec[UInt] = Vec(NumRobReadEntry, UInt(InstructionIdWidth.W))

  val fromVfpu = new RobFromVfpu
}

class RobRetireOutput extends Bundle {
  val retire = new Bundle() {
    val updateGateClkValid  : Bool = Bool()
    val entryUpdateValidVec : Vec[Bool] = Vec(NumRobReadEntry, Bool())
    val except = new Bundle() {
      val inst0Abnormal : Bool = Bool()
      val inst0Valid    : Bool = Bool()
    }
  }

  val toRob = new Bundle() {
    val debugCommit0    : Bool = Bool()
    // Todo: check if needed move into retire bundle
    val exceptInst0Iid  : UInt = UInt(InstructionIdWidth.W)
  }
  val toPst       = Vec(3, new Bundle() {
    val iid           : UInt = UInt(InstructionIdWidth.W)
    val gateClkValid  : Bool = Bool()
  })
  val toRetire = new RobToRetireBundle {}
  val toTop = new Bundle() {
    val robCurPc : UInt = UInt(7.W)
  }
  val toCpu = new RobToCpu
  val toHad = new RobToHad
  val toHpcp = new RobToHpcp
  val toIdu = new Bundle() {
    val retireInterruptValid : Bool = Bool()
  }
  val toIu = new RobToIu
  val toPad = new RobToPad
  val yyXx = new RobYyXx
}

class RobRetireIO extends Bundle {
  val in  : RobRetireInput   = Input(new RobRetireInput)
  val out : RobRetireOutput  = Output(new RobRetireOutput)
}

class RobRetire extends Module {
  /**
   * Config
   */
  def RetireEntryDataBits = 52
  def PcAddend1Bits = 5
  def PcFifoPopBits = 48
  def RtBits = 52
  /**
   *   Config for fpga
   */
  def fpga = false

  val io : RobRetireIO = IO(new RobRetireIO)
  class DebugRetireInfo extends Bundle {
    // Todo: imm
    val info        : UInt = UInt(22.W)
    val jmpPcOffset : UInt = UInt(17.W)
    val pc          : UInt = UInt(PcWidth.W)
  }

  class ExtraRetireEntryContent extends Bundle {
    val abnormal        : Bool = Bool()
    val bjuLength       : Bool = Bool()
    val ctcFlush        : Bool = Bool()
    val dataBreakpoint  : Bool = Bool()
    val instBreakpoint  : Bool = Bool()
    val debugDisable    : Bool = Bool()
    val intMask         : Bool = Bool()
    val pcal            : Bool = Bool()
    val ras             : Bool = Bool()
    val pcCur           : UInt = UInt(PcWidth.W)
    val interruptVec    : ValidIO[UInt] = ValidIO(UInt(InterruptVecWidth.W))
  }

  class NormalRetireEntryContent extends Bundle {
    val pcAddend0 : UInt = UInt(PcWidth.W)
    val pcAddend1 : UInt = UInt(PcAddend1Bits.W)
    val data      = new RetireEntryData
  }

  class RetireEntryData extends Bundle {
    val vlPred          : Bool = Bool()
    val vl              : UInt = UInt(VlmaxBits.W)
    val vecDirty        : Bool = Bool()
    val vsetvli         : Bool = Bool()
    val vsew            : UInt = UInt(VsewBits.W)
    val vlmul           : UInt = UInt(VlmulBits.W)
    val noSpec          = new RobNoSpecBundle
    val load            : Bool = Bool()
    val fpDirty         : Bool = Bool()
    val breakpointInst  = new RobBreakpointInstBundle
    val breakpointData  = new RobBreakpointDataBundle
    val store           : Bool = Bool()
    val instNum         : UInt = UInt(2.W)
    val pret            : Bool = Bool() // from pcfifo data
    val pcOffset        : UInt = UInt(RobPcOffsetBits.W)
    val jmp             : Bool = Bool() // from pcfifo data
    val condBrTaken     : Bool = Bool() // from pcfifo data
    val condBranch      : Bool = Bool() // from pcfifo data
    // Todo: imm
    val checkIdx        : UInt = UInt(8.W) // from pcfifo data
    val bju             : Bool = Bool()
    val split           : Bool = Bool()
    val iid             : UInt = UInt(InstructionIdWidth.W)
  }

  /**
   * Regs
   */

  private val debugRetireInfoVec : Vec[ValidIO[DebugRetireInfo]] =
    RegInit(VecInit(Seq.fill(NumRetireEntry)(0.U.asTypeOf(ValidIO(new DebugRetireInfo)))))
  private val noRetire : Bool = RegInit(false.B)

  private val retireCnt : UInt = if(fpga) RegInit(0.U(32.W)) else null
  private val retireInstVec = RegInit(VecInit(
    Seq.fill(NumRetireEntry)(0.U.asTypeOf(ValidIO(new NormalRetireEntryContent)))
  ))
  private val retireInst0Special = RegInit(0.U.asTypeOf(Output(new ExtraRetireEntryContent)))

  private val retirePstPregInstValidVec = RegInit(VecInit(Seq.fill(NumRetireEntry)(false.B)))
  private val retirePstEregInstValidVec = RegInit(VecInit(Seq.fill(NumRetireEntry)(false.B)))
  private val retirePstVregInstValidVec = RegInit(VecInit(Seq.fill(NumRetireEntry)(false.B)))

  // Todo: Rename
  private val retireRetireInstValidVec = RegInit(VecInit(Seq.fill(NumRetireEntry)(false.B)))

  private val robCommitIid = Reg(Vec(NumCommitEntry, ValidIO(UInt(InstructionIdWidth.W))))

  private val robPcCurAddend0 = RegInit(0.U(PcWidth.W))
  private val robPcCurAddend1 = RegInit(0.U(PcAddend1Bits.W))
  private val robRead2PcCurAddend0 = RegInit(0.U(PcWidth.W))

  /**
   * Wires
   */

  private val robRead2PcNextAddend0 = WireInit(0.U(PcWidth.W))
  private val robRead2PcNextAddend1 = WireInit(0.U(PcAddend1Bits.W))
  private val robRead2PcFifoData = WireInit(0.U(PcFifoPopBits.W))

  //==========================================================
  //            Retire Entry Valid update signals
  //==========================================================
  //----------------------------------------------------------
  //                   rename for input
  //----------------------------------------------------------

  private val robReadDataVec = WireInit(io.in.fromRobReadData)
  private val robReadIidVec = WireInit(io.in.fromRobReadIid)
  // output for pst iid update
  for (i <- 0 until NumRobReadEntry) {
    io.out.toPst(i).gateClkValid := robReadDataVec(i).ctrl.valid
  }
  private val pipeCmpltVec = Wire(Vec(NumPipeline, Bool()))
  pipeCmpltVec(0) := io.in.fromIu.pipe0.cmplt
  pipeCmpltVec(1) := io.in.fromIu.pipe1.cmplt
  pipeCmpltVec(2) := io.in.fromIu.pipe2.cmplt
  pipeCmpltVec(3) := io.in.fromLsu.pipe3.cmplt
  pipeCmpltVec(4) := io.in.fromLsu.pipe4.cmplt
  pipeCmpltVec(5) := io.in.fromVfpu.pipe6.cmplt
  pipeCmpltVec(6) := io.in.fromVfpu.pipe7.cmplt
  private val pipeIidVec = Wire(Vec(NumPipeline, UInt(InstructionIdWidth.W)))
  pipeIidVec(0) := io.in.fromIu.pipe0.iid
  pipeIidVec(1) := io.in.fromIu.pipe1.iid
  pipeIidVec(2) := io.in.fromIu.pipe2.iid
  pipeIidVec(3) := io.in.fromLsu.pipe3.iid
  pipeIidVec(4) := io.in.fromLsu.pipe4.iid
  pipeIidVec(5) := io.in.fromVfpu.pipe6.iid
  pipeIidVec(6) := io.in.fromVfpu.pipe7.iid
  private val pipeAbnormalVec = Wire(Vec(NumPipeline, Bool()))
  private val abnormalPipeNumSeq = Seq(0, 2, 3, 4)
  pipeAbnormalVec(0) := io.in.fromIu.pipe0.abnormal
  pipeAbnormalVec(1) := false.B
  pipeAbnormalVec(2) := io.in.fromIu.pipe2.abnormal
  pipeAbnormalVec(3) := io.in.fromLsu.pipe3.abnormal
  pipeAbnormalVec(4) := io.in.fromLsu.pipe4.abnormal
  pipeAbnormalVec(5) := false.B
  pipeAbnormalVec(6) := false.B
  private val pipeBreakpointVec = Wire(Vec(NumPipeline, new RobBreakpointDataBundle))
  private val breakpointDataPipeSeq = Seq(3, 4)
  pipeBreakpointVec(0) := 0.U.asTypeOf(new RobBreakpointDataBundle)
  pipeBreakpointVec(1) := 0.U.asTypeOf(new RobBreakpointDataBundle)
  pipeBreakpointVec(2) := 0.U.asTypeOf(new RobBreakpointDataBundle)
  pipeBreakpointVec(3) := io.in.fromLsu.pipe3.breakpointData
  pipeBreakpointVec(4) := io.in.fromLsu.pipe4.breakpointData
  pipeBreakpointVec(5) := 0.U.asTypeOf(new RobBreakpointDataBundle)
  pipeBreakpointVec(6) := 0.U.asTypeOf(new RobBreakpointDataBundle)
  private val pipeNoSpecVec = Wire(Vec(NumPipeline, new RobNoSpecBundle))
  private val noSpecPipeSeq = Seq(3, 4)
  pipeNoSpecVec(0) := 0.U.asTypeOf(new RobNoSpecBundle)
  pipeNoSpecVec(1) := 0.U.asTypeOf(new RobNoSpecBundle)
  pipeNoSpecVec(2) := 0.U.asTypeOf(new RobNoSpecBundle)
  pipeNoSpecVec(3) := io.in.fromLsu.pipe3.noSpec
  pipeNoSpecVec(4) := io.in.fromLsu.pipe4.noSpec
  pipeNoSpecVec(5) := 0.U.asTypeOf(new RobNoSpecBundle)
  pipeNoSpecVec(6) := 0.U.asTypeOf(new RobNoSpecBundle)

  //----------------------------------------------------------
  //               ROB read 0/1/2 completing
  //----------------------------------------------------------
  private val robReadPipeCmpltVec = Wire(Vec(NumRobReadEntry, Vec(NumPipeline, Bool())))
  private val robReadExceptEntryValidVec = Wire(Vec(NumRobReadEntry, Bool()))


  for (i <- 0 until NumRobReadEntry) {
    for (j <- 0 until NumPipeline) {
      // pipe(j) read(i) complete
      robReadPipeCmpltVec(i)(j) := pipeCmpltVec(i) && robReadIidVec(i) === pipeIidVec(i)
    }
    robReadExceptEntryValidVec(i):= io.in.fromExptEntry.valid && robReadIidVec(i) === io.in.fromExptEntry.iid
  }

  //----------------------------------------------------------
  //               ROB read 0/1/2 abnormal
  //----------------------------------------------------------
  // Only pipe 0, 2, 3, 4 produce exception?
  private val robReadPipeAbnormalVec = Wire(Vec(NumRobReadEntry, Vec(NumPipeline, Bool())))
  for (i <- 0 until NumRobReadEntry) {
    for (j <- 0 until NumPipeline) {
      if (abnormalPipeNumSeq.contains(j)) {
        // pipe(j) read(i) abnormal
        robReadPipeAbnormalVec(i)(j) := robReadPipeCmpltVec(i)(j) && pipeAbnormalVec(j)
      }else{
        robReadPipeAbnormalVec(i)(j) := false.B
      }
    }
  }

  //----------------------------------------------------------
  //               ROB read 0/1/2 breakpoint
  //----------------------------------------------------------
  private val robReadPipeBreakpointDataVec = Wire(Vec(NumRobReadEntry, Vec(NumPipeline, new RobBreakpointDataBundle)))
  for (i <- 0 until NumRobReadEntry) {
    for (j <- 0 until NumPipeline) {
      if (breakpointDataPipeSeq.contains(j)) {
        robReadPipeBreakpointDataVec(i)(j).a := robReadPipeCmpltVec(i)(j) && pipeBreakpointVec(j).a
        robReadPipeBreakpointDataVec(i)(j).b := robReadPipeCmpltVec(i)(j) && pipeBreakpointVec(j).b
      }else{
        robReadPipeBreakpointDataVec(i)(j) := 0.U.asTypeOf(new RobBreakpointDataBundle)
      }
    }
  }

  //----------------------------------------------------------
  //               ROB read 0/1/2 no speculation
  //----------------------------------------------------------
  private val robReadPipeNoSpecVec = Wire(Vec(NumRobReadEntry, Vec(NumPipeline, new RobNoSpecBundle)))
  for (i <- 0 until NumRobReadEntry) {
    for (j <- 0 until NumPipeline) {
      if (breakpointDataPipeSeq.contains(j)) {
        robReadPipeNoSpecVec(i)(j).hit      := robReadPipeCmpltVec(i)(j) && pipeNoSpecVec(j).hit
        robReadPipeNoSpecVec(i)(j).miss     := robReadPipeCmpltVec(i)(j) && pipeNoSpecVec(j).miss
        robReadPipeNoSpecVec(i)(j).mispred  := robReadPipeCmpltVec(i)(j) && pipeNoSpecVec(j).mispred
      }else{
        robReadPipeNoSpecVec(i)(j) := 0.U.asTypeOf(new RobNoSpecBundle)
      }
    }
  }

  //----------------------------------------------------------
  //            Prepare fold inst cmplt number
  //----------------------------------------------------------
  // Todo: imm
  private val needFoldPipeSeq = Seq(0, 1, 5, 6) // map to pipe 0, 1, 6, 7
  private val foldReadPipeCmpltVec = Wire(Vec(NumRobReadEntry, Vec(needFoldPipeSeq.length, Bool())))
  for (i <- 0 until NumRobReadEntry) {
    foldReadPipeCmpltVec(i)(0) := robReadPipeCmpltVec(i)(0)
    foldReadPipeCmpltVec(i)(1) := robReadPipeCmpltVec(i)(1)
    foldReadPipeCmpltVec(i)(2) := robReadPipeCmpltVec(i)(5)
    foldReadPipeCmpltVec(i)(3) := robReadPipeCmpltVec(i)(6)
  }

  private val foldReadPipeCmpltCntVec = Wire(Vec(NumRobReadEntry, UInt(log2Up(NumRobReadEntry).W)))
  for (i <- 0 until NumRobReadEntry) {
    foldReadPipeCmpltCntVec(i) := foldReadPipeCmpltVec(i).count(item=>item)
  }

  //----------------------------------------------------------
  //         Prepare retire entry update valid signal
  //----------------------------------------------------------

  private val robReadCmpltedVec = Wire(Vec(NumRobReadEntry, Bool()))
  for (i <- 0 until NumRobReadEntry) {
    robReadCmpltedVec(i) := robReadDataVec(i).ctrl.valid &&
      (
        //1.if rob read0 already cmplt
        robReadDataVec(i).ctrl.cmpltValid ||
          //2.1 1 fold inst cmplting and cmplt cnt is 1
          foldReadPipeCmpltCntVec(i) === 1.U && robReadDataVec(i).ctrl.cmpltCnt === 1.U ||
          //2.2 2 fold inst cmplting and cmplt cnt is 2
          foldReadPipeCmpltCntVec(i) === 2.U && robReadDataVec(i).ctrl.cmpltCnt === 2.U ||
          //2.3 3 fold inst cmplting and cmplt cnt must be 3
          foldReadPipeCmpltCntVec(i) === 3.U ||
          //2.4 other inst cmplt no matter cmplt cnt
          robReadPipeCmpltVec(i)(2) ||
          robReadPipeCmpltVec(i)(3) ||
          robReadPipeCmpltVec(i)(4)
        )
  }
  // Todo: figure out
  private val robRead0CmpltedGateClk = robReadDataVec(0).ctrl.valid &&
    (
      //1. if rob read0 already cmplt
      robReadDataVec(0).ctrl.cmpltValid ||
        //2. or rob read0 cmplting
        robReadPipeCmpltVec.asUInt.orR
    )

  private val robReadAbnormalVec = Wire(Vec(NumRobReadEntry, Bool()))
  for (i <- 0 until NumRobReadEntry) {
    robReadAbnormalVec(i) :=
      robReadPipeAbnormalVec(i)(0) ||
        robReadPipeAbnormalVec(i)(2) ||
        robReadPipeAbnormalVec(i)(3) ||
        robReadPipeAbnormalVec(i)(4) ||
        robReadExceptEntryValidVec(i)
  }
  // Todo: move
  private val robCommitMaskVec = Wire(Vec(NumCommitEntry, Bool()))
  //rob read0 will unconditionally update retire entry0
  // Todo: figure out why need not !robReadAbnormal(0), srtEn
  io.out.retire.entryUpdateValidVec(0) := robReadCmpltedVec(0) &&
    !io.in.fromRetire.rtMask &&
    !robCommitMaskVec(0)
  //rob read1 update retire entry1 when:
  // Todo: figure out srtEn, rtMask, commit mask
  io.out.retire.entryUpdateValidVec(1) := robReadCmpltedVec(1) &&
    !io.in.fromRetire.srtEn &&
    !io.in.fromRetire.rtMask &&
    !robCommitMaskVec(1) &&
    io.out.retire.entryUpdateValidVec(0) &&
    !robReadAbnormalVec(0) &&
    !robReadAbnormalVec(1) &&
    !robReadDataVec(1).data.ras

  //rob read1 update retire entry2 when:
  io.out.retire.entryUpdateValidVec(2) := robReadCmpltedVec(2) &&
    !io.in.fromRetire.srtEn &&
    !io.in.fromRetire.rtMask &&
    !robCommitMaskVec(2) &&
    io.out.retire.entryUpdateValidVec(1) &&
    !robReadAbnormalVec(2) &&
    !robReadDataVec(2).data.ras

  //retire entry0 update gateclk valid
  io.out.retire.updateGateClkValid := robRead0CmpltedGateClk

  //inst valid for mem bkpt, ignore cmplt
  private val retireRead0InstBreakpointInstValid =
    robReadDataVec(0).ctrl.valid &&
      !io.in.fromRetire.rtMask &&
      !robCommitMaskVec(0)

  //----------------------------------------------------------
  //           HAD Instruction Execution Dead Lock
  //----------------------------------------------------------
  //if rob read 0 inst valid but not cmplt when dbgreq_ack, signal exe dead
  // Todo: figure out
  io.out.toHad.instExeDead := robReadDataVec(0).ctrl.valid && !robReadCmpltedVec(0)

  //----------------------------------------------------------
  //                 HAD Memory BKPT signal
  //----------------------------------------------------------

  private val robReadCmpltedBreakpointDataVec = Wire(Vec(NumRobReadEntry, new RobBreakpointDataBundle))
  for (i <- 0 until NumRobReadEntry) {
    robReadCmpltedBreakpointDataVec(i).a :=
      robReadDataVec(i).data.breakpointData.a ||
      robReadPipeBreakpointDataVec(i)(3).a ||
      robReadPipeBreakpointDataVec(i)(4).a
    robReadCmpltedBreakpointDataVec(i).b :=
      robReadDataVec(i).data.breakpointData.b ||
        robReadPipeBreakpointDataVec(i)(3).b ||
        robReadPipeBreakpointDataVec(i)(4).b
  }

  io.out.toHad.instBreakpointInstValid := retireRead0InstBreakpointInstValid
  io.out.toHad.instBreakpoint := robReadDataVec(0).data.breakpointInst
  // dataBreakpoint from read entry0 and pipe3, pipe4
  io.out.toHad.dataBreakpoint := robReadCmpltedBreakpointDataVec(0)
  io.out.toHad.breakpointDataSt := robReadDataVec(0).data.store
  io.out.toHad.xxMBreakpointChangeFlow := robReadDataVec(0).data.bju
  io.out.toHad.instSplit := robReadDataVec(0).data.split

  //----------------------------------------------------------
  //                 no spec cmplt signal
  //----------------------------------------------------------

  private val robReadCmpltedNoSpecVec = Wire(Vec(NumRobReadEntry, new RobNoSpecBundle))
  for (i <- 0 until NumRobReadEntry) {
    robReadCmpltedNoSpecVec(i).hit := robReadDataVec(0).data.noSpec.hit ||
      robReadPipeNoSpecVec(i)(3).hit || robReadPipeNoSpecVec(i)(4).hit
    robReadCmpltedNoSpecVec(i).miss := robReadDataVec(0).data.noSpec.miss ||
      robReadPipeNoSpecVec(i)(3).miss || robReadPipeNoSpecVec(i)(4).miss
    robReadCmpltedNoSpecVec(i).mispred := robReadDataVec(0).data.noSpec.mispred ||
      robReadPipeNoSpecVec(i)(3).mispred || robReadPipeNoSpecVec(i)(4).mispred
  }

  //----------------------------------------------------------
  //                 PCFIFO Pop Signal
  //----------------------------------------------------------

  for (i <- 0 until NumRobReadEntry) {
    io.out.toIu.robReadPcFifoValid(i) := io.out.retire.entryUpdateValidVec(i) && robReadDataVec(i).data.pcFifo
  }
  io.out.toIu.robReadPcFifoGateClkValid := robReadDataVec.map {
    case readData => readData.ctrl.valid && readData.data.pcFifo
  }.reduce(_ || _)

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  private val entryClkEnVec = Wire(Vec(NumRobReadEntry, Bool()))
  // Todo: figure out why always use robReadDataVec(0).valid
  entryClkEnVec(0) := robReadDataVec(0).ctrl.valid || retireInstVec(0).valid || io.in.fromRetire.flushGateClk
  entryClkEnVec(1) := robReadDataVec(0).ctrl.valid || retireInstVec(0).valid
  entryClkEnVec(2) := robReadDataVec(0).ctrl.valid || retireInstVec(0).valid

  // Todo: gated clk for entry

  //==========================================================
  //                   Retire Entry Valid
  //==========================================================
  // Todo: move
  private val retireInst0InstBreakpointUpdate = Wire(Bool())

  // Todo: figure out
  private val robRead0PstRetireValid = !retireInst0InstBreakpointUpdate &&
    !(robReadAbnormalVec(0) && io.in.fromExptEntry.validUpdate)
  private val robRead0PstEregRetireValid = !retireInst0InstBreakpointUpdate

  // Todo: figure out
  when (io.in.fromRetire.flush) {
    retireInstVec(0).valid := false.B
    retireRetireInstValidVec(0) := false.B
    retirePstPregInstValidVec(0) := false.B
    retirePstVregInstValidVec(0) := false.B
    retirePstEregInstValidVec(0) := false.B
  }.elsewhen(io.out.retire.entryUpdateValidVec(0)) {
    retireInstVec(0).valid := true.B
    retireRetireInstValidVec(0) := true.B
    retirePstPregInstValidVec(0) := robRead0PstRetireValid
    retirePstVregInstValidVec(0) := robRead0PstRetireValid
    retirePstEregInstValidVec(0) := robRead0PstEregRetireValid
  }.elsewhen(retireInstVec(0).valid) {
    retireInstVec(0).valid := false.B
    retireRetireInstValidVec(0) := false.B
    retirePstPregInstValidVec(0) := false.B
    retirePstVregInstValidVec(0) := false.B
    retirePstEregInstValidVec(0) := false.B
  }

  when (io.in.fromRetire.flush) {
    retireInstVec(1).valid := false.B
    retireRetireInstValidVec(1) := false.B
    retirePstPregInstValidVec(1) := false.B
    retirePstVregInstValidVec(1) := false.B
    retirePstEregInstValidVec(1) := false.B
  }.elsewhen(io.out.retire.entryUpdateValidVec(1)) {
    retireInstVec(1).valid := true.B
    retireRetireInstValidVec(1) := true.B
    retirePstPregInstValidVec(1) := true.B
    retirePstVregInstValidVec(1) := true.B
    retirePstEregInstValidVec(1) := true.B
  }.elsewhen(retireInstVec(1).valid) {
    retireInstVec(1).valid := false.B
    retireRetireInstValidVec(1) := false.B
    retirePstPregInstValidVec(1) := false.B
    retirePstVregInstValidVec(1) := false.B
    retirePstEregInstValidVec(1) := false.B
  }

  when (io.in.fromRetire.flush) {
    retireInstVec(2).valid := false.B
    retireRetireInstValidVec(2) := false.B
    retirePstPregInstValidVec(2) := false.B
    retirePstVregInstValidVec(2) := false.B
    retirePstEregInstValidVec(2) := false.B
  }.elsewhen(io.out.retire.entryUpdateValidVec(2)) {
    retireInstVec(2).valid := true.B
    retireRetireInstValidVec(2) := true.B
    retirePstPregInstValidVec(2) := true.B
    retirePstVregInstValidVec(2) := true.B
    retirePstEregInstValidVec(2) := true.B
  }.elsewhen(retireInstVec(2).valid) {
    retireInstVec(2).valid := false.B
    retireRetireInstValidVec(2) := false.B
    retirePstPregInstValidVec(2) := false.B
    retirePstVregInstValidVec(2) := false.B
    retirePstEregInstValidVec(2) := false.B
  }

  // Todo: vectorize
  io.out.yyXx.retire(0) := retireInstVec(0).valid
  io.out.yyXx.retire(1) := retireInstVec(1).valid
  io.out.yyXx.retire(2) := retireInstVec(2).valid

  io.out.toPad.retirePc(0).valid := retireInstVec(0).valid
  io.out.toPad.retirePc(1).valid := retireInstVec(1).valid
  io.out.toPad.retirePc(2).valid := retireInstVec(2).valid

  io.out.toRetire.instVec(0).valid := retireRetireInstValidVec(0)
  io.out.toRetire.instVec(1).valid := retireRetireInstValidVec(1)
  io.out.toRetire.instVec(2).valid := retireRetireInstValidVec(2)

  io.out.toRetire.instVec(0).pstPregValid := retirePstPregInstValidVec(0)
  io.out.toRetire.instVec(1).pstPregValid := retirePstPregInstValidVec(1)
  io.out.toRetire.instVec(2).pstPregValid := retirePstPregInstValidVec(2)

  io.out.toRetire.instVec(0).pstVregValid := retirePstVregInstValidVec(0)
  io.out.toRetire.instVec(1).pstVregValid := retirePstVregInstValidVec(1)
  io.out.toRetire.instVec(2).pstVregValid := retirePstVregInstValidVec(2)

  io.out.toRetire.instVec(0).pstEregValid := retirePstEregInstValidVec(0)
  io.out.toRetire.instVec(1).pstEregValid := retirePstEregInstValidVec(1)
  io.out.toRetire.instVec(2).pstEregValid := retirePstEregInstValidVec(2)

  io.out.retire.except.inst0Valid := retireInstVec(0).valid

  //==========================================================
  //                 PCFIFO Pop Data select
  //==========================================================
  // Todo: check imm NumCommitEntry
  private val robReadPcFifoDataVec = Wire(Vec(NumCommitEntry, new PcFifoData))
  robReadPcFifoDataVec(0) := io.in.fromIu.pcFifoPopDataVec(0)
  robReadPcFifoDataVec(1) := Mux(
    robReadDataVec(0).data.pcFifo,
    io.in.fromIu.pcFifoPopDataVec(1),
    io.in.fromIu.pcFifoPopDataVec(0)
  )
  robReadPcFifoDataVec(2) := MuxCase(robReadPcFifoDataVec(0), Seq(
    // Already read pcFifo(1,0)
    (robReadDataVec(0).data.pcFifo && robReadDataVec(1).data.pcFifo)
      -> io.in.fromIu.pcFifoPopDataVec(2),
    // Already read pcFifo(1) or pcFifo(0)
    (robReadDataVec(0).data.pcFifo || robReadDataVec(1).data.pcFifo)
      -> io.in.fromIu.pcFifoPopDataVec(1)
  ))

  // Todo: check imm NumCommitEntry
  private val robReadNextPcVec = Wire(Vec(NumCommitEntry, UInt(PcWidth.W)))
  robReadNextPcVec := robReadPcFifoDataVec.map(_.pcNext)

  //==========================================================
  //                  Retire Entry Content
  //==========================================================
  private val retireInstDataCreateVec = Wire(Vec(NumRetireEntry, new RetireEntryData))
  for (i <- 0 until NumRetireEntry) {
    retireInstDataCreateVec(i).vlPred         := robReadDataVec(i).data.vlPred
    retireInstDataCreateVec(i).vl             := robReadDataVec(i).data.vl
    retireInstDataCreateVec(i).vecDirty       := robReadDataVec(i).data.vecDirty
    retireInstDataCreateVec(i).vsetvli        := robReadDataVec(i).data.vsetvli
    retireInstDataCreateVec(i).vsew           := robReadDataVec(i).data.vsew
    retireInstDataCreateVec(i).vlmul          := robReadDataVec(i).data.vlmul
    retireInstDataCreateVec(i).noSpec         := robReadDataVec(i).data.noSpec
    retireInstDataCreateVec(i).load           := robReadDataVec(i).data.load
    retireInstDataCreateVec(i).fpDirty        := robReadDataVec(i).data.fpDirty
    retireInstDataCreateVec(i).breakpointData := robReadDataVec(i).data.breakpointData
    retireInstDataCreateVec(i).breakpointInst := robReadDataVec(i).data.breakpointInst
    retireInstDataCreateVec(i).store          := robReadDataVec(i).data.store
    retireInstDataCreateVec(i).instNum        := robReadDataVec(i).data.instNum
    retireInstDataCreateVec(i).pret           := robReadPcFifoDataVec(i).pret
    retireInstDataCreateVec(i).pcOffset       := robReadDataVec(i).data.pcOffset
    retireInstDataCreateVec(i).jmp            := robReadPcFifoDataVec(i).jmp
    // Todo: figure out why condBrTaken = bhtPred ^ bhtMispred
    retireInstDataCreateVec(i).condBrTaken    := robReadPcFifoDataVec(i).bhtPred ^
                                                  robReadPcFifoDataVec(i).bhtMispred
    retireInstDataCreateVec(i).condBranch     := robReadPcFifoDataVec(i).condBranch
    retireInstDataCreateVec(i).checkIdx       := robReadPcFifoDataVec(i).checkIdx
    retireInstDataCreateVec(i).bju            := robReadDataVec(i).data.bju
    retireInstDataCreateVec(i).split          := robReadDataVec(i).data.split
    retireInstDataCreateVec(i).iid            := robReadIidVec(i)
  }

  for (i <- 0 until NumRetireEntry) {
    // Todo: check timing optimization, use entry0 update valid
    when(io.out.retire.entryUpdateValidVec(i)) {
      retireInstVec(i).bits.data := retireInstDataCreateVec(i)
    }
  }

  //----------------------------------------------------------
  //                   Rename for output
  //----------------------------------------------------------

  for (i <- 0 until NumRetireEntry) {
    io.out.toPst(i).iid := retireInstVec(i).bits.data.iid
  }
  io.out.toRob.exceptInst0Iid := retireInstVec(0).bits.data.iid
  // Todo: check if need to add inst1/2.iid output
  io.out.toRetire.instExtra.iid := retireInstVec(0).bits.data.iid
  io.out.toRetire.instExtra.pReturn := retireInstVec(0).bits.data.bju && retireInstVec(0).bits.data.pret

  // robToRetireInstVec
  for (i <- 0 until NumRetireEntry) {
    io.out.toRetire.instVec(i).split        := retireInstVec(i).bits.data.split
    io.out.toRetire.instVec(i).bju          := retireInstVec(i).bits.data.bju
    io.out.toRetire.instVec(i).checkIdx     := retireInstVec(i).bits.data.checkIdx
    // Todo: check if need remove && bju
    io.out.toRetire.instVec(i).condBranch   := retireInstVec(i).bits.data.condBranch && retireInstVec(i).bits.data.bju
    io.out.toRetire.instVec(i).condBrTaken  := retireInstVec(i).bits.data.condBrTaken
    // Todo: check if need remove && bju
    io.out.toRetire.instVec(i).jmp          := retireInstVec(i).bits.data.jmp && retireInstVec(i).bits.data.bju
    io.out.toRetire.instVec(i).pcOffset     := retireInstVec(i).bits.data.pcOffset
    io.out.toRetire.instVec(i).num          := retireInstVec(i).bits.data.instNum
    io.out.toRetire.instVec(i).store        := retireInstVec(i).bits.data.store
    io.out.toRetire.instVec(i).load         := retireInstVec(i).bits.data.load
    io.out.toRetire.instVec(i).fpDirty      := retireInstVec(i).bits.data.fpDirty
    io.out.toRetire.instVec(i).vecDirty     := retireInstVec(i).bits.data.vecDirty
    io.out.toRetire.instVec(i).noSpec       := retireInstVec(i).bits.data.noSpec
    io.out.toRetire.instVec(i).vlmul        := retireInstVec(i).bits.data.vlmul
    io.out.toRetire.instVec(i).vsew         := retireInstVec(i).bits.data.vsew
    io.out.toRetire.instVec(i).vsetvli      := retireInstVec(i).bits.data.vsetvli
    io.out.toRetire.instVec(i).vl           := retireInstVec(i).bits.data.vl
    io.out.toRetire.instVec(i).vlPred       := retireInstVec(i).bits.data.vlPred
  }

  for (i <- 0 until NumRetireEntry){
    io.out.toHad.instNonIrvBreakpoint(i).data := retireInstVec(i).bits.data.breakpointData
    io.out.toHad.instNonIrvBreakpoint(i).inst := retireInstVec(i).bits.data.breakpointInst
  }

  //==========================================================
  //            Retire Inst 0 Special Entry Content
  //==========================================================
  // Todo: check if need to add Inst Memory breakpoint segment

  //----------------------------------------------------------
  //                     Interrupt
  //----------------------------------------------------------
  // Todo: document
  //interrupt will trigger single retire, int can only hit retire/read
  //inst 0. when in trace mode, disable interrupt

  //if there are 3 inst commiting (rob_commit2), inst0 must be retiring
  //(because of rob_commit1). rob_read0/1 may both retire at next cycle
  //if set srt mode and signal int_vld, rob_read1 cannot retire at next cycle
  //and will be wrongly flushed after committed. so do not signal int when
  //3 inst commiting.
  //if there are 2 inst commiting, there will be at most 1 inst retiring at
  //next cycle, so can signal int when 1/2 inst committing

  // Todo: figure out cp0.xxIntB, cp0.xxVec
  private val robRead0InterruptVec = Wire(ValidIO(UInt(InterruptVecWidth.W)))
  robRead0InterruptVec.valid :=
    !io.in.fromCp0.xxIntB && !io.in.fromHad.xxTme && !robCommitIid(2).valid
  robRead0InterruptVec.bits := io.in.fromCp0.xxVec

  io.out.toRetire.intSrtEn := robRead0InterruptVec.valid
  private val interruptCommitMask = !io.in.fromCp0.xxIntB && !io.in.fromHad.xxTme

  //----------------------------------------------------------
  //                     Debug Disable
  //----------------------------------------------------------
  //similar to interrupt, the debug request can only hit retire/read
  //inst0 and cannot interrupt inst already committed

  private val robRead0DebugDisable = io.in.fromHad.debugReqEn && !robCommitIid(2).valid
  private val debugCommitMask = io.in.fromHad.debugReqEn

  //----------------------------------------------------------
  //                     CTC Flush Request
  //----------------------------------------------------------
  // Todo: figure out CTC
  private val robRead0CtcFlush = io.in.fromRetire.ctcFLushReq && !robCommitIid(2).valid

  io.out.toRetire.ctcFlushSrtEn := robRead0CtcFlush
  private val ctcFlushCommitMask = io.in.fromRetire.ctcFLushReq

  //----------------------------------------------------------
  //            retire inst0 special entry content
  //----------------------------------------------------------

  // special entry only bound to retire entry0
  when(io.out.retire.entryUpdateValidVec(0)) {
    retireInst0Special.abnormal      := robReadAbnormalVec(0)
    retireInst0Special.interruptVec  := robRead0InterruptVec
    retireInst0Special.ras           := robReadDataVec(0).data.ras
    retireInst0Special.pcal          := robReadPcFifoDataVec(0).pcall
    retireInst0Special.bjuLength     := robReadPcFifoDataVec(0).length
    retireInst0Special.intMask       := robReadDataVec(0).data.intMask
    retireInst0Special.dataBreakpoint:= io.in.fromHad.dataBreakPointDebugReq
    retireInst0Special.debugDisable  := robRead0DebugDisable
    retireInst0Special.ctcFlush      := robRead0CtcFlush
  }

  io.out.retire.except.inst0Abnormal  := retireInst0Special.abnormal

  io.out.toRetire.instExtra.interruptVec  := retireInst0Special.interruptVec
  io.out.toRetire.instExtra.interruptMask := retireInst0Special.intMask
  io.out.toRetire.instExtra.pCall          := retireInstVec(0).bits.data.bju && retireInst0Special.pcal
  io.out.toRetire.instExtra.ras           := retireInstVec(0).bits.data.bju && retireInst0Special.ras
  io.out.toRetire.instExtra.dataBreakpoint:= retireInst0Special.dataBreakpoint
  io.out.toRetire.instExtra.debugDisable  := retireInst0Special.debugDisable
  io.out.toRetire.instExtra.ctcFlush      := retireInst0Special.ctcFlush

  io.out.toIdu.retireInterruptValid       := retireInst0Special.interruptVec.valid

  //----------------------------------------------------------
  //                   inst breakpoint valid
  //----------------------------------------------------------

  retireInst0InstBreakpointUpdate := Mux(
    retireRead0InstBreakpointInstValid,
    io.in.fromHad.instBreakPointDebugReq,
    retireInst0Special.instBreakpoint
  )

  when(io.in.fromRetire.flush) {
    retireInst0Special.instBreakpoint := false.B
  }.elsewhen(retireRead0InstBreakpointInstValid) {
    retireInst0Special.instBreakpoint := io.in.fromHad.instBreakPointDebugReq
  }

  io.out.toRetire.instExtra.instBreakPoint := retireInst0Special.instBreakpoint

  //==========================================================
  //                Retire inst 0/1/2 Current PC
  //==========================================================
  //----------------------------------------------------------
  //                 prepare pc maintain signals
  //----------------------------------------------------------
  //add offset
  private def Addend1Bits : Int = RobPcOffsetBits + 2

  private val robReadPcOffsetExtVec = Wire(Vec(NumRobReadEntry, UInt(Addend1Bits.W)))
  for (i <- 0 until NumRobReadEntry) {
    robReadPcOffsetExtVec(i) := Cat(0.U(2.W), robReadDataVec(i).data.pcOffset)
  }

  //----------------------------------------------------------
  //                 ROB Read 0/1/2 Current PC
  //             (Retire inst 0/1/2 cur pc update val)
  //----------------------------------------------------------
  // Todo: figure out
  //when split fof flush, ifu change flow pc should be next pc of split
  //fof inst whose addend1 must be 0 (it is split inst), so change flow
  //pc will be addend0 + 0 (addend1) + 2

  private val robPcCurPlus2 = Wire(UInt(PcWidth.W))
  private val robPcCur      = Wire(UInt(PcWidth.W))

  private val robRead0PcCur = Mux(
    io.in.fromRetire.splitFofFlush,
    robPcCurPlus2,
    robPcCur
  )

  private val robRead1PcCurAddend0 = Mux(
    robReadDataVec(0).data.bju,
    robReadNextPcVec(0),
    robPcCur
  )
  private val robRead1PcCurAddend1 = robReadPcOffsetExtVec(0)

  when(robReadDataVec(1).data.bju) {
    robRead2PcCurAddend0 := robReadNextPcVec(1)
  }.elsewhen(robReadDataVec(0).data.bju) {
    robRead2PcCurAddend0 := robReadNextPcVec(0)
  }.otherwise {
    robRead2PcCurAddend0 := robPcCur
  }

  // Todo: figure out
  //pc offset will be 0 if bju
  private val robRead2PcCurAddend1 = Wire(UInt(PcWidth.W))
  robRead2PcCurAddend1 := Mux(
    robReadDataVec(1).data.bju,
    0.U,
    robReadPcOffsetExtVec(0) + robReadPcOffsetExtVec(1)
  )

  //----------------------------------------------------------
  //                  ROB Read 2 Next PC
  //                 (ROB cur pc update val)
  //----------------------------------------------------------

  when(robReadDataVec(2).data.bju) {
    robRead2PcNextAddend0 := robReadNextPcVec(2)
  }.elsewhen(robReadDataVec(1).data.bju) {
    robRead2PcNextAddend0 := robReadNextPcVec(1)
  }.elsewhen(robReadDataVec(0).data.bju) {
    robRead2PcNextAddend0 := robReadNextPcVec(0)
  }.otherwise {
    robRead2PcNextAddend0 := robPcCur
  }

  //pc offset will be 0 if bju
  when(robReadDataVec(2).data.bju) {
    robRead2PcNextAddend1 := 0.U
  }.elsewhen(robReadDataVec(1).data.bju) {
    robRead2PcNextAddend1 := robReadPcOffsetExtVec(2)
  }.otherwise {
    robRead2PcNextAddend1 := robReadPcOffsetExtVec(2) +
      robReadPcOffsetExtVec(1) +
      robReadPcOffsetExtVec(0)
  }

  //----------------------------------------------------------
  //               Retire inst 0 Current PC
  //----------------------------------------------------------
  //if retire inst flush, flop rob cur pc and then output to IFU for timing
  //if inst breakpoint, flop rob_read0_cur_pc and then output for HAD

  private val inst0PcCurUpdateValid =
    io.out.retire.entryUpdateValidVec(0) ||
      io.in.fromRetire.flush ||
      io.in.fromHad.instBreakPointDebugReq

  when(inst0PcCurUpdateValid) {
    retireInst0Special.pcCur := robRead0PcCur
  }

  io.out.toRetire.instVec(0).pc := retireInst0Special.pcCur
  io.out.toPad.retirePc(0).bits := Cat(retireInst0Special.pcCur, 0.U(1.W))
  // Todo: figure out bjuLength
  //retire inst0 branch increase pc
  io.out.toRetire.instExtra.bjuIncPc := retireInst0Special.pcCur +
    Cat(retireInst0Special.bjuLength, !retireInst0Special.bjuLength)
  //retire inst0 next pc
  io.out.toRetire.instVec(0).npc := Mux(
    retireInstVec(1).valid,
    io.out.toRetire.instVec(1).pc,
    robPcCur
  )

  //----------------------------------------------------------
  //               Retire inst 1 Current PC
  //----------------------------------------------------------

  when(io.out.retire.entryUpdateValidVec(1)) {
    retireInstVec(1).bits.pcAddend0 := robRead1PcCurAddend0
    retireInstVec(1).bits.pcAddend1 := robRead1PcCurAddend1
  }

  io.out.toRetire.instVec(1).pc := retireInstVec(1).bits.pcAddend0 + retireInstVec(1).bits.pcAddend1
  io.out.toPad.retirePc(1).bits := Cat(retireInstVec(1).bits.pcAddend0, 0.U(1.W))
  io.out.toRetire.instVec(1).npc := Mux(
    retireInstVec(2).valid,
    io.out.toRetire.instVec(2).pc,
    robPcCur
  )

  //----------------------------------------------------------
  //               Retire inst 2 Current PC
  //----------------------------------------------------------

  when(io.out.retire.entryUpdateValidVec(2)) {
    retireInstVec(2).bits.pcAddend0 := robRead2PcCurAddend0
    retireInstVec(2).bits.pcAddend1 := robRead2PcCurAddend1
  }
  io.out.toRetire.instVec(2).pc := retireInstVec(2).bits.pcAddend0 + retireInstVec(2).bits.pcAddend1
  io.out.toPad.retirePc(2).bits := Cat(retireInstVec(2).bits.pcAddend0, 0.U(1.W))
  io.out.toRetire.instVec(2).npc := robPcCur

  //==========================================================
  //                   ROB Current PC
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  // Todo: figure out fromIfu.curPcLoad
  private val pcClkEn = io.in.fromRetire.flushGateClk ||
    robReadDataVec(0).ctrl.valid || io.in.fromIfu.curPcLoad
  // Todo: gated clk for pc clk


  //----------------------------------------------------------
  //              Prepare ROB cur PC condition
  //----------------------------------------------------------

  //if rob read0 is rte/rfi, rob read0 next pc will be EPC/FPC from CP0
  //rte/rfi is fence inst, ROB can only bypass rte/rfi info from Cbus,
  //no rob read 1/2 inst, rob cur pc will be EFPC

  private val robRead0Rte = io.in.fromIu.pipe0.cmplt && io.in.fromIu.pipe0.efPc.valid

  //----------------------------------------------------------
  //                 ROB Current PC Register
  //----------------------------------------------------------

  //if flush by split vector first only fault inst
  //rtu should skip left inst by add pc 4,
  //retire must flush rob at this time

  when(io.in.fromRetire.splitFofFlush) {
    robPcCurAddend0 := robPcCurAddend0
    robPcCurAddend1 := robPcCurAddend1 + 2.U
  }.elsewhen(io.in.fromRetire.flush) {
    robPcCurAddend0 := robPcCurAddend0
    robPcCurAddend1 := robPcCurAddend1 + 2.U
  }.elsewhen(io.in.fromIfu.curPcLoad) {
    robPcCurAddend0 := io.in.fromIfu.curPc
    robPcCurAddend1 := 0.U
  }.elsewhen(io.out.retire.entryUpdateValidVec(2)) {
    // retire 3 insts
    robPcCurAddend0 := robRead2PcNextAddend0
    robPcCurAddend1 := robRead2PcNextAddend1
  }.elsewhen(io.out.retire.entryUpdateValidVec(1)) {
    // retire 2 insts
    robPcCurAddend0 := robRead2PcCurAddend0
    robPcCurAddend1 := robRead2PcCurAddend1
  }.elsewhen(io.out.retire.entryUpdateValidVec(0) && robRead0Rte) {
    robPcCurAddend0 := io.in.fromIu.pipe0.efPc.bits
    robPcCurAddend1 := 0.U
  }.elsewhen(io.out.retire.entryUpdateValidVec(0)) {
    // retire 1 inst
    robPcCurAddend0 := robRead1PcCurAddend0
    robPcCurAddend1 := robRead1PcCurAddend1
  }

  robPcCur := robPcCurAddend0 + robPcCurAddend1

  //when split fof flush, ifu change flow pc should be next pc of split
  //fof inst whose addend1 must be 0 (it is split inst), so change flow
  //pc will be addend0 + 0 (addend1) + 2
  robPcCurPlus2 := robPcCurAddend0 + 2.U

  io.out.toRetire.robCurPc := robPcCur
  io.out.toTop.robCurPc := robPcCur

  //==========================================================
  //                  Jump Offset for HPCP
  //==========================================================

  private val retireInst2JmpPcOffset = robPcCurAddend0 - retireInstVec(2).bits.pcAddend0
  private val retireInst1JmpPcOffset = Mux(
    !retireInstVec(2).valid,
    robPcCurAddend0 - retireInstVec(1).bits.pcAddend0,
    retireInstVec(2).bits.pcAddend0 - retireInstVec(1).bits.pcAddend0
  )
   private val retireInst0JmpPcOffset = MuxCase(
     retireInstVec(1).bits.pcAddend0 - retireInst0Special.pcCur, Seq(
       !retireInstVec(2).valid -> (robPcCurAddend0 - retireInst0Special.pcCur),
       !retireInstVec(1).valid -> (retireInstVec(2).bits.pcAddend0 - retireInst0Special.pcCur)
     )
   )

  //==========================================================
  //                   Retire PC for HAD
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  private val debugInfoEn = io.in.fromHad.debugRetireInfoEn || io.in.fromHpcp.cntEn
  private val debugClkEn = debugInfoEn && retireInstVec(0).valid || debugRetireInfoVec(0).valid

  // Todo: gated clk for debug clk

  io.out.toHad.robEmpty := DontCare

  // Todo: toHad, toHPCP
  io.out.toHad.retireInstInfo <> DontCare
  io.out.toHpcp <> DontCare

  //==========================================================
  //                    Commit Signals
  //==========================================================
  //----------------------------------------------------------
  //                 Instance of Gated Cell
  //----------------------------------------------------------
  private val commitClkEn = robReadDataVec(0).ctrl.valid ||
    robReadDataVec(1).ctrl.valid ||
    robReadDataVec(2).ctrl.valid ||
    robCommitIid(0).valid
  // Todo: gated clk for commit clk

  //----------------------------------------------------------
  //                 Prepare commit signal
  //----------------------------------------------------------
  //if current retire inst is flush, rob read0 cannot commit
  private val robReadCommitVec = Wire(Vec(NumCommitEntry, Bool()))
  robReadCommitVec(0) := robReadDataVec(0).ctrl.valid &&
    !io.in.fromHad.instBreakPointDebugReq &&
    !io.in.fromRetire.rtMask
  robReadCommitVec(1) := robReadDataVec(1).ctrl.valid &&
    !io.in.fromHad.instBreakPointDebugReq &&
    !io.in.fromRetire.rtMask &&
    !io.in.fromRetire.srtEn &&
    robReadCmpltedVec(0) &&
    !robReadAbnormalVec(0)
  robReadCommitVec(2) := robReadDataVec(2).ctrl.valid &&
    !io.in.fromHad.instBreakPointDebugReq &&
    !io.in.fromRetire.rtMask &&
    !io.in.fromRetire.srtEn &&
    robReadCmpltedVec(0) &&
    robReadCmpltedVec(1) &&
    !robReadAbnormalVec(0) &&
    !robReadAbnormalVec(1)

  //----------------------------------------------------------
  //          Asynchronous Exception Mask Signals
  //----------------------------------------------------------
  //asynchronous exception need to stop any new inst retire and commit
  //asynchronous exception cannot mask inst already committed
  //the async expt commit mask should wait for commit new inst
  // Todo: figure out Asynchronous Exception Mask Signals
  private val commitAsyncExceptMaskVec = Wire(Vec(NumCommitEntry, Bool()))
  commitAsyncExceptMaskVec(0) :=
    io.in.fromRetire.asyncExceptionCommitMask &&
      !(robCommitIid(0).valid && (robReadIidVec(0) === robCommitIid(0).bits) ||
        robCommitIid(1).valid && (robReadIidVec(0) === robCommitIid(1).bits) ||
        robCommitIid(2).valid && (robReadIidVec(0) === robCommitIid(2).bits)
        )
  commitAsyncExceptMaskVec(1) :=
    io.in.fromRetire.asyncExceptionCommitMask &&
      !(robCommitIid(1).valid && (robReadIidVec(1) === robCommitIid(1).bits) ||
        robCommitIid(2).valid && (robReadIidVec(1) === robCommitIid(2).bits)
        )
  commitAsyncExceptMaskVec(2) :=
    io.in.fromRetire.asyncExceptionCommitMask &&
      !(robCommitIid(2).valid && (robReadIidVec(2) === robCommitIid(2).bits)
        )
  //----------------------------------------------------------
  //                 Sync Commit Mask Signals
  //----------------------------------------------------------
  //sync commit mask need single retire, so no more than 1 inst
  //can commit at same time
  //sync commit mask cannot mask inst already committed
  //the sync commit mask need not to mask commit0 for new inst
  private val commitSyncMask =
    interruptCommitMask || debugCommitMask || ctcFlushCommitMask
  private val commitSyncMaskVec = Wire(Vec(NumCommitEntry, Bool()))
  commitSyncMaskVec(0) := false.B
  commitSyncMaskVec(1) := commitSyncMask &&
    !(robCommitIid(1).valid && robCommitIid(1).bits === robReadIidVec(1) ||
      robCommitIid(2).valid && robCommitIid(2).bits === robReadIidVec(1)
      )
  commitSyncMaskVec(2) := commitSyncMask &&
    !(robCommitIid(2).valid && robCommitIid(2).bits === robReadIidVec(2)
      )

  //----------------------------------------------------------
  //                  Flop commit signals
  //----------------------------------------------------------
  for (i <- 0 until NumCommitEntry) {
    robCommitMaskVec(i) := commitAsyncExceptMaskVec(i) || commitSyncMaskVec(i)
  }

  for (i <- 0 until NumCommitEntry) {
    when(io.in.fromRetire.flush) {
      robCommitIid(i).valid := false.B
    }.otherwise {
      robCommitIid(i).valid := robReadCommitVec(i) && !robCommitMaskVec(i)
    }

    robCommitIid(i).bits := robReadIidVec(i)
  }

  for (i <- 0 until NumCommitEntry) {
    io.out.toRetire.commitValidVec(i) := robCommitIid(i).valid
    io.out.yyXx.commitIid(i) := robCommitIid(i)
  }
  io.out.toRob.debugCommit0 := robCommitIid(0).valid

  //----------------------------------------------------------
  //                 Debug Counter for FPGA
  //----------------------------------------------------------

  if (fpga) {
    when(retireInstVec(0).valid) {
      retireCnt := 0.U
    }.otherwise {
      retireCnt := retireCnt + 1.U
    }

    when(retireInstVec(0).valid) {
      noRetire := false.B
    }.elsewhen(retireCnt === 2048.U) {
      noRetire := true.B
    }
    io.out.toCpu.noRetire := !noRetire && retireCnt === 2048.U && io.in.fromIdu.fenceIdle
  } else {
    io.out.toCpu.noRetire := false.B
  }

  io.out.toRetire.instExtra.bhtMispred := DontCare
  io.out.toRetire.instExtra.breakPoint := DontCare
  io.out.toRetire.instExtra.efPcValid := DontCare
  io.out.toRetire.instExtra.exceptionVec := DontCare
  io.out.toRetire.instExtra.highHwException := DontCare
  io.out.toRetire.instExtra.instMmuException := DontCare
  io.out.toRetire.instExtra.instFlush := DontCare
  io.out.toRetire.instExtra.jmpMispred := DontCare
  io.out.toRetire.instExtra.mtval := DontCare
  io.out.toRetire.instExtra.specFail := DontCare
  io.out.toRetire.instExtra.specFailNoSsf := DontCare
  io.out.toRetire.instExtra.specFailSsf := DontCare
  io.out.toRetire.instExtra.vsetvl := DontCare
  io.out.toRetire.instExtra.vstart := DontCare

  io.out.toRetire.splitSpecFailSrt := DontCare
  io.out.toRetire.intSrtEn := DontCare
  io.out.toRetire.ssfIid := DontCare

}
