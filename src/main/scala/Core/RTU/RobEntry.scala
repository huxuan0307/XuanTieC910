package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._

class RobEntryCtrlPath extends Bundle {
  val valid       : Bool = Bool()
  val cmpltCnt    : UInt = UInt(NumCmpltBits.W)
  val cmpltValid  : Bool = Bool()
}

class RobEntryDataPath extends Bundle {
  val instr           : UInt = UInt(32.W)
  val vlPred          : Bool = Bool()
  val vl              : UInt = UInt(VlmaxBits.W)
  val vecDirty        : Bool = Bool()
  val vsetvli         : Bool = Bool()
  val vsew            : UInt = UInt(VsewBits.W)
  val vlmul           : UInt = UInt(VlmulBits.W)
  val noSpec          = new Bundle {
    val mispred       : Bool = Bool()
    val miss          : Bool = Bool()
    val hit           : Bool = Bool()
  }
  val load            : Bool = Bool()
  val fpDirty         : Bool = Bool()
  // Todo: imm
  val instNum         : UInt = UInt(2.W)
  val breakpointData = new RobBreakpointDataBundle
  val breakpointInst = new RobBreakpointInstBundle
  val store           : Bool = Bool()
  val ras             : Bool = Bool()
  val pcFifo          : Bool = Bool()
  val bju             : Bool = Bool()
  // Todo: figure out
  val intMask         : Bool = Bool()
  val split           : Bool = Bool()
  val pcOffset        : UInt = UInt(RobPcOffsetBits.W)
}

class RobEntryData extends Bundle {
  val ctrl = new RobEntryCtrlPath
  val data = new RobEntryDataPath
}

class RobEntryPipe extends Bundle {
  val wbBreakpointData = new RobBreakpointDataBundle
  val wbNoSpec          : RobNoSpecBundle = new RobNoSpecBundle
}

class RobEntryInput extends Bundle {
  val fromCp0 = new Bundle() {
    val icgEn  : Bool = Bool()
    val yyClkEn   : Bool = Bool()
  }
  val fromIdu = new Bundle() {
    val createDataVec : Vec[RobEntryData] = Vec(NumCreateEntry, new RobEntryData)
  }
  val fromLsu = new Bundle() {
    val miscCmpltGateClkEn : Bool = Bool()
    val pipe3 = new RobEntryPipe
    val pipe4 = new RobEntryPipe
  }
  val fromPad = new Bundle() {
    val yyIcgScanEn : Bool = Bool()
  }
  val fromRetire = new Bundle() {
    val flush         : Bool = Bool()
    val flushGateClk  : Bool = Bool()
  }
  val x = new Bundle() {
    val cmpltGateClkValid : Bool = Bool()
    val cmpltValidVec     : Vec[Bool] = Vec(NumPipeline, Bool())
    val createDpEn        : Bool = Bool()
    val createEn          : Bool = Bool()
    val createGateClkEn   : Bool = Bool()
    val createSel         : Vec[Bool] = Vec(NumCreateEntry, Bool())
    val popEn             : Bool = Bool()
  }
}

class RobEntryOutput extends Bundle {
  val readData : RobEntryData = new RobEntryData
}

class RobEntryIO extends Bundle {
  val in  : RobEntryInput   = Input(new RobEntryInput)
  val out : RobEntryOutput  = Output(new RobEntryOutput)
}

class RobEntry extends Module {
  val io : RobEntryIO = IO(new RobEntryIO)

  // Todo: bind pipeline number dynamically
  /**
   * Regs
   */

  private val entryData   = RegInit(0.U.asTypeOf(new RobEntryData))
  private val xCreateData = WireInit(0.U.asTypeOf(new RobEntryData))

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================

  private val entryClkEn = io.in.fromRetire.flushGateClk ||
    io.in.x.createGateClkEn ||
    io.in.x.cmpltGateClkValid ||
    io.in.x.popEn

  private val createClkEn = io.in.x.createGateClkEn

  private val lsuCmpltClkEn = io.in.x.createGateClkEn || io.in.fromLsu.miscCmpltGateClkEn

  // Todo: gated clk for entry, entry create, lsu cmplt

  //==========================================================
  //                      Create Port
  //==========================================================

  xCreateData := MuxLookup(io.in.x.createSel.asUInt, 0.U, Seq(
    "b0001".U -> io.in.fromIdu.createDataVec(0).asUInt,
    "b0010".U -> io.in.fromIdu.createDataVec(1).asUInt,
    "b0100".U -> io.in.fromIdu.createDataVec(2).asUInt,
    "b1000".U -> io.in.fromIdu.createDataVec(3).asUInt,
  )).asTypeOf(new RobEntryData)

  //==========================================================
  //                      Entry Valid
  //==========================================================

  when(io.in.fromRetire.flush) {
    entryData.ctrl.valid := false.B
  }.elsewhen(io.in.x.createEn) {
    entryData.ctrl.valid := xCreateData.ctrl.valid
  }.elsewhen(io.in.x.popEn) {
    entryData.ctrl.valid := false.B
  }

  //==========================================================
  //               Cmplt counter and Cmplt bit
  //==========================================================
  //----------------------------------------------------------
  //         Prepare cmplt cnt create and cmplt value
  //----------------------------------------------------------

  // Todo: imm
  private val cmpltFoldValid = VecInit(
    io.in.x.cmpltValidVec(0),
    io.in.x.cmpltValidVec(1),
    io.in.x.cmpltValidVec(5),
    io.in.x.cmpltValidVec(6),
  )
  private val cmpltFoldValidCnt = cmpltFoldValid.count(b => b)
  // Todo: imm
  private val cmpltFoldValidCntOH = Wire(Vec(4, Bool()))
  for (i <- 0 until 4) {
    cmpltFoldValidCntOH(i) := cmpltFoldValidCnt === i.U
  }

  private val cmpltCntWithCreate = Wire(UInt(NumCmpltBits.W))
  // Todo: figure out cmpltCntCmpltExist
  // Todo: imm
  private val cmpltCntCmpltExist =
    // 2.1 0 inst cmplt
    Fill(NumCmpltBits, !io.in.x.cmpltValidVec.asUInt(6,0).orR) & cmpltCntWithCreate |
      // 2.2 1 fold inst cmplt
      Fill(NumCmpltBits, cmpltFoldValidCntOH(1)) & (cmpltCntWithCreate - 1.U) |
      // 2.3 2 fold inst cmplt
      Fill(NumCmpltBits, cmpltFoldValidCntOH(2)) & (cmpltCntWithCreate - 2.U) |
      // 2.4 3 fold inst cmplt
      Fill(NumCmpltBits, cmpltFoldValidCntOH(3)) & 0.U(NumCmpltBits.W) |
      // 2.5 other inst cmplt
      Fill(NumCmpltBits, io.in.x.cmpltValidVec.asUInt(6,2).orR) & 0.U(NumCmpltBits.W)

  //----------------------------------------------------------
  //                        cmplt cnt
  //----------------------------------------------------------
  cmpltCntWithCreate := Mux(
    io.in.x.createEn,
    xCreateData.ctrl.cmpltCnt,
    entryData.ctrl.cmpltCnt
  )

  when (io.in.x.cmpltValidVec.asUInt.orR) {
    entryData.ctrl.cmpltCnt := cmpltCntCmpltExist
  }.elsewhen (io.in.x.createEn) {
    entryData.ctrl.cmpltCnt := xCreateData.ctrl.cmpltCnt
  }

  //----------------------------------------------------------
  //         Prepare cmplt create and cmplt value
  //----------------------------------------------------------
  //1.if create to new or exist entry, cmplt will be 0
  //2.if no create in and inst cmplt, cmplt will be 1
  private val cmpltUpdate = Wire(Bool())
  cmpltUpdate :=
    (cmpltFoldValidCntOH(1) && cmpltCntWithCreate === 1.U) ||
      (cmpltFoldValidCntOH(2) && cmpltCntWithCreate === 2.U) ||
      cmpltFoldValidCntOH(3) ||
      io.in.x.cmpltValidVec.asUInt(4,2).orR

  //----------------------------------------------------------
  //                         cmplt
  //----------------------------------------------------------
  when (io.in.x.cmpltValidVec.asUInt.orR) {
    entryData.ctrl.cmpltValid := cmpltUpdate
  }.elsewhen(io.in.x.createEn) {
    entryData.ctrl.cmpltValid := xCreateData.ctrl.cmpltValid
  }

  //==========================================================
  //              Instruction Complete Information
  //==========================================================
  //bkpta_data and bkptb_data can only from pipe3/4
  // Todo : imm
  private val breakpointADataUpdate =
    io.in.x.cmpltValidVec(3) && io.in.fromLsu.pipe3.wbBreakpointData.a ||
      io.in.x.cmpltValidVec(4) && io.in.fromLsu.pipe4.wbBreakpointData.a
  private val breakpointBDataUpdate =
    io.in.x.cmpltValidVec(3) && io.in.fromLsu.pipe3.wbBreakpointData.b ||
      io.in.x.cmpltValidVec(4) && io.in.fromLsu.pipe4.wbBreakpointData.b
  private val noSpecHitUpdate =
    io.in.x.cmpltValidVec(3) && io.in.fromLsu.pipe3.wbNoSpec.hit ||
      io.in.x.cmpltValidVec(4) && io.in.fromLsu.pipe4.wbNoSpec.hit
  private val noSpecMissUpdate =
    io.in.x.cmpltValidVec(3) && io.in.fromLsu.pipe3.wbNoSpec.miss ||
      io.in.x.cmpltValidVec(4) && io.in.fromLsu.pipe4.wbNoSpec.miss
  private val noSpecMispredUpdate =
    io.in.x.cmpltValidVec(3) && io.in.fromLsu.pipe3.wbNoSpec.mispred ||
      io.in.x.cmpltValidVec(4) && io.in.fromLsu.pipe4.wbNoSpec.mispred

  //==========================================================
  //              Instruction Create Information
  //==========================================================

  // first, accept completed signal from pipeline fu
  // second, accept create signal from idu
  when (io.in.x.cmpltValidVec.asUInt(4,3).orR) {
    entryData.data.breakpointData.a := breakpointADataUpdate
    entryData.data.breakpointData.b := breakpointBDataUpdate
    entryData.data.noSpec.hit      := noSpecHitUpdate
    entryData.data.noSpec.miss     := noSpecMissUpdate
    entryData.data.noSpec.mispred  := noSpecMispredUpdate
    // other signals maintain themselves
  }.elsewhen (io.in.x.createDpEn) {
    entryData.data := xCreateData.data
  }

  io.out.readData := entryData
}
