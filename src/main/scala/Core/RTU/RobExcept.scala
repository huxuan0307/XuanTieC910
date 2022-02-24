package Core.RTU

import Core.AddrConfig._
import Core.ExceptionConfig._
import Core.IntConfig._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import chisel3._
import chisel3.util._

// Todo: figure out: ssf: Split instruction spec fail
object SsfState {
  def size : Int = 3
  def bits : Int = log2Up(size)
  val idle :: wfRetire :: retiring :: Nil = Enum(3)
}

protected trait RobExceptConfig {
  /**
   *
   * @return Number of pipeline producing exception
   */
  def NumExceptPipeline : Int = 4

  /**
   *
   * @return Number of sources of exception, including RobExcept itself
   */
  def NumExceptSource   : Int = NumExceptPipeline + 1
}

class RobExceptEntryBundle extends Bundle {
  val validUpdate  : Bool = Bool()
  val iid             : UInt = UInt(InstructionIdWidth.W)
  val valid           : Bool = Bool()
}

class RobExceptEntry extends Bundle {
  val vstart    : ValidIO[UInt] = ValidIO(UInt(VlmaxBits.W))
  val vsetvl    : Bool = Bool()
  val efPcValid : Bool = Bool()
  val specFail  : Bool = Bool()
  val breakpoint: Bool = Bool()
  val flush     : Bool = Bool()
  val jmpMispred: Bool = Bool()
  val bhtMispred: Bool = Bool()

  val mtval     : UInt = UInt(MtvalWidth.W)
  val instMmu   : Bool = Bool()
  val highHw    : Bool = Bool()
  val exceptVec : ValidIO[UInt] = ValidIO(UInt(ExceptionVecWidth.W))
  val iid       : UInt = UInt(InstructionIdWidth.W)
}

class RobExceptInput extends Bundle {
  val fromCp0 = new RobFromCp0Bundle
  val fromIu = new Bundle() {
    val pipe0 = new Bundle() {
      val iid           : UInt = UInt(InstructionIdWidth.W)
      val cmplt         : Bool = Bool()
      val abnormal      : Bool = Bool()
      val breakPoint    : Bool = Bool()
      val efPc          : ValidIO[UInt] = ValidIO(UInt(PcWidth.W))
      val exceptVec     : ValidIO[UInt] = ValidIO(UInt(ExceptionVecWidth.W))
      val flush         : Bool = Bool()
      val highHwExcept  : Bool = Bool()
      val instMmuExcept : Bool = Bool()
      val mtval         : UInt = UInt(MtvalWidth.W)
      val vsetvl        : Bool = Bool()
      val vstart        : ValidIO[UInt] = ValidIO(UInt(VlmaxBits.W))
    }
    val pipe2 = new Bundle() {
      val iid         : UInt = UInt(InstructionIdWidth.W)
      val cmplt       : Bool = Bool()
      val abnormal    : Bool = Bool()
      val bhtMispred  : Bool = Bool()
      val jmpMispred  : Bool = Bool()
    }
  }
  val fromLsu = new Bundle() {
    val pipe3 : RobFromLsuPipeCommonBundle = new RobFromLsuPipeCommonBundle {}
    val pipe4 : RobFromLsuPipeCommonBundle = new RobFromLsuPipeCommonBundle {}
  }
  val fromPad = new RobFromPad
  val fromRetire = new Bundle {
    val inst0Abnormal : Bool = Bool()
    val inst0Valid    : Bool = Bool()
    val flush         : Bool = Bool()
  }
  val fromRob = new Bundle() {
    val inst0Iid    : UInt = UInt(InstructionIdWidth.W)
    val inst0Split  : Bool = Bool()
  }
  val fromRtu = new Bundle() {
    val yyXxFlush   : Bool = Bool()
  }
}

class RobExceptOutput extends Bundle {
  val except = new RobExceptEntryBundle
  val toRetire = new Bundle() {
    val inst0 = new Bundle() {
      val bhtMispred      : Bool = Bool()
      val breakpoint      : Bool = Bool()
      val efPcValid       : Bool = Bool()
      val exceptionVec    : ValidIO[UInt] = ValidIO(UInt(ExceptionVecWidth.W))
      val highHwException : Bool = Bool()
      val instMmuException: Bool = Bool()
      val instFlush       : Bool = Bool()
      val jmpMispred      : Bool = Bool()
      val mtval           : UInt = UInt(MtvalWidth.W)
      val specFail        : Bool = Bool()
      val specFailNoSsf   : Bool = Bool()
      val specFailSsf     : Bool = Bool()
      val vsetvl          : Bool = Bool()
      val vstart          : ValidIO[UInt] = ValidIO(UInt(VlmaxBits.W))
    }
    val splitSpecFailSrt  : Bool = Bool()
    val ssfIid            : UInt = UInt(InstructionIdWidth.W)
  }
  val toTop = new Bundle() {
    val ssfStateCur : UInt = UInt(SsfState.bits.W)
  }
}

class RobExceptIO extends Bundle {
  val in  : RobExceptInput  = Input(new RobExceptInput)
  val out : RobExceptOutput = Output(new RobExceptOutput)
}

class RobExcept extends Module with RobExceptConfig {
  val io : RobExceptIO = IO(new RobExceptIO)

  //==========================================================
  //                          Regs
  //==========================================================

  private val exceptEntryData       : RobExceptEntry = RegInit(0.U.asTypeOf(Output(new RobExceptEntry)))
  private val exceptEntryDataUpdate : RobExceptEntry = RegInit(0.U.asTypeOf(Output(new RobExceptEntry)))
  private val exceptEntryValid      = RegInit(false.B)
  private val ssfStateCur           = RegInit(0.U(2.W))
  private val ssfStateNext          = RegInit(0.U(2.W))
  private val ssfIid                = RegInit(0.U(InstructionIdWidth.W))

  private val pipe0ExceptCmplt = io.in.fromIu.pipe0.cmplt && io.in.fromIu.pipe0.abnormal
  private val pipe2ExceptCmplt = io.in.fromIu.pipe2.cmplt && io.in.fromIu.pipe2.abnormal
  private val pipe3ExceptCmplt = io.in.fromLsu.pipe3.wb.cmplt && io.in.fromLsu.pipe3.wb.abnormal
  private val pipe4ExceptCmplt = io.in.fromLsu.pipe4.wb.cmplt && io.in.fromLsu.pipe4.wb.abnormal
  private val exceptCmplt = pipe0ExceptCmplt || pipe2ExceptCmplt || pipe3ExceptCmplt || pipe4ExceptCmplt

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================

  private val entryClkEn = exceptCmplt || exceptEntryValid
  // Todo: gated clk for entry

  //==========================================================
  //                 Exception Cmplt Order
  //==========================================================
  def IidFlag(iid: UInt) : Bool = {
    iid(NumRobEntryBits)
  }
  def IidNum(iid: UInt) : UInt = {
    iid(NumRobEntryBits - 1, 0)
  }

  def CompareIidLess(left: UInt, right: UInt) : Bool = {
    val res = Wire(Bool())
    res := Mux(
      IidFlag(left) === IidFlag(right),
      IidNum(left) < IidNum(right),
      IidNum(left) > IidNum(right)
    )
    res
  }

  private val pipe4Iid = io.in.fromLsu.pipe4.wb.iid
  private val pipe3Iid = io.in.fromLsu.pipe3.wb.iid
  private val pipe2Iid = io.in.fromIu.pipe2.iid
  private val pipe0Iid = io.in.fromIu.pipe0.iid
  private val exceptIid = exceptEntryData.iid

  private val pipe4lt3 = CompareIidLess(pipe4Iid, pipe3Iid)
  private val pipe4lt2 = CompareIidLess(pipe4Iid, pipe2Iid)
  private val pipe4lt0 = CompareIidLess(pipe4Iid, pipe0Iid)
  private val pipe4lte = CompareIidLess(pipe4Iid, exceptIid)
  private val pipe3lt2 = CompareIidLess(pipe2Iid, pipe3Iid)
  private val pipe3lt0 = CompareIidLess(pipe2Iid, pipe3Iid)
  private val pipe3lte = CompareIidLess(pipe2Iid, pipe3Iid)
  private val pipe2lt0 = CompareIidLess(pipe0Iid, pipe2Iid)
  private val pipe2lte = CompareIidLess(exceptIid, pipe2Iid)
  private val pipe0lte = CompareIidLess(exceptIid, pipe0Iid)

  private val exceptEntryWriteSel = Wire(Vec(NumExceptSource, Bool()))
  // select oldest completed pipe
  exceptEntryWriteSel(0) := exceptEntryValid &&
                            (!pipe0ExceptCmplt || !pipe0lte) &&
                            (!pipe2ExceptCmplt || !pipe2lte) &&
                            (!pipe3ExceptCmplt || !pipe3lte) &&
                            (!pipe4ExceptCmplt || !pipe4lte)
  exceptEntryWriteSel(1) := pipe0ExceptCmplt &&
                            (!exceptEntryValid || pipe0lte) &&
                            (!pipe2ExceptCmplt || !pipe2lt0) &&
                            (!pipe3ExceptCmplt || !pipe3lt0) &&
                            (!pipe4ExceptCmplt || !pipe4lt0)
  exceptEntryWriteSel(2) := pipe2ExceptCmplt &&
                            (!exceptEntryValid || pipe2lte) &&
                            (!pipe0ExceptCmplt || pipe2lt0) &&
                            (!pipe3ExceptCmplt || !pipe3lt2) &&
                            (!pipe4ExceptCmplt || !pipe4lt2)
  exceptEntryWriteSel(3) := pipe3ExceptCmplt &&
                            (!exceptEntryValid || pipe3lte) &&
                            (!pipe0ExceptCmplt || pipe3lt0) &&
                            (!pipe2ExceptCmplt || pipe3lt2) &&
                            (!pipe4ExceptCmplt || !pipe4lt3)
  exceptEntryWriteSel(4) := pipe4ExceptCmplt &&
                            (!exceptEntryValid || pipe4lte) &&
                            (!pipe0ExceptCmplt || pipe4lt0) &&
                            (!pipe2ExceptCmplt || pipe4lt2) &&
                            (!pipe3ExceptCmplt || pipe4lt3)

  private val pipe4ExceptEntryData : RobExceptEntry = WireInit(0.U.asTypeOf(Output(new RobExceptEntry)))
  private val pipe3ExceptEntryData : RobExceptEntry = WireInit(0.U.asTypeOf(Output(new RobExceptEntry)))
  private val pipe2ExceptEntryData : RobExceptEntry = WireInit(0.U.asTypeOf(Output(new RobExceptEntry)))
  private val pipe0ExceptEntryData : RobExceptEntry = WireInit(0.U.asTypeOf(Output(new RobExceptEntry)))

  pipe4ExceptEntryData.vstart       := io.in.fromLsu.pipe4.wb.vstart
  pipe4ExceptEntryData.vsetvl       := false.B
  pipe4ExceptEntryData.efPcValid    := false.B
  pipe4ExceptEntryData.specFail     := io.in.fromLsu.pipe4.wb.specFail
  pipe4ExceptEntryData.breakpoint   := false.B
  pipe4ExceptEntryData.flush        := io.in.fromLsu.pipe4.wb.flush
  pipe4ExceptEntryData.jmpMispred   := false.B
  pipe4ExceptEntryData.bhtMispred   := false.B
  pipe4ExceptEntryData.mtval        := io.in.fromLsu.pipe4.wb.mtval
  pipe4ExceptEntryData.instMmu      := false.B
  pipe4ExceptEntryData.highHw       := false.B
  pipe4ExceptEntryData.exceptVec    := io.in.fromLsu.pipe4.wb.exceptVec
  pipe4ExceptEntryData.iid          := io.in.fromLsu.pipe4.wb.iid

  pipe3ExceptEntryData.vstart       := io.in.fromLsu.pipe3.wb.vstart
  pipe3ExceptEntryData.vsetvl       := io.in.fromLsu.pipe3.wb.vsetvl
  pipe3ExceptEntryData.efPcValid    := false.B
  pipe3ExceptEntryData.specFail     := io.in.fromLsu.pipe3.wb.specFail
  pipe3ExceptEntryData.breakpoint   := false.B
  pipe3ExceptEntryData.flush        := io.in.fromLsu.pipe3.wb.flush
  pipe3ExceptEntryData.jmpMispred   := false.B
  pipe3ExceptEntryData.bhtMispred   := false.B
  pipe3ExceptEntryData.mtval        := io.in.fromLsu.pipe3.wb.mtval
  pipe3ExceptEntryData.instMmu      := false.B
  pipe3ExceptEntryData.highHw       := false.B
  pipe3ExceptEntryData.exceptVec    := io.in.fromLsu.pipe3.wb.exceptVec
  pipe3ExceptEntryData.iid          := io.in.fromLsu.pipe3.wb.iid

  pipe2ExceptEntryData.jmpMispred   := io.in.fromIu.pipe2.jmpMispred
  pipe2ExceptEntryData.bhtMispred   := io.in.fromIu.pipe2.bhtMispred
  pipe2ExceptEntryData.iid          := io.in.fromIu.pipe2.iid

  pipe0ExceptEntryData.vstart       := io.in.fromIu.pipe0.vstart
  pipe0ExceptEntryData.vsetvl       := io.in.fromIu.pipe0.vsetvl
  pipe0ExceptEntryData.efPcValid    := io.in.fromIu.pipe0.efPc.valid
  pipe0ExceptEntryData.specFail     := false.B
  pipe0ExceptEntryData.breakpoint   := io.in.fromIu.pipe0.breakPoint
  pipe0ExceptEntryData.flush        := io.in.fromIu.pipe0.flush
  pipe0ExceptEntryData.jmpMispred   := false.B
  pipe0ExceptEntryData.bhtMispred   := false.B
  // Todo: figure out this Cat
  pipe0ExceptEntryData.mtval        := Cat(0.U(8.W), io.in.fromIu.pipe0.mtval(31, 0))
  pipe0ExceptEntryData.instMmu      := io.in.fromIu.pipe0.instMmuExcept
  pipe0ExceptEntryData.highHw       := io.in.fromIu.pipe0.highHwExcept
  pipe0ExceptEntryData.exceptVec    := io.in.fromIu.pipe0.exceptVec
  pipe0ExceptEntryData.iid          := io.in.fromIu.pipe0.iid

  exceptEntryDataUpdate := MuxLookup(exceptEntryWriteSel.asUInt, 0.U, Seq(
    UIntToOH(0.U) -> exceptEntryData.asUInt,
    UIntToOH(1.U) -> pipe0ExceptEntryData.asUInt,
    UIntToOH(2.U) -> pipe2ExceptEntryData.asUInt,
    UIntToOH(3.U) -> pipe3ExceptEntryData.asUInt,
    UIntToOH(4.U) -> pipe4ExceptEntryData.asUInt,
  )).asTypeOf(Output(new RobExceptEntry))

  //==========================================================
  //                 Exception Entry Valid
  //==========================================================
  when(io.in.fromRetire.flush || io.in.fromRtu.yyXxFlush) {
    //flush with rob, if spec inst on wrong path set expt, flush again
    exceptEntryValid := false.B
  }.elsewhen(exceptCmplt) {
    exceptEntryValid := true.B
  }.elsewhen(io.in.fromRetire.inst0Valid && io.in.fromRetire.inst0Abnormal) {
    exceptEntryValid := false.B
  }
  io.out.except.valid := exceptEntryValid
  io.out.except.iid   := exceptEntryData.iid

  //==========================================================
  //                     Except Complete Info
  //==========================================================
  io.out.except.validUpdate := Mux(
    exceptCmplt,
    exceptEntryDataUpdate.exceptVec.valid,
    exceptEntryData.exceptVec.valid
  )

  when(exceptCmplt) {
    exceptEntryData := exceptEntryDataUpdate
  }

//  val exceptEntry = WireInit(0.U.asTypeOf(new RobExceptEntry))
//  exceptEntry := exceptEntryData

  //----------------------------------------------------------
  //                 Rename for output
  //----------------------------------------------------------
  private val ssfSplitSpecFailFlush     = Wire(Bool())

  io.out.toRetire.inst0.exceptionVec.valid  :=  exceptEntryData.exceptVec.valid &&
                                                io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.exceptionVec.bits   :=  exceptEntryData.exceptVec.bits
  io.out.toRetire.inst0.instMmuException    :=  exceptEntryData.instMmu
  io.out.toRetire.inst0.highHwException     :=  exceptEntryData.highHw
  io.out.toRetire.inst0.instFlush           :=  (exceptEntryData.flush &&
                                                  !exceptEntryData.exceptVec.valid &&
                                                  io.in.fromRetire.inst0Abnormal) ||
                                                (!io.in.fromRetire.inst0Abnormal &&
                                                  ssfSplitSpecFailFlush)
  io.out.toRetire.inst0.jmpMispred          :=  exceptEntryData.jmpMispred &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.bhtMispred          :=  exceptEntryData.bhtMispred &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.mtval               :=  exceptEntryData.mtval
  io.out.toRetire.inst0.breakpoint          :=  exceptEntryData.breakpoint &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.specFailNoSsf       :=  exceptEntryData.specFail &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.specFailSsf         :=  ssfSplitSpecFailFlush &&
                                                  !io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.specFail            :=  io.out.toRetire.inst0.specFailSsf || io.out.toRetire.inst0.specFailNoSsf
  io.out.toRetire.inst0.efPcValid           :=  exceptEntryData.efPcValid &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.vsetvl              :=  exceptEntryData.vsetvl &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.vstart.valid        :=  exceptEntryData.vstart.valid &&
                                                  io.in.fromRetire.inst0Abnormal
  io.out.toRetire.inst0.vstart.bits         :=  exceptEntryData.vstart.bits

  //==========================================================
  //               Split instruction spec fail
  //==========================================================


  //----------------------------------------------------------
  //                Instance of Gated Cell
  //----------------------------------------------------------
  private val ssfSmStart = Wire(Bool())
  private val ssfClkEn = ssfSmStart || (ssfStateCur =/= SsfState.idle)
  // Todo: gated clk for ssf

  //----------------------------------------------------------
  //              control signal for ssf FSM
  //----------------------------------------------------------
  private val ssfSplitSpecFailRetire  = Wire(Bool())
  private val ssfInstRetireNoSplit    = Wire(Bool())

  ssfSmStart              := io.in.fromLsu.pipe3.da.splitSpecFailIid.valid || io.in.fromLsu.pipe4.da.splitSpecFailIid.valid
  ssfSplitSpecFailRetire  := io.in.fromRetire.inst0Valid && (io.in.fromRob.inst0Iid === ssfIid)
  ssfInstRetireNoSplit    := io.in.fromRetire.inst0Valid && !io.in.fromRob.inst0Split

  //----------------------------------------------------------
  //             FSM of inst ssf ctrl logic
  //----------------------------------------------------------
  // State Description:
  // IDLE       : no split instruction speculation failed
  // WF_RETIRE  : wait for split spec fail inst retire
  // RETIRING   : split spec fail inst is retring

  when(io.in.fromRetire.flush || io.in.fromRtu.yyXxFlush) {
    ssfStateCur := SsfState.idle
  }.otherwise {
    ssfStateCur := ssfStateNext
  }

  switch(ssfStateCur) {
    is(SsfState.idle) {
      when(ssfSmStart) {
        ssfStateNext := SsfState.wfRetire
      }.otherwise {
        ssfStateNext := SsfState.idle
      }
    }
    is(SsfState.wfRetire) {
      when(ssfSplitSpecFailRetire) {
        ssfStateNext := SsfState.retiring
      }.otherwise {
        ssfStateNext := SsfState.wfRetire
      }
    }
    is(SsfState.retiring) {
      when(ssfInstRetireNoSplit) {
        ssfStateNext := SsfState.idle
      }.otherwise {
        ssfStateNext := SsfState.retiring
      }
    }
  }

  //----------------------------------------------------------
  //                   control signals
  //----------------------------------------------------------
  //if there is split inst spec fail, to simplify the design, enable srt
  //wait for split inst retire
  io.out.toRetire.splitSpecFailSrt := ssfStateCur === SsfState.retiring
  //if in RETIRING state, wait for next no split inst and generate
  //spec fail flush if it is not abnormal
  ssfSplitSpecFailFlush := ssfStateCur === SsfState.wfRetire && ssfInstRetireNoSplit

  io.out.toTop.ssfStateCur := ssfStateCur

  //----------------------------------------------------------
  //                  Split Spec fail Age
  //----------------------------------------------------------
  private val ssfPipe4Iid = io.in.fromLsu.pipe4.da.splitSpecFailIid.bits
  private val ssfPipe3Iid = io.in.fromLsu.pipe3.da.splitSpecFailIid.bits

  private val ssfPipe4lt3 = CompareIidLess(ssfPipe4Iid, ssfPipe3Iid)
  private val ssfPipe4ltSm = CompareIidLess(ssfPipe4Iid, ssfIid)
  private val ssfPipe3ltSm = CompareIidLess(ssfPipe3Iid, ssfIid)

  //if older split spec fail occurs, update ssf_iid with older iid
  //and hold ssf sm state if it is not idle
  //because lsu should signal split spec fail before complete


  private val ssfPipe4IidValidUpdate = Wire(Bool())
  private val ssfPipe3IidValidUpdate = Wire(Bool())
  private val ssfSmIidValidUpdate = Wire(Bool())

  ssfPipe4IidValidUpdate :=
    io.in.fromLsu.pipe4.da.splitSpecFailIid.valid &&
      (!io.in.fromLsu.pipe3.da.splitSpecFailIid.valid || ssfPipe4lt3) &&
      ((ssfStateCur === SsfState.idle) || ssfPipe4ltSm)
  ssfPipe3IidValidUpdate :=
    io.in.fromLsu.pipe3.da.splitSpecFailIid.valid &&
      (!io.in.fromLsu.pipe4.da.splitSpecFailIid.valid || !ssfPipe4lt3) &&
      ((ssfStateCur === SsfState.idle) || ssfPipe3ltSm)
  ssfSmIidValidUpdate :=
    (!ssfStateCur === SsfState.idle) &&
      (!io.in.fromLsu.pipe3.da.splitSpecFailIid.valid || !ssfPipe3ltSm) &&
      (!io.in.fromLsu.pipe4.da.splitSpecFailIid.valid || !ssfPipe4ltSm)

  //----------------------------------------------------------
  //               Split spec fail inst iid
  //----------------------------------------------------------
  private val ssfIidUpdate = Wire(UInt(InstructionIdWidth.W))
  ssfIidUpdate := MuxCase(0.U, Seq(
    ssfPipe3IidValidUpdate  -> ssfPipe3Iid,
    ssfPipe4IidValidUpdate  -> ssfPipe4Iid,
    ssfSmIidValidUpdate     -> ssfIid
  ))

  when(reset.asBool) {

  }.otherwise {
    ssfIid := ssfIidUpdate
  }
  io.out.toRetire.ssfIid := ssfIid
}
