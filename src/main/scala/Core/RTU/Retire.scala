package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.AddrConfig._
import Core.Config.XLEN
import Core.IntConfig._
import Core.ExceptionConfig._
import Core.PipelineConfig.NumPipeline
import Core.VectorUnitConfig._
import Utils.Bits.{sext, zext}

// State Description:
// AE_IDLE    : no asynchronous exception or LSU trigger async expt
// AE_WFC     : wait for commiting inst retire, stop new inst commit
// AE_WFI     : stop rob retire entry valid, wait for retire inst 0/1/2
//              not valid and FLUSH state machine IDLE and ifu fetch vec addr
// AE_EXPT    : signal IFU expt valid, trigger FLUSH state machine
object AsyncExceptState {
  def size : Int = 4
  def width : Int = log2Up(size)
  val idle :: wfc :: wfi :: except :: Nil = Enum(size)
}

// State Description:
// FLUSH_IDLE  : no flush or retiring inst 0 will trigger flush.
//               if triggering, stop commit, flush ROB, retire/expt entry
// FLUSH_IS    : flush IDU IS/RF, flush IDU ptag pool, start to stall
//               IDU ID, stall IDU ID
// FLUSH_FE    : flush IFU and IDU ID/IR/IS/RF, flush IDU ptag pool,
//               stall IDU ID
// WF_EMPTY    : wait PST retired and released entry WB, stall IDU ID
// FLUSH_BE    : flush PST, recover rename table,
//               stop IDU ID mispred stall, stall IDU ID
// FLUSH_IS_BE : flush IS and flush backend
// FLUSH_FE_BE : flush frontend and flush backend
object FlushState {
  def size = 7

  def width = 5

  def idle    : UInt = "b00001".U(width.W)
  def is      : UInt = "b00010".U(width.W)
  def fe      : UInt = "b00100".U(width.W)
  def wfEmpty : UInt = "b01000".U(width.W)
  def be      : UInt = "b10000".U(width.W)
  //
  def is_be   : UInt = (is.litValue | be.litValue).U
  def fe_be   : UInt = (fe.litValue | be.litValue).U

  def hasStateIs(src : UInt) : Bool = src(1)

  def hasStateFe(src : UInt) : Bool = src(2)

  def hasStateBe(src : UInt) : Bool = src(4)
}

class RetireFromCp0Bundle extends Bundle {
  val icgEn : Bool = Bool()
  val srtEn : Bool = Bool()
  val yyClkEn : Bool = Bool()
}

class RetireFromHadBundle extends Bundle {
  val debugDisable : Bool = Bool()
  val debugReqEn : Bool = Bool()
  val eventDebugReq : Bool = Bool()
  // Todo: figure out
  val fdb : Bool = Bool()
  val hwDebugReq : Bool = Bool()
  val hwDebugReqGateClk : Bool = Bool()
  val nonIrvBreakpointDebugReq : Bool = Bool()
  val pop1Disable : Bool = Bool()
  val traceDebugReq : Bool = Bool()
  val traceEn : Bool = Bool()
  val xxJdbReq : Bool = Bool()
  val yyXxExitDebug : Bool = Bool()
}

class RetireFromLsuBundle extends Bundle {
  val allCommitDataValid : Bool = Bool()
  // Todo: imm
  val asyncExceptAddr : UInt = UInt(40.W)
  val asyncExceptValid : Bool = Bool()
  val ctcFlushValid : Bool = Bool()
}

class RetireFromMmuBundle extends Bundle {
  val mmuEn : Bool = Bool()
}

class RetireFromPstBundle extends Bundle {
  val retiredRegWb : Bool = Bool()
}

class RetireInput extends Bundle {
  val fromCp0 = new RetireFromCp0Bundle
  val fromHad = new RetireFromHadBundle
  val fromHpcp = new RtuFromHpcpBundle
  val fromLsu = new RetireFromLsuBundle
  val fromMmu = new RetireFromMmuBundle
  val fromPad = new RtuFromPadBundle
  val fromPst = new RetireFromPstBundle
  val fromRob = Output(new RobToRetireBundle {})
}

class RetireToPst extends Bundle {
  val asyncFlush : Bool = Bool()
  val wbInstPregValid : Vec[Bool] = Vec(3, Bool())
  val wbInstVregValid : Vec[Bool] = Vec(3, Bool())
  val wbInstEregValid : Vec[Bool] = Vec(3, Bool())
}

class RtuToCp0Bundle extends Bundle {
  val epc : UInt = UInt(XLEN.W)
  val exceptGateClkValid : Bool = Bool()
  // Todo: imm
  val exceptMtval : UInt = UInt(64.W)
  val exceptValid : Bool = Bool()
  val fpDirty : Bool = Bool()
  val vecDirty : Bool = Bool()
  val intAck : Bool = Bool()
  val vConfigVill : Bool = Bool()
  val vConfigVl = ValidIO(UInt(VlmaxBits.W))
  val vConfigVlmul : UInt = UInt(VlmulBits.W)
  val vConfigVsew : UInt = UInt(VsewBits.W)
  val vConfigVtypeValid : Bool = Bool()
  val vStart = ValidIO(UInt(VlmaxBits.W))
}

class RetireToHadBundle extends Bundle {
  val debugAckInfo : Bool = Bool()
  val debugReqAck : Bool = Bool()
  val inst0BreakpointInst : Bool = Bool()
  val debugAckPc : Bool = Bool()
  val mBreakpointDataAck : Bool = Bool()
  val mBreakpointInstAck : Bool = Bool()
  val pc : UInt = UInt(PcWidth.W)
  class RetireToHadPcFifo extends Bundle {
    val changeFlow : Bool = Bool()
    val condBr : Bool = Bool()
    val condBrTaken : Bool = Bool()
    val jmp : Bool = Bool()
    val npc : UInt = UInt(PcWidth.W)
    val pCall : Bool = Bool()
    val pReturn : Bool = Bool()
  }
  val pcFifoData = new Bundle {
    val instVec : Vec[RetireToHadPcFifo] = Vec(3, new RetireToHadPcFifo)
    val inst0Iid : UInt = UInt(InstructionIdWidth.W)
  }
  val splitInst = Bool()
}

class RetireToHpcp extends Bundle {
  class RetireToHpcpCommon extends Bundle {
    val condBr : Bool = Bool()
    val jmp : Bool = Bool()
    // Todo: imm
    val num : UInt = UInt(2.W)
    // Todo: imm
    val pcOffset : UInt = UInt(3.W)
    val split : Bool = Bool()
    val store : Bool = Bool()
    val valid : Bool = Bool()
  }
  class RetireToHpcpExtra extends Bundle {
    val ackInt : Bool = Bool()
    val bhtMispred : Bool = Bool()
    val jmpMispred : Bool = Bool()
    val specFail : Bool = Bool()
  }
  class RetireToHpcpTrace extends Bundle {
    val changeFlow : Bool = Bool()
    val npc : UInt = UInt(PcWidth.W)
  }
  // Todo: imm
  val instVec : Vec[RetireToHpcpCommon] = Vec(3, new RetireToHpcpCommon)
  val inst0 : RetireToHpcpExtra = new RetireToHpcpExtra
  val traceVec : Vec[RetireToHpcpTrace] = Vec(3, new RetireToHpcpTrace)
}

class RetireToIduBundle extends Bundle {
  val flushFe : Bool = Bool()
  val flushIs : Bool = Bool()
  val flushStall : Bool = Bool()
  val retire0InstValid : Bool = Bool()
  val srtEn : Bool = Bool()
}

class RetireToIfuBundle extends Bundle {
  val changeFlowPc : UInt = UInt(PcWidth.W)
  val changeFlowValid : Bool = Bool()
  val flush : Bool = Bool()
  class RetireToIfuRetireCommon extends Bundle {
    // Todo: imm
    val checkIdx : UInt = UInt(8.W)
    val condBr : Bool = Bool()
    val condBrTaken : Bool = Bool()
    val jmp : Bool = Bool()
  }
  class RetireToIfuRetireExtra extends Bundle {
    // Todo: figure out naming
    val incPc : UInt = UInt(PcWidth.W)
    val jmpMispred : Bool = Bool()
    val mispred : Bool = Bool()
    val npc : UInt = UInt(PcWidth.W)
    val pCall : Bool = Bool()
    val pReturn : Bool = Bool()
  }
  val retireVec : Vec[RetireToIfuRetireCommon] = Vec(3, new RetireToIfuRetireCommon)
  val retire0 : RetireToIfuRetireExtra = new RetireToIfuRetireExtra
  class RetireToIfuRetireInstCommon extends Bundle {
    val pc : UInt = UInt(PcWidth.W)
    val load : Bool = Bool()
    val noSpec = new RobNoSpecBundle
    val store : Bool = Bool()
    val vlPred : Bool = Bool()
  }
  class RetireToIfuRetireInstExtra extends Bundle {
    val vlHit : Bool = Bool()
    val vlMispred : Bool = Bool()
    val vlMiss : Bool = Bool()
  }
  // Todo: imm
  val retireInstVec : Vec[RetireToIfuRetireInstCommon] = Vec(3, new RetireToIfuRetireInstCommon)
  val retireInst0 = new RetireToIfuRetireInstExtra
  val debugOn : Bool = Bool()
  // Todo: imm
  // Todo: figure out why width 6
  val exceptVec = ValidIO(UInt(6.W))
}

class RetireToIu extends Bundle {
  val flushChangeFlowMask : Bool = Bool()
  val flushFe : Bool = Bool()
}

class RetireToLsu extends Bundle {
  val asyncFlush : Bool = Bool()
  val eretFlush : Bool = Bool()
  val exceptFlush : Bool = Bool()
  val specFailFlush : Bool = Bool()
  val specFailIid : UInt = UInt(InstructionIdWidth.W)
}

class RetireToMmuBundle extends Bundle {
  // Todo: imm
  val badVpn : UInt = UInt(27.W)
  val exceptValid : Bool = Bool()
}

class RetireOutput extends Bundle {
  val toPst = new RetireToPst
  val toRob = new RobFromRetire {}
  val toTop = new Bundle {
    // Todo: imm
    val aeState : UInt = UInt(2.W)
  }
  val toCp0 = new RtuToCp0Bundle
  val toHad = new RetireToHadBundle
  val toHpcp = new RetireToHpcp
  val toIdu = new RetireToIduBundle
  val toIfu = new RetireToIfuBundle
  val toIu = new RetireToIu
  val toLsu = new RetireToLsu
  val toMmu = new RetireToMmuBundle
  val yyXx = new Bundle {
    val debugOn : Bool = Bool()
    val exceptVec : UInt = UInt(6.W)
    val flush : Bool = Bool()
    val retire0Normal : Bool = Bool()
  }
}

class RetireIO extends Bundle {
  val in : RetireInput = Flipped(Output(new RetireInput))
  val out : RetireOutput = Output(new RetireOutput)
}

class Retire extends Module {
  val io : RetireIO = IO(new RetireIO)

  private val had = io.in.fromHad
  private val lsu = io.in.fromLsu
  private val cp0 = io.in.fromCp0
  private val mmu = io.in.fromMmu
  private val pst = io.in.fromPst
  private val hpcp = io.in.fromHpcp
  private val rob = io.in.fromRob
  private val pad = io.in.fromPad

  /**
   * States
   */
  private val aeStateCur = RegInit(AsyncExceptState.idle)
  // Todo: figure out why wire type need this initial value when AsyncExceptState is sequential
  val aeStateNext = WireInit(AsyncExceptState.idle)

  private val flushStateCur = RegInit(FlushState.idle)
  // Flush State is not sequential, initial value needed here
  val flushStateNext = WireInit(FlushState.idle)
  /**
   * Regs
   */
  // Todo: imm
  private val aePhyAddr = RegInit(0.U(40.W))
  private val debugModeOn = RegInit(false.B)
  private val flushEret = RegInit(false.B)
  private val flushExcept = RegInit(false.B)
  private val flushSpecFail = RegInit(false.B)
  private val ifuDebugModeOn = RegInit(false.B)
  private val ctcFlushReq = RegInit(false.B)
  private val toHpcp = RegInit(0.U.asTypeOf(new RetireToHpcp))
  private val toIfuChangeFlowValid = RegInit(false.B)
  // Todo: imm
  private val toIfuExceptVec = RegInit(0.U(6.W))
  private val toIfuExceptValid = RegInit(false.B)
  private val specFailIid = RegInit(0.U(InstructionIdWidth.W))

  /**
   * Wires
   */
  private val exceptMtvalSrc = Wire(UInt(40.W))

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  // Todo: gated clk
  // val retireClkEn =

  //==========================================================
  //                  Single Retire Mode
  //==========================================================
  // Todo: figure out idu, cp0
  //when meet following condition, RTU will enable single retire
  //mode: IDU stop folding, ROB read 1/2 will not valid
  private val srtEn = had.pop1Disable || had.debugReqEn || cp0.srtEn
  io.out.toRob.srtEn := srtEn || rob.splitSpecFailSrt ||
    rob.intSrtEn || rob.ctcFlushSrtEn
  io.out.toIdu.srtEn := srtEn

  //==========================================================
  //                   Retire valid signals
  //==========================================================
  //retire inst 0 may expt vld, but retire inst 1/2 are always normal
  private val instNormalRetireVec = Wire(Vec(NumRetireEntry, Bool()))
  instNormalRetireVec(0) := rob.instVec(0).valid && !rob.instExtra.exceptionVec.valid
  instNormalRetireVec(1) := rob.instVec(1).valid
  instNormalRetireVec(2) := rob.instVec(2).valid

  io.out.toIdu.retire0InstValid := rob.instVec(0).valid
  io.out.yyXx.retire0Normal := instNormalRetireVec(0)

  // Todo: move bundles into rob
  //if inst bkpt or expt vld, retire inst0 cannot write back
  io.out.toPst.wbInstPregValid := rob.instVec.map(_.pstPregValid)
  io.out.toPst.wbInstVregValid := rob.instVec.map(_.pstVregValid)
  io.out.toPst.wbInstEregValid := rob.instVec.map(_.pstEregValid)


  //==========================================================
  //             Retire (Inst 0) Exception Process
  //==========================================================
  //Exception, Interrupt and debug can ONLY hit retire inst 0

  //----------------------------------------------------------
  //                 Prepare Exception Source
  //----------------------------------------------------------
  private val exceptInstValid = rob.instExtra.exceptionVec.valid
  // Todo: figure out exception vector
  private val exceptMmuBadVpn = rob.instExtra.exceptionVec.valid &&
    rob.instExtra.exceptionVec.bits(3,2) === "b11".U

  //----------------------------------------------------------
  //                 Prepare Interrupt Source
  //----------------------------------------------------------
  private val interValid = rob.instExtra.interruptVec.valid &&
    !rob.instVec(0).split && !rob.instExtra.interruptMask

  //----------------------------------------------------------
  //                    Exception Vector
  //----------------------------------------------------------
  // interrupt and exception
  private val exceptVec = Wire(UInt(6.W))
  exceptVec := Mux(
    interValid,
    Cat(true.B, rob.instExtra.interruptVec.bits.asUInt),
    Cat(0.U(2.W), rob.instExtra.exceptionVec.bits.asUInt)
  )

  //----------------------------------------------------------
  //                         MTVAL
  //----------------------------------------------------------
  private val ackInterrupt = Wire(Bool())
  when(aeStateCur === AsyncExceptState.except) {
    exceptMtvalSrc := aePhyAddr
  }.elsewhen(ackInterrupt) {
    exceptMtvalSrc := 0.U
  }.elsewhen(rob.instExtra.instMmuException && !rob.instExtra.highHwException) {
    exceptMtvalSrc := Cat(rob.instVec(0).pc, 0.U(1.W))
  }.elsewhen(rob.instExtra.instMmuException) {
    // Todo: imm
    //32 bit inst cross 4k page fault, high half-word is 4k align of next pc
    exceptMtvalSrc := Cat(rob.instVec(0).npc(38,11), 0.U(12.W))
  }.otherwise {
    exceptMtvalSrc := rob.instExtra.mtval
  }

  // Todo: figure out mtval
  // Todo: check if meet risc-v privilege spec
  private val exceptMtval = Mux(
    mmu.mmuEn && !aeStateCur === AsyncExceptState.except,
    sext(64, exceptMtvalSrc),
    zext(64, exceptMtvalSrc)
  )

  //----------------------------------------------------------
  //             Exception and Interrupt Priority
  //----------------------------------------------------------
  private val ackMmu = Wire(Bool())
  private val exceptValid = Wire(Bool())
  private val exceptGateClkValid = Wire(Bool())

  private val debugReqAck = Wire(Bool())

  ackInterrupt := interValid
  ackMmu := exceptInstValid && exceptMmuBadVpn && !interValid
  exceptValid := rob.instVec(0).valid && !debugReqAck &&
    (exceptInstValid || interValid)
  exceptGateClkValid := rob.instVec(0).valid && (exceptInstValid || interValid)
  io.out.toRob.debug.exceptionValid := exceptGateClkValid

  //----------------------------------------------------------
  //            IFU Exception and Interrupt Output
  //----------------------------------------------------------

  private val toIfuExceptValidUpdate =
    (exceptValid || aeStateCur === AsyncExceptState.except) && !debugModeOn
  //  Tick 1             Tick 0
  toIfuExceptValid := toIfuExceptValidUpdate
  io.out.toIfu.exceptVec.valid := toIfuExceptValid

  // Todo: move
  private val asyncExceptVec = Wire(UInt(6.W))

  private val toIfuExceptVecUpdate = Mux(
    aeStateCur === AsyncExceptState.except,
    asyncExceptVec,
    exceptVec
  )
  io.out.yyXx.exceptVec := toIfuExceptVecUpdate

  when(toIfuExceptValidUpdate) {
    toIfuExceptVec := toIfuExceptVecUpdate
  }
  io.out.toIfu.exceptVec.bits := toIfuExceptVec

  //----------------------------------------------------------
  //            MMU Exception and Interrupt Output
  //----------------------------------------------------------
  io.out.toMmu.exceptValid := rob.instVec(0).valid && ackMmu && !debugModeOn

  // Todo: imm
  io.out.toMmu.badVpn := exceptMtval(38,12)

  //----------------------------------------------------------
  //            CP0 Exception and Interrupt Output
  //----------------------------------------------------------
  io.out.toCp0.exceptValid := (exceptValid || aeStateCur === AsyncExceptState.except) && !debugModeOn
  io.out.toCp0.exceptGateClkValid := exceptGateClkValid || aeStateCur === AsyncExceptState.except
  io.out.toCp0.exceptMtval := exceptMtval

  /**
   * epc = pc, if exception or breakpoint
   * epc = npc, else
   */
  private val inst0epc = Mux(
    rob.instExtra.exceptionVec.valid || rob.instExtra.instBreakPoint,
    rob.instVec(0).pc,
    rob.instVec(0).npc
  )

  // Todo: figure out
  private val cp0epc = Mux(
    aeStateCur === AsyncExceptState.except,
    rob.robCurPc,
    inst0epc
  )

  io.out.toCp0.epc := Mux(
    mmu.mmuEn,
    sext(XLEN, Cat(cp0epc, 0.U(1.W))),
    zext(XLEN, Cat(cp0epc, 0.U(1.W)))
  )
  io.out.toCp0.intAck := rob.instVec(0).valid && ackInterrupt && !debugReqAck && !debugModeOn
  io.out.toCp0.fpDirty := rob.instVec.map {
    inst => inst.pstVregValid && inst.fpDirty
  }.reduce(_||_)

  io.out.toCp0.vecDirty := rob.instVec.map {
    inst => inst.pstVregValid && inst.vecDirty
  }.reduce(_||_)

  io.out.toRob.debug.instAckInt := rob.instVec(0).valid && ackInterrupt

  //----------------------------------------------------------
  //                  CP0 Vector Values
  //----------------------------------------------------------
  private val instVsetvliVec = Wire(Vec(NumRetireEntry, Bool()))
  for (i <- 0 until NumRetireEntry) {
    instVsetvliVec(i) := instNormalRetireVec(i) && rob.instVec(i).vsetvli
  }
  // Todo: figure out
  private val instVsetvlx = instNormalRetireVec(0) && rob.instExtra.vsetvl
  private val vConfigIllegal    = instVsetvlx && rob.instExtra.mtval(13)
  private val vConfigVlMispred  = instVsetvlx && rob.instExtra.mtval(14)
  private val vConfigVlFof      = instVsetvlx && rob.instExtra.mtval(15)

  io.out.toRob.splitFofFlush := vConfigVlFof && rob.instVec(0).split
  io.out.toCp0.vConfigVill := vConfigIllegal
  io.out.toCp0.vConfigVl.valid := instVsetvliVec.reduce(_||_) ||
    vConfigIllegal || vConfigVlMispred || vConfigVlFof
  io.out.toCp0.vConfigVtypeValid := instVsetvliVec.reduce(_||_) ||
    instVsetvlx && !vConfigVlFof

  when(instVsetvlx) {
    // Todo: fix mtval
    io.out.toCp0.vConfigVlmul := rob.instExtra.mtval(1,0)
    io.out.toCp0.vConfigVsew := rob.instExtra.mtval(4,2)
    io.out.toCp0.vConfigVl.bits := rob.instExtra.mtval(12,5)
  }.elsewhen(instVsetvliVec(2)) {
    io.out.toCp0.vConfigVlmul := rob.instVec(2).vlmul
    io.out.toCp0.vConfigVsew := rob.instVec(2).vsew
    io.out.toCp0.vConfigVl.bits := rob.instVec(2).vl
  }.elsewhen(instVsetvliVec(1)) {
    io.out.toCp0.vConfigVlmul := rob.instVec(1).vlmul
    io.out.toCp0.vConfigVsew := rob.instVec(1).vsew
    io.out.toCp0.vConfigVl.bits := rob.instVec(1).vl
  }.otherwise {
    io.out.toCp0.vConfigVlmul := rob.instVec(0).vlmul
    io.out.toCp0.vConfigVsew := rob.instVec(0).vsew
    io.out.toCp0.vConfigVl.bits := rob.instVec(0).vl
  }
  io.out.toCp0.vStart.valid := rob.instVec(0).valid && rob.instExtra.vstart.valid
  io.out.toCp0.vStart.bits := rob.instExtra.vstart.bits

  //==========================================================
  //                    RTU IFU Interface
  //==========================================================
  io.out.toIfu.retire0.mispred := instNormalRetireVec(0) &&
    (rob.instExtra.bhtMispred || rob.instExtra.jmpMispred)

  io.out.toRob.debug.mispred := instNormalRetireVec(0) &&
    (rob.instExtra.bhtMispred || rob.instExtra.jmpMispred)

  //----------------------------------------------------------
  //                    Conditional Branch
  //----------------------------------------------------------
  private val instCondBrVec = Wire(Vec(NumRetireEntry, Bool()))
  for (i <- 0 until NumRetireEntry) {
    instCondBrVec(i) := instNormalRetireVec(i) && rob.instVec(i).condBranch
    io.out.toIfu.retireVec(i).condBr := instCondBrVec(i)
    io.out.toIfu.retireVec(i).condBrTaken := instNormalRetireVec(i) && rob.instVec(i).condBrTaken
  }

  //----------------------------------------------------------
  //                      Return Stack
  //----------------------------------------------------------
  io.out.toIfu.retire0.pCall := instNormalRetireVec(0) && rob.instExtra.pCall
  io.out.toIfu.retire0.pReturn := instNormalRetireVec(0) && rob.instExtra.pReturn
  io.out.toIfu.retire0.incPc := rob.instExtra.bjuIncPc

  //----------------------------------------------------------
  //                      Indirect Jump
  //----------------------------------------------------------
  private val jmpMispred = instNormalRetireVec(0) && rob.instExtra.jmpMispred && !rob.instExtra.pReturn
  private val instJmpVec = Wire(Vec(NumRetireEntry, Bool()))

  io.out.toIfu.retire0.jmpMispred := jmpMispred
  instJmpVec(0) := instNormalRetireVec(0) &&
    rob.instVec(0).jmp && !rob.instExtra.pReturn
  instJmpVec(1) := instNormalRetireVec(1) &&
    rob.instVec(1).jmp
  instJmpVec(2) := instNormalRetireVec(2) &&
    rob.instVec(2).jmp

  for (i <- 0 until NumRetireEntry) {
    io.out.toIfu.retireVec(i).jmp := instJmpVec(i)
    io.out.toIfu.retireVec(i).checkIdx := rob.instVec(i).checkIdx
  }
  io.out.toIfu.retire0.npc := rob.instVec(0).npc

  //----------------------------------------------------------
  //                         No Spec
  //----------------------------------------------------------

  for(i <- 0 until NumRetireEntry) {
    io.out.toIfu.retireInstVec(i).load := instNormalRetireVec(i) && rob.instVec(i).load
    io.out.toIfu.retireInstVec(i).store := instNormalRetireVec(i) && rob.instVec(i).store
    io.out.toIfu.retireInstVec(i).noSpec := rob.instVec(i).noSpec
    io.out.toIfu.retireInstVec(i).pc := rob.instVec(i).pc
  }

  //----------------------------------------------------------
  //                          Vl
  //----------------------------------------------------------

  for(i <- 0 until NumRetireEntry) {
    io.out.toIfu.retireInstVec(i).vlPred := instVsetvliVec(i) && rob.instVec(i).vlPred
  }
  // vl need not output

  // Todo: figure out
  io.out.toIfu.retireInst0.vlMispred := vConfigVlMispred && instVsetvliVec(0) && rob.instVec(0).vlPred
  io.out.toIfu.retireInst0.vlHit := !vConfigVlMispred && instVsetvliVec(0) && rob.instVec(0).vlPred
  io.out.toIfu.retireInst0.vlMiss := vConfigVlMispred && instVsetvliVec(0) && !rob.instVec(0).vlPred

  //----------------------------------------------------------
  //                   RTU IFU Change Flow
  //----------------------------------------------------------
  //if flush inst retires without exception, signal rob to flop rob cur pc
  //into retire inst0 pc and then output to IFU PC MUX

  private val inst0InstFlush = instNormalRetireVec(0) &&
    (rob.instExtra.instFlush || rob.instExtra.ctcFlush) &&
    !rob.instVec(0).split
  io.out.toRob.instFlush := inst0InstFlush
  io.out.toRob.debug.instflush := inst0InstFlush

  // Todo: figure out why changeFlowValid need reg next, while changeFlowPc not
  //flop and then signal IFU to changeflow
  io.out.toIfu.changeFlowValid := RegNext(inst0InstFlush)
  //at this time, flush change flow pc is in retire inst0 cur pc

  val pc0Vec = io.in.fromRob.instVec(0).debug.pc
  io.out.toIfu.changeFlowPc := RegNext(Cat(pc0Vec(0), 0.U(1.W))(63, 0) | BigInt("80000000", 16).U)

  //==========================================================
  //                    Debug Interface
  //==========================================================

  //----------------------------------------------------------
  //                  Prepare Debug Source
  //----------------------------------------------------------
  private val debugReqAckHw         = rob.instVec(0).valid && !rob.instVec(0).split && had.hwDebugReq
  private val debugReqAckBreakpoint = rob.instVec(0).valid && rob.instExtra.breakPoint && had.fdb
  private val debugReqAckTrace      = had.traceDebugReq
  private val debugReqAckEvent      = rob.instVec(0).valid && !rob.instVec(0).split && had.eventDebugReq
  private val debugReqAckJdbReq     = had.xxJdbReq && !had.debugDisable
  private val debugReqAckMBreakpoint = rob.instExtra.instBreakPoint || instNormalRetireVec(0) && rob.instExtra.dataBreakpoint
  private val debugReqAckNonIrv     = rob.instVec(0).valid && !rob.instVec(0).split && had.nonIrvBreakpointDebugReq

  //cannot enter into debug mode if commit1/2 valid, debug request will
  //trigger single retire mode and can only ack at retire inst0
  //except async jdbreq, which ignores any disable
  private val debugReqAckWithOutEvent = !rob.instExtra.debugDisable &&
    (debugReqAckHw || debugReqAckBreakpoint ||
      debugReqAckTrace || debugReqAckMBreakpoint || debugReqAckNonIrv) ||
    debugReqAckJdbReq

  debugReqAck := (debugReqAckWithOutEvent || !rob.instExtra.debugDisable && debugReqAckEvent) &&
    !had.debugDisable

  // Todo: figure out
  io.out.toHad.debugReqAck := debugReqAckWithOutEvent

  private val debugReqAckGateClk = had.hwDebugReqGateClk ||
    debugReqAckBreakpoint || had.traceEn || debugReqAckMBreakpoint ||
    debugReqAckJdbReq || debugReqAckEvent || debugReqAckNonIrv

  //----------------------------------------------------------
  //              Debug Ack and Mode on signal
  //----------------------------------------------------------
  when(FlushState.hasStateBe(flushStateCur) && ifuDebugModeOn) {
    debugModeOn := true.B
  }.elsewhen(had.yyXxExitDebug) {
    debugModeOn := false.B
  }

  io.out.yyXx.debugOn := debugModeOn

  //stop ifu fetch new inst immediate after dbgack
  when(debugReqAck) {
    ifuDebugModeOn := true.B
  }.elsewhen(had.yyXxExitDebug) {
    ifuDebugModeOn := false.B
  }

  io.out.toIfu.debugOn            := ifuDebugModeOn
  io.out.toRob.debug.debugModeOn  := ifuDebugModeOn

  //----------------------------------------------------------
  //                      Debug Inteface
  //----------------------------------------------------------
  //for HAD CPUSCR Todo: figure out
  io.out.toHad.debugAckPc := debugReqAck
  //for had info
  io.out.toHad.debugAckInfo := debugReqAckJdbReq && !ifuDebugModeOn

  io.out.toHad.pc := Mux(
    rob.instExtra.exceptionVec.valid || rob.instExtra.instBreakPoint,
    rob.instVec(0).pc,
    rob.robCurPc
  )

  io.out.toHad.splitInst := rob.instVec(0).split

  //for HAD PCFIFO
  for (i <- 0 until NumRetireEntry) {
    io.out.toHad.pcFifoData.instVec(i).changeFlow := instNormalRetireVec(i) && rob.instVec(i).bju
    io.out.toHad.pcFifoData.instVec(i).condBr := rob.instVec(i).condBranch
    io.out.toHad.pcFifoData.instVec(i).condBrTaken := rob.instVec(i).condBrTaken
    io.out.toHad.pcFifoData.instVec(i).npc := rob.instVec(i).npc
  }
  io.out.toHad.pcFifoData.instVec(0).pCall := rob.instExtra.pCall
  io.out.toHad.pcFifoData.instVec(1).pCall := 0.U
  io.out.toHad.pcFifoData.instVec(2).pCall := 0.U
  io.out.toHad.pcFifoData.instVec(0).pReturn := rob.instExtra.pReturn
  io.out.toHad.pcFifoData.instVec(1).pReturn := 0.U
  io.out.toHad.pcFifoData.instVec(2).pReturn := 0.U
  io.out.toHad.pcFifoData.instVec(0).jmp := rob.instVec(0).jmp && !rob.instExtra.pCall && !rob.instExtra.ras
  io.out.toHad.pcFifoData.instVec(1).jmp := rob.instVec(1).jmp
  io.out.toHad.pcFifoData.instVec(2).jmp := rob.instVec(2).jmp
  io.out.toHad.pcFifoData.inst0Iid := rob.instExtra.iid

  //for breakpoint
  io.out.toHad.inst0BreakpointInst := rob.instExtra.breakPoint

  //for memory breakpoint Todo: figure out
  io.out.toHad.mBreakpointInstAck := rob.instExtra.instBreakPoint && !rob.instExtra.debugDisable
  io.out.toHad.mBreakpointDataAck := instNormalRetireVec(0) &&
    rob.instExtra.dataBreakpoint && !rob.instExtra.debugDisable

  for(i <- 0 until NumRetireEntry) {
    io.out.toHpcp.traceVec(i).changeFlow := instNormalRetireVec(i) && rob.instVec(i).bju
    io.out.toHpcp.traceVec(i).npc := rob.instVec(i).npc
  }

  //----------------------------------------------------------
  //                 Performance Monitor
  //----------------------------------------------------------

  when(hpcp.cntEn && rob.instVec(0).valid) {
    for (i <- 0 until NumRetireEntry) {
      toHpcp.instVec(i).valid     := rob.instVec(i).valid
      toHpcp.instVec(i).split     := rob.instVec(i).split
      toHpcp.instVec(i).num       := rob.instVec(i).num
      toHpcp.instVec(i).pcOffset  := rob.instVec(i).pcOffset
      toHpcp.instVec(i).condBr    := instCondBrVec(i)
      // tick 1                       tick 0
      toHpcp.instVec(i).jmp       := instJmpVec(i)
      toHpcp.instVec(i).store     := rob.instVec(i).store
    }
    toHpcp.inst0.bhtMispred := instNormalRetireVec(0) && rob.instExtra.bhtMispred
    toHpcp.inst0.jmpMispred := rob.instExtra.jmpMispred
    toHpcp.inst0.specFail := rob.instExtra.specFail
    toHpcp.inst0.ackInt := !debugModeOn && ackInterrupt
  }.otherwise {
    for (i <- 0 until NumRetireEntry) {
      toHpcp.instVec(i).valid := false.B
    }
  }

  io.out.toHpcp := toHpcp

  for (i <- 0 until NumRetireEntry) {
    // tick 1                tick 1
    io.out.toRob.instJmp(i) := toHpcp.instVec(i).jmp
  }

  //==========================================================
  //                    Flush Control
  //==========================================================

  //----------------------------------------------------------
  //              Prepare state machine signals
  //----------------------------------------------------------

  private val inst0flush = exceptValid || inst0InstFlush || debugReqAck || aeStateCur === AsyncExceptState.except
  private val inst0mispred = instNormalRetireVec(0) && (rob.instExtra.jmpMispred || rob.instExtra.bhtMispred)
  private val inst0flushGateClk = exceptGateClkValid || inst0InstFlush || debugReqAck
  private val flushPipelineEmpty = pst.retiredRegWb && lsu.allCommitDataValid

  // Todo: move
  private val asyncFlush = Wire(Bool())
  when(asyncFlush) {
    flushStateCur := FlushState.fe_be
  }.otherwise {
    flushStateCur := flushStateNext
  }

  switch(flushStateCur) {
    is(FlushState.idle) {
      when(inst0flush && flushPipelineEmpty) {
        flushStateNext := FlushState.fe_be
      }.elsewhen(inst0flush) {
        flushStateNext := FlushState.fe
      }.elsewhen(inst0mispred && flushPipelineEmpty) {
        flushStateNext := FlushState.is_be
      }.elsewhen(inst0mispred) {
        flushStateNext := FlushState.is
      }.otherwise {
        flushStateNext := FlushState.idle
      }
    }
    is(FlushState.is) {
      when(flushPipelineEmpty) {
        flushStateNext := FlushState.be
      }.otherwise {
        flushStateNext := FlushState.wfEmpty
      }
    }
    is(FlushState.fe) {
      when(flushPipelineEmpty) {
        flushStateNext := FlushState.be
      }.otherwise {
        flushStateNext := FlushState.wfEmpty
      }
    }
    is(FlushState.wfEmpty) {
      when(flushPipelineEmpty) {
        flushStateNext := FlushState.be
      }.otherwise {
        flushStateNext := FlushState.wfEmpty
      }
    }
    is(FlushState.is_be) {
      // Todo: figure out why not wait for empty
      flushStateNext := FlushState.idle
    }
    is(FlushState.fe_be) {
      flushStateNext := FlushState.idle
    }
    is(FlushState.be) {
      flushStateNext := FlushState.idle
    }
  }

  //----------------------------------------------------------
  //                   Control Siganls
  //----------------------------------------------------------
  val retire_flush_is = flushStateCur(1).asBool
  val retire_flush_fe = flushStateCur(2).asBool
  val retire_flush_be = flushStateCur(4).asBool
  io.out.toIfu.flush    := retire_flush_fe
  io.out.toIdu.flushFe  := retire_flush_fe
  io.out.toIdu.flushIs  := retire_flush_is
  io.out.toIu.flushFe   :=retire_flush_fe

  io.out.yyXx.flush     := retire_flush_be
  io.out.toRob.flush    := inst0flush || inst0mispred ||
    retire_flush_is ||
    retire_flush_fe

  io.out.toRob.flushGateClk := inst0flushGateClk ||
    retire_flush_is ||
    retire_flush_fe

  // Todo: move
  private val flushStateNotIdle = !flushStateCur(0).asBool

  io.out.toIdu.flushStall := flushStateNotIdle
  //mask iu change flow on wrong path during flush state machine
  //when mispred iu will mask wrong path change flow by itself
  io.out.toIu.flushChangeFlowMask := flushStateNotIdle
  io.out.toRob.flushState := flushStateCur

  //----------------------------------------------------------
  //                   Asynchronous Flush
  //----------------------------------------------------------
  //when sync flush, lsu entry do not flush commit inst, but
  //lsu pipeline flush inst without considering its commit state.
  //when async flush, the committed lsu inst may die when flushed
  //at lsu pipeline while not flushed in lsu entry.
  //so add async flush to flush lsu entry committed inst.
  //force flush state machine to flush fe and be state
  //force pst preg all wb
  //force async expt to idle
  //the async expt will not interrupt commit inst execute in lsu
  //so do not need async lsu flush
  asyncFlush := debugReqAckJdbReq

  io.out.toPst.asyncFlush := RegNext(asyncFlush)
  io.out.toLsu.asyncFlush := RegNext(asyncFlush)

  //----------------------------------------------------------
  //                    Flush Expt
  //----------------------------------------------------------
  //rtu should signal lsu whether flush is triggered by
  //1. expt, include expt, int and async expt
  //2. exception return, include rte/rfi
  //available only when flush

  when(inst0flush) {
    flushExcept   := toIfuExceptValid
    flushEret     := rob.instExtra.efPcValid
    flushSpecFail := rob.instExtra.specFail
  }.elsewhen(flushStateCur === FlushState.be) {
    flushExcept   := false.B
    flushEret     := false.B
    flushSpecFail := false.B
  }

  io.out.toLsu.exceptFlush    := flushExcept
  io.out.toLsu.eretFlush      := flushEret
  io.out.toLsu.specFailFlush  := flushSpecFail

  //----------------------------------------------------------
  //                      Spec fail IID
  //----------------------------------------------------------

  when(asyncFlush) {
    specFailIid := 0.U
  }.elsewhen(instNormalRetireVec(0) && rob.instExtra.specFailNoSsf) {
    specFailIid := rob.instExtra.iid
  }.elsewhen(instNormalRetireVec(0) && rob.instExtra.specFailSsf) {
    specFailIid := rob.ssfIid
  }

  io.out.toLsu.specFailIid := specFailIid

  //==========================================================
  //                  Asynchronous Exception
  //==========================================================

  //----------------------------------------------------------
  //              Prepare state machine signals
  //----------------------------------------------------------

  private val asyncExcept = lsu.asyncExceptValid && !debugReqAck && !debugModeOn
  private val asyncExceptNoCommit = !rob.commitValidVec.reduce(_||_)
  private val asyncExceptNoRetire =
    !(rob.instVec.map(_.valid).reduce(_||_) || flushStateNotIdle || !pst.retiredRegWb)

  //----------------------------------------------------------
  //                 Save physical address
  //----------------------------------------------------------
  when(lsu.asyncExceptValid) {
    aePhyAddr := lsu.asyncExceptAddr
  }

  //----------------------------------------------------------
  //                    FSM of Async Expt
  //----------------------------------------------------------

  when(asyncFlush) {
    aeStateCur := AsyncExceptState.idle
  }.otherwise {
    aeStateCur := aeStateNext
  }

  switch(aeStateCur) {
    is(AsyncExceptState.idle) {
      when(asyncExcept) {
        aeStateNext := AsyncExceptState.wfc
      }.otherwise {
        aeStateNext := AsyncExceptState.idle
      }
    }
    is(AsyncExceptState.wfc) {
      when(asyncExceptNoCommit) {
        aeStateNext := AsyncExceptState.wfi
      }.otherwise {
        aeStateNext := AsyncExceptState.wfc
      }
    }
    is(AsyncExceptState.wfi) {
      when(asyncExceptNoRetire) {
        aeStateNext := AsyncExceptState.except
      }.otherwise {
        aeStateNext := AsyncExceptState.wfi
      }
    }
    is(AsyncExceptState.except) {
      aeStateNext := AsyncExceptState.idle
    }
  }

  //----------------------------------------------------------
  //                   Control Siganls
  //----------------------------------------------------------
  private val asyncExceptNoIdle = aeStateCur =/= AsyncExceptState.idle
  //stop new inst commit, do not stop existent commit
  io.out.toRob.asyncExceptionCommitMask := aeStateCur === AsyncExceptState.wfc
  //stop rob retire new inst
  io.out.toRob.rtMask := aeStateCur === AsyncExceptState.wfi
  //async expt valid will flush rob, including commit
  private val aeStateIsExcept = aeStateCur === AsyncExceptState.except
  //access error Todo: figure out
  asyncExceptVec := 5.U

  io.out.toTop.aeState := aeStateCur
  //when lsu request ctc flush, rtu should req sync flush to
  //rob read0 inst like int, ctc flush req should be clear when
  //flush fe generated by ctc flush req or other rtu flush request.
  //cannot use flush be because mispred may flush fe before ctc flush
  //request, ctc flush req may be clear after mispred
  private val ctcFlushLsuReq = lsu.ctcFlushValid
  when(ctcFlushLsuReq) {
    ctcFlushReq := true.B
  }.elsewhen(flushStateCur === FlushState.fe) {
    ctcFlushReq := false.B
  }
  io.out.toRob.ctcFLushReq := ctcFlushReq

  //==========================================================
  //                  Retire Empty Signals
  //==========================================================
  io.out.toRob.retireEmpty := lsu.allCommitDataValid
}
