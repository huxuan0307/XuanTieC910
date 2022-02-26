package Core.RTU

import chisel3._
import chisel3.util._
import Core.IntConfig._
import Core.ROBConfig._

class PstPregEntryData extends Bundle {
  val reg : UInt = UInt(NumLogicRegsBits.W)
  val iid : UInt = UInt(InstructionIdWidth.W)
  val preg : UInt = UInt(NumPhysicRegsBits.W)
}

class PstPregFromIduBundle extends Bundle {
  val instVec                 : Vec[PstPregEntryData] = Vec(NumCreateEntry, new PstPregEntryData)
}

class PstPregFromRobBundle extends Bundle {
  val bits = Vec(NumRetireEntry, new Bundle() {
    val gateClkValid  : Bool = Bool()
    val iidUpdate     : UInt = UInt(InstructionIdWidth.W)
  })
}

class PstPregFromRtuBundle extends Bundle {
  val yyXxFlush : Bool = Bool()
}

class PstPregEntryInterconnectInput extends Bundle {
  val createValidOH : Vec[Bool] = Vec(NumCreateEntry, Bool())
  // Todo: figure out
  val deallocMask   : Bool      = Bool()
  val deallocValid  : Bool      = Bool()
  val releaseValid  : Bool      = Bool()
  val resetDestReg  : UInt      = UInt(NumLogicRegsBits.W)
  val resetMapped   : Bool      = Bool()
  val wbValid       : Bool      = Bool()
}

class PstPregEntryInterconnectOutput extends Bundle {
  val empty             : Bool      = Bool()
  val destRegOH         : Vec[Bool] = Vec(NumLogicRegs, Bool())
  val releasePregOH     : Vec[Bool] = Vec(NumPhysicRegs, Bool())
  val retiredReleasedWb : Bool      = Bool()
}

class PstPregEntryInterconnectBundle extends Bundle {
  val in  : PstPregEntryInterconnectInput  = Input(new PstPregEntryInterconnectInput)
  val out : PstPregEntryInterconnectOutput = Output(new PstPregEntryInterconnectOutput)
}

class PstPregEntryInput extends Bundle {
  val deallocValidGateClk     : Bool              = Bool()
  val fromCp0                 : RegEntryFromCp0Bundle    = new RegEntryFromCp0Bundle
  val fromIdu                 : PstPregFromIduBundle = new PstPregFromIduBundle
  val fromIfu                 : RegEntryFromIfuBundle   = new RegEntryFromIfuBundle
  val fromPad = new RobFromPad
  val fromRetire           : PregFromRetire     = new PregFromRetire(NumRetireEntry)
  val fromRob = new PstPregFromRobBundle
  val fromRtu = new PstPregFromRtuBundle
}

class PstPregEntryOutput extends Bundle

class PstPregEntryIO extends Bundle {
  val in  : PstPregEntryInput  = Input(new PstPregEntryInput)
  val out : PstPregEntryOutput = Output(new PstPregEntryOutput)
  val x   : PstPregEntryInterconnectBundle = new PstPregEntryInterconnectBundle
}

class PstPregEntry extends Module {
  val io : PstPregEntryIO = IO(new PstPregEntryIO)

  //----------------------------------------------------------
  //                  FSM of Preg lifecycle
  //----------------------------------------------------------
  // State Description:
  // DEALLOC    : preg is released and written back, could
  //              allocate to new producer
  // WF_ALLOC   : preg is in allocate preg register
  // ALLOC      : preg is allocated to producer
  // RETIRE     : producer is retired but preg is not released
  // RELEASE    : preg is released

  object PregState {
    def size = 5
    val dealloc :: wf_alloc :: alloc :: retire :: release :: Nil = Enum(size)
  }

  //----------------------------------------------------------
  //                 FSM of Preg write back
  //----------------------------------------------------------
  // State Description:
  // IDLE       : preg is not written back
  // WB         : preg is written back

  object WbState {
    def size = 2
    val idle :: wb :: Nil = Enum(size)
  }

  //==========================================================
  //                           Regs
  //==========================================================

  private val entryCreate = RegInit(0.U.asTypeOf(new PstPregEntryData))
  private val entry   = RegInit(0.U.asTypeOf(new PstPregEntryData))
  private val retireIidMatchVec = RegInit(VecInit(Seq.fill(NumRetireEntry)(false.B)))
  private val lifecycleStateCur   = RegInit(PregState.dealloc)
  private val wbStateCur    = RegInit(WbState.idle)


  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================

  // Todo: gated clk cell for sm, alloc

  //==========================================================
  //               Preg Lifecycle State Machine
  //==========================================================

  // Lifecycle State
  private val lifecycleStateNext  = WireInit(PregState.dealloc)

  //----------------------------------------------------------
  //             Data Prepare for Preg Lifecycle
  //----------------------------------------------------------

  private val createValidOH = io.x.in.createValidOH
  private val lifecycleStateReset : UInt = Mux(io.x.in.resetMapped, PregState.retire, PregState.dealloc)
  private val createValid   : Bool = createValidOH.asUInt.orR
  private val releaseValid  : Bool = io.x.in.releaseValid
  private val retireValid   : Bool = io.in.fromRetire.wbRetireInstPregValid.zip(retireIidMatchVec).map {
    case (wbValid, iidMatch) => wbValid && iidMatch
  }.reduce(_||_)

  //----------------------------------------------------------
  //             Preg Lifecycle State Transfer
  //----------------------------------------------------------

  when(io.in.fromIfu.xxSyncReset) {
    lifecycleStateCur := lifecycleStateReset
  }.otherwise {
    lifecycleStateCur := lifecycleStateNext
  }

  switch(lifecycleStateCur) {
    is(PregState.dealloc) {
      when(io.x.in.deallocValid && !io.in.fromRtu.yyXxFlush) {
        lifecycleStateNext := PregState.wf_alloc
      }.otherwise {
        lifecycleStateNext := PregState.dealloc
      }
    }
    is(PregState.wf_alloc) {
      when(io.in.fromRtu.yyXxFlush) {
        lifecycleStateNext := PregState.dealloc
      }.elsewhen(createValid) {
        lifecycleStateNext := PregState.alloc
      }.otherwise {
        lifecycleStateNext := PregState.wf_alloc
      }
    }
    is(PregState.alloc) {
      when(io.in.fromRtu.yyXxFlush) {
        lifecycleStateNext := PregState.dealloc
      }.elsewhen(releaseValid && wbStateCur === WbState.wb && io.x.in.deallocMask) {
        lifecycleStateNext := PregState.dealloc
      }.elsewhen(releaseValid) {
        lifecycleStateNext := PregState.release
      }.elsewhen(retireValid) {
        lifecycleStateNext := PregState.retire
      }.otherwise{
        lifecycleStateNext := PregState.alloc
      }
    }
    is(PregState.retire) {
      when(releaseValid && wbStateCur === WbState.wb && io.x.in.deallocMask) {
        lifecycleStateNext := PregState.dealloc
      }.elsewhen(releaseValid) {
        lifecycleStateNext := PregState.release
      }.otherwise{
        lifecycleStateNext := PregState.retire
      }
    }
    is(PregState.release) {
      when(wbStateCur === WbState.wb && io.x.in.deallocMask) {
        lifecycleStateNext := PregState.dealloc
      }.otherwise{
        lifecycleStateNext := PregState.release
      }
    }
  }

  io.x.out.empty := lifecycleStateCur === PregState.dealloc

  //==========================================================
  //               Preg Write Back State Machine
  //==========================================================

  private val wbStateNext   = RegInit(WbState.idle)
  private val wbStateReset  = Mux(io.x.in.resetMapped, WbState.wb, WbState.idle)

  //----------------------------------------------------------
  //             Data Prepare for Preg Write Back State
  //----------------------------------------------------------

  private val wbValid = io.x.in.wbValid

//  private val wb_cur_state_wb = Wire(Bool())
//  private val wb_cur_state_wb_masked = Wire(Bool())

  //----------------------------------------------------------
  //             Preg Write Back State Transfer
  //----------------------------------------------------------

  when(io.in.fromRetire.asyncFlush) {
    wbStateCur := WbState.wb
  }.elsewhen(io.in.fromIfu.xxSyncReset) {
    wbStateCur := wbStateReset
  }.elsewhen(createValid) {
    wbStateCur := WbState.idle
  }.otherwise{
    wbStateCur := wbStateNext
  }

  switch(wbStateCur) {
    is(WbState.idle) {
      when(wbValid) {
        wbStateNext := WbState.wb
      }.otherwise {
        wbStateNext := WbState.idle
      }
    }
    is(WbState.wb) {
      when(lifecycleStateCur === PregState.dealloc) {
        wbStateNext := WbState.idle
      }.otherwise{
        wbStateNext := WbState.wb
      }
    }
  }

  //==========================================================
  //                    Preg information
  //==========================================================
  //----------------------------------------------------------
  //              Prepare allocate create data
  //----------------------------------------------------------

  entryCreate := MuxLookup(io.x.in.createValidOH.asUInt, 0.U.asTypeOf(new PstPregEntryData), Seq(
    "b0001".U -> io.in.fromIdu.instVec(0),
    "b0010".U -> io.in.fromIdu.instVec(1),
    "b0100".U -> io.in.fromIdu.instVec(2),
    "b1000".U -> io.in.fromIdu.instVec(3),
  ))

  //----------------------------------------------------------
  //                  Information Register
  //----------------------------------------------------------
  when(io.in.fromIfu.xxSyncReset) {
    entry := 0.U.asTypeOf(new PstPregEntryData)
    entry.reg := io.x.in.resetDestReg
  }.elsewhen(createValid) {
    entry := entryCreate
  }.otherwise {
    entry := entry
  }
  //----------------------------------------------------------
  //                Retire IID Match Register
  //----------------------------------------------------------
  // Todo: verilog: timing optimization: compare retire inst iid before retire

  when(io.in.fromIfu.xxSyncReset) {
    retireIidMatchVec := 0.U(retireIidMatchVec.getWidth.W).asBools
  }.elsewhen(lifecycleStateCur === PregState.alloc){
    // Only in alloc state, entry need to retire
    for (i <- 0 until NumRetireEntry) {
      // Todo: figure out why need gate clk valid
      when(io.in.fromRob.bits(i).gateClkValid) {
        retireIidMatchVec(i) := io.in.fromRob.bits(i).iidUpdate === entry.iid
      }
    }
  }

  //==========================================================
  //                       Retire signal
  //==========================================================
  private val retireGateClkValid : Bool = io.in.fromRetire.wbRetireInstPregValid.asUInt.orR

  //==========================================================
  //                      Release signal
  //==========================================================
  io.x.out.releasePregOH := VecInit(Mux(
    lifecycleStateCur === PregState.alloc && retireValid,
    UIntToOH(entry.preg, NumPhysicRegs),
    0.U
  ).asBools)

  //==========================================================
  //              Rename Table Recovery signal
  //==========================================================
  io.x.out.destRegOH := VecInit(Mux(
    lifecycleStateCur === PregState.retire,
    UIntToOH(entry.reg, NumLogicRegs),
    0.U
  ).asBools)

  //==========================================================
  //          Fast Retired Instruction Write Back
  //==========================================================
  //this signal will be used at retiring cycle by FLUSH state machine
  //should consider the retiring insts
  io.x.out.retiredReleasedWb := Mux(
    lifecycleStateCur === PregState.alloc && retireValid ||
      lifecycleStateCur=== PregState.retire ||
      lifecycleStateCur=== PregState.release,
    wbStateCur === WbState.wb,
    true.B
  )

}
