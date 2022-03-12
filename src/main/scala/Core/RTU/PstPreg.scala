package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.IntConfig._
import Core.PipelineConfig._

import scala.language.postfixOps

class PstPregOHBundle extends Bundle {
  val pregOH = Vec(NumPhysicRegs, Bool())
}

class PstPregInput extends Bundle {
  val fromCp0 = new RegEntryFromCp0Bundle
  val idu2rtuIrPregAllocVld = Vec(NumCreateEntry, Bool())
  val idu2rtuIrPregAllocGateClkVld = Bool()
  val fromIdu = new Bundle() {
    val alloc : Vec[ValidIO[PstPregEntryData]] = Vec(NumCreateEntry, ValidIO(new PstPregEntryData))
    val pregDeallocMaskOH : Vec[Bool] = Vec(NumPhysicRegs, Bool())
  }
  val fromIfu = new Bundle() {
    val xxSyncReset = Bool()
  }
  val fromIuEx2 = Vec(NumIuPipe, ValidIO(new PstPregOHBundle))
  val fromLsu = Vec(NumLsuPipe, ValidIO(new PstPregOHBundle))
  val fromPad = new RtuFromPad
  val retiredEregWb = Bool()
  val retiredFregWb = Bool()
  val retiredVregWb = Bool()
  val retirePstAsyncFlush = Bool()
  // Todo: imm
  val retirePstWbRetireInstPregVld = Vec(3, Bool())
  // Todo: imm
  val fromRob = Vec(3, new Bundle() {
    val gateClkValid = Bool()
    val iidUpdate = UInt(InstructionIdWidth.W)
  })
  val fromRtu = new Bundle() {
    val yyXxFlush = Bool()
  }
}

class PstPregOutput extends Bundle {
  val retireRetiredRegWb = Bool()
  val topRetiredPregWb  = Bool()
  val topRetiredVFregWb = Bool()
  val topRetiredEregWb  = Bool()
  val hadInstNotWb = Bool()
  val toIdu = new Bundle{
    // Todo: imm
    val alloc = Vec(4, ValidIO(UInt(NumPhysicRegsBits.W)))
    val empty = Bool()
    val rtRecoverPreg = Vec(NumLogicRegs, UInt(NumPhysicRegsBits.W))
  }
}

class PstPregIO extends Bundle {
  val in : PstPregInput = Input(new PstPregInput)
  val out : PstPregOutput = Output(new PstPregOutput)
}

class PstPreg extends Module {
  val io : PstPregIO = IO(new PstPregIO)

  // Regs
  private val allocPreg : Vec[ValidIO[UInt]] = RegInit(VecInit(
    Seq.fill(NumCreateEntry)(0.U.asTypeOf(ValidIO(UInt(NumPhysicRegsBits.W))))
  ))
  private val dealloc3Vec : Vec[Bool] = RegInit(VecInit(Seq.fill(NumPhysicRegs)(false.B)))
  private val deallocPreg3 = RegInit(0.U(NumPhysicRegsBits.W))

  // entry input
  val deallocValidForGateClk = Wire(Bool())
  val pregCreateValid   = Wire(Vec(NumPhysicRegs, UInt(NumCreateEntry.W)))
  val deallocValid      = Wire(Vec(NumPhysicRegs, Bool()))
  val releaseValid      = Wire(Vec(NumPhysicRegs, Bool()))
  val wbValid           = Wire(Vec(NumPhysicRegs, Bool()))
  // entry output
  val pregEmpty         = Wire(Vec(NumPhysicRegs, Bool()))
  val dregOHVec         = Wire(Vec(NumPhysicRegs, UInt(NumLogicRegs.W)))
  val relPregOHVec      = Wire(Vec(NumPhysicRegs, UInt(NumPhysicRegs.W)))
  val retiredReleaseWb  = Wire(Vec(NumPhysicRegs, Bool()))

  val pregResetMapped = Wire(Vec(NumPhysicRegs, Bool()))


  //==========================================================
  //                   Instance Entries
  //==========================================================
  val entries = Seq.fill(NumPhysicRegs)(Module(new PstPregEntry))
  for (i <- 1 until NumPhysicRegs) {
    entries(i).io.in.fromCp0                    := io.in.fromCp0
    entries(i).io.in.deallocValidGateClk        := deallocValidForGateClk
    entries(i).io.in.fromIdu.instVec            := io.in.fromIdu.alloc.map(_.bits)
    entries(i).io.in.fromIfu                    := io.in.fromIfu
    entries(i).io.in.fromPad                    := io.in.fromPad
    entries(i).io.in.fromRetire.asyncFlush      := io.in.retirePstAsyncFlush
    entries(i).io.in.fromRetire.wbRetireInstPregValid := io.in.retirePstWbRetireInstPregVld
    entries(i).io.in.fromRob                    := io.in.fromRob
    entries(i).io.in.fromRtu                    := io.in.fromRtu.yyXxFlush

    entries(i).io.x.in.createValidOH      := pregCreateValid(i)
    entries(i).io.x.in.deallocMask        := io.in.fromIdu.pregDeallocMaskOH(i)
    entries(i).io.x.in.deallocValid       := deallocValid(i)
    entries(i).io.x.in.releaseValid       := releaseValid(i)
    entries(i).io.x.in.resetDestReg       := Mux(i.U < NumLogicRegs.U, i.U, 0.U)
    entries(i).io.x.in.resetMapped        := pregResetMapped(i)
    entries(i).io.x.in.wbValid            := wbValid(i)

    pregEmpty(i)        := entries(i).io.x.out.empty
    dregOHVec(i)        := entries(i).io.x.out.destRegOH
    relPregOHVec(i)     := entries(i).io.x.out.releasePregOH
    retiredReleaseWb(i) := entries(i).io.x.out.retiredReleasedWb
  }

  //==========================================================
  //          PST GPR Physical Register (Preg) Logic
  //==========================================================

  //==========================================================
  //                  Reset initial states
  //==========================================================
  // After reset, p0-p31 will be mapped to r0-r31, p31-p63 will be dealloc
  pregResetMapped.zipWithIndex.foreach{ case(b, idx) => b := (idx < NumLogicRegs).B }

  //==========================================================
  //                 Dispatch Create signals
  //==========================================================


  //----------------------------------------------------------
  //                Instance of alloc registers
  //----------------------------------------------------------
  private val allocPregInvalid = Wire(Vec(NumCreateEntry, Bool()))
  private val allocPreg3DeallocValid = Wire(Bool())
  for (i <- 0 until NumCreateEntry) {
    //alloc preg 0/1/2/3 will be allocated to ir inst 0/1/2/3
    allocPregInvalid(i) :=
      (!allocPreg(i).valid || io.in.fromIdu.alloc(i).valid) && !io.in.fromIfu.xxSyncReset
  }
  // Todo: figure out
  //preg3 reuse preg0~preg2 if any of them does not need dealloc
  allocPreg3DeallocValid := allocPreg(3).valid && !allocPregInvalid(0) && allocPregInvalid(1)

  when(io.in.fromRtu.yyXxFlush) {
    allocPreg.foreach(preg => {
      preg.valid := false.B
      preg.bits  := 0.U
    })
  }

  // Wires
  /**
   * register associated table
   * rat[regIdx] = pregIdx
   */
  private val rat = WireInit(VecInit(Seq.fill(NumLogicRegs)(0.U(NumPhysicRegsBits.W))))
  for (i <- 1 until NumLogicRegs) {
    rat(i) := OHToUInt(
      dregOHVec.map(item => item(i))
    )
  }

  //==========================================================
  //          Fast Retired Instruction Write Back
  //==========================================================
  private val retiredPregWb = retiredReleaseWb.asUInt.andR
  private val retiredRegWb  =
    retiredPregWb &&
    io.in.retiredVregWb &&
    io.in.retiredFregWb &&
    io.in.retiredEregWb

  // output
  io.out.retireRetiredRegWb := retiredRegWb
  io.out.topRetiredPregWb   := retiredPregWb
  io.out.topRetiredVFregWb  := io.in.retiredFregWb || io.in.retiredVregWb
  io.out.topRetiredEregWb   := io.in.retiredEregWb

  io.out.hadInstNotWb       := !retiredRegWb
  io.out.toIdu.empty        := retiredRegWb
  io.out.toIdu.alloc        := allocPreg
  io.out.toIdu.rtRecoverPreg:= rat
}
