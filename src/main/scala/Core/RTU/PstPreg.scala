package Core.RTU

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.IntConfig._
import Core.PipelineConfig._
import Utils.ParallelXOR
import chisel3.util.experimental.BoringUtils

import scala.language.postfixOps

class PstPregInput extends Bundle {
  class PstFromIdu extends Bundle {
    val allocGateClkValid : Bool = Bool()
    val alloc : Vec[ValidIO[PstPregEntryCreateBundle]] = Vec(NumCreateEntry, ValidIO(new PstPregEntryCreateBundle))
    val deallocMaskOH : Vec[Bool] = Vec(NumPhysicRegs, Bool())
  }
  class PstFromIfu extends Bundle {
    val xxSyncReset : Bool = Bool()
  }
  class PstFromRetire extends Bundle {
    val asyncFlush : Bool = Bool()
    // Todo: check imm
    val wbRetireInstPregValid : Vec[Bool] = Vec(NumRetireEntry, Bool())
  }
  class PstFromRob extends Bundle {
    val gateClkValid = Vec(NumRetireEntry, Bool())
    val iidUpdate = Vec(NumRetireEntry, UInt(InstructionIdWidth.W))
  }
  class PstFromRtu extends Bundle {
    val yyXxFlush : Bool = Bool()
  }
  class PstFromIu extends Bundle {
    val wbData : Vec[ValidIO[Vec[Bool]]] = Vec(NumIuPipe, ValidIO(Vec(NumPhysicRegs, Bool())))
  }
  class PstFromLsu extends Bundle {
    val wbData : Vec[ValidIO[Vec[Bool]]] = Vec(NumLu, ValidIO(Vec(NumPhysicRegs, Bool())))
  }
  val fromCp0 = new RegEntryFromCp0Bundle
  val fromIdu = new PstFromIdu
  val fromIfu = new PstFromIfu
  val fromIuEx2 = new PstFromIu
  val fromLsu = new PstFromLsu
  val fromPad = new RtuFromPadBundle
  val retiredEregWb : Bool = Bool()
  val retiredFregWb : Bool = Bool()
  val retiredVregWb : Bool = Bool()
  val fromRetire = new PstFromRetire
  val fromRob = new PstFromRob
  val fromRtu = new PstFromRtu
}

class PstPregOutput extends Bundle {
  class PstToRetire extends Bundle {
    val retiredRegWb : Bool = Bool()
  }
  class PstToTop extends Bundle {
    // Todo: imm
    /**
     * 0: preg
     * 1: fvreg
     * 2: ereg
     */
    val retiredRegWb : Vec[Bool] = Vec(3, Bool())
  }
  class PstToHad extends Bundle {
    val instNotWb : Bool = Bool()
  }
  class PstToIdu extends Bundle {
    val alloc : Vec[ValidIO[UInt]] = Vec(NumCreateEntry, ValidIO(UInt(NumPhysicRegsBits.W)))
    val empty : Bool = Bool()
    val rtRecoverPreg : Vec[UInt] = Vec(NumLogicRegs, UInt(NumPhysicRegsBits.W))
  }
  val toRetire = new PstToRetire
  val toTop = new PstToTop
  val toHad = new PstToHad
  val toIdu = new PstToIdu
}

class PstPregIO extends Bundle {
  val in : PstPregInput = Input(new PstPregInput)
  val out : PstPregOutput = Output(new PstPregOutput)
}

class PstPreg extends Module {
  val io : PstPregIO = IO(new PstPregIO)

  private val cp0 = io.in.fromCp0
  private val idu = io.in.fromIdu
  private val ifu = io.in.fromIfu
  private val iu = io.in.fromIuEx2
  private val lsu = io.in.fromLsu
  private val pad = io.in.fromPad
  private val retire = io.in.fromRetire
  private val rob = io.in.fromRob
  private val rtu = io.in.fromRtu

  /**
   * Regs
   */
  private val allocPregValidVec = RegInit(VecInit(Seq.fill(NumCreateEntry)(false.B)))
  private val allocPregs = RegInit(VecInit(Seq.fill(NumCreateEntry)(0.U(NumPhysicRegsBits.W))))

  /**
   * Wires
   */
  private val dealloc3Vec : Vec[Bool] = Wire(Vec(NumPhysicRegs, Bool()))
  private val deallocPregVec = Wire(Vec(NumCreateEntry, UInt(NumPhysicRegsBits.W)))
  private val allocPregInvalidVec = Wire(Vec(NumCreateEntry, Bool()))
  //==========================================================
  //                   Instance Entries
  //==========================================================

  // entry input
  private val deallocValidForGateClk = Wire(Bool())
  private val pregCreateValid   = Wire(Vec(NumPhysicRegs, UInt(NumCreateEntry.W)))
  private val releaseValidOH      = Wire(Vec(NumPhysicRegs, Bool()))
  private val deallocValidOH = Wire(Vec(NumPhysicRegs, Bool()))
  private val resetDestReg      = Wire(Vec(NumPhysicRegs, UInt(NumLogicRegs.W)))
  resetDestReg.zipWithIndex.foreach {
    case (reg, i) =>
      if (i < NumLogicRegs)
        reg := i.U
      else
        reg := 0.U
  }
  private val resetMapped = Wire(Vec(NumPhysicRegs, Bool()))

  private val wbValid           = Wire(Vec(NumPhysicRegs, Bool()))

//  // entry output
  private val pregOutEmpty        = Wire(Vec(NumPhysicRegs, Bool()))
  private val pregOutDestRegOHVec    = Wire(Vec(NumPhysicRegs, Vec(NumLogicRegs, Bool())))
  private val pregOutReleasePregOHVec   = Wire(Vec(NumPhysicRegs, Vec(NumPhysicRegs, Bool())))
  private val pregOutRetiredReleaseWb = Wire(Vec(NumPhysicRegs, Bool()))

  //==========================================================
  //                   Instance Entries
  //==========================================================
  private val entries = Seq.fill(NumPhysicRegs)(Module(new PstPregEntry))
  for (i <- 0 until NumPhysicRegs) {
    val in = entries(i).io.in
    val x = entries(i).io.x
    if (i != 0) {
      in.deallocValidGateClk := deallocValidForGateClk
      in.fromCp0 := io.in.fromCp0
      in.fromIdu.instVec.zipWithIndex.foreach{
        case (data, i) =>
          data.preg := io.in.fromIdu.alloc(i).bits.preg
          data.iid  := io.in.fromIdu.alloc(i).bits.iid
          data.reg  := io.in.fromIdu.alloc(i).bits.dstReg
      }
      in.fromIfu.xxSyncReset := io.in.fromIfu.xxSyncReset
      in.fromPad := io.in.fromPad
      in.fromRetire.wbRetireInstPregValid := io.in.fromRetire.wbRetireInstPregValid
      in.fromRetire.asyncFlush := io.in.fromRetire.asyncFlush
      in.fromRob.bits.zipWithIndex.foreach {
        case (entry, i) =>
          entry.iidUpdate := io.in.fromRob.iidUpdate(i)
          entry.gateClkValid := io.in.fromRob.gateClkValid(i)
      }
      in.fromRtu.yyXxFlush := io.in.fromRtu.yyXxFlush

      x.in.createValidOH := VecInit(pregCreateValid(i).asBools)
      x.in.deallocMask := io.in.fromIdu.deallocMaskOH(i)
      x.in.releaseValid := releaseValidOH(i)
      x.in.deallocValid := deallocValidOH(i)
      x.in.resetDestReg := resetDestReg(i)
      x.in.resetMapped := resetMapped(i)
      x.in.wbValid := wbValid(i)

      pregOutEmpty(i) := x.out.empty
      pregOutDestRegOHVec(i) := x.out.destRegOH
      pregOutReleasePregOHVec(i) := x.out.releasePregOH
      pregOutRetiredReleaseWb(i) := x.out.retiredReleasedWb
    } else {
      in := DontCare
      x.in := DontCare
      pregOutEmpty(i) := DontCare
      pregOutDestRegOHVec(i) := DontCare
      pregOutReleasePregOHVec(i) := DontCare
      pregOutRetiredReleaseWb(i) := DontCare
    }
  }

  //==========================================================
  //          PST GPR Physical Register (Preg) Logic
  //==========================================================

  //==========================================================
  //                  Reset initial states
  //==========================================================
  // After reset, p0-p31 will be mapped to r0-r31, p31-p95 will be dealloc
  resetMapped.zipWithIndex.foreach{ case(b, idx) => b := (idx < NumLogicRegs).B }

  //==========================================================
  //                 Dispatch Create signals
  //==========================================================
  private val disPregOHVec = Wire(Vec(NumCreateEntry, Vec(NumPhysicRegs, Bool())))

  disPregOHVec.zipWithIndex.foreach {
    case (value, i) =>
      value := VecInit(UIntToOH(idu.alloc(i).bits.preg, NumPhysicRegs).asBools)
  }

  pregCreateValid.zipWithIndex.foreach {
    case (createValid, i) =>
      createValid := VecInit(disPregOHVec.map(_(i))).asUInt
  }

  //==========================================================
  //                    Write back signals
  //==========================================================
  wbValid.zipWithIndex.foreach {
    case (wb_valid, i) =>
      wb_valid := iu.wbData.map(data=>data.valid && data.bits(i)).reduce(_||_) ||
        lsu.wbData.map(data=>data.valid && data.bits(i)).reduce(_||_)
  }

  //==========================================================
  //                     Release signals
  //==========================================================
  releaseValidOH.zipWithIndex.foreach {
    case (relValid, i) => relValid := pregOutReleasePregOHVec.map(_(i)).reduce(_|_)
  }

  //==========================================================
  //                     Dealloc signals
  //==========================================================
  //----------------------------------------------------------
  //                calculate dealloc vector
  //----------------------------------------------------------
  //get all entry dealloc bits
  private val deallocOH = Wire(Vec(NumPhysicRegs, Bool()))
  deallocOH := pregOutEmpty

  //one-hot dealloc preg 0,
  //search priority is from p0 to p95
  private val dealloc0OH = PriorityEncoderOH(deallocOH)
  //one-hot dealloc preg 1,
  //search priority is from p95 to p0
  private val dealloc1OH = PriorityEncoderOH(deallocOH.reverse).reverse
  //one-hot dealloc preg 2,
  //remove dealloc preg 0, and then search priority is from p0 to p95
  private val deallocNo0OH = deallocOH.zip(dealloc0OH).map {
    case (empty, alloc0) => empty & !alloc0
  }
  private val dealloc2OH = PriorityEncoderOH(deallocNo0OH)

  //one-hot dealloc preg 3,
  //remove dealloc preg 1, and then search priority is from p95 to p0
  // Todo: figure out why remove this in OpenC910
//  val deallocNo1 = deallocOH.zip(dealloc1OH).map {
//    case (empty, alloc1) => empty & !alloc1
//  }
//  val alloc3OH = PriorityEncoderOH(allocNo1.reverse)

  //----------------------------------------------------------
  //                deallocate preg and valid
  //----------------------------------------------------------
  //deallocate preg valid
  private val deallocPregValidVec = Wire(Vec(NumCreateEntry, Bool()))
  deallocPregValidVec(0) := deallocOH.reduce(_|_)
  deallocPregValidVec(1) := deallocNo0OH.reduce(_|_)
  deallocPregValidVec(2) := (0 until NumPhysicRegs).map(i =>
    deallocOH(i) & !dealloc0OH(i) & !dealloc1OH(i)).reduce(_|_)
//  deallocPregValidVec(3) := (0 until NumPhysicRegs).map(i =>
//    deallocOH(i) & !dealloc0OH(i) & !dealloc1OH(i) & !dealloc2OH(i)).reduce(_|_)

  deallocPregVec(0) := OHToUInt(dealloc0OH)
  deallocPregVec(1) := OHToUInt(dealloc1OH)
  deallocPregVec(2) := OHToUInt(dealloc2OH)

  //preg3 reuse preg0~preg2 if any of them does not need dealloc
  deallocPregValidVec(3) := deallocPregValidVec(0) && !allocPregInvalidVec(0) ||
    deallocPregValidVec(1) && !allocPregInvalidVec(1)

  //----------------------------------------------------------
  //                   deallocate signals
  //----------------------------------------------------------
  //deallocate vector without redundancy:
  //if dealloc preg is same as others, the vector is all 0
  private val dealloc0Vec = dealloc0OH
  private val dealloc1Vec = (0 until NumPhysicRegs).map(i =>
    dealloc1OH(i) & !dealloc0OH(i))
  private val dealloc2Vec = (0 until NumPhysicRegs).map(i =>
    dealloc2OH(i) & !dealloc1OH(i))
//  val dealloc3Vec = (0 until NumPhysicRegs).map(i =>
//    dealloc3OH(i) & !dealloc2OH(i) & !dealloc0OH(i))

  //alloc preg 0/1/2/3 will be allocated to ir inst 0/1/2/3
  allocPregInvalidVec.zipWithIndex.foreach {
    case (allocPregInvalid, i) =>
      allocPregInvalid := (!allocPregValidVec(i) || idu.alloc(i).valid) && !ifu.xxSyncReset
    //                     invalid past           || invalid now
  }

  private val allocPreg3DeallocValid = Wire(Bool())
  //preg3 reuse preg0~preg2 if any of them does not need dealloc
  allocPreg3DeallocValid := allocPregInvalidVec(3) &&
    !(allocPregInvalidVec(0) && allocPregInvalidVec(1))

  when(!allocPregInvalidVec(0)) {
    dealloc3Vec := dealloc0Vec
    deallocPregVec(3) := deallocPregVec(0)
  }.otherwise {
    dealloc3Vec := dealloc1Vec
    deallocPregVec(3) := deallocPregVec(1)
  }

  deallocValidOH.zipWithIndex.foreach {
    case (deallocValid, i) =>
      deallocValid := allocPregInvalidVec(0) & dealloc0Vec(i) |
      allocPregInvalidVec(1) & dealloc1Vec(i) |
      allocPregInvalidVec(2) & dealloc2Vec(i) |
      allocPreg3DeallocValid & dealloc3Vec(i)
  }

  deallocValidForGateClk := allocPregValidVec.map(!_).reduce(_||_) || idu.allocGateClkValid

  //==========================================================
  //                  Allocate Preg Registers
  //==========================================================

  //----------------------------------------------------------
  //                  Instance of Gated Cell
  //----------------------------------------------------------
  private val allocPregClkEn = rtu.yyXxFlush || allocPregValidVec.map(!_).reduce(_||_) ||
    idu.alloc.map(_.valid).reduce(_||_)

  // Todo: gated clk

  //----------------------------------------------------------
  //                Instance of alloc registers
  //----------------------------------------------------------

  for (i <- 0 until NumCreateEntry) {
    when (rtu.yyXxFlush) {
      allocPregValidVec(i) := false.B
      allocPregs(i) := 0.U
    }.elsewhen(allocPregInvalidVec(i)) {
      allocPregValidVec(i) := deallocPregValidVec(i)
      allocPregs(i) := deallocPregVec(i)
    }
  }

  //alloc preg 0/1/2/3 will be allocated to ir inst 0/1/2/3
  io.out.toIdu.alloc.zipWithIndex.foreach {
    case (alloc, i) =>
      alloc.valid := allocPregValidVec(i)
      alloc.bits := allocPregs(i)
  }

  //==========================================================
  //          Fast Retired Instruction Write Back
  //==========================================================

  private val pregWb = pregOutRetiredReleaseWb.asUInt.andR

  private val regWb =
    pregWb &&
    io.in.retiredVregWb &&
    io.in.retiredFregWb &&
    io.in.retiredEregWb

  // output
  io.out.toTop.retiredRegWb(0) := pregWb
  io.out.toTop.retiredRegWb(1) := io.in.retiredVregWb || io.in.retiredFregWb
  io.out.toTop.retiredRegWb(2) := io.in.retiredEregWb

  io.out.toRetire.retiredRegWb := regWb
  // all preg has write back
  io.out.toIdu.empty := regWb
  io.out.toHad.instNotWb := !regWb

  //==========================================================
  //                  Recovery Rename Table
  //==========================================================
  //the preg_x_dreg[31:0] indicates preg-reg mapping of retired entry.
  //rename table is reg indexed, while pst is preg indexed.
  //transpose the mappings from preg index to reg index.
  private val regPregOHVec = Wire(Vec(NumLogicRegs, Vec(NumPhysicRegs, Bool())))
  for (i <- 0 until NumLogicRegs) {
    regPregOHVec(i) := VecInit(pregOutDestRegOHVec.map(_(i)))
  }
  private val regPregVec = Wire(Vec(NumLogicRegs, UInt(NumPhysicRegsBits.W)))
  for (i <- 0 until NumLogicRegs) {
    regPregVec(i) := OHToUInt(regPregOHVec(i))
  }

  io.out.toIdu.rtRecoverPreg := regPregVec


  //==========================================================
  //          to Difftest
  //==========================================================
  val temdestReg0 = Seq.fill(NumPhysicRegs)(WireInit(0.U(NumLogicRegsBits.W)))
  val tempreg0 = Seq.fill(NumPhysicRegs)(WireInit(0.U(NumLogicRegsBits.W)))
  val temdestReg1 = Seq.fill(NumPhysicRegs)(WireInit(0.U(NumLogicRegsBits.W)))
  val tempreg1 = Seq.fill(NumPhysicRegs)(WireInit(0.U(NumLogicRegsBits.W)))
  val temdestReg2 = Seq.fill(NumPhysicRegs)(WireInit(0.U(NumLogicRegsBits.W)))
  val tempreg2 = Seq.fill(NumPhysicRegs)(WireInit(0.U(NumLogicRegsBits.W)))
  entries.zipWithIndex.foreach{
    case (entries, i) =>
      val destReg = OHToUInt(entries.io.x.out.destRegOH)
      val preg = OHToUInt(entries.io.x.out.releasePregOH)
      val retireValidVec = entries.io.x.out.retireValidVec
      temdestReg0(i) := retireValidVec(0) & destReg
      tempreg0(i)  := retireValidVec(0) & preg
      temdestReg1(i) := retireValidVec(1) & destReg
      tempreg1(i)  := retireValidVec(1) & preg
      temdestReg2(i) := retireValidVec(2) & destReg
      tempreg2(i)  := retireValidVec(2) & preg
  }
  val diffcommitdestReg = WireInit(VecInit(Seq.fill(NumRetireEntry)(0.U(NumLogicRegsBits.W))))
  val diffcommitpreg = WireInit(VecInit(Seq.fill(NumRetireEntry)(0.U(NumLogicRegsBits.W))))
  diffcommitdestReg(0) := ParallelXOR(temdestReg0)
  diffcommitpreg(0) := ParallelXOR(tempreg0)
  diffcommitdestReg(1) := ParallelXOR(temdestReg1)
  diffcommitpreg(1) := ParallelXOR(tempreg1)
  diffcommitdestReg(2) := ParallelXOR(temdestReg2)
  diffcommitpreg(2) := ParallelXOR(tempreg2)

  BoringUtils.addSource(diffcommitdestReg,"diffcommitdestReg")
  BoringUtils.addSource(diffcommitpreg,"diffcommitpreg")
}
