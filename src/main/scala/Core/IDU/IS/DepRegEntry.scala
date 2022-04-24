package Core.IDU.IS

import chisel3._
import chisel3.util._
import Core.ROBConfig._
import Core.VectorUnitConfig._
import Core.PipelineConfig._
import Core.IntConfig._

trait DepRegEntryConfig {
  def alu0Idx = 0

  def alu1Idx = 1

  def mulIdx = 2

  def divIdx = 3

  def loadIdx = 4

  def vfpu0Idx = 5

  def vfpu1Idx = 6

  def WbNum = 3
}

class DepRegEntryData extends Bundle {
  val preg : UInt = UInt(NumPhysicRegsBits.W)
  val lsuMatch : Bool = Bool()
  val ready : Bool = Bool()
  val wb : Bool = Bool()
}

class DepRegEntryCreateData extends DepRegEntryData

class DepRegEntryReadData extends DepRegEntryData {
  val readyForBypass : Bool = Bool()
  val readyForIssue : Bool = Bool()
}

class FwdValidBundle extends Bundle {
  val alu : Vec[Bool] = Vec(NumAlu, Bool())
  val load : Vec[Bool] = Vec(NumLu, Bool())
}

class IqEntryFromCp0 extends Bundle {
  val icgEn : Bool = Bool()
  val yyClkEn : Bool = Bool()
}

class DepRegEntryInput extends Bundle with DepRegEntryConfig {
  val fwdValid = new FwdValidBundle
  val fromCp0 = new IqEntryFromCp0
  /**
   * Include alu0, alu1, mul, div, load, vfpu0, vfpu1
   */
  val fuDstPreg : Vec[ValidIO[UInt]] = Vec(NumFuHasDstReg, ValidIO(UInt(NumPhysicRegsBits.W)))

  /**
   * Include pipe0,1,3 wb
   */
  val wbPreg : Vec[ValidIO[UInt]] = Vec(WbNum, ValidIO(UInt(NumPhysicRegsBits.W)))

  val loadPreg = ValidIO(UInt(NumPhysicRegsBits.W))

  val flush = new Bundle {
    val fe : Bool = Bool()
    val is : Bool = Bool()
  }

  val createData = new DepRegEntryCreateData
  val gateClkIdxWen : Bool = Bool()
  val gateClkWen : Bool = Bool()
  val readyClear : Bool = Bool()
  val wen : Bool = Bool()
}

class DepRegEntryOutput extends Bundle {
  val readData = new DepRegEntryReadData
}

class DepRegEntryIO extends Bundle {
  val in : DepRegEntryInput = Flipped(Output(new DepRegEntryInput))
  val out : DepRegEntryOutput = Output(new DepRegEntryOutput)
}

class DepRegEntry extends Module with DepRegEntryConfig {
  val io : DepRegEntryIO = IO(new DepRegEntryIO)

  private val dataCreate = io.in.createData
  private val readData = Wire(new DepRegEntryReadData)
  private val flush = io.in.flush
  /**
   * Regs
   */

  private val dataInit = Wire(new DepRegEntryData)
  dataInit.ready := 1.U
  dataInit.wb := 1.U
  dataInit.lsuMatch := 0.U
  dataInit.preg := 0.U

  private val data = RegInit(dataInit)
  private val dataUpdate = Wire(new DepRegEntryData)
  dataUpdate.preg := DontCare
  // Todo: gated clk


  //==========================================================
  //                       Ready Bit
  //==========================================================
  //ready bit shows the result of source is predicted to be ready:
  //1 stands for the result may be forwarded

  private val dataReady = io.in.fuDstPreg.map {
    case preg => preg.valid && (preg.bits === data.preg)
  }.reduce(_ || _)

  def issueNum = 3 // alu0, alu1, lsu

  //bypass data ready for issue
  private val issueDataReadyVec = Wire(Vec(issueNum, Bool()))
  issueDataReadyVec(0) := io.in.fwdValid.alu(0)
  issueDataReadyVec(1) := io.in.fwdValid.alu(1)
  issueDataReadyVec(2) := io.in.fwdValid.load(0) && data.lsuMatch

  private val wakeUp = data.wb
  private val readyClear = io.in.readyClear

  //1.if ready is already be 1, just hold 1
  //2.if producer are presumed to produce the result two cycles later,
  //  set ready to 1
  //3.if producer wake up, set ready to 1 // wb = 1
  //4.clear ready to 0

  dataUpdate.ready := (data.ready || dataReady || wakeUp) && !readyClear

  //==========================================================
  //              LSU reg Match for Bypass Ready
  //==========================================================
  private val loadPreg = io.in.loadPreg
  dataUpdate.lsuMatch := loadPreg.valid && loadPreg.bits === data.preg

  //==========================================================
  //                     Write Back Valid
  //==========================================================
  //write back valid shows whether the result is written back
  //into PRF : 1 stands for the result is in PRF

  //-------------Update value of Write Back Bit---------------
  //prepare write back signal
  private val pipeWb = io.in.wbPreg.map {
    case preg => preg.valid && preg.bits === data.preg
  }

  //1.if wb_vld is already be 1, just hold 1
  //2.if this result is writing back to PRF, set wb to 1
  dataUpdate.wb := data.wb || pipeWb.reduce(_ || _)


  readData.ready := dataUpdate.ready
  //the following signals are for Issue Queue bypass/issue logic
  //                         cycle0        cycle1
  readData.readyForIssue := data.ready || issueDataReadyVec.reduce(_ || _)
  readData.readyForBypass := data.ready

  readData.lsuMatch := dataUpdate.lsuMatch
  readData.wb := dataUpdate.wb
  readData.preg := data.preg

  when(flush.fe || flush.is) {
    data.ready := dataInit.ready
    data.lsuMatch := dataInit.lsuMatch
    data.wb := dataInit.wb
  }.elsewhen(io.in.wen) {
    data := dataCreate
  }.otherwise {
    data.ready := dataInit.ready
    data.lsuMatch := dataInit.lsuMatch
    data.wb := dataInit.wb
  }

  io.out.readData := readData
}