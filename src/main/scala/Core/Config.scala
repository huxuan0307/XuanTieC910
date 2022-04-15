package Core

import chisel3.util._
import chisel3._

trait IntConfig {
  def NumAlu = 2
  def NumLu = 1 // Load
  def NumSu = 1 // Store
  def NumMu = 1 // Mul
  def NumDu = 1 // Div
  def NumVfUnit = 2 // Vector and Float
  def NumFu : Int = NumAlu + NumLu + NumSu +
    NumMu + NumDu + NumVfUnit
  def NumFuHasDstReg : Int = NumAlu + NumLu + NumMu + NumDu + NumVfUnit
  def NumLogicRegs = 32
  def NumFPregs = 64
  def NumVPregs = 64
  def NumEregs = 32
  def NumPhysicRegs = 96
  def OpcodeBits = 32
  def NumInstructionId = 128
  def NumLogicRegsBits : Int = log2Up(NumLogicRegs)
  def NumFPregsBits : Int = log2Up(NumFPregs)
  def NumVPregsBits : Int = log2Up(NumVPregs)
  def NumEregsBits : Int = log2Up(NumEregs)
  def NumPhysicRegsBits : Int = log2Up(NumPhysicRegs)
  def InstructionIdWidth : Int = log2Up(NumInstructionId)
  def XLEN = 64
}

trait ROBConfig {
  def NumRobEntry     = 64
  def NumRobEntryBits : Int = log2Up(NumRobEntry)
  def RobPtrWidth     : Int = NumRobEntry
  def RobPtrNum       = 4
  def RobReadPtrNum   = 6
  def NumRobReadEntry = 3
  def NumCreateEntry  = 4
  def NumCommitEntry  = 3
  def NumRetireEntry  = 3
  def NumPopEntry     = 3
  def RobPcOffsetBits = 3
  // Todo: Num of complete
  def NumCmpltBits    = 2
}

trait PipelineConfig {
  def NumIuPipe = 2
  def NumLsuPipe = 2
  def NumLuPipe = 1
  def NumSuPipe = 1
  def NumPipeline = 7
}

trait AddrConfig {
  def PcWidth = 39
}

trait ExceptionConfig {
  def ExceptionVecWidth = 5
  def InterruptVecWidth = 5
  def MtvalWidth = 32
}

trait VectorUnitConfig {
  def VlmaxBits = 8
  def VlmulBits = 3
  def VsewBits = 3
  def VregNum = 64
  def VregNumBits : Int = log2Up(VregNum)
}

trait FuTypeConfig {
  def ALU      = "b0000000001"
  def BJU      = "b0000000010"
  def MULT     = "b0000000100"
  def DIV      = "b0000001000"
  def LSU_P5   = "b0000110000"
  def LSU      = "b0000010000"
  def PIPE67   = "b0001000000"
  def PIPE6    = "b0010000000"
  def PIPE7    = "b0100000000"
  def SPECIAL  = "b1000000000"
}

object IntConfig extends IntConfig
object ROBConfig extends ROBConfig
object PipelineConfig extends PipelineConfig
object AddrConfig extends AddrConfig
object ExceptionConfig extends ExceptionConfig
object VectorUnitConfig extends VectorUnitConfig
object FuTypeConfig extends FuTypeConfig


abstract class CoreBundle extends Bundle with Config {}

trait Config {
  val PcStart = "h80000000"
  val XLEN = 64
  val VAddrBits = 39
  val PAddrBits = 40

  //IFU
  val uBTBSize = 16
  val pre_array_data_size = 32
  val ghr_size = 22
  val rtu_ras = 6
  val ifu_ras = 12
  val IBufSize = 32

  //CtrlBlock
  val RobSize = 128
  val NRPhyRegs = 64
  //LSU
  val LSQueueSize = 32
  val StoreBufferSize = 16

  //DCache
  val DCacheSize    = 64 //KB
  val CacheLineBits = 512
  val CacheLineSize = CacheLineBits/8
  val DCacheWays    = 2
  val DCacheSets    = DCacheSize*1024/CacheLineSize/DCacheWays //512
  val OffsetBits    = log2Up(CacheLineSize) //6
  val IndexBits     = log2Up(DCacheSets) //9
  val DCacheTagBits = PAddrBits - 12
  val DCRefillBits  = 64
  val RefillCnt     = CacheLineBits/DCRefillBits
}

object Config extends Config{}

object LSUOpType {
  def lb   = "b0000000".U
  def lh   = "b0000001".U
  def lw   = "b0000010".U
  def ld   = "b0000011".U
  def lbu  = "b0000100".U
  def lhu  = "b0000101".U
  def lwu  = "b0000110".U
  def sb   = "b0001000".U
  def sh   = "b0001001".U
  def sw   = "b0001010".U
  def sd   = "b0001011".U

  def lr   = "b0100000".U
  def sc   = "b0100001".U

  def isStore(func: UInt): Bool = func(3)
  def isLoad(func: UInt): Bool = !isStore(func)
  def isLR(func: UInt): Bool = func === lr
  def isSC(func: UInt): Bool = func === sc
  def needMemRead(func: UInt): Bool  = isLoad(func) || isLR(func)
  def needMemWrite(func: UInt): Bool = isStore(func) || isSC(func)
}

object FuncOpType {
  def width = 7.W
  def uwidth = UInt(width)
}