package Core

import chisel3.util._

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
  def InstBits = 32
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

object IntConfig extends IntConfig
object ROBConfig extends ROBConfig
object PipelineConfig extends PipelineConfig
object AddrConfig extends AddrConfig
object ExceptionConfig extends ExceptionConfig
object VectorUnitConfig extends VectorUnitConfig