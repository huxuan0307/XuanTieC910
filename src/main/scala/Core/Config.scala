package Core

import chisel3.util._

trait IntConfig {
  def LogicRegsNum = 32
  def PhysicRegsNum = 96
  def InstructionIdNum = 128
  def LogicRegsNumBits : Int = log2Up(LogicRegsNum)
  def PhysicRegsNumBits : Int = log2Up(PhysicRegsNum)
  def InstructionIdWidth : Int = log2Up(InstructionIdNum)
}

trait ROBConfig {
  def NumRobEntry     = 64
  def NumRobEntryBits : Int = log2Up(NumRobEntry)
  def RobPtrWidth     : Int = NumRobEntry
  def RobPtrNum       = 4
  def RobReadPtrNum   = 6
  def NumRobReadEntry = 3
  def NumCreateEntry  = 4
  def NumRetireEntry  = 3
  def NumPopEntry     = 3
  def NumCommitEntry  = 3

  def ROB_WIDTH           = 40
  def ROB_VL_PRED         = 39
  def ROB_VL              = 38
  def ROB_VEC_DIRTY       = 30
  def ROB_VSETVLI         = 29
  def ROB_VSEW            = 28
  def ROB_VLMUL           = 25
  def ROB_NO_SPEC_MISPRED = 23
  def ROB_NO_SPEC_MISS    = 22
  def ROB_NO_SPEC_HIT     = 21
  def ROB_LOAD            = 20
  def ROB_FP_DIRTY        = 19
  def ROB_INST_NUM        = 18
  def ROB_BKPTB_INST      = 16
  def ROB_BKPTA_INST      = 15
  def ROB_BKPTB_DATA      = 14
  def ROB_BKPTA_DATA      = 13
  def ROB_STORE           = 12
  def ROB_RAS             = 11
  def ROB_PCFIFO          = 10
  def ROB_BJU             = 9
  def ROB_INTMASK         = 8
  def ROB_SPLIT           = 7
  def ROB_PC_OFFSET       = 6
  def ROB_CMPLT_CNT       = 3
  def ROB_CMPLT           = 1
  def ROB_VLD             = 0
}

trait PipelineConfig {
  def NumIuPipe = 2
  def NumLsuPipe = 1
  def NumPipeline = 7
}

trait AddrConfig {
  def PcWidth = 39
}

trait ExceptionConfig {
  def ExceptionVecWidth = 5
  def interruptVecWidth = 5
  def MtvalWidth = 32
}

trait VectorUnitConfig {
  def VlmaxWidth = 8
  def VlmulWidth = 3
  def VsewWidth = 3
}

object IntConfig extends IntConfig
object ROBConfig extends ROBConfig
object PipelineConfig extends PipelineConfig
object AddrConfig extends AddrConfig
object ExceptionConfig extends ExceptionConfig
object VectorUnitConfig extends VectorUnitConfig