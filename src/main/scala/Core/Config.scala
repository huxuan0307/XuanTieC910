package Core

import chisel3.util._

trait IntConfig {
  def LogicRegsNum = 32
  def PhysicRegsNum = 96
  def InstructionIdNum = 128
  def LogicRegsNumBits : Int = log2Up(LogicRegsNum)
  def PhysicRegsNumBits : Int = log2Up(PhysicRegsNum)
  def InstructionIdNumBits : Int = log2Up(InstructionIdNum)
}

trait ROBConfig {
  def NumCreateEntry = 4
  def NumRetireEntry = 3
}

object IntConfig extends IntConfig
object ROBConfig extends ROBConfig