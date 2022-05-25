package Core.IDU.Opcode

import chisel3._

trait ShiftOpcode extends Opcode {
  // shift
  def SLL   : UInt = "b0100000".U
  def SRL   : UInt = "b0100001".U
  def SRA   : UInt = "b0100011".U
  def SLLW  : UInt = "b0100100".U
  def SRLW  : UInt = "b0100101".U
  def SRAW  : UInt = "b0100111".U
  // logic
  def AND   : UInt = "b0101000".U
  def OR    : UInt = "b0101001".U
  def XOR   : UInt = "b0101010".U
}

object ShiftOpcode extends ShiftOpcode
