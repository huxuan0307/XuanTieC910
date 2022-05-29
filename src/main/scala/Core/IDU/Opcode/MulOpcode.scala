package Core.IDU.Opcode

import chisel3._

trait MulOpcode extends Opcode {
  def MUL     : UInt = "b0001100".U
  def MULH    : UInt = "b0000100".U
  def MULHU   : UInt = "b0000101".U
  def MULHSU  : UInt = "b0000111".U
  def MULW    : UInt = "b0001000".U
  // for vector
  def MULA    : UInt = "b0101100".U
  def MULS    : UInt = "b0101101".U
  def MULAW   : UInt = "b0101000".U
  def MULSW   : UInt = "b0101001".U
  def MULAH   : UInt = "b0100100".U
  def MULSH   : UInt = "b0100101".U
}

object MulOpcode extends MulOpcode
