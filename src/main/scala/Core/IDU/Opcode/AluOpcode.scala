package Core.IDU.Opcode

import chisel3._

trait AluOpcode extends Opcode {
  // alu
  def ADD   : UInt = "b0000000".U
  def ADDW  : UInt = "b0000001".U
  def SUB   : UInt = "b0000010".U
  def SUBW  : UInt = "b0000011".U
  def SLT   : UInt = "b0000110".U
  def SLTU  : UInt = "b0001110".U
  def LUI   : UInt = "b0001000".U
  // pseudo
  def MIN   : UInt = "b0010100".U
  def MINU  : UInt = "b0011100".U
  def MINW  : UInt = "b0010101".U
  def MINUW : UInt = "b0011101".U
  def MAX   : UInt = "b0010000".U
  def MAXU  : UInt = "b0011000".U
  def MAXW  : UInt = "b0010001".U
  def MAXUW : UInt = "b0011001".U
}

object AluOpcode extends AluOpcode