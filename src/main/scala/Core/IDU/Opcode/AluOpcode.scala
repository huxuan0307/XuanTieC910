package Core.IDU.Opcode

import chisel3._

trait AluOpcode extends Opcode {
  // alu
  def AUI_PC = "b1011111".U
  def ADD   : UInt = "b0000000".U
  def SUB   : UInt = "b0000001".U
  def SLT   : UInt = "b0000011".U
  def LUI   : UInt = "b0101100".U
  def ADDW  : UInt = "b0100000".U
  def SUBW  : UInt = "b0100001".U
  def SLTU  : UInt = "b1000011".U
  // shift
  def SLL   : UInt = "b0001000".U
  def SRL   : UInt = "b0001010".U
  def SRA   : UInt = "b0001011".U
  def SLLW  : UInt = "b0101000".U
  def SRLW  : UInt = "b0101010".U
  def SRAW  : UInt = "b0101011".U
  // logic
  def AND   : UInt = "b0010000".U
  def OR    : UInt = "b0010001".U
  def XOR   : UInt = "b0010010".U
  // pseudo
  def MIN   : UInt = "b0010000".U
  def MINU  : UInt = "b1010000".U
  def MINW  : UInt = "b0110000".U
  def MINUW : UInt = "b1110000".U
  def MAX   : UInt = "b0010001".U
  def MAXU  : UInt = "b1010001".U
  def MAXW  : UInt = "b0110001".U
  def MAXUW : UInt = "b1110001".U

  def isWordOp(op: UInt)    : Bool = op(5)
  def isUnsigned(op: UInt)  : Bool = op(6)
}

object AluOpcode extends AluOpcode