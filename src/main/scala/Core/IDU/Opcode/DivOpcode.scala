package Core.IDU.Opcode

import chisel3._

trait DivOpcode extends Opcode {
  def DIV   : UInt = "b1000101".U
  def DIVU  : UInt = "b1000100".U
  def REM   : UInt = "b1001001".U
  def REMU  : UInt = "b1001000".U
  def DIVW  : UInt = "b1000111".U
  def DIVUW : UInt = "b1000110".U
  def REMW  : UInt = "b1001011".U
  def REMUW : UInt = "b1001010".U
  def isDu(opcode: UInt)  : Bool = opcode(6,5) === "b10".U
}

object DivOpcode extends DivOpcode
