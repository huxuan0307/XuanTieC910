package Core.IDU.Opcode

import chisel3._

trait BjuOpcode extends Opcode {
  def JAL     : UInt = "b1000000".U
  def JALR    : UInt = "b1000001".U
  def BEQ     : UInt = "b0100000".U
  def BNE     : UInt = "b0100001".U
  def BLT     : UInt = "b0100010".U
  def BLTU    : UInt = "b0100011".U
  def BGE     : UInt = "b0100100".U
  def BGEU    : UInt = "b0100101".U
  def isJmp(op: UInt): Bool = op(6)
  def isBr(op: UInt): Bool = op(5)
}

object BjuOpcode extends BjuOpcode
