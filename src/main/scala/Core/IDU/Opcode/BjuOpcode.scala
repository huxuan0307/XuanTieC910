package Core.IDU.Opcode

import chisel3._

trait BjuOpcode extends Opcode {
  def JAL     : UInt = "b0000000".U
  def JALR    : UInt = "b0000001".U
  def BEQ     : UInt = "b0001000".U
  def BNE     : UInt = "b0001001".U
  def BLT     : UInt = "b0001010".U
  def BLTU    : UInt = "b0001011".U
  def BGE     : UInt = "b0001100".U
  def BGEU    : UInt = "b0001101".U
}

object BjuOpcode extends BjuOpcode
