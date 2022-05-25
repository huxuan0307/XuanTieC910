package Core.IDU.Opcode

import chisel3._

trait Opcode {
  def width = 7
  def NOP           : UInt = "b0000000".U
}

object Opcode extends Opcode
