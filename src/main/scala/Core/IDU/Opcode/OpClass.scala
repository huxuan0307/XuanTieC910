package Core.IDU.Opcode

import chisel3._

trait OpClass {
  def ADDER   : UInt = "b00001".U
  def SHIFTER : UInt = "b00010".U
  def LOGIC   : UInt = "b00011".U
  def MISC    : UInt = "b00100".U
}
