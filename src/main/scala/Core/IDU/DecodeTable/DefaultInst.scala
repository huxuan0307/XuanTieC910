package Core.IDU.DecodeTable

import Core.IDU.FuncUnit
import Core.IDU.Opcode.Opcode
import Utils.Bits.BinaryConst.{F, T}
import chisel3._

object DefaultInst {
  def inst = List(FuncUnit.NON, Opcode.NOP, F, F, F)
}
