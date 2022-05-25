package Core.IDU.Opcode

import chisel3._

trait SpecialOpcode extends Opcode {
  def ECALL         : UInt = "b0000010".U
  def EBREAK        : UInt = "b0000011".U
  def AUIPC         : UInt = "b0000100".U
  def PSEUDO_AUIPC  : UInt = "b0000101".U
  def VSETVLI       : UInt = "b0000100".U
  def VSETVL        : UInt = "b0000111".U
}

object SpecialOpcode extends SpecialOpcode

