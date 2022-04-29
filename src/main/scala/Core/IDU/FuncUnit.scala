package Core.IDU

import chisel3._
import chisel3.util._

object FuncUnit {
  def OH = true
  def size = 8
  def width   : Int = size
  def NON     : UInt = 0.U(width.W)
  def ALU     : UInt = UIntToOH(0.U, width)
  def SPECIAL : UInt = UIntToOH(1.U, width)
  def CP0     : UInt = UIntToOH(2.U, width)
  def MU      : UInt = UIntToOH(3.U, width)
  def DU      : UInt = UIntToOH(4.U, width)
  def BJU     : UInt = UIntToOH(5.U, width)
  def LU      : UInt = UIntToOH(6.U, width)
  def SU      : UInt = UIntToOH(7.U, width)
}
