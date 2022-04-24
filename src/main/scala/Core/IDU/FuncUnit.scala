package Core.IDU

import chisel3._

object FuncUnit {
  def ALU     : UInt = "b0001".U
  def SPECIAL : UInt = "b0010".U
  def CP0     : UInt = "b0011".U
  def MU      : UInt = "b0100".U
  def DU      : UInt = "b0101".U
  def BJU     : UInt = "b1000".U
  def LU      : UInt = "b1001".U
  def SU      : UInt = "b1010".U
}
