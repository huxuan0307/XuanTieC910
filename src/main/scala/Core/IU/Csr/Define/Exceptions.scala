package Core.IU.Csr.Define

import  chisel3._
object Exceptions {
  object ExceptionVec {
    def apply(): Vec[Bool] = Vec(16, Bool())
  }

  def InstAddressMisaligned = 0

  def InstAccessFault = 1

  def IllegalInst = 2

  def Breakpoint = 3

  def LoadAddrMisaligned = 4

  def LoadAccessFault = 5

  def StoreAddrMisaligned = 6

  def StoreAccessFault = 7

  def UECall = 8

  def SECall = 9

  def MECall = 11

  def InstPageFault = 12

  def LoadPageFault = 13

  def StorePageFault = 15
}
