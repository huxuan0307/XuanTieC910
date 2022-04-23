package Core.IU.Cp0.Define

import chisel3.{Bool, UInt, Vec, fromStringToLiteral}

object Interrupts {

  object InterruptVec {
    def apply(): Vec[Bool] = Vec(12, Bool())
  }
  def UserSoft = 0

  def SupervisorSoft = 1

  def MachineSoft = 3

  def UserTime = 4

  def SupervisorTime = 5

  def MachineTime = 7

  def UserExtern = 8

  def SupervisorExtern = 9

  def MachineExtern = 11

  def isUser(cause : UInt) : Bool = cause(1, 0) === "b00".U

  def isSupervisor(cause : UInt) : Bool = cause(1, 0) === "b01".U

  def isMachine(cause : UInt) : Bool = cause(1, 0) === "b11".U

  def isSoft(cause : UInt) : Bool = cause(3, 2) === "b00".U

  def isTime(cause : UInt) : Bool = cause(3, 2) === "b01".U

  def isExtern(cause : UInt) : Bool = cause(3, 2) === "b10".U
}
