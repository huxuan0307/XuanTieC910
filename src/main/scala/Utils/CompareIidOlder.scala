package Utils

import Core.ROBConfig.NumRobEntryBits
import chisel3.{Bool, Mux, UInt, Wire}

object CompareIidOlder {
  private def IidFlag(iid: UInt) : Bool = {
    iid(NumRobEntryBits)
  }
  private def IidNum(iid: UInt) : UInt = {
    iid(NumRobEntryBits - 1, 0)
  }
  def apply(left: UInt, right: UInt) : Bool = {
    val res = Wire(Bool())
    res := Mux(
      IidFlag(left) === IidFlag(right),
      IidNum(left) < IidNum(right),
      IidNum(left) > IidNum(right)
    )
    res
  }
}
