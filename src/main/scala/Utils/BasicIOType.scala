package Utils
import chisel3._
import chisel3.internal.firrtl.Width

object OutUInt {
  def apply(w : Int) : UInt = Output(UInt(w.W))
  def apply(w : Width) : UInt = Output(UInt(w))
}

object InUInt {
  def apply(w: Int) : UInt = Input(UInt(w.W))
  def apply(w: Width) : UInt = Input(UInt(w))
}

object OutBool {
  def apply() : Bool = Output(Bool())
}

object InBool {
  def apply() : Bool = Input(Bool())
}
