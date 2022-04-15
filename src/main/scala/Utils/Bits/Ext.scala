package Utils.Bits

import chisel3._
import chisel3.util._

object msb {
  def apply(src: UInt): Bool = src(src.getWidth - 1).asBool
}

object sext {
  def apply(width: Int, src: UInt) : UInt = {
    assert(width > 0)
    if (width - src.getWidth == 0)
      src
    else
      Cat(Fill(width - src.getWidth, msb(src).asUInt), src)
  }
}

object zext {
  def apply(width: Int, src: UInt) : UInt = {
    assert(width > 0)
    if (width - src.getWidth == 0)
      src
    else
      Cat(Fill(width - src.getWidth, 0.U(1.W)), src)
  }
}
