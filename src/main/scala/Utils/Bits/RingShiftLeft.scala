package Utils.Bits

import chisel3.util.Cat
import chisel3.{UInt, assert}

object RingShiftLeft {
  def apply(src : UInt, offset : Int) : UInt = {
    assert(offset >= 0 && offset <= src.getWidth)
    if (offset == 0)
      src
    else {
      Cat(
        src(src.getWidth - offset - 1, 0),
        src(src.getWidth - 1, src.getWidth - offset)
      )
    }
  }
}
