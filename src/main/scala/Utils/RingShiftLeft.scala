package Utils

import chisel3._
import chisel3.util._

object RingShiftLeft {
  def apply(src: UInt, offset: Int) : UInt = {
    assert(offset >= 0)
    if (offset == 0)
      src
    else
      Cat(src(src.getWidth - offset - 1, 0), src(src.getWidth - 1, src.getWidth - offset))
  }
}

