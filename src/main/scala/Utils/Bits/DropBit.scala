package Utils.Bits

import chisel3._
import chisel3.util._

object DropBit {
  def apply (src: UInt, idx: Int) : UInt = {
    val width = src.getWidth
    if (idx == 0)
      src(width-1, 1)
    else if(idx == width - 1)
      src(width-2, 0)
    else if(idx > 0 && idx < width - 1)
      Cat(src(width-1, idx+1), src(idx-1, 0))
    else {
      assert(cond = false, "idx out of width of src")
      0.U
    }
  }
  def apply(src: Vec[Bool], idx: Int) : UInt = {
    this.apply(src.asUInt, idx)
  }
}
