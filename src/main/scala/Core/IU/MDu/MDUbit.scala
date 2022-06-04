package Core.IU.Du

import Core.Config.XLEN
import Core.IUConfig
import Utils.ZeroExt
import chisel3._

class MDUbit(val src: UInt) extends Bundle with IUConfig  {
  def abs(a: UInt,lenx: Int):  UInt = {
    val s = a(lenx - 1)
    val temp = Wire(UInt((lenx+1).W))
    temp := Mux(s, -a, a)
    temp(lenx-1, 0)
  }                                  //取得绝对值
  def single(x: UInt, lenx: Int):UInt = ZeroExt(abs(x,lenx),XLEN)
  def half(x: UInt, lenx: Int) = ZeroExt(abs(x,lenx),32)
}

