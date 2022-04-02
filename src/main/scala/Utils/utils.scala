package Utils

import chisel3._
import chisel3.util._

object LookupTree {
  def apply[T <: Data](key: UInt, mapping: Iterable[(UInt, T)]): T =
    Mux1H(mapping.map(p => (p._1 === key, p._2)))
}   // mux select

object SignExt {
  def apply(a: UInt, len: Int) : UInt = {
    val aLen = a.getWidth
    val signBit = a(aLen-1)
    if (aLen >= len) a(len-1,0) else Cat(Fill(len - aLen, signBit), a)
  }
}  //bread says it is buqi weishu

object ZeroExt {
  def apply(a: UInt, len: Int) : UInt = {
    val aLen = a.getWidth
    if (aLen >= len) a(len-1,0) else Cat(0.U((len - aLen).W), a)
  }
}
object UIntToMask {
  def apply(ptr: UInt, length: Integer) = leftmask(ptr, length)
  def reverseUInt(input: UInt): UInt = {
    VecInit(input.asBools.reverse).asUInt
  }
  def leftmask(ptr: UInt, length: Integer) = UIntToOH(ptr)(length - 1, 0) - 1.U
  def rightmask(ptr: UInt, length: Integer) = reverseUInt(reverseUInt(UIntToOH(ptr)(length - 1, 0)) - 1.U)
}

