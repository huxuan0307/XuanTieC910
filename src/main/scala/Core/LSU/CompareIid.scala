package Core.LSU
import Core.ROBConfig.{IidWidth, NumRobEntryBits}
import chisel3._
import chisel3.util._
class CompareIid extends Module{
  val io = IO(new Bundle() {
    val iid0 = Input(UInt(IidWidth.W))
    val iid1 = Input(UInt(IidWidth.W))
    val older = Output(Bool())
  })
  //==========================================================
  //            Compare order of two 7 bits IIDs
  //==========================================================
  val mis_match = io.iid0(IidWidth-1) ^ io.iid1(IidWidth-1)
  val (ptr0,ptr1) = (io.iid0(NumRobEntryBits-1,0),io.iid1(NumRobEntryBits-1,0))
  val match_0_older = ptr0 < ptr1
  val mis_match_0_older = ptr0 < ptr1
  io.older := Mux(mis_match,mis_match_0_older,match_0_older)
}
