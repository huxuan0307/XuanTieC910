package Core.LSU.DCache
import chisel3._
import chisel3.experimental.ExtModule
import chisel3.util._


class ct_spsram_2048x32 extends ExtModule with HasExtModuleResource {
  val A = IO(Input(UInt(11.W)))
  val CEN = IO(Input(Bool()))
  val CLK = IO(Input(Clock()))
  val D = IO(Input(UInt(32.W)))
  val GWEN = IO(Input(Bool()))
  val Q = IO(Output(UInt(32.W)))
  val WEN = IO(Input(UInt(32.W)))

  addResource("/ct_spsram_2048x32.v")
}

class ct_spsram_512x54 extends ExtModule with HasExtModuleResource {
  val A = IO(Input(UInt(9.W)))
  val CEN = IO(Input(Bool()))
  val CLK = IO(Input(Clock()))
  val D = IO(Input(UInt(54.W)))
  val GWEN = IO(Input(Bool()))
  val Q = IO(Output(UInt(54.W)))
  val WEN = IO(Input(UInt(54.W)))

  addResource("/ct_spsram_512x54.v")
}

class ct_spsram_512x52 extends ExtModule with HasExtModuleResource {
  val A = IO(Input(UInt(9.W)))
  val CEN = IO(Input(Bool()))
  val CLK = IO(Input(Clock()))
  val D = IO(Input(UInt(52.W)))
  val GWEN = IO(Input(Bool()))
  val Q = IO(Output(UInt(52.W)))
  val WEN = IO(Input(UInt(52.W)))

  addResource("/ct_spsram_512x52.v")
}

class ct_spsram_512x7 extends ExtModule with HasExtModuleResource {
  val A = IO(Input(UInt(9.W)))
  val CEN = IO(Input(Bool()))
  val CLK = IO(Input(Clock()))
  val D = IO(Input(UInt(7.W)))
  val GWEN = IO(Input(Bool()))
  val Q = IO(Output(UInt(7.W)))
  val WEN = IO(Input(UInt(7.W)))

  addResource("/ct_spsram_512x7.v")
}

