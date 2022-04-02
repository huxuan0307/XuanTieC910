package Core.IFU
import Core.Config
import chisel3._
import chisel3.util._


class PCGen extends Module with Config {
  val io = IO(new PCGenIO)

  val pcReg = RegInit(PcStart.U(VAddrBits.W))
  val pcWire = WireInit(PcStart.U(VAddrBits.W))
  val pc = WireInit(0.U(VAddrBits.W))

  pc := PriorityMux(Seq(
    //    io.redirect(3).valid -> io.redirect(3).bits,
    //    io.redirect(2).valid -> io.redirect(2).bits,
    //    io.redirect(1).valid -> io.redirect(1).bits,
    //    io.redirect(0).valid -> io.redirect(0).bits,
    true.B               -> pcReg,
    io.continue          -> (Cat(pcReg(VAddrBits-1,4), 0.U(4.W)) + 16.U)
  ))
  //pcWire := Mux(io.continue,Cat(pcReg(VAddrBits-1,4), 0.U(4.W)) + 16.U,pcReg)
  pcReg := Mux(io.continue,Cat(pcReg(VAddrBits-1,4), 0.U(4.W)) + 16.U,pcReg)
  io.pc := pc
}
