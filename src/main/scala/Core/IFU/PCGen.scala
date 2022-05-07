package Core.IFU
import Core.Config
import chisel3._
import chisel3.util._


class PCGen extends Module with Config {
  val io = IO(new PCGenIO)

  val pcReg = RegInit(PcStart.U(VAddrBits.W))
  val pcWire = WireInit(PcStart.U(VAddrBits.W))
  val pc = WireInit(0.U(VAddrBits.W))
  val rtu_cur_pc = WireInit(0.U(VAddrBits.W))
  val rtu_cur_pc_load = WireInit(false.B)
  val ifu_rtu_cur_pcReg = RegInit(PcStart.U(VAddrBits.W))
  val ifu_rtu_cur_pc_loadReg = WireInit(false.B)

  // &Force("bus","ipctrl_pcgen_taken_pc",38,0); @28
  //==========================================================
  //                PC MUX of Change Flow
  //==========================================================
  //The Priority of PC is as following:
  //  PC from Had
  //  PC from Vector
  //  PC from RTU
  //  PC from IU
  //  PC from Addrgen
  //  PC from IFU IB Stage Change Flow
  //  PC from IFU IP Stage Reissue
  //  PC from IFU IP Stage Change Flow
  //  PC from IFU IF Stage Reissue
  //  PC Increase
  // &CombBeg; @43
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

  //==========================================================
  //                  Interface with RTU
  //==========================================================
  rtu_cur_pc_load := io.had_ifu_pcload || io.vector_pcgen_pcload
  rtu_cur_pc := Mux(io.had_ifu_pcload, io.had_ifu_pc, io.vector_pcgen_pc)

  ifu_rtu_cur_pc_loadReg := rtu_cur_pc_load
  ifu_rtu_cur_pcReg := Mux(io.continue, pc, Mux(rtu_cur_pc_load, rtu_cur_pc, ifu_rtu_cur_pcReg)) //////todo: fix this logic

  io.ifu_rtu_cur_pc_load := ifu_rtu_cur_pc_loadReg
  io.ifu_rtu_cur_pc := ifu_rtu_cur_pcReg
}
