package Core.IFU
import Core.Config
import chisel3._
import chisel3.util._


class PCGen extends Module with Config {
  val io = IO(new PCGenIO)

  val pcReg = RegInit(PcStart.U(VAddrBits.W))
  val pcWire = WireInit(PcStart.U(VAddrBits.W))
  val ifpc_chgflw_pre = Wire(UInt(VAddrBits.W))
  val inc_pc = WireInit(0.U(VAddrBits.W))
  val rtu_cur_pc = WireInit(0.U(VAddrBits.W))
  val rtu_cur_pc_load = WireInit(false.B)
  val ifu_rtu_cur_pcReg = RegInit(PcStart.U(VAddrBits.W))
  val ifu_rtu_cur_pc_loadReg = WireInit(false.B)
  val pcgen_chgflw = io.redirect.map(_.valid).reduce(_||_)
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
  ifpc_chgflw_pre := PriorityMux(Seq(
      io.rtu_ifu_chgflw_vld -> io.rtu_ifu_chgflw_pc,
      io.redirect(3).valid -> io.redirect(3).bits,//io.redirect(3).bits,
      io.redirect(2).valid -> io.redirect(2).bits,//io.redirect(2).bits,
      io.redirect(1).valid -> io.redirect(1).bits,//io.redirect(1).bits,
      io.redirect(0).valid -> io.redirect(0).bits,//io.redirect(0).bits,
      //io.redirect(0).valid -> io.redirect(0).bits,
      true.B          -> "h80000000".U
    ))

  inc_pc := Cat(pcReg(VAddrBits-1,4), 0.U(4.W)) + 16.U
  //true.B, io.redirect(0).bits can ?
  //false.B, io.redirect(0).bits can be compiled !!!
  //io.redirect(0).valid, io.redirect(0).bits can not be compiled !!!
  //io.redirect(1).valid can be compiled !!!
  //io.redirect(0).valid can be compiled !!!
  //false.B can be compiled
  //pc := Mux(io.redirect(3).valid,io.redirect(3).bits,Mux(io.redirect(2).valid,io.redirect(2).bits,Mux(io.redirect(1).valid,io.redirect(1).bits,Mux(io.redirect(0).valid,io.redirect(0).bits,(Cat(pcReg(VAddrBits-1,4), 0.U(4.W)) + 16.U)))))
  //pcWire := Mux(io.continue,Cat(pcReg(VAddrBits-1,4), 0.U(4.W)) + 16.U,pcReg)
  when(io.IbufAllowEnq){
    pcReg := Mux(pcgen_chgflw,ifpc_chgflw_pre,Mux(io.continue,inc_pc,pcReg))
  }

  io.pc := pcReg

  //==========================================================
  //                  Interface with RTU
  //==========================================================
  rtu_cur_pc_load := io.had_ifu_pcload || io.vector_pcgen_pcload
  rtu_cur_pc := Mux(io.had_ifu_pcload, io.had_ifu_pc, io.vector_pcgen_pc)

  ifu_rtu_cur_pc_loadReg := rtu_cur_pc_load
  ifu_rtu_cur_pcReg := Mux(io.continue, pcReg, Mux(rtu_cur_pc_load, rtu_cur_pc, ifu_rtu_cur_pcReg)) //////todo: fix this logic

  io.ifu_rtu_cur_pc_load := ifu_rtu_cur_pc_loadReg
  io.ifu_rtu_cur_pc := ifu_rtu_cur_pcReg

  //-----------------------IB Cancel--------------------------
  io.ibctrl_cancel := io.had_ifu_pcload ||
    io.vector_pcgen_pcload ||
    io.rtu_ifu_chgflw_vld ||
    io.redirect(3).valid
    //|| io.rtu_ifu_xx_expt_vld todo
    //|| dbg_cancel


}
