package Core.IU

import Core.AddrConfig.PcWidth
import Core.IU.Bju.BjuOut
import Core.IUConfig
import chisel3._
import chisel3.util._
class CbusOut2Rtu extends Bundle with IUConfig {
val pipe0Abnormal = Bool()
val pipe0Bkpt = Bool()
val pipe0Cmplt = Bool()
val pipe0Efpc = UInt(PcWidth.W)
val pipe0EfpcVld = Bool()
val pipe0ExptVec = UInt(5.W)
val pipe0ExptVld = Bool()
val pipe0Flush = Bool()
val pipe0HighHwExpt = Bool()
val pipe0Iid = UInt(7.W)
val pipe0Mtval = Bool()
val pipe1Cmplt = Bool()
val pipe1Iid = Bool()
val pipe2Abnormal = Bool()
val pipe2BhtMispred = Bool()
val pipe2Cmplt = Bool()
val pipe2Iid = UInt(7.W)
val pipe2JmpMispred = Bool()
}

class CbusNoEcep extends Bundle with IUConfig{
  val divSel = Bool()
  val multSel = Bool()
  val pipe0Iid = UInt(7.W)
  val pipe0Sel = Bool()
  val pipe1Iid = UInt(7.W)
  val pipe1Sel = Bool()
}

class Cp0In extends Bundle with IUConfig{
  val abnormal = Bool()
  val efpc = UInt(PcWidth.W)
  val efpcVld = Bool()
  val exptVec = UInt(5.W)
  val exptVld = Bool()
  val flush = Bool()
  val iid = UInt(7.W)
  val instVld = Bool()
  val mtval = UInt(32.W)
  val en = Bool()
}

class CbusIO extends Bundle with IUConfig{
  val normIn = Input(new CbusNoEcep)
  val specialIn = Input(new SpecialOut)
  val cp0In = Input(new Cp0In)
  val out = Output(new CbusOut2Rtu)
  val bjuIn = Input(new BjuOut)
}

class Cbus extends Module with IUConfig{
  val io = IO(new CbusIO)
  val flush = false.B
  val cbus_pipe0_cmplt = io.normIn.pipe0Sel || io.normIn.divSel || io.specialIn.instVld || io.cp0In.instVld
  val cbus_pipe0_inst_vld = RegInit(cbus_pipe0_cmplt ,!flush)

  // pipe 0
  // Mux the iid - to Rob
  val cbus_pipe0_src_iid = (Cat(Fill(7, io.normIn.pipe0Sel)) & io.normIn.pipe0Iid) |
    (Cat(Fill(7, io.normIn.divSel)) & io.normIn.pipe0Iid)|
    (Cat(Fill(7, io.specialIn.instVld)) & io.specialIn.iid)|
    (Cat(Fill(7, io.cp0In.instVld)) & io.cp0In.iid)

  val cbus_pipe0_src_abnormal = (io.specialIn.instVld && io.specialIn.abnormal) || (io.cp0In.abnormal && io.cp0In.instVld)

  val cbus_pipe0_src_expt_vld = (io.specialIn.instVld && io.specialIn.exptVld) || (io.cp0In.exptVld && io.cp0In.instVld)

  val cbus_pipe0_src_expt_vec = (Cat(Fill(7, io.specialIn.instVld)) & io.specialIn.exptVec) |
    (Cat(Fill(7, io.cp0In.instVld)) & io.cp0In.exptVec)

  val cbus_pipe0_src_high_hw_expt = io.specialIn.instVld && io.specialIn.highHwExpt

  val cbus_pipe0_src_bkpt = io.specialIn.instVld && io.specialIn.bkpt

  val cbus_pipe0_src_mtval = (Cat(Fill(7, io.specialIn.instVld)) & io.specialIn.mtval)|
    (Cat(Fill(7, io.cp0In.instVld)) & io.cp0In.mtval)

  val cbus_pipe0_src_flush = (io.cp0In.instVld && io.cp0In.flush) || (io.specialIn.instVld && io.specialIn.flush)

  val cbus_pipe0_src_efpc_vld =  io.cp0In.instVld && io.cp0In.efpcVld

  val cbus_pipe0_src_efpc = Cat(Fill(PcWidth, io.cp0In.instVld)) & io.cp0In.efpc


  val cbus_pipe0_expt_vld      = RegEnable( cbus_pipe0_src_expt_vld, cbus_pipe0_cmplt)
  val cbus_pipe0_expt_vec      = RegEnable( cbus_pipe0_src_expt_vec, cbus_pipe0_cmplt)
  val cbus_pipe0_high_hw_expt  = RegEnable( cbus_pipe0_src_high_hw_expt, cbus_pipe0_cmplt)
  val cbus_pipe0_bkpt          = RegEnable( cbus_pipe0_src_bkpt, cbus_pipe0_cmplt)
  val cbus_pipe0_mtval         = RegEnable( cbus_pipe0_src_mtval, cbus_pipe0_cmplt)
  val cbus_pipe0_flush         = RegEnable( cbus_pipe0_src_flush, cbus_pipe0_cmplt)
  val cbus_pipe0_efpc_vld      = RegEnable( cbus_pipe0_src_efpc_vld, cbus_pipe0_cmplt)
  val cbus_pipe0_efpc          = RegEnable( cbus_pipe0_src_efpc, cbus_pipe0_cmplt)
  val cbus_pipe0_iid           = RegEnable( cbus_pipe0_src_iid, cbus_pipe0_cmplt) // TODO CLK is different  @ct_iu_cbus @439
  val cbus_pipe0_abnormal      = RegEnable( cbus_pipe0_abnormal, cbus_pipe0_cmplt)
  // out put
  io.out.pipe0Cmplt         := cbus_pipe0_cmplt
  io.out.pipe0Iid           := cbus_pipe0_iid
  io.out.pipe0Abnormal      := cbus_pipe0_abnormal
  io.out.pipe0ExptVld       := cbus_pipe0_expt_vld
  io.out.pipe0ExptVec       := cbus_pipe0_expt_vec
  io.out.pipe0HighHwExpt    := cbus_pipe0_high_hw_expt
  io.out.pipe0Bkpt          := cbus_pipe0_bkpt
  io.out.pipe0Mtval         := cbus_pipe0_mtval
  io.out.pipe0Flush         := cbus_pipe0_flush
  io.out.pipe0EfpcVld       := cbus_pipe0_efpc_vld
  io.out.pipe0Efpc          := cbus_pipe0_efpc


  // pipe 1

  val cbus_pipe1_cmplt    = io.normIn.pipe1Sel  || io.normIn.multSel
  val cbus_pipe1_inst_vld = RegInit(cbus_pipe1_cmplt ,!flush)
  val cbus_pipe1_src_iid  = io.normIn.pipe1Iid
  val cbus_pipe1_iid      = RegEnable(cbus_pipe1_src_iid,cbus_pipe1_cmplt)
  io.out.pipe1Iid         := cbus_pipe1_iid
  io.out.pipe2Abnormal    := io.bjuIn.toCbus.sel
  io.out.pipe2BhtMispred  := io.bjuIn.toCbus.abnormal
  io.out.pipe2Cmplt       := io.bjuIn.toCbus.jmpMispred
  io.out.pipe2JmpMispred  := io.bjuIn.toCbus.bhtMispred
}
