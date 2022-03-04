package Core.IU

import Core.AddrConfig.PcWidth
import Core.IU.Bju.BjuOut
import Core.IUConfig
import chisel3._
import chisel3.util._
class CbusOut2Rtu extends Bundle with IUConfig {
val pipe0_abnormal = Bool()
val pipe0_bkpt = Bool()
val pipe0_cmplt = Bool()
val pipe0_efpc = UInt(PcWidth.W)
val pipe0_efpc_vld = Bool()
val pipe0_expt_vec = UInt(5.W)
val pipe0_expt_vld = Bool()
val pipe0_flush = Bool()
val pipe0_high_hw_expt = Bool()
val pipe0_iid = UInt(7.W)
val pipe0_mtval = Bool()
val pipe1_cmplt = Bool()
val pipe1_iid = Bool()
val pipe2_abnormal = Bool()
val pipe2_bht_mispred = Bool()
val pipe2_cmplt = Bool()
val pipe2_iid = UInt(7.W)
val pipe2_jmp_mispred = Bool()
}

class CbusNoEcep extends Bundle with IUConfig{
  val div_sel = Bool()
  val mult_sel = Bool()
  val pipe0_iid = UInt(7.W)
  val pipe0_sel = Bool()
  val pipe1_iid = UInt(7.W)
  val pipe1_sel = Bool()
}

class Cp0In extends Bundle with IUConfig{
  val abnormal = Bool()
  val efpc = UInt(PcWidth.W)
  val efpc_vld = Bool()
  val expt_vec = UInt(5.W)
  val expt_vld = Bool()
  val flush = Bool()
  val iid = UInt(7.W)
  val inst_vld = Bool()
  val mtval = UInt(32.W)
  val en = Bool()
}

class CbusIO extends Bundle with IUConfig{
  val norm_in = Input(new CbusNoEcep)
  val special_in = Input(new SpecialOut)
  val cp0_in = Input(new Cp0In)
  val out = Output(new CbusOut2Rtu)
  val bju_in = Input(new BjuOut)
}

class Cbus extends Module with IUConfig{
  val io = IO(new CbusIO)
  val flush = false.B
  val cbus_pipe0_cmplt = io.norm_in.pipe0_sel || io.norm_in.div_sel || io.special_in.inst_vld || io.cp0_in.inst_vld
  val cbus_pipe0_inst_vld = RegInit(cbus_pipe0_cmplt ,!flush)

  // pipe 0
  // Mux the iid - to Rob
  val cbus_pipe0_src_iid = (Cat(Fill(7, io.norm_in.pipe0_sel)) & io.norm_in.pipe0_iid) |
    (Cat(Fill(7, io.norm_in.div_sel)) & io.norm_in.pipe0_iid)|
    (Cat(Fill(7, io.special_in.inst_vld)) & io.special_in.iid)|
    (Cat(Fill(7, io.cp0_in.inst_vld)) & io.cp0_in.iid)

  val cbus_pipe0_src_abnormal = (io.special_in.inst_vld && io.special_in.abnormal) || (io.cp0_in.abnormal && io.cp0_in.inst_vld)

  val cbus_pipe0_src_expt_vld = (io.special_in.inst_vld && io.special_in.expt_vld) || (io.cp0_in.expt_vld && io.cp0_in.inst_vld)

  val cbus_pipe0_src_expt_vec = (Cat(Fill(7, io.special_in.inst_vld)) & io.special_in.expt_vec) |
    (Cat(Fill(7, io.cp0_in.inst_vld)) & io.cp0_in.expt_vec)

  val cbus_pipe0_src_high_hw_expt = io.special_in.inst_vld && io.special_in.high_hw_expt

  val cbus_pipe0_src_bkpt = io.special_in.inst_vld && io.special_in.bkpt

  val cbus_pipe0_src_mtval = (Cat(Fill(7, io.special_in.inst_vld)) & io.special_in.mtval)|
    (Cat(Fill(7, io.cp0_in.inst_vld)) & io.cp0_in.mtval)

  val cbus_pipe0_src_flush = (io.cp0_in.inst_vld && io.cp0_in.flush) || (io.special_in.inst_vld && io.special_in.flush)

  val cbus_pipe0_src_efpc_vld =  io.cp0_in.inst_vld && io.cp0_in.efpc_vld

  val cbus_pipe0_src_efpc = Cat(Fill(PcWidth, io.cp0_in.inst_vld)) & io.cp0_in.efpc


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
  io.out.pipe0_cmplt := cbus_pipe0_cmplt
  io.out.pipe0_iid           := cbus_pipe0_iid
  io.out.pipe0_abnormal      := cbus_pipe0_abnormal
  io.out.pipe0_expt_vld      := cbus_pipe0_expt_vld
  io.out.pipe0_expt_vec      := cbus_pipe0_expt_vec
  io.out.pipe0_high_hw_expt  := cbus_pipe0_high_hw_expt
  io.out.pipe0_bkpt          := cbus_pipe0_bkpt
  io.out.pipe0_mtval         := cbus_pipe0_mtval
  io.out.pipe0_flush         := cbus_pipe0_flush
  io.out.pipe0_efpc_vld      := cbus_pipe0_efpc_vld
  io.out.pipe0_efpc          := cbus_pipe0_efpc


  // pipe 1

  val cbus_pipe1_cmplt  = io.norm_in.pipe1_sel  || io.norm_in.mult_sel
  val cbus_pipe1_inst_vld = RegInit(cbus_pipe1_cmplt ,!flush)
  val cbus_pipe1_src_iid = io.norm_in.pipe1_iid
  val cbus_pipe1_iid = RegEnable(cbus_pipe1_src_iid,cbus_pipe1_cmplt)
  io.out.pipe1_iid     := cbus_pipe1_iid
  io.out.pipe2_abnormal    := io.bju_in.toCbus.sel
  io.out.pipe2_bht_mispred := io.bju_in.toCbus.abnormal
  io.out.pipe2_cmplt       := io.bju_in.toCbus.jmpMispred
  io.out.pipe2_jmp_mispred := io.bju_in.toCbus.bhtMispred
}
