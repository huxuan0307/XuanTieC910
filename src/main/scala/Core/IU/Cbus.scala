package Core.IU

import Core.AddrConfig.PcWidth
import Core.IU.Bju.{BjuCmpltSignal, BjuOut}
import Core.IUConfig
import Core.IntConfig.InstructionIdWidth
import chisel3._
import chisel3.util._
class CbusOut extends Bundle with IUConfig {
val pipe0Abnormal = Bool()
val pipe0Bkpt = Bool()
val pipe0Cmplt = Bool()
val pipe0Efpc = UInt(PcWidth.W)
val pipe0EfpcVld = Bool()
val pipe0ExptVec = UInt(5.W)
val pipe0ExptVld = Bool()
val pipe0Flush = Bool()
val pipe0HighHwExpt = Bool()
val pipe0Iid = UInt(InstructionIdWidth.W)
val pipe0Mtval = Bool()
val pipe1Cmplt = Bool()
val pipe1Iid = UInt(InstructionIdWidth.W)
val pipe2Abnormal = Bool()
val pipe2BhtMispred = Bool()
val pipe2Cmplt = Bool()
val pipe2Iid = UInt(InstructionIdWidth.W)
val pipe2JmpMispred = Bool()
}

class CbusNoEcep extends Bundle with IUConfig{
  val divSel   = Bool()
  val multSel  = Bool()
  val pipe0Iid = UInt(InstructionIdWidth.W)
  val pipe0Sel = Bool()
  val pipe1Iid = UInt(InstructionIdWidth.W)
  val pipe1Sel = Bool()
}

class CbusIO extends Bundle with IUConfig{
  val normIn    = Input(new CbusNoEcep)
  val specialIn = Input(new SpecialCmpltSignal)
  val cp0In     = Input(new Cp0ToCbus)
  val bjuIn     = Input(new BjuCmpltSignal)
  val flush     = Input(Bool())
  val out       = Output(new CbusOut)
}

class Cbus extends Module with IUConfig{
  val io = IO(new CbusIO)
  val flush = io.flush
  val special_in = io.specialIn
  val norm_in    = io.normIn
  val cp0_in     = io.cp0In
  val bju_in     = io.bjuIn
  val cbus_pipe0_cmplt    = norm_in.pipe0Sel || norm_in.divSel || special_in.instVld || cp0_in.instVld
  val cbus_pipe1_cmplt    = norm_in.pipe1Sel  || norm_in.multSel
  val cbus_pipe0_inst_vld = RegInit(false.B)
  val cbus_pipe1_inst_vld = RegInit(false.B)
  when(flush){
    cbus_pipe0_inst_vld := false.B
    cbus_pipe1_inst_vld := false.B
  }.otherwise{
    cbus_pipe0_inst_vld := cbus_pipe0_cmplt
    cbus_pipe1_inst_vld := cbus_pipe1_cmplt
  }

  // pipe 0
  // Mux the iid - to Rob
  val cbus_pipe0_src_iid = (Cat(Fill(7, norm_in.pipe0Sel)) & norm_in.pipe0Iid) |
    (Cat(Fill(7, norm_in.divSel)) & norm_in.pipe0Iid)|
    (Cat(Fill(7, special_in.instVld)) & special_in.iid)|
    (Cat(Fill(7, cp0_in.instVld)) & cp0_in.iid)

  val cbus_pipe0_src_abnormal = (special_in.instVld && special_in.abnormal) || (cp0_in.abnormal && cp0_in.instVld)

  val cbus_pipe0_src_expt_vld = (special_in.instVld && special_in.exptVld) || (cp0_in.exptVld && cp0_in.instVld)

  val cbus_pipe0_src_expt_vec = (Cat(Fill(7, special_in.instVld)) & special_in.exptVec) |
    (Cat(Fill(7, cp0_in.instVld)) & cp0_in.exptVec)

  val cbus_pipe0_src_high_hw_expt = special_in.instVld && special_in.highHwExpt

  val cbus_pipe0_src_bkpt = special_in.instVld && special_in.bkpt

  val cbus_pipe0_src_mtval = (Cat(Fill(7, special_in.instVld)) & special_in.mtval)|
    (Cat(Fill(7, cp0_in.instVld)) & cp0_in.mtval)

  val cbus_pipe0_src_flush = (cp0_in.instVld && cp0_in.flush) || (special_in.instVld && special_in.flush)

  val cbus_pipe0_src_efpc_vld =  cp0_in.instVld && cp0_in.efpcVld

  val cbus_pipe0_src_efpc = Cat(Fill(PcWidth, cp0_in.instVld)) & cp0_in.efpc


  val cbus_pipe0_expt_vld      = RegEnable( cbus_pipe0_src_expt_vld, cbus_pipe0_src_expt_vld,     cbus_pipe0_cmplt)
  val cbus_pipe0_expt_vec      = RegEnable( cbus_pipe0_src_expt_vec, cbus_pipe0_src_expt_vec,     cbus_pipe0_cmplt)
  val cbus_pipe0_high_hw_expt  = RegEnable( cbus_pipe0_src_high_hw_expt, cbus_pipe0_src_high_hw_expt, cbus_pipe0_cmplt)
  val cbus_pipe0_bkpt          = RegEnable( cbus_pipe0_src_bkpt,  cbus_pipe0_src_bkpt,        cbus_pipe0_cmplt)
  val cbus_pipe0_mtval         = RegEnable( cbus_pipe0_src_mtval, cbus_pipe0_src_mtval,       cbus_pipe0_cmplt)
  val cbus_pipe0_flush         = RegEnable( cbus_pipe0_src_flush, cbus_pipe0_src_flush,      cbus_pipe0_cmplt)
  val cbus_pipe0_efpc_vld      = RegEnable( cbus_pipe0_src_efpc_vld, cbus_pipe0_src_efpc_vld,     cbus_pipe0_cmplt)
  val cbus_pipe0_efpc          = RegEnable( cbus_pipe0_src_efpc, cbus_pipe0_src_efpc,         cbus_pipe0_cmplt)
  val cbus_pipe0_iid           = RegEnable( cbus_pipe0_src_iid, cbus_pipe0_src_iid,         cbus_pipe0_cmplt) // TODO CLK is different  @ct_iu_cbus @439
  val cbus_pipe0_abnormal      = RegEnable( cbus_pipe0_src_abnormal, cbus_pipe0_src_abnormal,     cbus_pipe0_cmplt)
  // out put
  io.out.pipe0Cmplt         := cbus_pipe0_inst_vld
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
  val cbus_pipe1_iid      = RegEnable(norm_in.pipe1Iid, norm_in.pipe1Iid, cbus_pipe1_cmplt)
  io.out.pipe1Cmplt       := cbus_pipe1_inst_vld
  io.out.pipe1Iid         := cbus_pipe1_iid
  // pipe 2
  io.out.pipe2Abnormal    := bju_in.sel
  io.out.pipe2BhtMispred  := bju_in.abnormal
  io.out.pipe2Cmplt       := bju_in.jmpMispred
  io.out.pipe2JmpMispred  := bju_in.bhtMispred
  io.out.pipe2Iid         := bju_in.iid
}
