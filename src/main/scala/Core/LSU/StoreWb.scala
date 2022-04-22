package Core.LSU
import Core.ExceptionConfig.ExceptionVecWidth
import Core.IU.Cp0.Define.Exceptions.ExceptionVec
import Core.IntConfig.{NumLogicRegsBits, NumPhysicRegs, XLEN}
import Core.LsuConfig
import Core.ROBConfig.IidWidth
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToStWb extends Bundle with LsuConfig{
  val icgEn = Bool()
  val clkEn  = Bool()
}
class WmbToWb extends Bundle with LsuConfig{
  val bkptaData = Bool()
  val bkptbData = Bool()
  val cmpltReq  = Bool()
  val iid        = UInt(IidWidth.W)
  val instFlush = Bool()
  val specFail  = Bool()
}
//----------------------------------------------------------
class StoreWbIn extends Bundle with LsuConfig{
  val cp0In  = new Cp0ToStWb
  val ctrlStClk = Bool()
  //{
  val stDaIn = new StDaToWb
  val bkptaData = Bool()
  val bkptbData = Bool()
  val instVld   = Bool()
  val iid       = UInt(IidWidth.W)
  //}
  val rtuFlush = Bool()
  val wmbIn   = new WmbToWb
}
//==========================================================
//                        Output
//==========================================================
class StWbToRtu extends Bundle with LsuConfig {
  val abnormal      = Bool()
  val bkptaData     = Bool()
  val bkptbData     = Bool()
  val cmplt         = Bool()
  val exptVec       = UInt(ExceptionVecWidth.W)
  val exptVld       = Bool()
  val flush         = Bool()
  val iid           = UInt(IidWidth.W)
  val mtval         = UInt(PA_WIDTH.W)
  val noSpechit     = Bool()
  val noSpecMispred = Bool()
  val noSpecMiss    = Bool()
  val specFail      = Bool()
}
//----------------------------------------------------------
class StoreWbOut extends Bundle with LsuConfig{
  val toRtu  = new StWbToRtu
  val instVld       = Bool()
  val wmbCmpltGrnt = Bool()
}
//==========================================================
//                          IO
//==========================================================
class StoreWbIO extends Bundle with LsuConfig{
  val in  = Input(new StoreWbIn)
  val out = Output(new StoreWbOut)
}
class StoreWb extends Module with LsuConfig{
  val io = IO(new StoreWbIO)
  //==========================================================
  //                 arbitrate WB stage request
  //==========================================================
  //------------------complete part---------------------------
  //-----------grant signal---------------
  val st_wb_da_cmplt_grnt = io.in.stDaIn.cmpltReq
  io.out.wmbCmpltGrnt := !io.in.stDaIn.cmpltReq && io.in.wmbIn.cmpltReq
  //-----------signal select--------------
  val st_wb_pre_inst_vld = io.in.stDaIn.cmpltReq || io.in.wmbIn.cmpltReq
  val st_wb_pre_inst_flush = io.out.wmbCmpltGrnt && io.in.wmbIn.instFlush
  val st_wb_pre_spec_fail  = st_wb_da_cmplt_grnt && io.in.stDaIn.specFail || io.out.wmbCmpltGrnt && io.in.wmbIn.specFail
  val st_wb_pre_bkpta_data = st_wb_da_cmplt_grnt && io.in.bkptaData || io.out.wmbCmpltGrnt && io.in.wmbIn.bkptaData
  val st_wb_pre_bkptb_data = st_wb_da_cmplt_grnt && io.in.bkptbData || io.out.wmbCmpltGrnt && io.in.wmbIn.bkptbData
  val st_wb_pre_vstart_vld = false.B
  val st_wb_pre_expt_vld   = Wire(Bool())
  val st_wb_pre_flush      = st_wb_pre_inst_flush || st_wb_pre_spec_fail || st_wb_pre_vstart_vld && !st_wb_pre_expt_vld
  st_wb_pre_expt_vld   := st_wb_da_cmplt_grnt   &&  io.in.stDaIn.exptVld
  val st_wb_pre_iid        = Mux(st_wb_da_cmplt_grnt,io.in.iid,0.U(IidWidth.W)) | Mux(io.out.wmbCmpltGrnt,io.in.wmbIn.iid,0.U(IidWidth.W))
  //for spec fail prediction
  val st_wb_pre_no_spec_miss    = st_wb_da_cmplt_grnt &&  io.in.stDaIn.noSpecMiss
  val st_wb_pre_no_spec_hit     = st_wb_da_cmplt_grnt &&  io.in.stDaIn.noSpecHit
  val st_wb_pre_no_spec_mispred = st_wb_da_cmplt_grnt &&  io.in.stDaIn.noSpecMispred
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val st_wb_cmplt_clk_en =   io.in.instVld  ||  io.in.wmbIn.cmpltReq
  val st_wb_expt_clk_en  =   io.in.instVld  &&  io.in.stDaIn.exptVld
  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------complete part---------------------------
  //+----------+----------+
  //| inst_vld | expt_vld |
  //+----------+----------+
  val st_wb_inst_vld = RegInit(false.B)
  when(io.in.rtuFlush){
    st_wb_inst_vld := false.B
  }.elsewhen(st_wb_pre_inst_vld){
    st_wb_inst_vld := true.B
  }.otherwise{
    st_wb_inst_vld := false.B
  }
  io.out.instVld := st_wb_inst_vld
  //+-----+-------+-----------+-----------+
  //| iid | flush | spec_fail | bkpt_data |
  //+-----+-------+-----------+-----------+
  val st_wb_expt_vld        = RegInit(false.B)
  val st_wb_iid             = RegInit(0.U(IidWidth.W))
  val st_wb_spec_fail       = RegInit(false.B)
  val st_wb_flush           = RegInit(false.B)
  val st_wb_bkpta_data      = RegInit(false.B)
  val st_wb_bkptb_data      = RegInit(false.B)
  val st_wb_no_spec_miss    = RegInit(false.B)
  val st_wb_no_spec_hit     = RegInit(false.B)
  val st_wb_no_spec_mispred = RegInit(false.B)
  when(st_wb_pre_inst_vld){
    st_wb_expt_vld        := st_wb_pre_expt_vld
    st_wb_iid             := st_wb_pre_iid
    st_wb_spec_fail       := st_wb_pre_spec_fail
    st_wb_flush           := st_wb_pre_flush
    st_wb_bkpta_data      := st_wb_pre_bkpta_data
    st_wb_bkptb_data      := st_wb_pre_bkptb_data
    st_wb_no_spec_miss    := st_wb_pre_no_spec_miss
    st_wb_no_spec_hit     := st_wb_pre_no_spec_hit
    st_wb_no_spec_mispred := st_wb_pre_no_spec_mispred
  }
  //+----------+----------+
  //| expt_vec | mt_value |
  //+----------+----------+
  val st_wb_expt_vec = RegInit(0.U(ExceptionVecWidth.W))
  val st_wb_mt_value = RegInit(0.U(PA_WIDTH.W))
  when(io.in.instVld  &&  io.in.stDaIn.exptVld){
    st_wb_expt_vec := io.in.stDaIn.exptVec
    st_wb_mt_value := io.in.stDaIn.mtValue
  }
  //==========================================================
  //                 Generate interface to rtu
  //==========================================================
  //------------------complete part---------------------------
  io.out.toRtu.cmplt         := st_wb_inst_vld
  io.out.toRtu.iid           := st_wb_iid
  io.out.toRtu.exptVld       := st_wb_expt_vld
  io.out.toRtu.exptVec       := st_wb_expt_vec
  io.out.toRtu.specFail      := st_wb_spec_fail
  io.out.toRtu.flush         := st_wb_flush
  io.out.toRtu.abnormal      := st_wb_expt_vld  ||  st_wb_flush

  io.out.toRtu.mtval         := st_wb_mt_value
  io.out.toRtu.bkptaData     := st_wb_bkpta_data
  io.out.toRtu.bkptbData     := st_wb_bkptb_data

  io.out.toRtu.noSpecMiss    := st_wb_no_spec_miss
  io.out.toRtu.noSpechit     := st_wb_no_spec_hit
  io.out.toRtu.noSpecMispred := st_wb_no_spec_mispred

}
