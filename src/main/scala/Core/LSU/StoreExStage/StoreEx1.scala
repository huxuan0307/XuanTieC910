package Core.LSU.StoreExStage
import Core.Config.XLEN
import Core.LsuConfig
import chisel3._
import chisel3.util._

class Cp0ToStEx1 extends Bundle with LsuConfig{
  val lsuIcgEn = Bool()
  val clkEn  = Bool()
}
class RfPipe5ToStEx1 extends Bundle with LsuConfig{
  val gateclkSel  = Bool()
  val sdiqEntry   = UInt(LSIQ_ENTRY.W)
  val sel         = Bool()
  val src0        = UInt(XLEN.W)
  val srcv0Fr     = UInt(XLEN.W)
  val srcv0FrVld  = Bool()
  val srcv0Vld    = Bool()
  val srcv0Vr0    = UInt(XLEN.W)
  val srcv0Vr1    = UInt(XLEN.W)
  val stdata1Vld  = Bool()
  val unalign     = Bool()
}
//----------------------------------------------------------
class StoreEx1In extends Bundle with LsuConfig{
  val cp0In    = new Cp0ToStEx1
  val iduIn    = new RfPipe5ToStEx1
  val rtuFlush = Bool()
}
//==========================================================
//                        Output
//==========================================================
class StExtoIdu extends Bundle with LsuConfig{
  val sdiqEntry   = UInt(LSIQ_ENTRY.W)
  val sdiqFrzClr  = Bool()
  val sdiqPopVld  = Bool()
}
//----------------------------------------------------------
class StoreEx1Out extends Bundle with LsuConfig{
  val toIdu = new StExtoIdu
  val sdEx1Data           = UInt(XLEN.W)
  val sdEx1DataBypass     = UInt((XLEN*2).W)
  val sdEx1InstVld        = Bool()
  val rfEx1Sdid           = UInt(SDID_WIDTH.W)
  val rfInstVldShort      = Bool()
}
//==========================================================
//                          IO
//==========================================================
class StoreEx1IO extends Bundle with LsuConfig{
  val in  = Input(new StoreEx1In)
  val out = Output(new StoreEx1Out)
}

class StoreEx1 extends Module with LsuConfig{
  val io = IO(new StoreEx1IO)
  val sd_ex1_clk_en = io.in.iduIn.gateclkSel
  val sd_ex1_data_clk_en  = io.in.iduIn.gateclkSel && (!io.in.iduIn.srcv0Vld)
  val sd_ex1_vdata_clk_en = io.in.iduIn.gateclkSel && (io.in.iduIn.srcv0Vld)
  //==========================================================
  //                      encode sdid
  //==========================================================
  io.out.rfEx1Sdid := OHToUInt(io.in.iduIn.sdiqEntry)
  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+
  //| inst_vld |
  //+----------+
  io.out.rfInstVldShort := io.in.iduIn.gateclkSel
  val sd_rf_ex1_inst_vld = io.in.iduIn.sel && (!io.in.rtuFlush)
  val sd_ex1_inst_vld = RegInit(false.B)
  sd_ex1_inst_vld := sd_rf_ex1_inst_vld
  io.out.sdEx1InstVld := sd_ex1_inst_vld
  //+------+------+----------+------+
  //| sdid | secd | boundary | data |
  //+------+------+----------+------+
  val sd_ex1_sdid_oh        = RegInit(0.U(LSIQ_ENTRY.W))
  val sd_ex1_secd           = RegInit(false.B)
  val sd_ex1_boundary       = RegInit(false.B)
  val sd_ex1_srcv0_vld      = RegInit(false.B)
  val sd_ex1_srcv0_fr_vld   = RegInit(false.B)
  when(sd_ex1_clk_en){
    sd_ex1_sdid_oh       := io.in.iduIn.sdiqEntry
    sd_ex1_secd          := io.in.iduIn.stdata1Vld
    sd_ex1_boundary      := io.in.iduIn.unalign
    sd_ex1_srcv0_vld     := io.in.iduIn.srcv0Vld
    sd_ex1_srcv0_fr_vld  := io.in.iduIn.srcv0FrVld
  }
  val sd_ex1_src0_data      = RegInit(0.U(XLEN.W))
  val sd_ex1_srcv0_vr1_data = RegInit(0.U(XLEN.W))
  val sd_ex1_srcv0_vr0_data = RegInit(0.U(XLEN.W))
  val sd_ex1_srcv0_fr_data  = RegInit(0.U(XLEN.W))
  when(sd_ex1_data_clk_en){
    sd_ex1_src0_data := io.in.iduIn.src0
  }
  when(sd_ex1_vdata_clk_en){
    sd_ex1_srcv0_vr1_data := io.in.iduIn.srcv0Vr1
    sd_ex1_srcv0_vr0_data := io.in.iduIn.srcv0Vr0
    sd_ex1_srcv0_fr_data  := io.in.iduIn.srcv0Fr
  }
  //==========================================================
  //        data select
  //==========================================================
  val sd_ex1_data_64  = Wire(UInt(XLEN.W))
  sd_ex1_data_64 := Mux(sd_ex1_srcv0_vld ,
    Mux(sd_ex1_srcv0_fr_vld, sd_ex1_srcv0_fr_data,sd_ex1_srcv0_vr0_data),
    sd_ex1_src0_data)
  io.out.sdEx1Data := sd_ex1_data_64
  io.out.sdEx1DataBypass := Cat(Cat(Seq.fill(XLEN)("b0".U)) ,sd_ex1_data_64)
  //==========================================================
  //        Generage interface to idu
  //==========================================================
  io.out.toIdu.sdiqPopVld := sd_ex1_inst_vld && (!sd_ex1_boundary  ||   sd_ex1_secd)
  io.out.toIdu.sdiqFrzClr := sd_ex1_inst_vld &&   sd_ex1_boundary  &&  (!sd_ex1_secd)
  io.out.toIdu.sdiqEntry  := sd_ex1_sdid_oh
}
