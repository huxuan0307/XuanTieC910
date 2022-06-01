package Core.LSU.Vb

import Core.BiuID.BIU_VB_ID_T
import Core.IntConfig.XLEN
import Core.LSU.CacheRelated.IccToVb
import Core.LSU.Lfb.{LfbToVb, VbToLfb}
import Core.LSU.StoreExStage.StDaToVb
import Core.{DCacheConfig, LsuConfig}
import chisel3.Module
import chisel3._
import chisel3.util._

trait DataEntryState {
  def IDLE            = 0.U(4.W)
  def GET_VB_DATA     = "b1000".U
  def REQ_WRITE_ADDR  = "b1001".U
  def REQ_WRITE_DATA  = "b1010".U
  def GRNT_WRITE_DATA = "b1011".U
  def WAIT_REQ        = "b0111".U
  def GET_SNQ_DATA    = "b0100".U
  def REQ_CD_CHANNEL  = "b0101".U
//  def VB_NOP          = 4'b1111,
}
//==========================================================
//                        Input
//==========================================================
class VbsdbDataEntryIn extends Bundle with LsuConfig with DCacheConfig {
  val cp0LsuIcgEn = Bool()
  val ldDaIn = new Bundle() {
    val data256        = UInt(256.W)
    val borrowVb_x = Bool()
  }
  val snqIn = new Bundle() {
    val dataBypassHit_x = Bool()
    val bypassInvalid_x = Bool()
    val bypassReadonce = Bool()
    val bypassStart_x = Bool()
  }
  val sdb = new Bundle() {
    val createDataOrder = UInt(2.W)
    val createEn_x       = Bool()
    val entryDataIndex  = UInt(4.W)
    val invEn_x          = Bool()
  }
  val vbIn = new Bundle() {
    val dataEntry = new Bundle() {
      val biuReqSuccess_x   = Bool()
      val createDpVld_x     = Bool()
      val createGateclkEn_x = Bool()
      val createVld_x        = Bool()
      val wdSmGrnt_x        = Bool()
    }
    val rclSm = new Bundle() {
      val addrId              = UInt(VB_ADDR_ENTRY.W)
      val dataDcacheDirty    = Bool()
      val dataSetDataDone_x = Bool()
      val inv                  = Bool()
      val lfbCreate           = Bool()
    }
    val wdSm = new Bundle() {
      val dataBias     = UInt(4.W)
      val dataPopReq_x = Bool()
    }
  }
  val wdSmDataPopReq_x = Bool()
}
//==========================================================
//                        Output
//==========================================================
class VbsdbDataEntryOut extends Bundle with LsuConfig with DCacheConfig {
  val sdb = new Bundle() {
    val  dataVld_x    = Bool()
    val entryAvail_x = Bool()
    val entryData_v  = UInt((2*XLEN).W)
    val vld_x        = Bool()
  }
  val toVb = new Bundle() {
    val addr_id_v            = UInt(VB_ADDR_ENTRY.W)
    val biu_req_x            = Bool()
    val bypass_pop_x         = Bool()
    val dirty_x              = Bool()
    val inv_x                = Bool()
    val lfb_create_x         = Bool()
    val normal_pop_x         = Bool()
    val req_success_x        = Bool()
    val vld_x                = Bool()
    val wd_sm_req_x          = Bool()
    val write_data128_v      = UInt((XLEN*2).W)
    val sdb_data_entry_vld_x = Bool()
  }

}
//==========================================================
//                          IO
//==========================================================
class VbsdbDataEntryIO extends Bundle {
  val in  = Input( new VbsdbDataEntryIn)
  val out = Output(new VbsdbDataEntryOut)
}
class VbsdbDataEntry extends Module with DCacheConfig with LsuConfig with DataEntryState{
  val io = IO(new VbsdbDataEntryIO)
  //==========================================================//==========================================================

  //                 Generate interface
  //------------------input-----------------------------------
  //-----------create signal--------------
  val vb_data_entry_create_vld        = io.in.vbIn.dataEntry.createVld_x
  val vb_data_entry_create_dp_vld     = io.in.vbIn.dataEntry.createDpVld_x
  val vb_data_entry_create_gateclk_en = io.in.vbIn.dataEntry.createGateclkEn_x
  //-----------grnt signal----------------
  val vb_data_entry_biu_req_success = io.in.vbIn.dataEntry.biuReqSuccess_x
  val vb_data_entry_wd_sm_grnt      = io.in.vbIn.dataEntry.wdSmGrnt_x
  //-----------other signal---------------
  val ld_da_vb_borrow_vb            = io.in.ldDaIn.borrowVb_x
  val vb_rcl_sm_data_set_data_done  = io.in.vbIn.rclSm.dataSetDataDone_x
  val vb_wd_sm_data_pop_req         = io.in.wdSmDataPopReq_x
  //----------- for snq--------------
  val snq_vb_bypass_start     = io.in.snqIn.bypassStart_x
  val snq_vb_bypass_invalid   = io.in.snqIn.bypassInvalid_x
  val snq_data_bypass_hit     = io.in.snqIn.dataBypassHit_x
  val sdb_create_en           = io.in.sdb.createEn_x
  val sdb_inv_en              = io.in.sdb.invEn_x
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //-----------entry gateclk--------------
  //normal gateclk ,open when create || entry_vld
//  val vb_data_entry_create_clk_en = vb_data_entry_create_gateclk_en
//  val vb_data_entry_clk_en       = vb_data_entry_vld  ||  sdb_vld    ||  sdb_create_en    ||  vb_data_entry_create_gateclk_en
//  val vb_data_entry_data0_clk_en = vb_data_entry_pass_data0_vld
//  val vb_data_entry_data1_clk_en = vb_data_entry_pass_data1_vld
  //==========================================================
  //                 Registers
  //==========================================================
  //+-------+
  //| state |
  //+-------+
  val vb_data_entry_state      = RegInit(IDLE)
  val vb_data_entry_vld        = vb_data_entry_state(3)
  val vb_data_entry_biu_req    = vb_data_entry_state === REQ_WRITE_ADDR
  val vb_data_entry_wd_sm_req  = vb_data_entry_state === REQ_WRITE_DATA
  //+-----------+
  //| data_bias |
  //+-----------+
  val vb_data_entry_data_bias = RegInit(1.U(2.W))
  when(vb_data_entry_create_dp_vld || sdb_create_en){
    vb_data_entry_data_bias := (1.U(2.W))
  }.elsewhen(ld_da_vb_borrow_vb){
    vb_data_entry_data_bias := Reverse(vb_data_entry_data_bias)
  }
  //+-------+------------+-----+---------+
  //| dirty | lfb_create | inv | addr_id |
  //+-------+------------+-----+---------+
  //dirty is read from cache
  //lfb create is read from addr entry
  val vb_data_entry_dirty     = RegInit(false.B)
  val vb_data_entry_lfb_create= RegInit(false.B)
  val vb_data_entry_inv       = RegInit(false.B)
  val vb_data_entry_addr_id   = RegInit(0.U(VB_ADDR_ENTRY))
  when(vb_data_entry_create_dp_vld){
    vb_data_entry_dirty      := io.in.vbIn.rclSm.dataDcacheDirty
    vb_data_entry_lfb_create := io.in.vbIn.rclSm.lfbCreate
    vb_data_entry_inv        := io.in.vbIn.rclSm.inv
    vb_data_entry_addr_id    := io.in.vbIn.rclSm.addrId
  }
  //+------+
  //| data |
  //+------+
  val vb_data_entry_pass_data_vld = Seq.fill(2)(Wire(Bool()))
  val vb_data_entry_data_vec  = Seq.fill(2)(RegInit(0.U(256.W)))
  val vb_data_entry_data = VecInit(vb_data_entry_data_vec).asUInt
  for(i<- 0 until(2)){
    when(vb_data_entry_pass_data_vld(i)){
      vb_data_entry_data_vec(i)  := io.in.ldDaIn.data256
    }
  }
  //for sdb return order
  //+--------------+
  //| return order |
  //+--------------+
  val sdb_start_bias = RegInit(0.U(2.W))
  when(sdb_create_en || snq_vb_bypass_start){
    sdb_start_bias := io.in.sdb.createDataOrder
  }
  //for readonce record
  val sdb_bypass_readonce= RegInit(false.B)
  when(sdb_create_en){
    sdb_bypass_readonce := false.B
  }.elsewhen(snq_vb_bypass_start){
    sdb_bypass_readonce := io.in.snqIn.bypassReadonce
  }
  //==========================================================
  //                    State machine
  //==========================================================
  val vb_data_entry_pop_vld = vb_wd_sm_data_pop_req
  switch(vb_data_entry_state) {
    is(IDLE) {
      when(vb_data_entry_create_vld) {
        vb_data_entry_state := GET_VB_DATA
      }.elsewhen(sdb_create_en) {
        vb_data_entry_state := GET_SNQ_DATA
      }
    }
    is(GET_VB_DATA) {
      when(vb_rcl_sm_data_set_data_done) {
        vb_data_entry_state := REQ_WRITE_ADDR
      }
    }
    is(REQ_WRITE_ADDR) {
      when(vb_data_entry_biu_req_success) {
        vb_data_entry_state := REQ_WRITE_DATA
      }.elsewhen(snq_data_bypass_hit) {
        vb_data_entry_state := WAIT_REQ
      }
    }
    is(REQ_WRITE_DATA) {
      when(vb_data_entry_wd_sm_grnt) {
        vb_data_entry_state := GRNT_WRITE_DATA
      }
    }
    is(GRNT_WRITE_DATA) {
      when(vb_data_entry_pop_vld) {
        vb_data_entry_state := IDLE
      }
    }
    is(WAIT_REQ) {
      when(snq_vb_bypass_start) {
        vb_data_entry_state := REQ_CD_CHANNEL
      }.elsewhen(snq_vb_bypass_invalid) {
        vb_data_entry_state := IDLE
      }
    }
    is(GET_SNQ_DATA) {
      when(vb_data_entry_pass_data_vld.reduce(_ || _)) {
        vb_data_entry_state := REQ_CD_CHANNEL
      }
    }
    is(REQ_CD_CHANNEL) {
      when(sdb_inv_en & sdb_bypass_readonce) {
        vb_data_entry_state := REQ_WRITE_ADDR
      }.elsewhen(sdb_inv_en) {
        vb_data_entry_state := IDLE
      }
    }
  }
  //==========================================================
  //                 State 1 : get data
  //==========================================================
  //when sdb get data,critial first
  val sdb_vld = !vb_data_entry_state(3) && vb_data_entry_state(2)
  val sdb_bypass_reverse = sdb_vld && sdb_start_bias(1).asBool
  for(i<- 0 until(2)){
    vb_data_entry_pass_data_vld(i)  := ld_da_vb_borrow_vb && (vb_data_entry_data_bias(i) && sdb_bypass_reverse)
  }
  //==========================================================
  //                 State 4 : grnt write data
  //==========================================================
  val bias_ptr = Seq.fill(4)(Wire(Bool()))
  bias_ptr.zipWithIndex.foreach {
    case (ptr, i) =>
      ptr := io.in.vbIn.wdSm.dataBias(i)
  }
  val data_vec =Seq.fill(4)(Wire(UInt(128.W)))
  data_vec.zipWithIndex.foreach {
    case (data, i) =>
      data := vb_data_entry_data((i+1)*128-1,i*128)
  }
  val vb_data_entry_write_data128 = bias_ptr.zip(data_vec).map {
    case (ptr, data) =>
      Mux(ptr, data, 0.U((128).W))
  }.reduce(_ | _)
  //==========================================================
  //                 Snoop signal
  //==========================================================
  val vb_sdb_data_entry_vld = vb_data_entry_state(3,2).orR
  val sdb_data_vld          =(vb_data_entry_state === REQ_CD_CHANNEL)
  val sdb_entry_avail       = !vb_sdb_data_entry_vld && !vb_data_entry_create_vld
  val sdb_return_order = RegInit(0.U(4.W))
  for(i<-0 until(4)){
    when(sdb_start_bias === i.U){
      sdb_return_order := ShiftRegister(io.in.sdb.entryDataIndex,i,sdb_start_bias === i.U)
    }
  }
  val sdb_bias_ptr = Seq.fill(4)(Wire(Bool()))
  sdb_bias_ptr.zipWithIndex.foreach {
    case (ptr, i) =>
      ptr := sdb_return_order(i)
  }
  val sdb_entry_data = sdb_bias_ptr.zip(data_vec).map {
    case (ptr, data) =>
      Mux(ptr, data, 0.U((128).W))
  }.reduce(_ | _)
  val vb_data_entry_req_success = (vb_data_entry_state === REQ_WRITE_ADDR) && vb_data_entry_biu_req_success ||
    (vb_data_entry_state === REQ_WRITE_DATA) || (vb_data_entry_state === GRNT_WRITE_DATA)
  val vb_data_bypass_pop = (vb_data_entry_state === WAIT_REQ) && (snq_vb_bypass_start && !io.in.snqIn.bypassReadonce || snq_vb_bypass_invalid)
  val vb_data_normal_pop = (vb_data_entry_state === GRNT_WRITE_DATA) && vb_data_entry_pop_vld
  //==========================================================
  //                 Generate interface
  //==========================================================
  //------------------output----------------------------------
  io.out.toVb.sdb_data_entry_vld_x  := vb_sdb_data_entry_vld
  io.out.toVb.vld_x                 := vb_data_entry_vld
  io.out.toVb.dirty_x               := vb_data_entry_dirty
  io.out.toVb.lfb_create_x          := vb_data_entry_lfb_create
  io.out.toVb.inv_x                 := vb_data_entry_inv
  io.out.toVb.addr_id_v             := vb_data_entry_addr_id
  //-----------request--------------------
  io.out.toVb.biu_req_x    := vb_data_entry_biu_req
  io.out.toVb.wd_sm_req_x  := vb_data_entry_wd_sm_req
  //-----------other signal---------------
  io.out.toVb.write_data128_v :=vb_data_entry_write_data128
  io.out.toVb.req_success_x   :=vb_data_entry_req_success
  io.out.toVb.bypass_pop_x    :=vb_data_bypass_pop
  io.out.toVb.normal_pop_x    :=vb_data_normal_pop
  //----------- for snq--------------
  io.out.sdb := DontCare

}
