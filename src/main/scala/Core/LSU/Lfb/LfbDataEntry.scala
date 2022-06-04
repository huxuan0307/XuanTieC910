package Core.LSU.Lfb
import Core.Config.XLEN
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class BiuToLfbDataEntry extends Bundle with LsuConfig {
  val rData = UInt((2*XLEN).W)
  val rLast = Bool()
  val rVld  = Bool()
}
class SnqToLfbDataEntry extends Bundle with LsuConfig {
  val bypassChgTag_x= Bool()
  val bypassInvalid_x= Bool()
}
//----------------------------------------------------------
class LfbDataEntryIn extends Bundle with LsuConfig with DCacheConfig {
  val cp0In   = new Cp0ToLfbEntry
  val biuAxiR = new BiuToLfbDataEntry
  val snqIn   = new SnqToLfbDataEntry
  val addrEntryLinefillAbort     = UInt(LFB_ADDR_ENTRY.W)
  val addrEntryLinefillPermit    = UInt(LFB_ADDR_ENTRY.W)
  val biuId2to0                   = UInt(2.W)
  val biuRIdHit                   = Bool()
  val dataEntryCreateDpVld_x      = Bool()
  val dataEntryCreateGateclkEn_x  = Bool()
  val dataEntryCreateVld_x        = Bool()
  val firstPassPtr                = Vec(4,Bool())
  val lfSmDataGrnt_x = Bool()
  val lfSmDataPopReq_x  = Bool()
  val rRespErr  = Bool()
  val rRespShare  = Bool()
}
//==========================================================
//                        Output
//==========================================================
class LfbDataEntryOut extends Bundle with LsuConfig with DCacheConfig {
  val addrId_v         = UInt(LFB_ADDR_ENTRY.W)
  val addrPopReq_v     = UInt(LFB_ADDR_ENTRY.W)
  val data_v           = UInt((XLEN*8).W)
  val dcacheShare_x    = Bool()
  val full_x           = Bool()
  val last_x           = Bool()
  val lfSmReq_x        = Bool()
  val vld_x            = Bool()
  val waitSurplus_x    = Bool()
}
//==========================================================
//                          IO
//==========================================================
class LfbDataEntryIO extends Bundle with LsuConfig {
  val in  = Input( new LfbDataEntryIn)
  val out = Output(new LfbDataEntryOut)
}

class LfbDataEntry extends Module with LsuConfig with DCacheConfig {
  val io = IO(new LfbDataEntryIO)
  //==========================================================
  //                 Generate interface
  //------------------input-----------------------------------
  //-----------create signal--------------
  val lfb_data_entry_create_vld        = io.in.dataEntryCreateVld_x
  val lfb_data_entry_create_dp_vld     = io.in.dataEntryCreateDpVld_x
  val lfb_data_entry_create_gateclk_en = io.in.dataEntryCreateGateclkEn_x
  //-----------grnt signal----------------
  val lfb_lf_sm_data_grnt = io.in.lfSmDataGrnt_x
  //-----------other signal---------------
  val lfb_lf_sm_data_pop_req = io.in.lfSmDataPopReq_x
  val snq_lfb_bypass_invalid = io.in.snqIn.bypassInvalid_x
  val snq_lfb_bypass_chg_tag = io.in.snqIn.bypassChgTag_x
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //-----------entry gateclk--------------
  //normal gateclk ,open when create || entry_vld
  val lfb_data_entry_vld  = RegInit(false.B)
  val lfb_data_entry_clk_en = lfb_data_entry_vld || lfb_data_entry_create_gateclk_en
  //-----------data gateclk---------------
  val lfb_data_entry_pass_data_vld = Wire(Bool())
//  val lfb_data_entry_pass_data2_vld = Wire(Bool())
//  val lfb_data_entry_pass_data3_vld = Wire(Bool())
  val lfb_data_entry_data_clk_en  = lfb_data_entry_pass_data_vld
//  val lfb_data_entry_data2_clk_en = lfb_data_entry_pass_data2_vld
//  val lfb_data_entry_data3_clk_en = lfb_data_entry_pass_data3_vld
  //==========================================================
  //                 Registers
  //==========================================================
  //+-----------+
  //| entry_vld |
  //+-----------+
  val lfb_data_entry_pop_vld = Wire(Bool())
  when(lfb_data_entry_pop_vld){
    lfb_data_entry_vld := false.B
  }.elsewhen(lfb_data_entry_create_vld){
    lfb_data_entry_vld := true.B
  }
  //+--------------------+
  //| addr_entry_id/r_id |
  //+--------------------+
  val lfb_data_entry_biu_id = RegInit(0.U(3.W))
  when(lfb_data_entry_create_dp_vld){
    lfb_data_entry_biu_id := io.in.biuId2to0
  }
  //+-------------+-----+------------+
  //| cache share | cnt | bypass_ptr |
  //+-------------+-----+------------+
  val lfb_data_entry_cnt     = RegInit(0.U(2.W))
  val lfb_data_entry_last    = RegInit(false.B)
  val lfb_data_entry_pass_ptr = Seq.fill(4)(RegInit(false.B))
  when(lfb_data_entry_create_dp_vld){
    lfb_data_entry_cnt      := (0.U(2.W))
    lfb_data_entry_last     := io.in.biuAxiR.rLast
    for(i<-1 until 4){
      lfb_data_entry_pass_ptr(i) := io.in.firstPassPtr(i-1)  // ptr shift
    }
    lfb_data_entry_pass_ptr(0) := io.in.firstPassPtr(3)
  }.elsewhen(lfb_data_entry_pass_data_vld){
    lfb_data_entry_cnt      := lfb_data_entry_cnt + (1.U(2.W)) // axi cnt + 1
    lfb_data_entry_last     := io.in.biuAxiR.rLast
    for(i<-1 until 4){
      lfb_data_entry_pass_ptr(i) := io.in.firstPassPtr(i-1)  // ptr shift
    }
    lfb_data_entry_pass_ptr(0) := io.in.firstPassPtr(3)
  }
  val lfb_data_entry_dcache_share = RegInit(false.B)
  when(lfb_data_entry_create_dp_vld){
    lfb_data_entry_dcache_share := io.in.rRespShare
  }.elsewhen(lfb_data_entry_pass_data_vld){
    lfb_data_entry_dcache_share := io.in.rRespShare
  }.elsewhen(snq_lfb_bypass_chg_tag){
    lfb_data_entry_dcache_share := true.B
  }
  //+-----------+
  //| bus error |
  //+-----------+
  val lfb_data_entry_bus_err = RegInit(false.B)
  when(lfb_data_entry_create_dp_vld){
    lfb_data_entry_bus_err := io.in.rRespErr
  }.elsewhen(lfb_data_entry_pass_data_vld && io.in.rRespErr){
    lfb_data_entry_bus_err := true.B
  }
  //+------+
  //| data |
  //+------+
  // todo entry data is 512bits, and divid into 4 part,(511,384)(383,256)(255,127)(128,0)
  // 4 parts MAYBE not only has 4 different clk, but also have one clk delay between 4 clk, because it from AXI-R-burst
  val lfb_data_entry_data_vec = Seq.fill(4)(RegInit(0.U((2*XLEN).W)))
  val lfb_data_entry_pass_data_vld_vec =  Seq.fill(4)(Wire(Bool()))
  for(i<- 0 until 4){
    when( lfb_data_entry_pass_data_vld_vec(i) ){
      lfb_data_entry_data_vec(i) := io.in.biuAxiR.rData
    }
  }
  val lfb_data_entry_data = VecInit(lfb_data_entry_data_vec).asUInt//Cat(lfb_data_entry_data_vec(3),lfb_data_entry_data_vec(2),lfb_data_entry_data_vec(1),lfb_data_entry_data_vec(0) )
  //+-------------------+
  //| lf_sm_req_success |
  //+-------------------+
  val lfb_data_entry_lf_sm_req_success = RegInit(false.B)
  when(lfb_data_entry_create_dp_vld){
    lfb_data_entry_lf_sm_req_success := false.B
  }.elsewhen(lfb_lf_sm_data_grnt){
    lfb_data_entry_lf_sm_req_success := true.B
  }
  //==========================================================
  //                        Wires
  //==========================================================
  //---------------------full signal--------------------------
  val lfb_data_entry_full = lfb_data_entry_vld  &&  lfb_data_entry_last
  //wait surplus means, though this entry is vld, but it hasn't received all data
  val lfb_data_entry_wait_surplus = lfb_data_entry_vld && !lfb_data_entry_last
  //------------------pass data signal------------------------
  val lfb_data_entry_r_id_hit=lfb_data_entry_vld && io.in.biuAxiR.rVld && io.in.biuRIdHit  &&
    (lfb_data_entry_biu_id === io.in.biuId2to0)
  lfb_data_entry_pass_data_vld := lfb_data_entry_create_dp_vld || lfb_data_entry_r_id_hit

  for(i<- 0 until 4){
    lfb_data_entry_pass_data_vld_vec(i) := lfb_data_entry_create_dp_vld && io.in.firstPassPtr(i) ||
      lfb_data_entry_r_id_hit && lfb_data_entry_pass_ptr(i)
  }
  //==========================================================
  //                 Generate req/pop signal
  //==========================================================
  //------------------last signal---------------------------
  val lfb_data_entry_pass_3times = lfb_data_entry_vld && (lfb_data_entry_cnt === "d2".U)
  val lfb_data_entry_finish_line   = lfb_data_entry_vld  &&  (lfb_data_entry_cnt ===  "d3".U)  &&  lfb_data_entry_last
  val lfb_data_entry_finish_once   = lfb_data_entry_vld  &&  (lfb_data_entry_cnt ===  "d0".U)  &&  lfb_data_entry_last
  val lfb_data_entry_pass_data_last  = lfb_data_entry_vld  &&  io.in.biuAxiR.rLast   &&  lfb_data_entry_pass_data_vld
  val lfb_data_entry_addr_id = RegInit(0.U(LSIQ_ENTRY.W))
  lfb_data_entry_addr_id := UIntToOH(lfb_data_entry_biu_id)
  val lfb_data_entry_linefill_permit = (lfb_data_entry_addr_id & io.in.addrEntryLinefillPermit).orR
  val lfb_data_entry_linefill_abort = (lfb_data_entry_addr_id & io.in.addrEntryLinefillAbort).orR
  val lfb_data_entry_abort = lfb_data_entry_finish_once || snq_lfb_bypass_invalid || lfb_data_entry_finish_line && (lfb_data_entry_linefill_abort || lfb_data_entry_linefill_permit && (lfb_data_entry_bus_err || !io.in.cp0In.lsuDcacheEn))
  val lfb_data_entry_addr_pop_req = lfb_data_entry_addr_id & Cat(Seq.fill(LFB_ADDR_ENTRY)(lfb_data_entry_abort))
  //------------------lf req signal---------------------------
  //if get all data already, or get last data this cycle, it will request linefill
  //state machine
  val lfb_data_entry_lf_sm_req  = lfb_data_entry_vld && !lfb_data_entry_lf_sm_req_success && !lfb_data_entry_bus_err  &&
    lfb_data_entry_linefill_permit && io.in.cp0In.lsuDcacheEn &&
    (lfb_data_entry_finish_line || lfb_data_entry_pass_3times && lfb_data_entry_pass_data_last &&  !io.in.rRespErr)
  lfb_data_entry_pop_vld := lfb_data_entry_abort || lfb_lf_sm_data_pop_req
  //==========================================================
  //                 Generate interface
  //==========================================================
  io.out.vld_x  := lfb_data_entry_vld
  io.out.addrId_v  := lfb_data_entry_addr_id
  io.out.dcacheShare_x  :=lfb_data_entry_dcache_share
  io.out.data_v  :=lfb_data_entry_data
  io.out.last_x    :=lfb_data_entry_last
  io.out.waitSurplus_x  :=lfb_data_entry_wait_surplus
  io.out.full_x  :=lfb_data_entry_full

  io.out.addrPopReq_v := lfb_data_entry_addr_pop_req
  io.out.lfSmReq_x := lfb_data_entry_lf_sm_req







}
