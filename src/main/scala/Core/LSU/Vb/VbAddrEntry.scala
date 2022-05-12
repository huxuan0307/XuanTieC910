package Core.LSU.Vb
import Core.IntConfig.XLEN
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._


//==========================================================
//                        Input
//==========================================================
class Cp0ToVbAddrEntry extends Bundle {
  val lsuIcgEn = Bool()
  val yyClkEn = Bool()
}
class VbAddrCtrl extends Bundle with LsuConfig {
  // create signal - from create ptr
  val createData_x = Bool()
  val createGateclk_en_x = Bool()
  val createVld_x = Bool()
  // pop signal
  val dataPop_x = Bool()
  val pop_x = Bool()
  val id = UInt(VB_DATA_ENTRY.W)
  val feedbackVld_x = Bool()
  // other module create date path
  val iccCreateDpVld_x = Bool()
  val lfbCreateDpVld_x = Bool()
  val wmbCreateDpVld_x = Bool()
  // rcl? state machine
  val rcl_sm_addr_grnt_x = Bool()
  val rcl_sm_done_x = Bool()
}
class WmbToVbAddrEntry extends Bundle with LsuConfig with DCacheConfig {
  val addrTto6     =  UInt((PA_WIDTH-OFFSET_WIDTH).W)
  val inv           = Bool()
  val setWayMode  = Bool()
  val writePtrEncode = UInt(VB_DATA_ENTRY.W)
  val writeReqAddr   = UInt(PA_WIDTH.W)
}
//----------------------------------------------------------
class VbAddrEntryIn extends Bundle with LsuConfig with DCacheConfig {
  val cp0In = new Cp0ToVbAddrEntry
  val evictReqSuccess = Bool()
  val iccIn = new Bundle() {
    val addrTto6 = UInt((PA_WIDTH-OFFSET_WIDTH).W) // top to 6 = 40 to 6
    val inv = Bool() // invalid
  }
  val lfbIn = new Bundle() {
    val addrTto6 = UInt((PA_WIDTH-OFFSET_WIDTH).W)
    val id = UInt(LFB_ID_WIDTH.W)
  }
  val pfuBiuReqAddr = UInt(PA_WIDTH.W)
  val rbBiuReqAddr = UInt(PA_WIDTH.W)
//  val padYyIcgScanEn
  val snqIn = new Bundle() {
   val bypassAddrTto6 = UInt((PA_WIDTH-OFFSET_WIDTH).W)
   val createAddr = UInt(PA_WIDTH.W)
  }
  val stDaFeedbackAddrTto14 = UInt((PA_WIDTH-OFFSET_WIDTH-INDEX_WIDTH).W)
  val vbIn = new VbAddrCtrl
  val wmbIn = new WmbToVbAddrEntry

}
//==========================================================
//                        Output
//==========================================================
class VbAddrEntryOut extends Bundle with LsuConfig with DCacheConfig {
 val addr_tto6_v             = UInt((PA_WIDTH-OFFSET_WIDTH).W)
 val db_id_v                 = UInt(VB_DATA_ENTRY.W)
 val dep_remove_x            = Bool()
 val inv_x                   = Bool()
 val lfb_create_x            = Bool()
 val lfb_vb_req_hit_idx_x    = Bool()
 val pfu_biu_req_hit_idx_x   = Bool()
 val rb_biu_req_hit_idx_x    = Bool()
 val rcl_sm_req_x            = Bool()
 val set_way_mode_x          = Bool()
 val snq_bypass_hit_x        = Bool()
 val snq_create_hit_idx_x    = Bool()
 val snq_start_wait_x        = Bool()
 val source_id_v             = UInt(VB_DATA_ENTRY.W)
 val vld_x                   = Bool()
 val wmb_create_x            = Bool()
 val wmb_write_req_hit_idx_x = Bool()
}
//==========================================================
//                          IO
//==========================================================
class VbAddrEntryIO extends Bundle with LsuConfig {
  val in  = Input( new VbAddrEntryIn)
  val out = Output(new VbAddrEntryOut)
}

class VbAddrEntry extends Module with DCacheConfig with LsuConfig {
  val io = IO(new VbAddrEntryIO)
  //------------------input-----------------------------------
  //-----------create signal--------------
  val vb_addr_entry_create_vld        = io.in.vbIn.createVld_x
  val vb_addr_entry_icc_create_dp_vld = io.in.vbIn.iccCreateDpVld_x
  val vb_addr_entry_lfb_create_dp_vld = io.in.vbIn.lfbCreateDpVld_x
  val vb_addr_entry_wmb_create_dp_vld = io.in.vbIn.wmbCreateDpVld_x
  val vb_addr_entry_create_gateclk_en = io.in.vbIn.createGateclk_en_x
  //-----------grnt signal----------------
  val vb_rcl_sm_addr_grnt  = io.in.vbIn.rcl_sm_addr_grnt_x
  //-----------other signal---------------
  val vb_addr_entry_feedback_vld  = io.in.vbIn.feedbackVld_x
  val vb_rcl_sm_done = io.in.vbIn.rcl_sm_done_x
  val vb_addr_entry_create_data = io.in.vbIn.createData_x
  val vb_addr_entry_pop  = io.in.vbIn.pop_x
  val vb_addr_data_pop   = io.in.vbIn.dataPop_x

  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //-----------create gateclk-------------
  val vb_addr_entry_create_clk_en  = vb_addr_entry_create_gateclk_en;
  //-----------entry gateclk--------------
  //normal gateclk ,open when create || entry_vld
  val vb_addr_entry_vld = RegInit(false.B)
  val vb_addr_entry_clk_en   = vb_addr_entry_vld  ||  vb_addr_entry_create_clk_en
  //--------feedback gateclk--------------
  val vb_addr_entry_feedback_clk_en =  vb_addr_entry_feedback_vld  ||  vb_addr_entry_create_clk_en
  //==========================================================
  //                 Register
  //==========================================================
  //+-----------+
  //| entry_vld |
  //+-----------+
  val vb_addr_entry_pop_vld = Wire(Bool())
  when(vb_addr_entry_pop_vld){
    vb_addr_entry_vld := false.B
  }.elsewhen(vb_addr_entry_create_vld){
    vb_addr_entry_vld := true.B
  }
  //+------+
  //| addr |
  //+------+
  val vb_addr_entry_addr_tag   = RegInit(0.U((INDEX_WIDTH-1).W))
  val vb_addr_entry_addr_index = RegInit(0.U((TAG_WIDTH+1).W))
  val vb_addr_entry_addr_tto6  = Cat(vb_addr_entry_addr_tag,vb_addr_entry_addr_index)
  when(vb_addr_entry_icc_create_dp_vld){
    vb_addr_entry_addr_index := io.in.iccIn.addrTto6(PA_WIDTH-7,8)
  }.elsewhen(vb_addr_entry_lfb_create_dp_vld){
    vb_addr_entry_addr_index := io.in.lfbIn.addrTto6(PA_WIDTH-7,8)
  }.elsewhen(vb_addr_entry_wmb_create_dp_vld){
    vb_addr_entry_addr_index := io.in.wmbIn.addrTto6(PA_WIDTH-7,8)
  }.elsewhen(vb_addr_entry_feedback_vld){
    vb_addr_entry_addr_index := io.in.stDaFeedbackAddrTto14
  }
  when(vb_addr_entry_icc_create_dp_vld){
    vb_addr_entry_addr_tag := io.in.iccIn.addrTto6(7,0)
  }.elsewhen(vb_addr_entry_lfb_create_dp_vld){
    vb_addr_entry_addr_tag := io.in.lfbIn.addrTto6(7,0)
  }.elsewhen(vb_addr_entry_wmb_create_dp_vld){
    vb_addr_entry_addr_tag := io.in.wmbIn.addrTto6(7,0)
  }
  //+--------------+-----------+-----+------------+------------+--------+
  //| set_way_mode | cache_way | inv | lfb_create | icc_create | lfb_id |
  //+--------------+-----------+-----+------------+------------+--------+
  val vb_addr_entry_set_way_mode = RegInit(false.B)
  val vb_addr_entry_inv          = RegInit(false.B)
  val vb_addr_entry_wmb_create   = RegInit(false.B)
  val vb_addr_entry_lfb_create   = RegInit(false.B)
  val vb_addr_entry_source_id    = RegInit(0.U(VB_DATA_ENTRY.W))
  when(vb_addr_entry_icc_create_dp_vld){
    vb_addr_entry_set_way_mode := true.B
    vb_addr_entry_inv          := io.in.iccIn.inv
    vb_addr_entry_wmb_create   := false.B
    vb_addr_entry_lfb_create   := false.B
    vb_addr_entry_source_id    := 0.U(3.W)
  }.elsewhen(vb_addr_entry_lfb_create_dp_vld){
    vb_addr_entry_set_way_mode := false.B
    vb_addr_entry_inv          := true.B
    vb_addr_entry_wmb_create   := false.B
    vb_addr_entry_lfb_create   := true.B
    vb_addr_entry_source_id    := io.in.lfbIn.id
  }.elsewhen(vb_addr_entry_wmb_create_dp_vld){
    vb_addr_entry_set_way_mode := io.in.wmbIn.setWayMode
    vb_addr_entry_inv          := io.in.wmbIn.inv
    vb_addr_entry_wmb_create   := true.B
    vb_addr_entry_lfb_create   := false.B
    vb_addr_entry_source_id    := io.in.wmbIn.writePtrEncode
  }
  val vb_addr_entry_rcl_sm_req_success = RegInit(false.B)
  when(vb_addr_entry_create_vld){
    vb_addr_entry_rcl_sm_req_success := false.B
  }.elsewhen(vb_rcl_sm_addr_grnt){
    vb_addr_entry_rcl_sm_req_success := true.B
  }
  //rcl_done,for snq dep
  val vb_addr_entry_rcl_done = RegInit(false.B)
  when(vb_addr_entry_create_vld){
    vb_addr_entry_rcl_done := false.B
  }.elsewhen(vb_rcl_sm_done){
    vb_addr_entry_rcl_done := true.B
  }
  //record vb data has been pop
  val vb_addr_entry_data_req_success = RegInit(false.B)
  when(vb_addr_entry_create_vld){
    vb_addr_entry_data_req_success := false.B
  }.elsewhen(vb_rcl_sm_done){
    vb_addr_entry_data_req_success := io.in.evictReqSuccess
  }.elsewhen(vb_addr_data_pop){
    vb_addr_entry_data_req_success := true.B
  }
  //data buffer id
  val vb_addr_entry_db_id = RegInit(0.U(VB_DATA_ENTRY.W))
  when(vb_addr_entry_create_data){
    vb_addr_entry_db_id := io.in.vbIn.id
  }
  //==========================================================
  //                Generate req/pop/resp signal
  //==========================================================
  val vb_addr_entry_rcl_sm_req = vb_addr_entry_vld  &&  !vb_addr_entry_rcl_sm_req_success
  vb_addr_entry_pop_vld := vb_addr_entry_pop
  //for snq dep remove
  val vb_addr_entry_dep_remove = !vb_addr_entry_vld || vb_addr_entry_rcl_done
  //==========================================================
  //                    Compare index
  //==========================================================
  //------------------compare rb_biu_req----------------------
  val vb_addr_entry_rb_biu_req_hit_idx    = vb_addr_entry_vld && (io.in.rbBiuReqAddr(13,6) === vb_addr_entry_addr_tto6(7,0))
  val vb_addr_entry_pfu_biu_req_hit_idx   = vb_addr_entry_vld && (io.in.pfuBiuReqAddr(13,6) === vb_addr_entry_addr_tto6(7,0))
  val vb_addr_entry_wmb_write_req_hit_idx = vb_addr_entry_vld && (io.in.wmbIn.writeReqAddr(13,6) === vb_addr_entry_addr_tto6(7,0))
  val vb_addr_entry_lfb_vb_req_hit_idx    = vb_addr_entry_vld && (io.in.lfbIn.addrTto6(7,0) === vb_addr_entry_addr_tto6(7,0))
  val vb_addr_entry_snq_create_hit_idx    = vb_addr_entry_vld && (io.in.snqIn.createAddr(13,6) === vb_addr_entry_addr_tto6(7,0))
  val vb_addr_entry_snq_bypass_hit        = vb_addr_entry_vld && (io.in.snqIn.bypassAddrTto6(7,0) === vb_addr_entry_addr_tto6(7,0)) &&
    vb_addr_entry_rcl_done && !vb_addr_entry_data_req_success
  val vb_addr_entry_snq_start_wait = vb_addr_entry_vld && vb_addr_entry_rcl_done && vb_addr_entry_data_req_success
  //==========================================================
  //                 Generate interface
  //==========================================================
  //------------------output----------------------------------
  //-----------entry signal---------------
  io.out.vld_x          := vb_addr_entry_vld
  io.out.addr_tto6_v    := vb_addr_entry_addr_tto6
  io.out.set_way_mode_x := vb_addr_entry_set_way_mode
  io.out.inv_x          := vb_addr_entry_inv
  io.out.lfb_create_x   := vb_addr_entry_lfb_create
  io.out.wmb_create_x   := vb_addr_entry_wmb_create
  io.out.source_id_v    := vb_addr_entry_source_id
  io.out.db_id_v        := vb_addr_entry_db_id
  io.out.dep_remove_x   := vb_addr_entry_dep_remove
  io.out.rcl_sm_req_x   := vb_addr_entry_rcl_sm_req
  //-----------hit idx--------------------
  io.out.rb_biu_req_hit_idx_x   := vb_addr_entry_rb_biu_req_hit_idx
  io.out.pfu_biu_req_hit_idx_x  := vb_addr_entry_pfu_biu_req_hit_idx
  io.out.wmb_write_req_hit_idx_x:= vb_addr_entry_wmb_write_req_hit_idx
  io.out.lfb_vb_req_hit_idx_x   := vb_addr_entry_lfb_vb_req_hit_idx
  io.out.snq_create_hit_idx_x   := vb_addr_entry_snq_create_hit_idx
  io.out.snq_start_wait_x       := vb_addr_entry_snq_bypass_hit
  io.out.snq_bypass_hit_x       := vb_addr_entry_snq_start_wait

}
