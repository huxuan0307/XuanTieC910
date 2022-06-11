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

trait RclStatus {
  def RCL_IDLE            = 0.U(4.W)
  def RCL_R_TAG_DIRTY     = "b1000".U
  def RCL_NOP             = "b1001".U
  def RCL_CMP_TAG         = "b1010".U
  def RCL_EVICT           = "b1011".U
  def RCL_INVALID         = "b1100".U
  def RCL_REQ_DATA_ENTRY  = "b1101".U
  def RCL_READ_DATA0      = "b1110".U
  def RCL_READ_DATA1      = "b1111".U
}
//==========================================================
//                        Input
//==========================================================
class BusArbToVb extends Bundle with LsuConfig with DCacheConfig {
  val awGrnt = Bool()
  val wGrnt = Bool()
}
class DcacheArbToVb extends Bundle with LsuConfig with DCacheConfig {
  val ldGrnt = Bool()
  val stGrnt = Bool()
}
class StDaToVbAll extends StDaToVb with LsuConfig with DCacheConfig {
  val eccErr            = Bool()
  val eccStall          = Bool()
  val feedbackDddrTto14 = UInt(26.W)
  val tagReissue        = Bool()
  val hit               = Bool()
}
class SdbToVbTop extends Bundle with LsuConfig with DCacheConfig {
  val addrId          = Vec(VB_ADDR_ENTRY, UInt(2.W))
  val biuReq          = UInt(VB_DATA_ENTRY.W)
  val bypassPop       = UInt(VB_DATA_ENTRY.W)
  val dirty           = UInt(VB_DATA_ENTRY.W)
  val inv             = UInt(VB_DATA_ENTRY.W)
  val lfbCreate       = UInt(VB_DATA_ENTRY.W)
  val normalPop       = UInt(VB_DATA_ENTRY.W)
  val reqSuccess      = UInt(VB_DATA_ENTRY.W)
  val vld             = UInt(VB_DATA_ENTRY.W)
  val wdSmReq         = UInt(VB_DATA_ENTRY.W)
  val writeData128    = Vec(3, UInt((XLEN*2).W))
  val sdbDataEntryVld = UInt(3.W) // todo maybe 3 is data id
}
class WmbToVbAll extends Bundle {
  val toEntry = new WmbToVbAddrEntry
  val createDpVld     = Bool()
  val createGateclkEn = Bool()
  val createReq       = Bool()
  val createVld       = Bool()
}
//----------------------------------------------------------
class VbIn extends Bundle with LsuConfig with DCacheConfig {
  val cp0In = new Cp0ToVbAddrEntry
  val biuIn = new Bundle() {
    val bId = (UInt(5.W))
    val bVld= Bool()
  }
  val busArbIn = new BusArbToVb
  val dcacheArbIn = new DcacheArbToVb
  val iccIn = new IccToVb
  val ldDaSnqDataReissue = Bool()
  val lfbIn = new LfbToVb
  val padYyIcgScanEn = Bool()
  val pfuBiuReqAddr = UInt(PA_WIDTH.W)
  val rbBiuReqAddr = UInt(PA_WIDTH.W)
  val snqIn = new Bundle() {
    val depdVbId = UInt(VB_ADDR_ENTRY.W)
    val bypassAddrTto6 = UInt((PA_WIDTH-OFFSET_WIDTH).W)
    val createAddr = UInt(PA_WIDTH.W)
    val bypassCheck = Bool()
  }
  val stDaIn = new StDaToVbAll
  val vbSdbIn = new SdbToVbTop // Sdb is data entry
  val wmbIn = new WmbToVbAll
}
//==========================================================
//                        Output
//==========================================================
class VbToBiu extends Bundle with LsuConfig with DCacheConfig {
  val aw_addr  = UInt(PA_WIDTH.W)
  val aw_bar   = UInt(2.W)
  val aw_burst = UInt(2.W)
  val aw_cache  = UInt(4.W)
  val aw_domain = UInt(2.W)
  val aw_dp_req = Bool()
  val aw_id  = UInt(5.W)
  val aw_len = UInt(2.W)
  val aw_lock = Bool()
  val aw_prot = UInt(3.W)
  val aw_req = Bool()
  val aw_req_gateclk_en = Bool()
  val aw_size  = UInt(3.W)
  val aw_snoop = UInt(3.W)
  val aw_unique = Bool()
  val aw_user = Bool()
  val w_data = UInt(128.W)
  val w_id = UInt(5.W)
  val w_last = Bool()
  val w_req = Bool()
  val w_strb = UInt(16.W)
  val w_vld = Bool()
}

class VbTopToSdb extends Bundle with LsuConfig with DCacheConfig {
  val biuReqSuccess = UInt(3.W)
  val createDpVld = UInt(3.W)
  val createGateclkEn = UInt(3.W)
  val createVld = UInt(3.W)
  val wdSmGrnt = UInt(3.W)

  val rclSmAddrId          = UInt(2.W)
  val rclSmDataDcacheDirty = Bool()
  val rclSmDataId          = UInt(3.W)
  val rclSmDataSetDataDone = UInt(3.W)
  val rclSmInv             = Bool()
  val rclSmLfbCreate       = Bool()
}
class VbToDcacheArb extends Bundle with LsuConfig with DCacheConfig {
  val borrow_addr        = UInt(PA_WIDTH.W)
  val data_way           = Bool()
  val dcache_replace     = Bool()

  val ld_borrow_req      = Bool()
  val ld_borrow_req_gate = Bool()
  val ld_data_gateclk_en = UInt(8.W)
  val ld_data_idx        = UInt(11.W)
  val ld_req             = Bool()
  val ld_tag_gateclk_en  = Bool()
  val ld_tag_idx         = UInt(INDEX_WIDTH.W)
  val ld_tag_req         = Bool()
  val ld_tag_wen         = UInt(2.W)

  val serial_req         = Bool()
  val set_way_mode       = Bool()

  val st_borrow_req      = Bool()
  val st_dirty_din       = UInt(7.W)
  val st_dirty_gateclk_en= Bool()
  val st_dirty_gwen      = Bool()
  val st_dirty_idx       = UInt(INDEX_WIDTH.W)
  val st_dirty_req       = Bool()
  val st_dirty_wen       = UInt(7.W)
  val st_req             = Bool()
  val st_tag_gateclk_en  = Bool()
  val st_tag_idx         = UInt(INDEX_WIDTH.W)
  val st_tag_req         = Bool()
}
class VbToSnq extends Bundle with LsuConfig with DCacheConfig {
  val bypass_db_id  = UInt(3.W)
  val bypass_hit    = Bool()
  val depd          = UInt(2.W)
  val depd_remove   = UInt(2.W)
  val start_hit_idx = Bool()
  val wait_remove   = UInt(2.W)
  val wait_vb_id    = UInt(2.W)
}
//----------------------------------------------------------
class VbOut extends Bundle with LsuConfig with DCacheConfig {
  // to had
  val snqDataBypassHit = UInt(3.W)
  val toLfb = new VbToLfb
  val toBiu = new VbToBiu
  val toSdb = new VbTopToSdb //to vb data entry
  val toDcacheArb = new VbToDcacheArb
  val vbEmpty = Bool()
  val iccCreateGrnt = Bool()
  val vbInvalidVld = Bool()
  val pfuHitIdx = Bool()
  val rbHitIdx = Bool()
  val toSnq = new VbToSnq
  val wdSmData = new Bundle() {
    val bias   = UInt(4.W)
    val popReq = UInt(3.W)
  }
  val toWmb = new Bundle() {
    val create_grnt        = Bool()
    val empty              = Bool()
    val entry_rcl_done     = UInt(8.W)
    val write_req_hit_idx  = Bool()
  }
  val victimAddr = UInt((PA_WIDTH-6).W)
}
//==========================================================
//                          IO
//==========================================================
class VbIO extends Bundle with LsuConfig {
  val in  = Input( new VbIn)
  val out = Output(new VbOut)
}
class Vb extends Module with DCacheConfig with LsuConfig with RclStatus{
  val io = IO(new VbIO)
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //rcl state machine
  val vb_rcl_sm_create_vld = Wire(Bool())
  val vb_rcl_sm_vld = Wire(Bool())
  val vb_rcl_sm_clk_en = vb_rcl_sm_create_vld || vb_rcl_sm_vld
  val vb_rcl_sm_create_clk_en = vb_rcl_sm_create_vld
  //wd state machine
  val vb_data_wd_sm_req = Wire(Bool())
  val vb_wd_sm_vld = RegInit(false.B)
  val vb_wd_sm_clk_en = vb_data_wd_sm_req  ||  vb_wd_sm_vld
  //==========================================================
  //              Instance addr entry
  //==========================================================
  val addr_entries = Seq.fill(VB_ADDR_ENTRY)(Module(new VbAddrEntry))
  //==========================================================
  //               addr entry output OH bundle
  //==========================================================
  val vb_addr_entry_vld = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_rcl_sm_req    =  Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_addr_tto6     = Seq.fill(VB_ADDR_ENTRY)(Wire(UInt((PA_WIDTH-6).W)))
  val vb_addr_entry_set_way_mode  = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_inv           = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_lfb_create    = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_source_id     = Seq.fill(VB_ADDR_ENTRY)(Wire(UInt((VB_DATA_ENTRY).W)))
  val vb_addr_entry_wmb_create    = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_rb_biu_req_hit_idx    = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_pfu_biu_req_hit_idx   = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_wmb_write_req_hit_idx = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_lfb_vb_req_hit_idx    = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  // output on the top
  addr_entries.zipWithIndex.foreach {
    case (entry, i) =>
      vb_addr_entry_vld(i)              := entry.io.out.vld_x
      vb_addr_entry_rcl_sm_req(i)       := entry.io.out.rcl_sm_req_x
      vb_addr_entry_addr_tto6(i)        := entry.io.out.addr_tto6_v
      vb_addr_entry_set_way_mode(i)     := entry.io.out.set_way_mode_x
      vb_addr_entry_inv(i)              := entry.io.out.inv_x
      vb_addr_entry_lfb_create(i)       := entry.io.out.lfb_create_x
      vb_addr_entry_source_id(i)        := entry.io.out.source_id_v
      vb_addr_entry_wmb_create(i)       := entry.io.out.wmb_create_x
      vb_addr_entry_rb_biu_req_hit_idx(i) := entry.io.out.rb_biu_req_hit_idx_x
      vb_addr_entry_pfu_biu_req_hit_idx(i) := entry.io.out.pfu_biu_req_hit_idx_x
      vb_addr_entry_wmb_write_req_hit_idx(i) := entry.io.out.wmb_write_req_hit_idx_x
      vb_addr_entry_lfb_vb_req_hit_idx(i) := entry.io.out.lfb_vb_req_hit_idx_x
  }
  //==========================================================
  //            Generate addr signal
  //==========================================================
  //------------------create ptr------------------------------
  val vb_addr_create_ptr = RegInit(0.U(VB_ADDR_ENTRY.W))
  vb_addr_create_ptr := PriorityEncoder(VecInit(vb_addr_entry_vld).asUInt)
  //------------------full signal-----------------------------
  val vb_addr_full   = vb_addr_entry_vld.reduce(_ && _) // if all invalid is full
  //------------------grnt signal to vb/pfu------------------
  io.out.iccCreateGrnt     := io.in.iccIn.createReq && !vb_addr_full
  io.out.toLfb.createGrnt  := io.in.lfbIn.createReq && (!vb_addr_full) && (!io.in.iccIn.createReq)
  io.out.toWmb.create_grnt := io.in.wmbIn.createReq && (!io.in.iccIn.createReq) && (!io.in.lfbIn.createReq) && (!vb_addr_full)
  //------------------create signal---------------------------
  // create
  val vb_icc_create_vld    = io.out.iccCreateGrnt      &&  io.in.iccIn.createVld
  val vb_lfb_create_vld    = io.out.toLfb.createGrnt   &&  io.in.lfbIn.createVld
  val vb_wmb_create_vld    = io.out.toWmb.create_grnt  &&  io.in.wmbIn.createVld
  val vb_addr_create_vld  = vb_icc_create_vld  ||  vb_lfb_create_vld    ||  vb_wmb_create_vld
  // dp create
  val vb_addr_icc_create_dp_vld  = io.out.iccCreateGrnt      &&  io.in.iccIn.createDpVld
  val vb_addr_lfb_create_dp_vld  = io.out.toLfb.createGrnt   &&  io.in.lfbIn.createDpVld
  val vb_addr_wmb_create_dp_vld  = io.out.toWmb.create_grnt  &&  io.in.wmbIn.createDpVld
  val vb_addr_create_gateclk_en = io.in.iccIn.createGateclkEn ||  io.in.lfbIn.createGateclkEn ||  io.in.wmbIn.createGateclkEn
  val vb_addr_entry_create_vld        = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_icc_create_dp_vld = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_lfb_create_dp_vld = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_wmb_create_dp_vld = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  val vb_addr_entry_create_gateclk_en = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  for(i<-0 until VB_ADDR_ENTRY){
    vb_addr_entry_create_vld(i)        := vb_addr_create_vld && vb_addr_create_ptr(i).asBool
    vb_addr_entry_icc_create_dp_vld(i) := vb_addr_icc_create_dp_vld && vb_addr_create_ptr(i).asBool
    vb_addr_entry_lfb_create_dp_vld(i) := vb_addr_lfb_create_dp_vld && vb_addr_create_ptr(i).asBool
    vb_addr_entry_wmb_create_dp_vld(i) := vb_addr_wmb_create_dp_vld && vb_addr_create_ptr(i).asBool
    vb_addr_entry_create_gateclk_en(i) := vb_addr_create_gateclk_en && vb_addr_create_ptr(i).asBool
  }
  //==========================================================
  //            read cache line state machine
  //==========================================================
  //---------------------registers----------------------------
  //+-------+
  //| state |
  //+-------+
  val vb_rcl_sm_state = RegInit(RCL_IDLE)
  vb_rcl_sm_vld := vb_rcl_sm_state(3) //state[3] is used for whether has set cache info
  //for timing dcache interface is set register
  val vb_dcache_arb_ld_req = RegInit(false.B)
  val vb_dcache_arb_st_req = RegInit(false.B)
  val vb_dcache_arb_ld_req_set = Wire(Bool())
  val vb_dcache_arb_st_req_set = Wire(Bool())
  when(vb_dcache_arb_ld_req_set){
    vb_dcache_arb_ld_req := true.B
  }.otherwise{
    vb_dcache_arb_ld_req := false.B
  }
  io.out.toDcacheArb.ld_req := vb_dcache_arb_ld_req
  when(vb_dcache_arb_st_req_set){
    vb_dcache_arb_st_req := true.B
  }.otherwise{
    vb_dcache_arb_st_req := false.B
  }
  io.out.toDcacheArb.st_req := vb_dcache_arb_st_req
  //+---------+------+
  //| addr_id | addr |
  //+---------+------+
  val vb_rcl_sm_addr_id       = RegInit(0.U(VB_ADDR_ENTRY.W))
  val vb_rcl_sm_addr_tto6     = RegInit(0.U((PA_WIDTH-7).W))
  val vb_rcl_sm_req_addr_ptr  = RegInit(0.U(VB_ADDR_ENTRY.W))
  val vb_rcl_sm_req_addr_tto6 = Wire(UInt(VB_ADDR_ENTRY.W))
  when(vb_rcl_sm_create_vld){
    vb_rcl_sm_addr_id   := vb_rcl_sm_req_addr_ptr
    vb_rcl_sm_addr_tto6 :=vb_rcl_sm_req_addr_tto6
  }
  io.out.toSdb.rclSmAddrId := vb_rcl_sm_addr_id // send addr id to data entry
  //+-------------+-----------+
  //| cache_dirty | cache_way |
  //+-------------+-----------+
  val vb_rcl_sm_dcache_dirty = RegInit(false.B)
  val vb_rcl_sm_dcache_way = RegInit(false.B)
  val rcl_cmp_tag_vld = Wire(Bool())
  val vb_rcl_sm_reg_set_dcache_dirty = Wire(Bool())
  val vb_rcl_sm_reg_set_dcache_way = Wire(Bool())
  when((rcl_cmp_tag_vld)){
    vb_rcl_sm_dcache_dirty := vb_rcl_sm_reg_set_dcache_dirty
    vb_rcl_sm_dcache_way   := vb_rcl_sm_reg_set_dcache_way
  }
  io.out.toDcacheArb.data_way := vb_rcl_sm_dcache_way
  //+---------+
  //| data_id |
  //+---------+
  val vb_data_create_ptr = RegInit(0.U(VB_DATA_ENTRY.W))
  val vb_rcl_sm_data_id = RegInit(0.U(VB_DATA_ENTRY.W))
  val vb_rcl_sm_data_entry_req_success= Wire(Bool())
  when(vb_rcl_sm_data_entry_req_success){
    vb_rcl_sm_data_id := vb_data_create_ptr
  }
  io.out.toSdb.rclSmDataId := vb_rcl_sm_data_id
  //------------------state change----------------------------
  val vb_addr_rcl_sm_req = Wire(Bool())
  val vb_data_full = Wire(Bool())
  val evict_enable = Wire(Bool())
  val evict_req_success = Wire(Bool())
  val evict_req_cancel = Wire(Bool())

  switch(vb_rcl_sm_state){
    is(RCL_IDLE){
      when(vb_addr_rcl_sm_req){
        vb_rcl_sm_state := RCL_R_TAG_DIRTY
      }
    }
    is(RCL_R_TAG_DIRTY){
      when(io.in.dcacheArbIn.stGrnt){
        vb_rcl_sm_state := RCL_NOP
      }
    }
    is(RCL_NOP){
      when(io.in.stDaIn.tagReissue){
        vb_rcl_sm_state := RCL_R_TAG_DIRTY
      }.otherwise{
        vb_rcl_sm_state := RCL_CMP_TAG
      }
    }
    //-----------------------------------------------
    is(RCL_CMP_TAG){ // ECC will stop the sm
      when(io.in.stDaIn.eccStall){
        vb_rcl_sm_state := RCL_CMP_TAG
      }.elsewhen(io.in.stDaIn.eccErr){
        vb_rcl_sm_state := RCL_IDLE
      }.elsewhen(io.out.toSdb.rclSmLfbCreate){ //lfb state
        //--------------------------------
        when(io.in.stDaIn.hit){
          vb_rcl_sm_state := RCL_IDLE
        }.elsewhen(!(io.in.stDaIn.replaceValid)){
          vb_rcl_sm_state := RCL_IDLE
        }.elsewhen(!(io.in.stDaIn.replaceDirty)){
          vb_rcl_sm_state := RCL_INVALID
        }.otherwise{
          //------------------
          when(!vb_data_full){
            vb_rcl_sm_state := RCL_READ_DATA0
          }.otherwise{
            vb_rcl_sm_state := RCL_REQ_DATA_ENTRY
          }
          //-----------------
        }
        //--------------------------------
      }
    }
    //-----------------------------------------------
    is(RCL_INVALID){
      when(io.in.dcacheArbIn.stGrnt && evict_enable){
        vb_rcl_sm_state := RCL_EVICT
      }.elsewhen(io.in.dcacheArbIn.stGrnt){
        vb_rcl_sm_state := RCL_IDLE
      }
    }
    is(RCL_EVICT){
      when(evict_req_success || evict_req_cancel){
        vb_rcl_sm_state := RCL_IDLE
      }
    }
    is(RCL_REQ_DATA_ENTRY){
      when(!vb_data_full){
        vb_rcl_sm_state := RCL_READ_DATA0
      }
    }
    is(RCL_READ_DATA0){
      when(io.in.dcacheArbIn.ldGrnt){
        vb_rcl_sm_state := RCL_READ_DATA1
      }
    }
    is(RCL_READ_DATA1){
      when(io.in.ldDaSnqDataReissue){
        vb_rcl_sm_state := RCL_READ_DATA0
      }.otherwise{
        vb_rcl_sm_state := RCL_IDLE
      }
    }
  }
  //------------------create singal---------------------------
  val vb_rcl_sm_permit     = !vb_rcl_sm_vld
  vb_addr_rcl_sm_req   := vb_addr_entry_rcl_sm_req.reduce(_ || _)
  vb_rcl_sm_create_vld := vb_addr_rcl_sm_req && vb_rcl_sm_permit
  //------------------create info----------------------------
  vb_rcl_sm_req_addr_ptr := PriorityEncoder(VecInit(vb_addr_entry_rcl_sm_req).asUInt)
  val vb_rcl_sm_addr_grnt = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  for(i<- 0 until VB_ADDR_ENTRY){
    vb_rcl_sm_addr_grnt(i) := vb_rcl_sm_create_vld && vb_rcl_sm_req_addr_ptr(i).asBool
  }
  vb_rcl_sm_req_addr_tto6 := (Cat(Seq.fill(PA_WIDTH-6)(vb_rcl_sm_req_addr_ptr(0))) & vb_addr_entry_addr_tto6(0)) |
    (Cat(Seq.fill(PA_WIDTH-6)(vb_rcl_sm_req_addr_ptr(1))) & vb_addr_entry_addr_tto6(1))
  //-------------rcl state info from addr entry---------------
  val vb_rcl_sm_set_way_mode = (vb_rcl_sm_addr_id & (VecInit(vb_addr_entry_set_way_mode).asUInt)).orR
  val vb_rcl_sm_inv = (vb_rcl_sm_addr_id & (VecInit(vb_addr_entry_inv).asUInt)).orR
  io.out.toSdb.rclSmInv := vb_rcl_sm_inv
  val vb_rcl_sm_lfb_create = (vb_rcl_sm_addr_id & (VecInit(vb_addr_entry_lfb_create).asUInt)).orR
  io.out.toSdb.rclSmLfbCreate := vb_rcl_sm_lfb_create
  val vb_addr_entry_source_id_ptr = Seq.fill(VB_ADDR_ENTRY)(Wire(UInt(8.W)))
  for(i<- 0 until VB_ADDR_ENTRY){
    vb_addr_entry_source_id_ptr(i) := UIntToOH(vb_addr_entry_source_id(i))
  }
  val vb_rcl_sm_source_id_expand = (Cat(Seq.fill(8)(vb_rcl_sm_addr_id(0))) & vb_addr_entry_source_id_ptr(0)) |
    (Cat(Seq.fill(8)(vb_rcl_sm_addr_id(1))) & vb_addr_entry_source_id_ptr(1))
  //--------------------CMP_TAG STATE-------------------------
  //-------------------feed back signal-----------------------
  rcl_cmp_tag_vld := (vb_rcl_sm_state === RCL_CMP_TAG) && !(io.in.stDaIn.eccStall) && !(io.in.stDaIn.eccErr)
  val vb_addr_feedback_vld = rcl_cmp_tag_vld
  val vb_addr_entry_feedback_vld = Mux(vb_addr_feedback_vld , vb_rcl_sm_addr_id, 0.U)
  //----------------------evict control-----------------------
  evict_enable := io.out.toSdb.rclSmLfbCreate
  //------------------done signal to lfb/wmb------------------
  val vb_rcl_lfb_unnecessary = rcl_cmp_tag_vld && (io.in.stDaIn.hit || !io.in.stDaIn.replaceValid)
  val vb_rcl_iccwmb_unnecessary = rcl_cmp_tag_vld && (io.in.stDaIn.miss || (!io.in.stDaIn.dirty && !vb_rcl_sm_inv))
  //for ecc err
  val vb_rcl_ecc_done        = (vb_rcl_sm_state ===  RCL_CMP_TAG)  && io.in.stDaIn.eccErr
  val vb_rcl_set_dirty_done  = (vb_rcl_sm_state ===  RCL_INVALID)  &&  !evict_enable  &&  io.in.dcacheArbIn.stGrnt
  val vb_rcl_req_data_done   = (vb_rcl_sm_state ===  RCL_READ_DATA1)  && !io.in.ldDaSnqDataReissue
  //------------------done signal to vb entry------------------
  //for snq dep, other done will invalid entry
  val vb_rcl_sm_done = Mux(vb_rcl_req_data_done||evict_req_success ,vb_rcl_sm_addr_id , 0.U)
  //---------------to lfb-----------------
  //vb_lfb_rcl_done
  io.out.toLfb.rclDone          := io.out.toSdb.rclSmLfbCreate && (vb_rcl_lfb_unnecessary
    ||  vb_rcl_set_dirty_done ||  evict_req_success    ||
    evict_req_cancel    ||  vb_rcl_ecc_done    ||  vb_rcl_req_data_done)
  io.out.toLfb.addrEntryRcldone := Mux(io.out.toLfb.rclDone  ,vb_rcl_sm_source_id_expand , 0.U)
  io.out.toLfb.dcacheHit        := rcl_cmp_tag_vld && io.in.stDaIn.hit
  io.out.toLfb.dcacheDirty      := io.in.stDaIn.dirty
  val vb_rcl_sm_sel_dcache_reg_info = Wire(Bool())
  io.out.toLfb.dcacheWay        := Mux(vb_rcl_sm_sel_dcache_reg_info,vb_rcl_sm_dcache_way ,vb_rcl_sm_reg_set_dcache_way)
  //---------------to wmb-----------------
  val vb_rcl_sm_addr_pop_req    = Wire(UInt(VB_ADDR_ENTRY.W))
  val vb_addr_entry_bypass_pop  = Wire(UInt(VB_ADDR_ENTRY.W))
  val vb_addr_entry_b_resp      = Wire(UInt(VB_ADDR_ENTRY.W))
  val vb_addr_entry_pop = vb_rcl_sm_addr_pop_req | vb_addr_entry_bypass_pop | vb_addr_entry_b_resp
  val vb_addr_entry_wmb_pop = vb_addr_entry_pop & VecInit(vb_addr_entry_wmb_create).asUInt
  io.out.toWmb.entry_rcl_done :=  (Cat(Seq.fill(8)(vb_addr_entry_wmb_pop(0))) & vb_addr_entry_source_id_ptr(0)) |
    (Cat(Seq.fill(8)(vb_addr_entry_wmb_pop(1))) & vb_addr_entry_source_id_ptr(1))

  //---------rcl get dcache info at CMP_TAG state-------------
  // if lfb create, then set fill_way and fill_dirty to reg and data entry,
  // if wmb create, then set hit way and hit dirty to reg and data entry
  vb_rcl_sm_reg_set_dcache_way   := Mux(io.out.toSdb.rclSmLfbCreate,io.in.stDaIn.replaceWay   , io.in.stDaIn.way )
  vb_rcl_sm_reg_set_dcache_dirty := Mux(io.out.toSdb.rclSmLfbCreate,io.in.stDaIn.replaceDirty , io.in.stDaIn.dirty )

  //-------------------create data entry----------------------
  val evict_req = vb_rcl_sm_state === RCL_EVICT
  vb_rcl_sm_sel_dcache_reg_info := vb_rcl_sm_state(2) || evict_req
  io.out.toSdb.rclSmDataDcacheDirty := Mux(vb_rcl_sm_sel_dcache_reg_info , vb_rcl_sm_dcache_dirty, vb_rcl_sm_reg_set_dcache_dirty)

  val vb_data_create_vld = rcl_cmp_tag_vld &&  (vb_rcl_sm_lfb_create && io.in.stDaIn.miss && io.in.stDaIn.replaceValid && io.in.stDaIn.replaceDirty ||
    !vb_rcl_sm_lfb_create    &&  io.in.stDaIn.hit    &&  io.in.stDaIn.dirty)  ||
    (vb_rcl_sm_state  ===  RCL_REQ_DATA_ENTRY)
  val vb_data_create_dp_vld = rcl_cmp_tag_vld || (vb_rcl_sm_state  ===  RCL_REQ_DATA_ENTRY)
  val vb_data_create_gateclk_en = vb_data_create_dp_vld
  vb_rcl_sm_data_entry_req_success := vb_data_create_vld  &&  !vb_data_full
  //--------------------cache interface set-------------------
  vb_dcache_arb_ld_req_set := (vb_rcl_sm_state  ===  RCL_READ_DATA0) || (vb_rcl_sm_state  ===  RCL_READ_DATA1) || (vb_rcl_sm_state  ===  RCL_INVALID)
  vb_dcache_arb_st_req_set := (vb_rcl_sm_state  ===  RCL_R_TAG_DIRTY) || (vb_rcl_sm_state  ===  RCL_READ_DATA1) || (vb_rcl_sm_state  ===  RCL_INVALID)
  //----------------------cache interface---------------------
  val vb_dcache_arb_write =  (vb_rcl_sm_state  ===  RCL_READ_DATA1) || (vb_rcl_sm_state  ===  RCL_INVALID)
  val vb_dcache_arb_serial_req = (vb_rcl_sm_state  ===  RCL_READ_DATA0)
  io.out.toDcacheArb.serial_req := vb_dcache_arb_serial_req
  val vb_dcache_arb_data_way = vb_rcl_sm_dcache_way
  //---------------tag array--------------
  io.out.toDcacheArb.ld_tag_req        := vb_dcache_arb_write  &&  vb_rcl_sm_inv
  io.out.toDcacheArb.ld_tag_gateclk_en := vb_dcache_arb_ld_req  &&  io.out.toDcacheArb.ld_tag_req
  io.out.toDcacheArb.ld_tag_idx        := vb_rcl_sm_addr_tto6(TAG_WIDTH-1,0)
  io.out.toDcacheArb.ld_tag_wen        := Cat(vb_rcl_sm_dcache_way, !vb_rcl_sm_dcache_way)

  io.out.toDcacheArb.st_tag_req         := vb_rcl_sm_state  ===  RCL_R_TAG_DIRTY
  io.out.toDcacheArb.st_tag_gateclk_en  := io.out.toDcacheArb.st_tag_req && io.out.toDcacheArb.st_req
  io.out.toDcacheArb.st_tag_idx         := vb_rcl_sm_addr_tto6(TAG_WIDTH-1,0)
  //---------------dirty array------------
  io.out.toDcacheArb.st_dirty_req        := io.out.toDcacheArb.st_req
  io.out.toDcacheArb.st_dirty_gateclk_en := io.out.toDcacheArb.st_req && io.out.toDcacheArb.st_dirty_req
  io.out.toDcacheArb.st_dirty_idx        := vb_rcl_sm_addr_tto6(TAG_WIDTH-1,0)
  io.out.toDcacheArb.st_dirty_din        := Cat(vb_rcl_sm_dcache_way,0.U(2.W),!vb_rcl_sm_inv,0.U(2.W) ,!vb_rcl_sm_inv )
  io.out.toDcacheArb.st_dirty_gwen       := vb_dcache_arb_write
  io.out.toDcacheArb.st_dirty_wen        := Mux(vb_dcache_arb_write,
    Cat(vb_rcl_sm_inv, Cat(Seq.fill(3)(vb_rcl_sm_dcache_way)) ,Cat(Seq.fill(3)(!vb_rcl_sm_dcache_way)) ),
  0.U(7.W))

  //---------------data array-------------
  io.out.toDcacheArb.ld_data_gateclk_en := Cat(Seq.fill(8)(vb_dcache_arb_ld_req))
  io.out.toDcacheArb.ld_data_idx := Cat(vb_rcl_sm_addr_tto6(TAG_WIDTH-1,0),vb_dcache_arb_write,vb_rcl_sm_dcache_way)
  //-------------------------borrow signal--------------------
  io.out.toDcacheArb.ld_borrow_req      := (vb_rcl_sm_state  ===  RCL_READ_DATA0) || (vb_rcl_sm_state  ===  RCL_READ_DATA1) && !io.in.ldDaSnqDataReissue
  io.out.toDcacheArb.ld_borrow_req_gate := (vb_rcl_sm_state  ===  RCL_READ_DATA0) || (vb_rcl_sm_state  ===  RCL_READ_DATA1)
  io.out.toDcacheArb.st_borrow_req      := (vb_rcl_sm_state  ===  RCL_R_TAG_DIRTY)
  io.out.toDcacheArb.borrow_addr        := Cat(vb_rcl_sm_addr_tto6 , 0.U(OFFSET_WIDTH.W))
  //-------------------other signal---------------------------
  io.out.toDcacheArb.set_way_mode   := vb_rcl_sm_set_way_mode
  io.out.toDcacheArb.dcache_replace := vb_rcl_sm_lfb_create
  //---------------------set data ready-----------------------
  //----------------------pop/done signal---------------------
  val vb_rcl_sm_set_data_done_vld = io.in.dcacheArbIn.ldGrnt && (vb_rcl_sm_state  ===  RCL_READ_DATA0)
  io.out.toSdb.rclSmDataSetDataDone := Mux(vb_rcl_sm_set_data_done_vld , vb_rcl_sm_data_id, 0.U(VB_ADDR_ENTRY.W))
  val vb_rcl_sm_addr_pop_vld =  vb_rcl_set_dirty_done  ||  evict_req_cancel ||
    io.out.toSdb.rclSmLfbCreate && vb_rcl_lfb_unnecessary ||
    !io.out.toSdb.rclSmLfbCreate && vb_rcl_iccwmb_unnecessary || vb_rcl_ecc_done
  vb_rcl_sm_addr_pop_req := Cat(Seq.fill(VB_ADDR_ENTRY)(vb_rcl_sm_addr_pop_vld)) & vb_rcl_sm_addr_id
  //==========================================================
  //                  Create data entry
  //==========================================================
  //------------------create ptr------------------------------
  vb_data_create_ptr := UIntToOH(io.in.vbSdbIn.vld) // input data entry id to OH
  vb_data_full := vb_data_create_ptr.andR
  //------------------create signal---------------------------
  io.out.toSdb.createVld       := Cat(Seq.fill(VB_DATA_ENTRY)(vb_data_create_vld)) & vb_data_create_ptr
  io.out.toSdb.createDpVld     := Cat(Seq.fill(VB_DATA_ENTRY)(vb_data_create_dp_vld)) & vb_data_create_ptr
  io.out.toSdb.createGateclkEn := Cat(Seq.fill(VB_DATA_ENTRY)(vb_data_create_gateclk_en)) & vb_data_create_ptr
  //for entry record
  val vb_data_entry_id  = vb_data_create_ptr
  val vb_addr_entry_create_data = Cat(Seq.fill(VB_DATA_ENTRY)(vb_rcl_sm_data_entry_req_success)) & vb_rcl_sm_addr_id

  //==========================================================
  //            write evict
  //==========================================================
  val evict_req_addr_tto6 =  (Cat(Seq.fill(PA_WIDTH-6)(vb_rcl_sm_addr_id(0))) & vb_addr_entry_addr_tto6(0)) |
    (Cat(Seq.fill(PA_WIDTH-6)(vb_rcl_sm_addr_id(1))) & vb_addr_entry_addr_tto6(1))
  val vb_data_biu_req = io.in.vbSdbIn.biuReq.orR
  evict_req_success := !vb_data_biu_req && io.in.busArbIn.awGrnt
  evict_req_cancel :=  evict_req  && ! io.in.busArbIn.awGrnt && (vb_rcl_sm_addr_id === io.in.snqIn.depdVbId)

  //for lm, evict or write back should clear lm
  io.out.vbInvalidVld := io.out.toDcacheArb.ld_tag_req
  io.out.victimAddr := evict_req_addr_tto6

  //==========================================================
  //            arbitrate req biu signal
  //==========================================================
  //-----------------biu aw req ptr---------------------------
  val vb_data_biu_req_ptr = UIntToOH(PriorityEncoder(io.in.vbSdbIn.biuReq))
  //-----------------biu aw req success-----------------------
  io.out.toSdb.biuReqSuccess := Mux(io.in.busArbIn.awGrnt,vb_data_biu_req_ptr , 0.U(VB_DATA_ENTRY))
  //-----------------biu aw req info--------------------------
  io.out.toBiu.aw_req            := vb_data_biu_req || evict_req
  io.out.toBiu.aw_dp_req         := io.out.toBiu.aw_req
  io.out.toBiu.aw_req_gateclk_en := io.out.toBiu.aw_req
  val vb_biu_aw_req_dirty      = (vb_data_biu_req_ptr & io.in.vbSdbIn.dirty).orR
  val vb_biu_aw_req_lfb_create = (vb_data_biu_req_ptr & io.in.vbSdbIn.lfbCreate).orR
  val vb_biu_aw_req_inv        = (vb_data_biu_req_ptr & io.in.vbSdbIn.inv).orR

  val biu_req_seq_ptr = Seq.fill(VB_DATA_ENTRY)(Wire(Bool()))
  biu_req_seq_ptr.zipWithIndex.foreach {
    case (ptr, i) =>
    ptr := vb_data_biu_req_ptr(i)
  }
  val vb_biu_aw_req_id     = biu_req_seq_ptr.zip(io.in.vbSdbIn.addrId).map{
    case(ptr, id) =>
      Mux(ptr, id,0.U)
  }.reduce(_ | _)
  val vb_biu_aw_addr_ptr = UIntToOH(vb_biu_aw_req_id)
  val biu_addr_seq_ptr = Seq.fill(VB_ADDR_ENTRY)(Wire(Bool()))
  biu_addr_seq_ptr.zipWithIndex.foreach {
    case (ptr, i) =>
      ptr := vb_biu_aw_addr_ptr(i)
  }
  val vb_biu_aw_req_addr_tto6 = biu_addr_seq_ptr.zip(vb_addr_entry_addr_tto6).map{
    case(ptr, addr) =>
      Mux(ptr, addr,0.U)
  }.reduce(_ | _)
  //-----------------interface to bus_arb---------------------
  io.out.toBiu.aw_id   := Mux(vb_data_biu_req ,Cat(BIU_VB_ID_T,vb_biu_aw_req_id) ,Cat(BIU_VB_ID_T,"b0".U,io.out.toSdb.rclSmAddrId(1)))
  io.out.toBiu.aw_addr := Mux(vb_data_biu_req,Cat(vb_biu_aw_req_addr_tto6,0.U(OFFSET_WIDTH.W)),
    Cat(evict_req_addr_tto6,0.U(OFFSET_WIDTH.W)))

  io.out.toBiu.aw_len   := "b11".U
  io.out.toBiu.aw_size  := "b110".U
  io.out.toBiu.aw_burst := "b01".U
  io.out.toBiu.aw_lock  := false.B
  //cacheable,weak order, bufferable
  io.out.toBiu.aw_cache := "b1111".U
  //data,security,supervisor
  io.out.toBiu.aw_prot := "b011".U
  io.out.toBiu.aw_user := true.B

  io.out.toBiu.aw_snoop  := DontCare
  io.out.toBiu.aw_domain := "b01".U
  io.out.toBiu.aw_bar    := Cat(vb_biu_aw_req_lfb_create,0.U(1.W))
  io.out.toBiu.aw_unique := !vb_biu_aw_req_dirty  &&  vb_biu_aw_req_lfb_create

  //==========================================================
  //              write data state machine
  //==========================================================
  //---------------------registers----------------------------
  //+-----+
  //| vld |
  //+-----+
  val vb_wd_sm_start_vld = Wire(Bool())
  val vb_wd_sm_inv = Wire(Bool())
  when(vb_wd_sm_start_vld){
    vb_wd_sm_vld := true.B
  }.elsewhen(vb_wd_sm_inv){
    vb_wd_sm_vld := false.B
  }
  //+---------+
  //| data_id |
  //+---------+
  val vb_data_wd_sm_req_ptr = io.in.vbSdbIn.wdSmReq
  val vb_wd_sm_data_id = RegInit(0.U(VB_DATA_ENTRY.W))
  when(vb_wd_sm_start_vld){
    vb_wd_sm_data_id := vb_data_wd_sm_req_ptr
  }
  //+-----------+
  //| data_bias |
  //+-----------+
  val vb_wd_sm_data_bias  =RegInit(0.U(4.W))
  when(vb_wd_sm_start_vld){
    vb_wd_sm_data_bias := 1.U(4.W)
  }.elsewhen(vb_wd_sm_vld  && io.in.busArbIn.wGrnt){
    vb_wd_sm_data_bias := Cat(io.out.wdSmData.bias,0.U(1.W))
  }
  io.out.wdSmData.bias := vb_wd_sm_data_bias
  //------------------create signal---------------------------
  val vb_wd_sm_vld_permit  = vb_wd_sm_vld  &&  vb_wd_sm_data_bias(3)   &&  io.in.busArbIn.wGrnt
  val vb_wd_sm_permit      = !vb_wd_sm_vld ||  vb_wd_sm_vld_permit
  vb_data_wd_sm_req    := io.in.vbSdbIn.wdSmReq.orR
  vb_wd_sm_start_vld   := vb_data_wd_sm_req &&  vb_wd_sm_permit
  //------------------create info-----------------------------
  //only 1 or 0 entry requests the wd_sm when the state machine is able to permit
  //request, so it doesn't need to arbitrate
  io.out.toSdb.wdSmGrnt := Mux(vb_wd_sm_permit,vb_data_wd_sm_req_ptr,0.U)
  //-----------------biu w req info---------------------------
  val wd_sm_data_seq_id = Seq.fill(VB_DATA_ENTRY)(Wire(Bool()))
  wd_sm_data_seq_id.zipWithIndex.foreach {
    case (id, i) =>
      id := vb_wd_sm_data_id(i)
  }
  val vb_wd_sm_w_req_data = wd_sm_data_seq_id.zip(io.in.vbSdbIn.writeData128).map {
    case (ptr, data) =>
      Mux(ptr, data, 0.U((XLEN*2).W))
  }.reduce(_ | _)
  //-----------------interface to bus_arb---------------------
  io.out.toBiu.w_req := vb_wd_sm_vld
  val vb_wd_sm_addr_id =  wd_sm_data_seq_id.zip(io.in.vbSdbIn.addrId).map {
    case (ptr, id) =>
      Mux(ptr, id, 0.U(VB_ADDR_ENTRY.W))
  }.reduce(_ | _)
  io.out.toBiu.w_id   := Cat(BIU_VB_ID_T,vb_wd_sm_addr_id)
  io.out.toBiu.w_data := vb_wd_sm_w_req_data
  io.out.toBiu.w_strb := "hffff".U
  io.out.toBiu.w_last := io.out.wdSmData.bias(3)
  io.out.toBiu.w_vld  := vb_wd_sm_vld
  //------------------pop signal------------------------------
  io.out.wdSmData.popReq := Mux(vb_wd_sm_vld_permit,vb_wd_sm_data_id,0.U)
  vb_wd_sm_inv := vb_wd_sm_vld_permit  &&  !vb_data_wd_sm_req
  //==========================================================
  //                      Compare b_id
  //==========================================================
  val vb_biu_b_id_hit = io.in.biuIn.bVld && (io.in.biuIn.bId === BIU_VB_ID_T)
  val vb_biu_id_1to0 = io.in.biuIn.bId(1,0)
  val vb_addr_b_resp_ptr = UIntToOH(vb_biu_id_1to0)
  vb_addr_entry_b_resp := Mux(vb_biu_b_id_hit ,vb_addr_b_resp_ptr ,0.U)
  //==========================================================
  //              Interface to other module
  //==========================================================
  //---------------------hit idx------------------------------
  io.out.rbHitIdx                 := vb_addr_entry_rb_biu_req_hit_idx.reduce(_ || _)
  io.out.pfuHitIdx                := vb_addr_entry_pfu_biu_req_hit_idx.reduce(_ || _)
  io.out.toWmb.write_req_hit_idx  := vb_addr_entry_wmb_write_req_hit_idx.reduce(_ || _)
  io.out.toLfb.vbReqHitIdx        := vb_addr_entry_lfb_vb_req_hit_idx.reduce(_ || _)
  //for snq
  io.out.toSnq := DontCare
  io.out.snqDataBypassHit := DontCare
  //vb addr pop when bypass
  val bypass_pop_seq_ptr = Seq.fill(VB_DATA_ENTRY)(Wire(Bool()))
  bypass_pop_seq_ptr.zipWithIndex.foreach {
    case (ptr, i) =>
      ptr := io.in.vbSdbIn.bypassPop(i)
  }
  vb_addr_entry_bypass_pop := bypass_pop_seq_ptr.zip(io.in.vbSdbIn.addrId).map {
    case (ptr, id) =>
      Mux(ptr, id, 0.U((VB_ADDR_ENTRY).W))
  }.reduce(_ | _)
  val normal_pop_seq_ptr = Seq.fill(VB_DATA_ENTRY)(Wire(Bool()))
  normal_pop_seq_ptr.zipWithIndex.foreach {
    case (ptr, i) =>
      ptr := io.in.vbSdbIn.normalPop(i)
  }
  val vb_addr_entry_data_pop = normal_pop_seq_ptr.zip(io.in.vbSdbIn.addrId).map {
    case (ptr, id) =>
      Mux(ptr, id, 0.U((VB_ADDR_ENTRY).W))
  }.reduce(_ | _)
  //------------------full/empty signal-----------------------
  val vb_addr_empty = !(VecInit(vb_addr_entry_vld).asUInt.orR)
  val vb_data_empty = !(io.in.vbSdbIn.vld.orR)
  io.out.vbEmpty     := vb_addr_empty &&  vb_data_empty
  io.out.toWmb.empty := !(io.in.vbSdbIn.vld & VecInit(vb_addr_entry_wmb_create).asUInt).orR

  // input on the bottom
  addr_entries.zipWithIndex.foreach{
    case(entry, i) =>
      // input
      entry.io.in.cp0In := io.in.cp0In
      entry.io.in.evictReqSuccess := evict_req_success
      // icc broadcast
      entry.io.in.iccIn.addrTto6 := io.in.iccIn.addrTto6
      entry.io.in.iccIn.inv      := io.in.iccIn.inv
      // lfb broadcast
      entry.io.in.lfbIn.addrTto6 := io.in.lfbIn.addrTto6
      entry.io.in.lfbIn.id       := io.in.lfbIn.id
      // snq broadcast
      entry.io.in.snqIn.bypassAddrTto6 := io.in.snqIn.bypassAddrTto6
      entry.io.in.snqIn.createAddr     := io.in.snqIn.createAddr
      // req addr broadcast
      entry.io.in.pfuBiuReqAddr := io.in.pfuBiuReqAddr
      entry.io.in.rbBiuReqAddr  := io.in.rbBiuReqAddr
      entry.io.in.stDaFeedbackAddrTto14 := io.in.stDaIn.feedbackDddrTto14
      // wmb broadcast
      entry.io.in.wmbIn := io.in.wmbIn.toEntry
      // ptr vld
      entry.io.in.vbIn.createVld_x        := vb_addr_entry_create_vld(i)
      entry.io.in.vbIn.iccCreateDpVld_x   := vb_addr_entry_icc_create_dp_vld(i)
      entry.io.in.vbIn.lfbCreateDpVld_x   := vb_addr_entry_lfb_create_dp_vld(i)
      entry.io.in.vbIn.wmbCreateDpVld_x   := vb_addr_entry_wmb_create_dp_vld(i)
      entry.io.in.vbIn.createGateclk_en_x := vb_addr_entry_create_gateclk_en(i)
      entry.io.in.vbIn.rcl_sm_addr_grnt_x := vb_rcl_sm_addr_grnt(i) // to close sm req


      entry.io.in.vbIn.id                 := vb_data_entry_id
      entry.io.in.vbIn.createData_x       := vb_addr_entry_create_data(i)

      entry.io.in.vbIn.feedbackVld_x    := vb_addr_entry_feedback_vld(i)
      entry.io.in.vbIn.pop_x            := vb_addr_entry_pop(i)
      entry.io.in.vbIn.dataPop_x        := vb_addr_entry_data_pop(i)

      entry.io.in.vbIn.rcl_sm_done_x    := vb_rcl_sm_done(i)

  }




}
