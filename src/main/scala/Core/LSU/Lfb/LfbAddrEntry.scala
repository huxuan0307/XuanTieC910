package Core.LSU.Lfb

import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToLfbEntry extends Bundle with LsuConfig {
  val lsuDcacheEn = Bool()
  val lsuIcgEn    = Bool()
  val yyClkEn     = Bool()
}
class LdDaToLfbAddrEntry extends Bundle with LsuConfig with DCacheConfig{
  val idx             = UInt(LFB_ADDR_ENTRY.W)
  val lfbDiscardGrnt  = Bool()
}
class LfbAddrCtrl extends Bundle with LsuConfig {
  val addrEntryCreateGateclkEn_x = Bool()
  val addrEntryPfuCreateDpVld_x  = Bool()
  val addrEntryPfuCreateVld_x    = Bool()
  val addrEntryRbCreateDpVld_x   = Bool()
  val addrEntryRbCreateVld_x     = Bool()
  val addrEntryRespSet_x         = Bool()
  val addrEntryVbPeReqGrnt_x     = Bool()
  val dataAddrPopReq_x           = Bool()
  val lfSmAddrPopReq_x           = Bool()
  val vbPeReq                    = Bool()
  val vbPeReqPermit              = Bool()
}
class PfuToLfbAddrEntry extends Bundle with LsuConfig with DCacheConfig{
  val addr = UInt(PA_WIDTH.W)
  val id = UInt(PFU_IDX.W)
}
class RbToLfbAddrEntry extends Bundle with LsuConfig{
  val biuReqAddr  = UInt(PA_WIDTH.W)
  val addrTto4  = UInt((PA_WIDTH-4).W)
  val atomic    = Bool()
  val depd      = Bool()
  val ldamo     = Bool()
}
class VbToLfbAddrEntry extends Bundle with LsuConfig{
  val addrEntryRclDone_x = Bool()
  val dcacheDirty        = Bool()
  val dcacheHit          = Bool()
  val dcacheWay          = Bool()
}
class WmbToLfbAddrEntry extends Bundle with LsuConfig{
  val readAddr = UInt(PA_WIDTH.W)
  val writeAddr = UInt(PA_WIDTH.W)
}
//----------------------------------------------------------
class LfbAddrEntryIn extends Bundle with LsuConfig {
  val cp0In  = new Cp0ToLfbEntry
  val ldDaIn = new LdDaToLfbAddrEntry
  val lfbIn  = new LfbAddrCtrl
  val lmAlreadySnoop = Bool()
  val lsuSpecialClk = Bool()
  val pfuIn  = new PfuToLfbAddrEntry
  val rbIn   = new RbToLfbAddrEntry
  val snqBypassAddrTto6 = UInt((PA_WIDTH-6).W)
  val stDaAddr = UInt(PA_WIDTH.W)
  val vbIn   = new VbToLfbAddrEntry
  val wmbReq = new WmbToLfbAddrEntry
}
//==========================================================
//                        Output
//==========================================================
class LfbAddrEntryOut extends Bundle with LsuConfig with DCacheConfig {
  val addrEntryAddrTto4_v          = UInt((PA_WIDTH-4).W)
  val addrEntryDcacheHit_x         = Bool()
  val addrEntryDepd_x              = Bool()
  val addrEntryDiscardVld_x        = Bool()
  val addrEntryLdDaHitIdx_x        = Bool()
  val addrEntryLdamo_x             = Bool()
  val addrEntryLinefillAbort_x     = Bool()
  val addrEntryLinefillPermit_x    = Bool()
  val addrEntryNotResp_x           = Bool()
  val addrEntryPfuBiuReqHitIdx_x   = Bool()
  val addrEntryPfuDcacheHit_v      = UInt(PFU_ENTRY.W)
  val addrEntryPfuDcacheMiss_v     = UInt(PFU_ENTRY.W)
  val addrEntryPopVld_x            = Bool()
  val addrEntryRbBiuReqHitIdx_x    = Bool()
  val addrEntryRclDone_x           = Bool()
  val addrEntryRefillWay_x         = Bool()
  val addrEntrySnqBypassHit_x      = Bool()
  val addrEntryStDaHitIdx_x        = Bool()
  val addrEntryVbPeReq_x           = Bool()
  val addrEntryVld_x               = Bool()
  val addrEntryWmbReadReqHitIdx_x  = Bool()
  val addrEntryWmbWriteReqHitIdx_x = Bool()
}
//==========================================================
//                          IO
//==========================================================
class LfbAddrEntryIO extends Bundle with LsuConfig {
  val in  = Input(new LfbAddrEntryIn)
  val out = Output(new LfbAddrEntryOut)
}

class LfbAddrEntry extends Module with LsuConfig with DCacheConfig {
  val io = IO(new LfbAddrEntryIO)


  //------------------input-----------------------------------
  //-----------create signal--------------/
  val lfb_addr_entry_vld = RegInit(false.B)
  val lfb_addr_entry_rb_create_vld     = io.in.lfbIn.addrEntryRbCreateVld_x
  val lfb_addr_entry_rb_create_dp_vld  = io.in.lfbIn.addrEntryRbCreateDpVld_x
  val lfb_addr_entry_pfu_create_vld    = io.in.lfbIn.addrEntryPfuCreateVld_x
  val lfb_addr_entry_pfu_create_dp_vld = io.in.lfbIn.addrEntryPfuCreateDpVld_x
  val lfb_addr_entry_create_gateclk_en = io.in.lfbIn.addrEntryCreateGateclkEn_x
  //-----------grnt signal----------------
  val lfb_addr_entry_vb_pe_req_grnt = io.in.lfbIn.addrEntryVbPeReqGrnt_x
  //-----------other signal---------------
  val vb_lfb_addr_entry_rcl_done  = io.in.vbIn.addrEntryRclDone_x
  val lfb_data_addr_pop_req       = io.in.lfbIn.dataAddrPopReq_x
  val lfb_lf_sm_addr_pop_req      = io.in.lfbIn.lfSmAddrPopReq_x
  val lfb_addr_entry_resp_set     = io.in.lfbIn.addrEntryRespSet_x
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //-----------create gateclk-------------
  val lfb_addr_entry_create_clk_en = lfb_addr_entry_create_gateclk_en
  //-----------entry gateclk--------------
  //normal gateclk ,open when create || entry_vld
  val lfb_addr_entry_clk_en = lfb_addr_entry_vld || lfb_addr_entry_create_clk_en
  //==========================================================
  //                 Register
  //==========================================================
  //+-----------+
  //| entry_vld |
  //+-----------+
  val lfb_addr_entry_pop_vld = Wire(Bool())
  when(lfb_addr_entry_pop_vld){
    lfb_addr_entry_vld := false.B  // if pop, not vld
  }.elsewhen(lfb_addr_entry_rb_create_vld || lfb_addr_entry_pfu_create_vld){
    lfb_addr_entry_vld := true.B // else create by rb / pfu
  }
  //+------+--------+
  //| addr | pfu_id |
  //+------+--------+
  val lfb_addr_entry_pfu_create = RegInit(false.B)
  val lfb_addr_entry_atomic     = RegInit(false.B)
  val lfb_addr_entry_ldamo      = RegInit(false.B)
  val lfb_addr_entry_pfu_id     = RegInit(0.U(PFU_IDX.W))
  val lfb_addr_entry_addr_tto4  = RegInit(0.U((PA_WIDTH-4).W))
  when(lfb_addr_entry_rb_create_dp_vld){ // switch choose rb / pfu, rb > pfu
    // rb data
    lfb_addr_entry_pfu_create := false.B
    lfb_addr_entry_atomic     := io.in.rbIn.atomic
    lfb_addr_entry_ldamo      := io.in.rbIn.ldamo
    lfb_addr_entry_pfu_id     := 0.U(PFU_IDX.W)
    lfb_addr_entry_addr_tto4  := io.in.rbIn.addrTto4
  }.elsewhen(lfb_addr_entry_pfu_create_dp_vld){
    // pfu data
    lfb_addr_entry_pfu_create := true.B
    lfb_addr_entry_atomic     := false.B
    lfb_addr_entry_ldamo      := false.B
    lfb_addr_entry_pfu_id     := io.in.pfuIn.id
    lfb_addr_entry_addr_tto4  := io.in.pfuIn.addr
  }
  //+--------------------+
  //| vb_pe_req_success |
  //+--------------------+
  val lfb_addr_entry_vb_pe_req_success = RegInit(false.B)
  val lfb_addr_entry_vb_pe_req_success_set = Wire(Bool())
  val lfb_addr_entry_create_dp_vld = Wire(Bool())
  when(lfb_addr_entry_vb_pe_req_success_set){
    lfb_addr_entry_vb_pe_req_success := true.B
  }.elsewhen(lfb_addr_entry_create_dp_vld){
    lfb_addr_entry_vb_pe_req_success := !io.in.cp0In.lsuDcacheEn
  }
  //+-----------------+
  //| cache line info |
  //+-----------------+
  val lfb_addr_entry_rcl_done     = RegInit(false.B)
  val lfb_addr_entry_refill_way   = RegInit(false.B)
  val lfb_addr_entry_dcache_hit   = RegInit(false.B)
  val lfb_addr_entry_dcache_dirty = RegInit(false.B)
  when(lfb_addr_entry_create_dp_vld){
    lfb_addr_entry_rcl_done     := !io.in.cp0In.lsuDcacheEn // from Cp0 - SHCR
    //HCR[1] - Dcache enable
    lfb_addr_entry_refill_way   := false.B
    lfb_addr_entry_dcache_hit   := false.B
    lfb_addr_entry_dcache_dirty := false.B
  }.elsewhen(vb_lfb_addr_entry_rcl_done){ // todo from vb?
    lfb_addr_entry_rcl_done     := true.B
    lfb_addr_entry_refill_way   := io.in.vbIn.dcacheWay
    lfb_addr_entry_dcache_hit   := io.in.vbIn.dcacheHit
    lfb_addr_entry_dcache_dirty := io.in.vbIn.dcacheDirty
  }
  //+---------------+
  //| already_reply |
  //+---------------+
  //if pfu create dcache hit, then reply dcache hit signal to prb
  val lfb_addr_entry_discard_vld = Wire(Bool())
  val lfb_addr_entry_already_reply = RegInit(false.B)
  when(lfb_addr_entry_create_dp_vld){
    lfb_addr_entry_already_reply := false.B
  }.elsewhen(lfb_addr_entry_rcl_done){
    lfb_addr_entry_already_reply := true.B
  }
  //+------+
  //| depd |
  //+------+
  val lfb_addr_entry_depd = RegInit(false.B)
  when(lfb_addr_entry_rb_create_dp_vld){
    lfb_addr_entry_depd := io.in.rbIn.depd
  }.elsewhen(lfb_addr_entry_pfu_create_dp_vld){
    lfb_addr_entry_depd := false.B
  }.elsewhen(lfb_addr_entry_discard_vld){
    lfb_addr_entry_depd := true.B
  }
  //+------+
  //| resp |
  //+------+
  val lfb_addr_entry_resp = RegInit(false.B)
  when(lfb_addr_entry_create_dp_vld){
    lfb_addr_entry_resp := false.B
  }.elsewhen(lfb_addr_entry_resp_set){
    lfb_addr_entry_resp := true.B
  }
  //==========================================================
  //                 Generate create signal
  //==========================================================
  lfb_addr_entry_create_dp_vld := lfb_addr_entry_rb_create_dp_vld  ||  lfb_addr_entry_pfu_create_dp_vld
  //==========================================================
  //                 Generate vb req siganl
  //==========================================================
  val lfb_addr_entry_vb_pe_req = lfb_addr_entry_vld && (!lfb_addr_entry_vb_pe_req_success)
  lfb_addr_entry_vb_pe_req_success_set := lfb_addr_entry_create_dp_vld &&
    io.in.lfbIn.vbPeReqPermit && (!io.in.lfbIn.vbPeReq) ||
    lfb_addr_entry_vb_pe_req && lfb_addr_entry_vb_pe_req_grnt
  //==========================================================
  //            Linefill permit
  //==========================================================
  // if snoop(coherence) & hit then abort
  val lfb_addr_entry_atomic_abort = (!io.in.lmAlreadySnoop) &&
    lfb_addr_entry_dcache_hit
  val lfb_addr_entry_linefill_permit = lfb_addr_entry_rcl_done && (!lfb_addr_entry_dcache_hit ||
    lfb_addr_entry_atomic && !lfb_addr_entry_atomic_abort)
  val lfb_addr_entry_linefill_abort  = lfb_addr_entry_rcl_done && lfb_addr_entry_dcache_hit &&
    (lfb_addr_entry_pfu_create || lfb_addr_entry_atomic && lfb_addr_entry_atomic_abort)
  val lfb_addr_entry_not_resp = lfb_addr_entry_vld && (!lfb_addr_entry_resp)
  //==========================================================
  //            Reply dcache hit signal to pfu
  //==========================================================
  val lfb_addr_entry_pfu_id_oh = Wire(Vec(PFU_ENTRY,Bool()))
  for(i<-0 until( PFU_ENTRY)){
    when(i.U === UIntToOH(lfb_addr_entry_pfu_id)){
      lfb_addr_entry_pfu_id_oh(i) := true.B
    }.otherwise{
      lfb_addr_entry_pfu_id_oh(i) := false.B
    }
  }
  val lfb_addr_entry_pfu_reply_vld = lfb_addr_entry_vld &&  lfb_addr_entry_pfu_create  &&
    lfb_addr_entry_rcl_done    &&  (!lfb_addr_entry_already_reply)
  val lfb_addr_entry_pfu_dcache_hit_vld  = lfb_addr_entry_pfu_reply_vld && lfb_addr_entry_dcache_hit
  val lfb_addr_entry_pfu_dcache_miss_vld = lfb_addr_entry_pfu_reply_vld &&  (!lfb_addr_entry_dcache_hit)
  val lfb_addr_entry_pfu_dcache_hit = Wire(Vec(PFU_ENTRY,Bool()))
  val lfb_addr_entry_pfu_dcache_miss = Wire(Vec(PFU_ENTRY,Bool()))
  for(i<-0 until( PFU_ENTRY)){
    lfb_addr_entry_pfu_dcache_hit(i) := lfb_addr_entry_pfu_dcache_hit_vld && lfb_addr_entry_pfu_id_oh(i)
    lfb_addr_entry_pfu_dcache_miss(i) := lfb_addr_entry_pfu_dcache_miss_vld && lfb_addr_entry_pfu_id_oh(i)
  }
  //==========================================================
  //                 Generate pop signal
  //=========================================================
  lfb_addr_entry_pop_vld := lfb_lf_sm_addr_pop_req || lfb_data_addr_pop_req
  //==========================================================
  //                    Compare index
  //==========================================================
  //------------------compare ld_da stage---------------------
  val lfb_addr_entry_ld_da_hit_idx = lfb_addr_entry_vld && (lfb_addr_entry_addr_tto4(9,2) === io.in.ldDaIn.idx)
  //------------------compare st_da stage---------------------
  val lfb_addr_entry_cmp_st_da_addr = io.in.stDaAddr
  val lfb_addr_entry_st_da_hit_idx = lfb_addr_entry_vld && (lfb_addr_entry_addr_tto4(9,2) === lfb_addr_entry_cmp_st_da_addr(13,6))
  //------------------depd_vld--------------------------------
  lfb_addr_entry_discard_vld := io.in.ldDaIn.lfbDiscardGrnt && lfb_addr_entry_ld_da_hit_idx
  //----------------compare rb biu req entry------------------
  val lfb_addr_entry_cmp_rb_biu_req_addr = io.in.rbIn.biuReqAddr
  val lfb_addr_entry_rb_biu_req_hit_idx = lfb_addr_entry_vld && (lfb_addr_entry_addr_tto4(9,2) === lfb_addr_entry_cmp_rb_biu_req_addr(13,6))
  //------------------compare pfu pop entry-------------------
  val lfb_addr_entry_cmp_pfu_biu_req_addr = io.in.pfuIn.addr
  val lfb_addr_entry_pfu_biu_req_hit_idx = lfb_addr_entry_vld && (lfb_addr_entry_addr_tto4(9,2) === lfb_addr_entry_cmp_pfu_biu_req_addr(13,6))
  //------------------compare wmb read req--------------------
  val lfb_addr_entry_cmp_wmb_read_req_addr = io.in.wmbReq.readAddr
  val lfb_addr_entry_wmb_read_req_hit_idx = lfb_addr_entry_vld && (lfb_addr_entry_addr_tto4(9,2) === lfb_addr_entry_cmp_wmb_read_req_addr(13,6))
  //------------------compare wmb write req-------------------
  val lfb_addr_entry_cmp_wmb_write_req_addr = io.in.wmbReq.writeAddr
  val lfb_addr_entry_wmb_write_req_hit_idx = lfb_addr_entry_vld && (lfb_addr_entry_addr_tto4(9,2) === lfb_addr_entry_cmp_wmb_write_req_addr(13,6))
  //---------------compare snq bypass req addr----------------
  val lfb_addr_entry_cmp_snq_bypass_addr_tto6 = io.in.snqBypassAddrTto6
  val lfb_addr_entry_snq_bypass_hit = lfb_addr_entry_vld  && lfb_addr_entry_resp    &&
    !lfb_addr_entry_rcl_done && (lfb_addr_entry_addr_tto4(PA_WIDTH-5,2) === lfb_addr_entry_cmp_wmb_write_req_addr(13,6))
  //==========================================================
  //                 Generate interface
  //==========================================================
  //------------------output----------------------------------
  //-----------entry signal---------------
  io.out.addrEntryVld_x       := lfb_addr_entry_vld
  io.out.addrEntryAddrTto4_v  := lfb_addr_entry_addr_tto4
  io.out.addrEntryRefillWay_x := lfb_addr_entry_refill_way
  io.out.addrEntryDepd_x      := lfb_addr_entry_depd
  io.out.addrEntryRclDone_x   := lfb_addr_entry_rcl_done
  io.out.addrEntryDcacheHit_x := lfb_addr_entry_dcache_hit
  io.out.addrEntryLdamo_x     := lfb_addr_entry_ldamo
  io.out.addrEntryNotResp_x   := lfb_addr_entry_not_resp
  //-----------request--------------------
  io.out.addrEntryVbPeReq_x       := lfb_addr_entry_vb_pe_req
  io.out.addrEntryPopVld_x        := lfb_addr_entry_pop_vld
  io.out.addrEntryDiscardVld_x    := lfb_addr_entry_discard_vld
  io.out.addrEntryPfuDcacheHit_v  := lfb_addr_entry_pfu_dcache_hit.asUInt
  io.out.addrEntryPfuDcacheMiss_v := lfb_addr_entry_pfu_dcache_miss.asUInt

  io.out.addrEntryLinefillPermit_x := lfb_addr_entry_linefill_permit
  io.out.addrEntryLinefillAbort_x  := lfb_addr_entry_linefill_abort

  io.out.addrEntryLdDaHitIdx_x := lfb_addr_entry_ld_da_hit_idx
  io.out.addrEntryStDaHitIdx_x := lfb_addr_entry_st_da_hit_idx

  io.out.addrEntryRbBiuReqHitIdx_x    := lfb_addr_entry_rb_biu_req_hit_idx
  io.out.addrEntryPfuBiuReqHitIdx_x   := lfb_addr_entry_pfu_biu_req_hit_idx
  io.out.addrEntryWmbReadReqHitIdx_x  := lfb_addr_entry_wmb_read_req_hit_idx
  io.out.addrEntryWmbWriteReqHitIdx_x := lfb_addr_entry_wmb_write_req_hit_idx
  io.out.addrEntrySnqBypassHit_x      := lfb_addr_entry_snq_bypass_hit

}
