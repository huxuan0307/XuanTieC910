package Core.LSU.Sq
import Core.IUConfig.MPPWidth
import Core.IntConfig.XLEN
import Core.LSU.RotData
import Core.LSU.StoreExStage.SqToStDc
import Core.LsuConfig
import Core.ROBConfig.IidWidth
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class SdEx1ToSqEntry extends Bundle with LsuConfig {
  val ex1InstVld     = Bool()
  val rfEx1Sdid      = UInt(SDID_WIDTH.W)
  val rfInstVldShort = Bool()
}
class SdEx1ToSq extends Bundle with LsuConfig {
  val toSqEntry          = new SdEx1ToSqEntry
  val sd_ex1_data        = UInt(XLEN.W)
  val sd_ex1_data_bypass = Vec(2,UInt(XLEN.W))
}
class IccToSq extends Bundle with LsuConfig {
  val idle   = Bool()
  val sqGrnt= Bool()
}
class LdDaToSqEntry extends Bundle with LsuConfig{
  val lsid                  = UInt(LSIQ_ENTRY.W)
  val sqDataDiscardVld   = Bool()
  val sqFwdId             = UInt(LSIQ_ENTRY.W)
  val sqFwdMultiVld      = Bool()
  val sqGlobalDiscardVld = Bool()
}
class RbToSq extends Bundle with LsuConfig{
  val rtuAllCommitLdDataVld = Bool()
  val sqPopHitIdx           = Bool()
}
class WmbToSq extends Bundle with LsuConfig{
  val addr         = UInt(PA_WIDTH.W)
  val dcacheSwInst = Bool()
  val sqPtr        = UInt(LSIQ_ENTRY.W)
  val ceVld        = Bool()
  val popGrnt      = Bool()
  val popToCeGrnt  = Bool()
}
//----------------------------------------------------------
class SqIn extends Bundle with LsuConfig{
  val sdEx1In  = new SdEx1ToSq
  val cp0In    = new Cp0ToSqEntry
  val dcacheIn = new DcacheToSqEntry
  val iccIn    = new IccToSq
  val ldDcIn   = new LdDcToSqEntry
  val ldDaIn   = new LdDaToSqEntry
  val rbIn     = new RbToSq
  val rtuIn    = new RtuToSqEntry
  val daIn     = new StDaToSqTotal
  val dcIn     = new StDcToSqTotal
  val wmbIn    = new WmbToSq
}
//==========================================================
//                        Output
//==========================================================
class SqToCtrl extends Bundle with LsuConfig{
  val dataDepdWakeup   = UInt(LSIQ_ENTRY.W)
  val empty            = Bool() // TODO to ctrl and icc
  val globalDepdWakeup = UInt(LSIQ_ENTRY.W)
}
class SqToIcc extends Bundle with LsuConfig{
  val clr = Bool()
  val inv = Bool()
  val req = Bool()
}
class SqToLdDa extends Bundle with LsuConfig{
  val fwdData                = UInt(XLEN.W)
  val fwdDataPe             = UInt(XLEN.W)
  }
class SqToLdDc extends Bundle with LsuConfig{
  val addr1DepDiscard     = Bool()
  val cancelAccReq        = Bool()
  val cancelAheadWb       = Bool()
  val dataDiscardReq      = Bool()
  val fwdBypassMulti      = Bool()
  val fwdBypassReq        = Bool()
  val fwdId               = UInt(LSIQ_ENTRY.W)
  val fwdMulti            = Bool()
  val fwdMultiMask        = Bool()
  val fwdReq              = Bool()
  val hasFwdReq           = Bool()
  val newestFwdDataVldReq = Bool()
  val otherDiscardReq     = Bool()
}
class SqToWmbCe extends Bundle with LsuConfig{
  val popAtomic    = Bool()
  val popBytesVld  = UInt(BYTES_ACCESS_WIDTH.W)
  val popIcc       = Bool()
  val popAddr      = UInt(PA_WIDTH.W)
  val popInstFlush = Bool()
  val popInstMode  = UInt(INST_MODE_WIDTH.W)
  val popInstSize  = UInt(INST_SIZE_WIDTH.W)
  val popInstType  = UInt(INST_TYPE_WIDTH.W)
  val popPageBuf   = Bool()
  val popPageCa    = Bool()
  val popPageSec   = Bool()
  val popPageShare = Bool()
  val popPageSo    = Bool()
  val popPageWa    = Bool()
  val popPrivMode  = UInt(MPPWidth.W)
  val popPtr       = UInt(LSIQ_ENTRY.W)
  val popSyncFence = Bool()
  val popWoSt      = Bool()
}
class SqToWmb extends Bundle with LsuConfig{
  val ce = new SqToWmbCe
  val mergeReq         = Bool()
  val mergeStallReq    = Bool()
  val popToCeDpReq     = Bool()
  val popToCeGateclkEn = Bool()
  val popToCeReq       = Bool()
  val bkptaData        = Bool()
  val bkptbData        = Bool()
  val createHitRbIdx   = Bool()
  val data128          = UInt((XLEN*2).W)
  val dcacheShare      = Bool()
  val dcacheValid      = Bool()
  val fenceMode        = UInt(FENCE_MODE_WIDTH.W)
  val iid              = UInt(IidWidth.W)
  val specFail         = Bool()
  val updateDcacheMesi = new DcacheDirtyDataEn
  val updateDcacheWay  = Bool()
  val vstartVld        = Bool()
}

//----------------------------------------------------------
class SqOut extends Bundle with LsuConfig{
  val toCtrl = new SqToCtrl
  val toIcc  = new SqToIcc
  val toLdDa = new SqToLdDa
  val toLdDc = new SqToLdDc
  val popSynciInst = Bool() // to PFU
  val toWmb  = new SqToWmb
  val SqNotFull = Bool() //to IDU
  val toDc   = new SqToStDc
  val AllCommitDataVld = Bool() // to RTU

}
//==========================================================
//                          IO
//==========================================================
class SqIO extends Bundle with LsuConfig{
  val in = Input(new SqIn)
  val out = Output(new SqOut)
}
class Sq extends Module with LsuConfig{
  val io = IO(new  SqIO)
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val sq_pop_depd_ff = RegInit(false.B)
  val sq_clk_en = !io.out.toCtrl.empty || io.in.dcIn.sq.sqCreateGateclkEn || sq_pop_depd_ff
  val sq_pop_req_unmask = Wire(Bool())
  val sq_create_pop_clk_en = io.in.dcIn.sq.sqCreateGateclkEn || sq_pop_req_unmask
  val sq_wakeup_queue_clk_en = io.in.ldDaIn.sqGlobalDiscardVld || sq_pop_depd_ff || io.in.rtuIn.flush
  val sq_newest_fwd_req_data_vld_short = Wire(Bool())
  val sq_fwd_data_pe_clk_en = sq_newest_fwd_req_data_vld_short
  val sq_pe_sel_age_vec_zero_entry_vld = Wire(Bool())
  val sq_pop_clk_en = sq_pe_sel_age_vec_zero_entry_vld || sq_pop_req_unmask
  //==========================================================
  //                 Instance 12 entries
  //==========================================================
  val entries = Seq.fill(LSIQ_ENTRY)(Module(new SqEntry))
  //==========================================================
  //                 entries input
  //==========================================================
  entries.zipWithIndex.foreach{
    case(entry, i) =>
      // binded io, dispatch from input
      entry.io.in.dcIn         := io.in.dcIn
      entry.io.in.daIn         := io.in.daIn
      entry.io.in.cp0In        := io.in.cp0In
      entry.io.in.dcacheIn     := io.in.dcacheIn
      entry.io.in.sdEx1In      := io.in.sdEx1In.toSqEntry
      entry.io.in.ldDaLsid     := io.in.ldDaIn.lsid
      entry.io.in.ldDcIn       := io.in.ldDcIn
      entry.io.in.rtuIn        := io.in.rtuIn
      entry.io.in.wmbSqPopGrnt := io.in.wmbIn.popGrnt
  }
  //==========================================================
  //                Generate full/create signal
  //==========================================================
  val sq_entry_cmit                  = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_vld                   = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_inst_hit              = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_st_dc_same_addr_newer = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_st_dc_create_age_vec  = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_settle_rot_sel_vec          = Seq.fill(LSIQ_ENTRY)(Wire(UInt(ROT_SEL_WIDTH_8.W)))
  val sq_entry_cancel_ahead_wb       = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_discard_req           = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_addr1_dep_discard     = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_cancel_acc_req        = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_fwd_bypass_req        = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_same_addr_newest      = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_data_discard_req_short= Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_data_discard_req      = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_newest_fwd_bypass_req_vec   = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_fwd_req               = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_newest_fwd_req_data_vld = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_newest_fwd_req_data_vld_short=Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_data_discard_req_short_vec=Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_data                  = Seq.fill(LSIQ_ENTRY)(Wire(UInt(XLEN.W)))

  //============================================================================

  val sq_entry_data_depd_wakeup      = Seq.fill(LSIQ_ENTRY)(Wire(UInt(LSIQ_ENTRY.W)))
  val sq_entry_age_vec_zero_ptr      = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_age_vec_surplus1_ptr  = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))

  //============================================================================

  val sq_entry_page_ca               = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_page_wa               = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_page_so               = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_page_buf              = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_page_sec              = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_page_share            = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))

  //============================================================================

  val sq_entry_atomic                = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_icc                   = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))

  //============================================================================

  val sq_entry_wo_st                 = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_sync_fence            = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_inst_flush            = Seq.fill(LSIQ_ENTRY)(Wire(UInt(LSIQ_ENTRY.W)))
  val sq_entry_inst_type             = Seq.fill(LSIQ_ENTRY)(Wire(UInt(INST_TYPE_WIDTH.W)))
  val sq_entry_inst_size             = Seq.fill(LSIQ_ENTRY)(Wire(UInt(INST_SIZE_WIDTH.W)))
  val sq_entry_inst_mode             = Seq.fill(LSIQ_ENTRY)(Wire(UInt(INST_MODE_WIDTH.W)))
  val sq_entry_priv_mode             = Seq.fill(LSIQ_ENTRY)(Wire(UInt(MPPWidth.W)))
  val sq_entry_fence_mode            = Seq.fill(LSIQ_ENTRY)(Wire(UInt(FENCE_MODE_WIDTH.W)))
  val sq_entry_addr0                 = Seq.fill(LSIQ_ENTRY)(Wire(UInt(PA_WIDTH.W)))

  //============================================================================

  val sq_entry_bytes_vld             = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_dcache_dirty          = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_dcache_info_vld       = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_dcache_share          = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_dcache_valid          = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_dcache_way            = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))

  //============================================================================

  val sq_entry_pop_req               = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_iid                   = Seq.fill(LSIQ_ENTRY)(Wire(UInt(IidWidth.W)))
  val sq_entry_spec_fail             = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_bkpta_data            = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_bkptb_data            = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_vstart_vld            = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))

  //============================================================================

  val sq_entry_depd                  = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_depd_set              = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))
  val sq_entry_cmit_data_vld         = Seq.fill(LSIQ_ENTRY)(Wire(Bool()))

  //==========================================================
  //                          Assign
  //==========================================================
  entries.zipWithIndex.foreach{
    case(entry, i) =>
      sq_entry_cmit(i)                  := entry.io.out.cmit_x && entry.io.out.vld_x
      sq_entry_vld(i)                   := entry.io.out.vld_x
      sq_entry_inst_hit(i)              := entry.io.out.instHit_x
      sq_entry_st_dc_same_addr_newer(i) := entry.io.out.stDcSameAddrNewer_x
      sq_entry_st_dc_create_age_vec(i)  := entry.io.out.stDcCreateAgeVec_x
      sq_settle_rot_sel_vec(i)          := Cat(Seq.fill(ROT_SEL_WIDTH_8)(entry.io.out.settleDataHit_x)) & entry.io.out.rotSel_v
      sq_entry_cancel_ahead_wb(i)       := entry.io.out.cancelAheadWb_x
      sq_entry_discard_req(i)           := entry.io.out.discardReq_x
      sq_entry_addr1_dep_discard(i)     := entry.io.out.addr1DepDiscard_x
      sq_entry_cancel_acc_req(i)        := entry.io.out.cancelAccReq_x
      sq_entry_fwd_bypass_req(i)        := entry.io.out.fwdBypassReq_x
      sq_entry_same_addr_newest(i)      := entry.io.out.sameAddrNewest_x
      sq_entry_data_discard_req_short(i):= entry.io.out.dataDiscardReqShort_x
      sq_entry_data_discard_req(i)      := entry.io.out.dataDiscardReq_x
      sq_newest_fwd_bypass_req_vec(i)   := entry.io.out.fwdBypassReq_x && entry.io.out.sameAddrNewest_x
      sq_entry_newest_fwd_req_data_vld(i):= entry.io.out.newestFwdReqDataVld_x
      sq_entry_newest_fwd_req_data_vld_short(i):=entry.io.out.newestFwdReqDataVldShort_x
      sq_entry_data_discard_req_short_vec(i):= entry.io.out.dataDiscardReqShort_x

      //============================================================================

      sq_entry_data(i)                  := entry.io.out.data_v
      sq_entry_data_depd_wakeup(i)      := entry.io.out.dataDepdWakeup_v
      sq_entry_age_vec_zero_ptr(i)      := entry.io.out.ageVecZeroPtr_x
      sq_entry_age_vec_surplus1_ptr(i)  := entry.io.out.ageVecSurplus1Ptr_x

      //============================================================================

      sq_entry_page_ca(i)               := entry.io.out.pageCa_x
      sq_entry_page_wa(i)               := entry.io.out.pageWa_x
      sq_entry_page_so(i)               := entry.io.out.pageSo_x
      sq_entry_page_buf(i)              := entry.io.out.pageBuf_x
      sq_entry_page_sec(i)              := entry.io.out.pageSec_x
      sq_entry_page_share(i)            := entry.io.out.pageShare_x

      //============================================================================

      sq_entry_atomic(i)                := entry.io.out.atomic_x
      sq_entry_icc(i)                   := entry.io.out.icc_x

      //============================================================================

      sq_entry_wo_st(i)                 := entry.io.out.woSt_x
      sq_entry_sync_fence(i)            := entry.io.out.syncFence_x
      sq_entry_inst_flush(i)            := entry.io.out.instFlush_x
      sq_entry_inst_type(i)             := entry.io.out.instType_v
      sq_entry_inst_size(i)             := entry.io.out.instSize_v
      sq_entry_inst_mode(i)             := entry.io.out.instMode_v
      sq_entry_priv_mode(i)             := entry.io.out.privMode_v
      sq_entry_fence_mode(i)            := entry.io.out.fenceMode_v
      sq_entry_addr0(i)                 := entry.io.out.addr0_v

      //============================================================================

      sq_entry_bytes_vld(i)             := entry.io.out.bytesVld_v
      sq_entry_dcache_dirty(i)          := entry.io.out.dcacheDirty_x
      sq_entry_dcache_info_vld(i)       := entry.io.out.dcacheInfo_vld_x
      sq_entry_dcache_share(i)          := entry.io.out.dcacheShare_x
      sq_entry_dcache_valid(i)          := entry.io.out.dcacheValid_x
      sq_entry_dcache_way(i)            := entry.io.out.dcacheWay_x

      //============================================================================

      sq_entry_pop_req(i)               := entry.io.out.dcacheWay_x
      sq_entry_iid(i)                   := entry.io.out.iid_v
      sq_entry_spec_fail(i)             := entry.io.out.specFail_x
      sq_entry_bkpta_data(i)            := entry.io.out.bkptaData_x
      sq_entry_bkptb_data(i)            := entry.io.out.bkptbData_x
      sq_entry_vstart_vld(i)            := entry.io.out.vstartVld_x

      //============================================================================

      sq_entry_depd(i)                  := entry.io.out.depd_x
      sq_entry_depd_set(i)              := entry.io.out.depdSet_x
      sq_entry_cmit_data_vld(i)         := entry.io.out.cmitData_vld_x
      sq_entry_fwd_req(i)               := entry.io.out.fwdReq_x
  }
  //------------------create ptr------------------------------
  //ptr 0 find empty entry from No.0
  val sq_create_ptr = Wire(UInt(LSIQ_ENTRY.W))
  val create_ptr_idx = PriorityEncoder((~(VecInit(sq_entry_vld).asUInt)).asUInt) //sq_entry_vld.takeWhile(_ == false.B).length // todo  use  PriorityEncoderOH
  sq_create_ptr := Mux(sq_entry_vld.reduce(_ && _) , 0.U(LSIQ_ENTRY.W),
    UIntToOH(create_ptr_idx))

  //------------------full signal-----------------------------
  val sq_has_cmit = sq_entry_cmit.reduce(_ || _)
  val sq_full     = sq_entry_vld.reduce(_ && _)
  for (i <- 0 until LSIQ_ENTRY) {
    if(create_ptr_idx == i.U){

    }
  }
//  val sq_empty_less2 = Mux(sq_entry_vld.reduce(_ && _) , true.B,
//    sq_entry_vld.drop(create_ptr_idx-1).reduce(_ && _) )
  val sq_empty_less2 = (VecInit(sq_entry_vld).asUInt | sq_create_ptr).andR
  io.out.toDc.instHit := sq_entry_inst_hit.reduce(_ || _)
  io.out.toDc.full    := sq_full || (!io.in.dcIn.sqda.old) && sq_empty_less2 && (!sq_has_cmit)
  //------------------empty signal----------------------------
  io.out.toCtrl.empty := !sq_entry_vld.reduce(_ || _)
  //------------------create signal---------------------------
  val sq_create_success = io.in.dcIn.sq.sqCreateVld && (! io.out.toDc.full) && (!io.in.rtuIn.flush)
  val sq_entry_create_vld        = Wire(UInt(LSIQ_ENTRY.W))
  val sq_entry_create_dp_vld     = Wire(UInt(LSIQ_ENTRY.W))
  val sq_entry_create_gateclk_en = Wire(UInt(LSIQ_ENTRY.W))
  sq_entry_create_vld        := Cat(Seq.fill(LSIQ_ENTRY)(sq_create_success)) & sq_create_ptr
  sq_entry_create_dp_vld     := Cat(Seq.fill(LSIQ_ENTRY)(io.in.dcIn.sq.sqCreateDpVld)) & sq_create_ptr
  sq_entry_create_gateclk_en := Cat(Seq.fill(LSIQ_ENTRY)(io.in.dcIn.sq.sqCreateGateclkEn)) & sq_create_ptr
  val sq_create_same_addr_newest = sq_entry_st_dc_same_addr_newer.reduce(_ || _)
  //------------------create age_vec signal-------------------
  val sq_create_vld     = sq_entry_create_vld
  val sq_create_age_vec = sq_entry_st_dc_create_age_vec
  val sq_age_vec_set    = sq_create_success || io.in.wmbIn.popToCeGrnt
  //==========================================================
  //            Settle data to memory mode
  //==========================================================
  //------------------settle data to register mode------------
  val sq_settle_rot_sel = sq_settle_rot_sel_vec.reduce(_ & _)
  val rot_mem_format = Module(new RotData)
  rot_mem_format.io.dataIn := io.in.sdEx1In.sd_ex1_data
  rot_mem_format.io.rotSel := sq_settle_rot_sel
  val sq_data_settle = rot_mem_format.io.dataSettle
  //==========================================================
  //            sq to ld_dc depd/discard signal
  //==========================================================
  //------------------request---------------------------------
  io.out.toLdDc.cancelAheadWb   := io.out.toLdDc.newestFwdDataVldReq && sq_entry_cancel_ahead_wb.reduce(_ || _) || io.out.toLdDc.otherDiscardReq
  io.out.toLdDc.otherDiscardReq := sq_entry_discard_req.reduce(_ || _)
  io.out.toLdDc.addr1DepDiscard := sq_entry_addr1_dep_discard.reduce(_ || _)
  io.out.toLdDc.cancelAccReq    := sq_entry_cancel_acc_req.reduce(_ || _)
  //------------------data depd-------------------------------
  val sq_fwd_bypass_req         = sq_entry_fwd_bypass_req.reduce(_ || _)
  val sq_newest_fwd_bypass_req  = sq_newest_fwd_bypass_req_vec.reduce(_ || _)
  val sq_data_discard_req_short = sq_entry_data_discard_req_short.reduce(_ || _)
  val sq_data_discard_req       = sq_entry_data_discard_req.reduce(_ || _)
  val sq_fwd_req                = sq_entry_fwd_req.reduce(_ || _)
  val sq_newest_fwd_req_id      = (VecInit(sq_entry_fwd_req).asUInt & VecInit(sq_entry_same_addr_newest).asUInt)
  val sq_newest_fwd_req = sq_newest_fwd_req_id.orR
  //------------------judge for depd--------------------------
  //to ld_dc fwd judge
  io.out.toLdDc.hasFwdReq := sq_fwd_req
  io.out.toLdDc.fwdReq    := sq_newest_fwd_req || (!sq_fwd_bypass_req) && sq_fwd_req
  io.out.toLdDc.newestFwdDataVldReq := sq_entry_newest_fwd_req_data_vld.reduce(_ || _)
  sq_newest_fwd_req_data_vld_short :=sq_entry_newest_fwd_req_data_vld_short.reduce(_ || _)
  io.out.toLdDc.dataDiscardReq := sq_data_discard_req && (!sq_newest_fwd_bypass_req) && (!sq_newest_fwd_req)
  io.out.toLdDc.fwdBypassReq := sq_newest_fwd_bypass_req || sq_fwd_bypass_req && (!sq_fwd_req)
  //if fwd_bypass_multi depd, then use sq_entry_data_discard_req_short id
  val sq_fwd_req_id = Mux(sq_newest_fwd_req , sq_newest_fwd_req_id ,VecInit(sq_entry_fwd_req).asUInt)
  io.out.toLdDc.fwdId := Mux(sq_newest_fwd_req &&(!sq_newest_fwd_req) , VecInit(sq_entry_data_discard_req_short).asUInt , sq_fwd_req_id)
  //---------multi depd-------------------
  val sq_fwd_multi = RegInit(true.B)
  when( (PopCount(VecInit(sq_entry_fwd_req).asUInt) === 1.U) || (PopCount(VecInit(sq_entry_fwd_req).asUInt) === 0.U) ){
    sq_fwd_multi := false.B
  }.otherwise{
    sq_fwd_multi := true.B
  }
  io.out.toLdDc.fwdMulti       := sq_fwd_multi
  io.out.toLdDc.fwdMultiMask   := sq_newest_fwd_req
  io.out.toLdDc.fwdBypassMulti := !sq_newest_fwd_bypass_req && !sq_newest_fwd_req && sq_fwd_bypass_req && sq_fwd_req
  //==========================================================
  //              Forward data pop entry
  //==========================================================
  val sq_fwd_data_sel = RegInit(0.U(XLEN.W))
  val sq_fwd_data_pe_req = io.out.toLdDc.newestFwdDataVldReq
  val sq_fwd_data_pe = RegInit(0.U(XLEN.W))
  when(sq_fwd_data_pe_req){
    sq_fwd_data_pe := sq_fwd_data_sel
  }
  for(i<- 0 until LSIQ_ENTRY){
    when(VecInit(sq_entry_newest_fwd_req_data_vld).asUInt === i.U){
      sq_fwd_data_sel :=  sq_entry_data(i)
    }
  }
  io.out.toLdDa.fwdDataPe := sq_fwd_data_pe
  //==========================================================
  //                forward data to ld_da
  //==========================================================
  for(i<- 0 until LSIQ_ENTRY){
    when(OHToUInt(io.in.ldDaIn.sqFwdId) === i.U){
      io.out.toLdDa.fwdData :=  sq_entry_data(i)
    }.otherwise{
      io.out.toLdDa.fwdData := DontCare
    }
  }
  //==========================================================
  //            ld_da to sq depd/discard signal
  //==========================================================
  //---------interface to sq_entry--------
  //set depd signal
  val sq_entry_fwd_multi_depd_set = Cat(Seq.fill(ROT_SEL_WIDTH_8)(io.in.ldDaIn.sqFwdMultiVld)) & VecInit(sq_entry_vld).asUInt & io.in.ldDaIn.sqFwdId
  //-------------data depd--------------
  //if more than 1 entry have depend relationship, see fwd_en signal,
  //if no entry has fwd_en signal, then select the biggest entry
  val sq_data_discard_id_sel = RegInit(0.U(LSIQ_ENTRY.W))
  val ld_fwd_id = Seq.fill(LSIQ_ENTRY)(Wire(UInt(1.W)))
  for(i<- 0 until LSIQ_ENTRY){
    ld_fwd_id(i) := io.in.ldDaIn.sqFwdId(i)
  }
  when(PopCount(io.in.ldDaIn.sqFwdId) >= 1.U){
    sq_data_discard_id_sel := UIntToOH(ld_fwd_id.takeWhile(_ == true.B).length.U)
  }.otherwise{
    sq_data_discard_id_sel := 0.U(LSIQ_ENTRY.W)
  }
  val sq_data_discard_newest_id = VecInit(sq_entry_same_addr_newest).asUInt & io.in.ldDaIn.sqFwdId
  val sq_data_discard_has_newest = sq_data_discard_newest_id.orR
  val sq_entry_wakeup_queue_set_id = Mux(sq_data_discard_has_newest , sq_data_discard_newest_id, sq_data_discard_id_sel)
  val sq_entry_data_discard_grnt = Cat(Seq.fill(ROT_SEL_WIDTH_8)(io.in.ldDaIn.sqDataDiscardVld)) & VecInit(sq_entry_vld).asUInt & sq_entry_wakeup_queue_set_id
  //==========================================================
  //            maintain restart wakeup queue
  //==========================================================
  //---------------------registers----------------------------
  //+--------------+
  //| wakeup_queue |
  //+--------------+
  //the queue stores the instructions waiting for wakeup
  val sq_wakeup_queue = RegInit(0.U(LSIQ_ENTRY.W))
  val sq_wakeup_queue_next = RegInit(0.U(LSIQ_ENTRY.W))
  when(io.in.rtuIn.flush){
    sq_wakeup_queue := 0.U(LSIQ_ENTRY.W)
  }.elsewhen(io.in.ldDaIn.sqGlobalDiscardVld|| sq_pop_depd_ff){
    sq_wakeup_queue := sq_wakeup_queue_next
  }
  //+-------------+
  //| depd_pop_ff |
  //+-------------+
  //if depd pop, this will set to 1, and clear wakeup_queue next cycle
  val wmb_ce_depd     = Wire(Bool())
  val wmb_ce_depd_set = Wire(Bool())
  when(io.in.wmbIn.popGrnt && (wmb_ce_depd || wmb_ce_depd_set)){
    sq_pop_depd_ff := true.B
  }.otherwise{
    sq_pop_depd_ff := false.B
  }
  //------------------update wakeup queue---------------------
  val sq_wakeup_queue_grnt = sq_wakeup_queue | (Cat(Seq.fill(ROT_SEL_WIDTH_8)(io.in.ldDaIn.sqGlobalDiscardVld)) & io.in.ldDaIn.lsid )
  sq_wakeup_queue_next := Mux(sq_pop_depd_ff, 0.U(LSIQ_ENTRY.W),sq_wakeup_queue_grnt)
  //-------------------------wakeup---------------------------
  io.out.toCtrl.globalDepdWakeup :=  Mux(sq_pop_depd_ff, sq_wakeup_queue_grnt, 0.U(LSIQ_ENTRY.W))
  io.out.toCtrl.dataDepdWakeup   :=  sq_entry_data_depd_wakeup.reduce(_ | _)
  //==========================================================
  //                pop entry
  //==========================================================
  //+----------+---------------+
  //| pop addr | pop_page_info |
  //+----------+---------------+
  // surplus
  val age_vec_sp = VecInit(sq_entry_age_vec_surplus1_ptr).asUInt
  val sq_pe_age_vec_surplus1_page_ca     = age_vec_sp & VecInit(sq_entry_page_ca).asUInt
  val sq_pe_age_vec_surplus1_page_wa     = age_vec_sp & VecInit(sq_entry_page_wa).asUInt
  val sq_pe_age_vec_surplus1_page_so     = age_vec_sp & VecInit(sq_entry_page_so).asUInt
  val sq_pe_age_vec_surplus1_page_buf    = age_vec_sp & VecInit(sq_entry_page_buf).asUInt
  val sq_pe_age_vec_surplus1_page_sec    = age_vec_sp & VecInit(sq_entry_page_sec).asUInt
  val sq_pe_age_vec_surplus1_page_share  = age_vec_sp & VecInit(sq_entry_page_share).asUInt
  val sq_pe_age_vec_surplus1_atomic      = age_vec_sp & VecInit(sq_entry_atomic).asUInt
  val sq_pe_age_vec_surplus1_icc         = age_vec_sp & VecInit(sq_entry_icc).asUInt
  val sq_pe_age_vec_surplus1_wo_st       = age_vec_sp & VecInit(sq_entry_wo_st).asUInt
  val sq_pe_age_vec_surplus1_sync_fence  = age_vec_sp & VecInit(sq_entry_sync_fence).asUInt
  val sq_pe_age_vec_surplus1_inst_flush  = age_vec_sp & VecInit(sq_entry_inst_flush).asUInt

  val sq_pe_age_vec_surplus1_addr = Wire(UInt(PA_WIDTH.W))
  val sq_pe_age_vec_surplus1_bytes_vld = Wire(UInt(BYTES_ACCESS_WIDTH.W))
  val sq_pe_age_vec_surplus1_inst_type = Wire(UInt(INST_TYPE_WIDTH.W))
  val sq_pe_age_vec_surplus1_inst_size = Wire(UInt(INST_SIZE_WIDTH.W))
  val sq_pe_age_vec_surplus1_inst_mode = Wire(UInt(INST_MODE_WIDTH.W))
  val sq_pe_age_vec_surplus1_priv_mode = Wire(UInt(MPPWidth.W))
  for(i<- 0 until LSIQ_ENTRY){
    if(OHToUInt(age_vec_sp) == i.U){
      sq_pe_age_vec_surplus1_inst_type := sq_entry_inst_type(i)
      sq_pe_age_vec_surplus1_inst_size := sq_entry_inst_size(i)
      sq_pe_age_vec_surplus1_inst_mode := sq_entry_inst_mode(i)
      sq_pe_age_vec_surplus1_priv_mode := sq_entry_priv_mode(i)
      sq_pe_age_vec_surplus1_addr      := sq_entry_addr0(i)
      sq_pe_age_vec_surplus1_bytes_vld := sq_entry_bytes_vld(i)
    }else{
      sq_pe_age_vec_surplus1_inst_type := DontCare
      sq_pe_age_vec_surplus1_inst_size := DontCare
      sq_pe_age_vec_surplus1_inst_mode := DontCare
      sq_pe_age_vec_surplus1_priv_mode := DontCare
      sq_pe_age_vec_surplus1_addr      := DontCare
      sq_pe_age_vec_surplus1_bytes_vld := DontCare
    }
  }
  // zero ptr
  val age_vec_zero = VecInit(sq_entry_age_vec_zero_ptr).asUInt
  val sq_pe_age_vec_zero_page_ca      = age_vec_sp & VecInit(sq_entry_page_ca).asUInt
  val sq_pe_age_vec_zero_page_wa      = age_vec_sp & VecInit(sq_entry_page_wa).asUInt
  val sq_pe_age_vec_zero_page_so      = age_vec_sp & VecInit(sq_entry_page_so).asUInt
  val sq_pe_age_vec_zero_page_buf     = age_vec_sp & VecInit(sq_entry_page_buf).asUInt
  val sq_pe_age_vec_zero_page_sec     = age_vec_sp & VecInit(sq_entry_page_sec).asUInt
  val sq_pe_age_vec_zero_page_share   = age_vec_sp & VecInit(sq_entry_page_share).asUInt
  val sq_pe_age_vec_zero_atomic       = age_vec_sp & VecInit(sq_entry_atomic).asUInt
  val sq_pe_age_vec_zero_icc          = age_vec_sp & VecInit(sq_entry_icc).asUInt
  val sq_pe_age_vec_zero_wo_st        = age_vec_sp & VecInit(sq_entry_wo_st).asUInt
  val sq_pe_age_vec_zero_sync_fence   = age_vec_sp & VecInit(sq_entry_sync_fence).asUInt
  val sq_pe_age_vec_zero_inst_flush   = age_vec_sp & VecInit(sq_entry_inst_flush).asUInt

  val sq_pe_age_vec_zero_addr = Wire(UInt(PA_WIDTH.W))
  val sq_pe_age_vec_zero_bytes_vld = Wire(UInt(BYTES_ACCESS_WIDTH.W))
  val sq_pe_age_vec_zero_inst_type = Wire(UInt(INST_TYPE_WIDTH.W))
  val sq_pe_age_vec_zero_inst_size = Wire(UInt(INST_SIZE_WIDTH.W))
  val sq_pe_age_vec_zero_inst_mode = Wire(UInt(INST_MODE_WIDTH.W))
  val sq_pe_age_vec_zero_priv_mode = Wire(UInt(MPPWidth.W))
  for(i<- 0 until LSIQ_ENTRY){
    if(OHToUInt(age_vec_zero) == i.U){
      sq_pe_age_vec_zero_inst_type := sq_entry_inst_type(i)
      sq_pe_age_vec_zero_inst_size := sq_entry_inst_size(i)
      sq_pe_age_vec_zero_inst_mode := sq_entry_inst_mode(i)
      sq_pe_age_vec_zero_priv_mode := sq_entry_priv_mode(i)
      sq_pe_age_vec_zero_addr      := sq_entry_addr0(i)
      sq_pe_age_vec_zero_bytes_vld := sq_entry_bytes_vld(i)
    }else {
      sq_pe_age_vec_zero_inst_type := DontCare
      sq_pe_age_vec_zero_inst_size := DontCare
      sq_pe_age_vec_zero_inst_mode := DontCare
      sq_pe_age_vec_zero_priv_mode := DontCare
      sq_pe_age_vec_zero_addr      := DontCare
      sq_pe_age_vec_zero_bytes_vld := DontCare
    }
  }
  val sq_pe_sel_age_vec_surplus1_entry_vld = Wire(Bool())
  sq_pe_sel_age_vec_zero_entry_vld := (VecInit(sq_entry_age_vec_zero_ptr).asUInt & (~VecInit(sq_entry_dcache_info_vld).asUInt).asUInt).orR
  sq_pe_sel_age_vec_surplus1_entry_vld := io.in.wmbIn.popToCeGrnt
  when(sq_pe_sel_age_vec_zero_entry_vld){
    io.out.toWmb.ce.popAddr       := sq_pe_age_vec_zero_addr
    io.out.toWmb.ce.popPageCa     := sq_pe_age_vec_zero_page_ca
    io.out.toWmb.ce.popPageWa     := sq_pe_age_vec_zero_page_wa
    io.out.toWmb.ce.popPageSo     := sq_pe_age_vec_zero_page_so
    io.out.toWmb.ce.popPageSec    := sq_pe_age_vec_zero_page_sec
    io.out.toWmb.ce.popPageBuf    := sq_pe_age_vec_zero_page_buf
    io.out.toWmb.ce.popPageShare  := sq_pe_age_vec_zero_page_share
    io.out.toWmb.ce.popAtomic     := sq_pe_age_vec_zero_atomic
    io.out.toWmb.ce.popIcc        := sq_pe_age_vec_zero_icc
    io.out.toWmb.ce.popWoSt       := sq_pe_age_vec_zero_wo_st
    io.out.toWmb.ce.popSyncFence  := sq_pe_age_vec_zero_sync_fence
    io.out.toWmb.ce.popInstFlush  := sq_pe_age_vec_zero_inst_flush
    io.out.toWmb.ce.popInstType   := sq_pe_age_vec_zero_inst_type
    io.out.toWmb.ce.popInstSize   := sq_pe_age_vec_zero_inst_size
    io.out.toWmb.ce.popInstMode   := sq_pe_age_vec_zero_inst_mode
    io.out.toWmb.ce.popBytesVld   := sq_pe_age_vec_zero_bytes_vld
    io.out.toWmb.ce.popPtr        := age_vec_zero
    io.out.toWmb.ce.popPrivMode   := sq_pe_age_vec_zero_priv_mode
  }.elsewhen(sq_pe_sel_age_vec_surplus1_entry_vld){
    io.out.toWmb.ce.popAddr       := sq_pe_age_vec_surplus1_addr
    io.out.toWmb.ce.popPageCa     := sq_pe_age_vec_surplus1_page_ca
    io.out.toWmb.ce.popPageWa     := sq_pe_age_vec_surplus1_page_wa
    io.out.toWmb.ce.popPageSo     := sq_pe_age_vec_surplus1_page_so
    io.out.toWmb.ce.popPageSec    := sq_pe_age_vec_surplus1_page_sec
    io.out.toWmb.ce.popPageBuf    := sq_pe_age_vec_surplus1_page_buf
    io.out.toWmb.ce.popPageShare  := sq_pe_age_vec_surplus1_page_share
    io.out.toWmb.ce.popAtomic     := sq_pe_age_vec_surplus1_atomic
    io.out.toWmb.ce.popIcc        := sq_pe_age_vec_surplus1_icc
    io.out.toWmb.ce.popWoSt       := sq_pe_age_vec_surplus1_wo_st
    io.out.toWmb.ce.popSyncFence  := sq_pe_age_vec_surplus1_sync_fence
    io.out.toWmb.ce.popInstFlush  := sq_pe_age_vec_surplus1_inst_flush
    io.out.toWmb.ce.popInstType   := sq_pe_age_vec_surplus1_inst_type
    io.out.toWmb.ce.popInstSize   := sq_pe_age_vec_surplus1_inst_size
    io.out.toWmb.ce.popInstMode   := sq_pe_age_vec_surplus1_inst_mode
    io.out.toWmb.ce.popBytesVld   := sq_pe_age_vec_surplus1_bytes_vld
    io.out.toWmb.ce.popPtr        := age_vec_sp
    io.out.toWmb.ce.popPrivMode   := sq_pe_age_vec_surplus1_priv_mode
  }.otherwise{
    io.out.toWmb.ce.popAddr       := DontCare
    io.out.toWmb.ce.popPageCa     := DontCare
    io.out.toWmb.ce.popPageWa     := DontCare
    io.out.toWmb.ce.popPageSo     := DontCare
    io.out.toWmb.ce.popPageSec    := DontCare
    io.out.toWmb.ce.popPageBuf    := DontCare
    io.out.toWmb.ce.popPageShare  := DontCare
    io.out.toWmb.ce.popAtomic     := DontCare
    io.out.toWmb.ce.popIcc        := DontCare
    io.out.toWmb.ce.popWoSt       := DontCare
    io.out.toWmb.ce.popSyncFence  := DontCare
    io.out.toWmb.ce.popInstFlush  := DontCare
    io.out.toWmb.ce.popInstType   := DontCare
    io.out.toWmb.ce.popInstSize   := DontCare
    io.out.toWmb.ce.popInstMode   := DontCare
    io.out.toWmb.ce.popBytesVld   := DontCare
    io.out.toWmb.ce.popPtr        := DontCare
    io.out.toWmb.ce.popPrivMode   := DontCare
  }
  //==========================================================
  //                request wmb ce
  //==========================================================
  //------------------pop info--------------------------------
  sq_pop_req_unmask := sq_entry_pop_req.reduce(_ || _)
  val sq_pop_st_inst =(!io.out.toWmb.ce.popAtomic) &&
  (!io.out.toWmb.ce.popIcc) &&  (!io.out.toWmb.ce.popSyncFence)
  val sq_pop_dcache_inst = (!io.out.toWmb.ce.popAtomic) && (io.out.toWmb.ce.popIcc) &&
    (io.out.toWmb.ce.popInstType === "b10".U)
  val sq_pop_dcache_all_inst   = sq_pop_dcache_inst && (io.out.toWmb.ce.popInstMode === "b00".U)
  val sq_pop_dcache_1line_inst = sq_pop_dcache_inst && (io.out.toWmb.ce.popInstMode =/= "b00".U)
  //------------------pop request-----------------------------
  //dcache all request icc success and wait for idle
  //index not hit include dcache write port not hit
  val sq_req_icc_success = RegInit(false.B)

  //==========================================================
  //                request icc
  //==========================================================
  //-----------------register---------------------------------
  when(io.in.iccIn.sqGrnt){
    sq_req_icc_success := true.B
  }.elsewhen(io.in.wmbIn.popToCeGrnt){
    sq_req_icc_success := false.B
  }
  val sq_pop_to_ce_req_unmask = sq_pop_req_unmask && (!sq_pop_dcache_all_inst || sq_req_icc_success && io.in.iccIn.idle)
  //because change mechanism, cache miss atomic can write, so must wait for hit_idx
  val sq_pop_to_ce_req = sq_pop_to_ce_req_unmask  && io.in.rtuIn.flush
  val sq_pop_merge_data_req_unmask = sq_pop_req_unmask && io.out.toWmb.ce.popWoSt
  //------------------pop grnt--------------------------------
  val sq_entry_pop_to_ce_grnt   = Cat(Seq.fill(LSIQ_ENTRY)(io.in.wmbIn.popToCeGrnt))  & VecInit(sq_entry_pop_req).asUInt
  val sq_entry_pop_to_ce_grnt_b = (~sq_entry_pop_to_ce_grnt).asUInt
  //------------------create signal---------------------------
  io.out.toWmb.mergeReq         := sq_pop_merge_data_req_unmask
  io.out.toWmb.popToCeReq       := sq_pop_to_ce_req
  io.out.toWmb.mergeStallReq    := io.out.toWmb.createHitRbIdx
  io.out.toWmb.popToCeDpReq     := sq_pop_to_ce_req_unmask
  io.out.toWmb.popToCeGateclkEn := sq_pop_to_ce_req_unmask
  //if hit rb idx, then cannot merge
  io.out.toWmb.createHitRbIdx := io.in.rbIn.sqPopHitIdx && (sq_pop_dcache_1line_inst || io.out.toWmb.ce.popAtomic || io.out.toWmb.ce.popWoSt)
  //-----------------wires-----------------------------------
  io.out.toIcc.req := sq_pop_req_unmask && sq_pop_dcache_all_inst && (!io.in.wmbIn.ceVld) && !sq_req_icc_success
  io.out.toIcc.clr := io.out.toWmb.ce.popInstSize(0)
  io.out.toIcc.inv := io.out.toWmb.ce.popInstSize(1)
  //==========================================================
  //                interface to pfu
  //==========================================================
  //clear all entry if sq pop a sync.i inst
  io.out.popSynciInst := sq_pop_req_unmask && (!io.out.toWmb.ce.popAtomic) && io.out.toWmb.ce.popSyncFence &&
    (io.out.toWmb.ce.popInstType === "b00".U) &&  (io.out.toWmb.ce.popInstFlush)
  //==========================================================
  //                wmb ce data path
  //==========================================================
  val wmb_ce_fence_mode = Wire(UInt(FENCE_MODE_WIDTH.W))
  val wmb_ce_iid        = Wire(UInt(IidWidth.W))
  val wmb_ce_data64 = RegInit(0.U(XLEN.W))
  for(i<- 0 until LSIQ_ENTRY){
    if(  OHToUInt(io.in.wmbIn.sqPtr) == i.U){
      wmb_ce_fence_mode := sq_entry_fence_mode(i)
      wmb_ce_iid        := sq_entry_iid(i)
      wmb_ce_data64     := sq_entry_data(i)
    }else{
      wmb_ce_fence_mode := DontCare
      wmb_ce_iid        := DontCare
      wmb_ce_data64     := DontCare
    }
  }
  io.out.toWmb.fenceMode := wmb_ce_fence_mode
  io.out.toWmb.iid       := wmb_ce_iid
  io.out.toWmb.data128   := Cat(wmb_ce_data64,wmb_ce_data64)
  io.out.toWmb.specFail  := (io.in.wmbIn.sqPtr & VecInit(sq_entry_spec_fail).asUInt).orR
  io.out.toWmb.bkptaData := (io.in.wmbIn.sqPtr & VecInit(sq_entry_bkpta_data).asUInt).orR
  io.out.toWmb.bkptbData := (io.in.wmbIn.sqPtr & VecInit(sq_entry_bkptb_data).asUInt).orR
  io.out.toWmb.vstartVld := (io.in.wmbIn.sqPtr & VecInit(sq_entry_vstart_vld).asUInt).orR
  val wmb_ce_dcache_mesi = WireInit(0.U.asTypeOf(new DcacheDirtyDataEn))
  wmb_ce_dcache_mesi.share := (io.in.wmbIn.sqPtr & VecInit(sq_entry_dcache_valid).asUInt).orR
  wmb_ce_dcache_mesi.valid := (io.in.wmbIn.sqPtr & VecInit(sq_entry_dcache_share).asUInt).orR
  wmb_ce_dcache_mesi.dirty := (io.in.wmbIn.sqPtr & VecInit(sq_entry_dcache_dirty).asUInt).orR
  io.out.toWmb.dcacheShare := wmb_ce_dcache_mesi.share
  io.out.toWmb.dcacheValid := wmb_ce_dcache_mesi.valid
  val wmb_ce_dcache_way    = (io.in.wmbIn.sqPtr & VecInit(sq_entry_dcache_way).asUInt).orR
  wmb_ce_depd      := (io.in.wmbIn.sqPtr & VecInit(sq_entry_depd).asUInt).orR
  wmb_ce_depd_set  := (io.in.wmbIn.sqPtr & VecInit(sq_entry_depd_set).asUInt).orR
  //==========================================================
  //                  LsuDcacheInfoUpdate
  //==========================================================
  val wmb_ce_dcache_update = Module(new LsuDcacheInfoUpdate)
  wmb_ce_dcache_update.io.in.originDcacheMesi  := wmb_ce_dcache_mesi
  wmb_ce_dcache_update.io.in.originDcacheWay   := wmb_ce_dcache_way
  wmb_ce_dcache_update.io.in.compareDcwpAddr   := io.in.wmbIn.addr
  wmb_ce_dcache_update.io.in.compareDcwpSwInst := io.in.wmbIn.dcacheSwInst
  wmb_ce_dcache_update.io.in.dcacheIn := io.in.dcacheIn
  val wmb_ce_dcache_hit_idx    = wmb_ce_dcache_update.io.out.compareDcwpHitIdx
  val wmb_ce_dcache_update_vld = wmb_ce_dcache_update.io.out.compareDcwpUpdateVld
  val wmb_ce_update_dcache_dirty = wmb_ce_dcache_update.io.out.updateDcacheMesi.dirty
  val wmb_ce_update_dcache_share = wmb_ce_dcache_update.io.out.updateDcacheMesi.share
  val wmb_ce_update_dcache_valid = wmb_ce_dcache_update.io.out.updateDcacheMesi.valid
  val wmb_ce_update_dcache_way   = wmb_ce_dcache_update.io.out.updateDcacheWay
  io.out.toWmb.updateDcacheWay := wmb_ce_dcache_update.io.out.updateDcacheWay
  io.out.toWmb.updateDcacheMesi := wmb_ce_dcache_update.io.out.updateDcacheMesi
  //==========================================================
  //                interface to idu
  //==========================================================
  io.out.SqNotFull := !sq_empty_less2
  val lsu_rtu_all_commit_st_data_vld = sq_entry_cmit_data_vld.reduce(_ && _)
  io.out.AllCommitDataVld := lsu_rtu_all_commit_st_data_vld && io.in.rbIn.rtuAllCommitLdDataVld

  //==========================================================
  //                 single input
  //==========================================================
  entries.zipWithIndex.foreach{
    case(entry, i) =>
      // binded io, dispatch from input
      entry.io.in.sqIn.ageVecSet             := sq_age_vec_set
      entry.io.in.sqIn.createAgeVec          := VecInit(sq_create_age_vec).asUInt
      entry.io.in.sqIn.createSameAddrNewest  := sq_create_same_addr_newest
      entry.io.in.sqIn.createSuccess         := sq_create_success
      entry.io.in.sqIn.createVld             := sq_create_vld
      entry.io.in.sqIn.dataSettle            := sq_data_settle
      entry.io.in.sqIn.createDpVldX          := sq_entry_create_dp_vld(i).asBool
      entry.io.in.sqIn.createGateclkEnX      := sq_entry_create_gateclk_en(i).asBool
      entry.io.in.sqIn.createVldX            := sq_entry_create_vld(i).asBool
      entry.io.in.sqIn.dataDiscardGrntX      := sq_entry_data_discard_grnt(i).asBool
      entry.io.in.sqIn.fwdMultiDepdSetX      := sq_entry_fwd_multi_depd_set(i).asBool
      entry.io.in.sqIn.popToCeGrntB          := sq_entry_pop_to_ce_grnt_b
      entry.io.in.sqIn.popToCeGrntX          := sq_entry_pop_to_ce_grnt(i)
      entry.io.in.sqIn.popPtrX               := io.out.toWmb.ce.popPtr(i)
  }
}