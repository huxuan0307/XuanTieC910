package Core.LSU.Lfb
import Core.BiuID.BIU_LFB_ID_T
import Core.IntConfig.XLEN
import Core.LSU.Lfb.BiuResp._
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._


object BiuResp {
  def OKAY    = "b00".U
  def EXOKAY  = "b01".U
  def SLVERR  = "b10".U
  def DECERR  = "b11".U
}


//==========================================================
//                        Input
//==========================================================
class BiuToLfb extends BiuToLfbDataEntry {
 val rId    = UInt(5.W)
 val rResp  = UInt(4.W)
}
class LmToLfb extends BiuToLfbDataEntry {
  val alreadySnoop   = Bool()
  val lfbDepdWakeup  = Bool()
  val stateIsAmoLock = Bool()
}
class PfuToLfb extends Bundle with DCacheConfig with LsuConfig {
  val reqAddr = UInt(PA_WIDTH.W)
  val createDpVld = Bool()
  val createGateclkEn = Bool()
  val createReq = Bool()
  val createVld = Bool()
  val id = UInt(PFU_IDX.W)
}
class RbToLfb extends Bundle with LsuConfig {
  val biuReqAddr = UInt((PA_WIDTH).W)
  val addrTto4 = UInt((PA_WIDTH-4).W)
  val atomic = Bool()
  val boundaryDepdWakeup = Bool()
  val createDpVld = Bool()
  val createGateclkEn = Bool()
  val createReq = Bool()
  val createVld = Bool()
  val depd = Bool()
  val ldamo = Bool()
}
class SnqToLfb extends Bundle with LsuConfig  {
  val bypassAddrTto6 = UInt((PA_WIDTH-6).W)
  val createLfbVbReqHitIdx = Bool()
  val bypassChgTag = UInt(2.W)
  val bypassInvalid = UInt(2.W)
  val vbReqHitIdx = Bool()
}
class VbToLfb extends Bundle with LsuConfig with DCacheConfig {
  val addrEntryRcldone = UInt(LFB_ADDR_ENTRY.W)
  val createGrnt = Bool()
  val dcacheDirty = Bool()
  val dcacheHit = Bool()
  val dcacheWay = Bool()
  val rclDone = Bool()
  val vbReqHitIdx = Bool()
}
//----------------------------------------------------------
class LfbIOIn extends Bundle with LsuConfig with DCacheConfig {
  val biuIn = new BiuToLfb
  val bus_arb_pfu_ar_sel = Bool() // todo whats this?
  val bus_arb_rb_ar_sel = Bool() // todo whats this?
  val cp0In = new Cp0ToLfbEntry
  val dcache_arb_lfb_ld_grnt = Bool()
  val ldDaIn = new Bundle {
    val idx             = UInt(LFB_ADDR_ENTRY.W)
    val discardGrnt     = Bool()
    val setWakeupQueue  = Bool()
    val wakeupQueueNext = Valid(UInt(LSIQ_ENTRY.W))
  }
  val lmIn = new LmToLfb
  val pfuIn = new PfuToLfb
  val rbIn = new RbToLfb
  val snqIn = new SnqToLfb
  val vbIn = new VbToLfb
  val rtuFlush = Bool()
  val stDaAddr = UInt(PA_WIDTH.W)
  val wmbIn = new WmbToLfbAddrEntry
}
//==========================================================
//                        Output
//==========================================================
class LfbToArb extends Bundle with LsuConfig with DCacheConfig {
  val ld_data_gateclk_en = UInt(LFB_ADDR_ENTRY.W)
  val ld_data_high_din = UInt((2*XLEN).W)
  val ld_data_idx = UInt(11.W)
  val ld_data_low_din = UInt((2*XLEN).W)
  val ld_req = Bool()
  val ld_tag_din = UInt(54.W)
  val ld_tag_gateclk_en = Bool()
  val ld_tag_idx = UInt(INDEX_WIDTH.W)
  val ld_tag_req = Bool()
  val ld_tag_wen = UInt(2.W)
  val serial_req = Bool()

  val st_dirty_din = UInt(7.W)
  val st_dirty_gateclk_en = Bool()
  val st_dirty_idx = UInt(INDEX_WIDTH.W)
  val st_dirty_req = Bool()
  val st_dirty_wen = UInt(7.W)
  val st_req = Bool()
  val st_tag_din = UInt(52.W)
  val st_tag_gateclk_en = Bool()
  val st_tag_idx = UInt(INDEX_WIDTH.W)
  val st_tag_req = Bool()
  val st_tag_wen = UInt(2.W)
}
class LfbToPfu extends Bundle with LsuConfig with DCacheConfig {
  val biuReqHitIdx = Bool()
  val createId   = UInt(PFU_IDX.W)
  val dcacheHit  = UInt(PFU_ENTRY.W)
  val dcacheMiss = UInt(PFU_ENTRY.W)
  val rreadyGrnt = Bool()
}
class LfbToRb extends Bundle with LsuConfig with DCacheConfig {
 val biuReqHitIdx = Bool()
 val caRreadyGrnt = Bool()
 val createId = UInt(5.W) // todo rb idx?
 val ncRreadyGrnt = Bool()
}
class LfbToSnq extends Bundle with LsuConfig with DCacheConfig {
  val bypassDataId =UInt(2.W) // todo snq id?
  val bypassHit = Bool()
  val bypassShare = Bool()
}
class LfbToVb extends Bundle with LsuConfig with DCacheConfig {
  val addrTto6 = UInt((PA_WIDTH-6).W)
  val createDpVld = Bool()
  val createGateclkEn = Bool()
  val createReq = Bool()
  val createVld = Bool()
  val id = UInt(3.W)  // todo snq id?
}
class LfbToWmb extends Bundle with LsuConfig with DCacheConfig {
  val readReqHitIdx= Bool()
  val writeReqHitIdx = Bool()
}
//----------------------------------------------------------
class LfbIOOut extends Bundle with LsuConfig with DCacheConfig {
  val ldHitPrefetch = Bool()
  val lfbAddrFull = Bool()
  val lfbAddrLess2 = Bool()
  val toArb = new LfbToArb
  val lfbDepdWakeup = UInt(LSIQ_ENTRY.W)
  val lfbEmpty = Bool()
  val ldDaHitIdx = Bool()
  val mcicWakeup = Bool()
  val toPfu = new LfbToPfu
  val toRb = new LfbToRb
  val toSnq = new LfbToSnq
  val toVb = new LfbToVb
  val toWmb = new LfbToWmb
  val popDepdFf = Bool()
  val stDaHitIdx = Bool()
  val lsuBiuRLinefillReady = Bool()
}
//==========================================================
//                          IO
//==========================================================
class LfbIO extends Bundle with LsuConfig {
  val in  = Input( new LfbIOIn)
  val out = Output(new LfbIOOut)
}
class Lfb extends Module with LsuConfig with DCacheConfig {
  val io = IO(new LfbIO)
  val lfb_clk_en = (!io.out.lfbEmpty) || io.in.rbIn.createGateclkEn || io.in.pfuIn.createGateclkEn ||
  io.in.rbIn.boundaryDepdWakeup || io.out.popDepdFf || io.in.lmIn.lfbDepdWakeup
  val lfb_vb_req_unmask = RegInit(false.B)
  val lfb_vb_pe_req = Wire(Bool())
  val lfb_create_vb_cancel = Wire(Bool())
  val lfb_create_vb_success = Wire(Bool())
  val lfb_vb_pe_clk_en = !lfb_vb_req_unmask && (lfb_vb_pe_req || io.in.rbIn.createReq ||  io.in.pfuIn.createReq)  ||
    lfb_create_vb_cancel || lfb_create_vb_success
  val lfb_lf_sm_req = Wire(Bool()) // sm is lfb state machine
  val lfb_lf_sm_vld = RegInit(false.B)
  val lfb_lf_sm_permit = Wire(Bool())
  val lfb_lf_sm_clk_en = lfb_lf_sm_req  ||  lfb_lf_sm_vld
  val lfb_lf_sm_req_clk_en = lfb_lf_sm_req  &&  lfb_lf_sm_permit
  val lfb_wakeup_queue_clk_en = io.out.popDepdFf || io.in.ldDaIn.setWakeupQueue || io.in.rtuFlush
  //==========================================================
  //              Instance addr entry
  //==========================================================
  //8 addr entry
  val lfb_addr_entry = Seq.fill(LFB_ADDR_ENTRY)(Module(new LfbAddrEntry))
  //wires define
  val addr_entry_vld                 = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_vb_pe_req       = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_addr_tto4       = Seq.fill(LFB_ADDR_ENTRY)(RegInit(0.U((PA_WIDTH-4).W)  ))
  val lfb_addr_entry_ldamo           = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_pfu_dcache_hit  = Seq.fill(LFB_ADDR_ENTRY)(RegInit(0.U((9).W)  ))
  val lfb_addr_entry_pfu_dcache_miss = Seq.fill(LFB_ADDR_ENTRY)(RegInit(0.U((9).W)  ))
  val lfb_addr_entry_depd            = Seq.fill(LFB_ADDR_ENTRY)(Wire(UInt(8.W)))
  val lfb_addr_entry_discard_vld     = Seq.fill(LFB_ADDR_ENTRY)(Wire(UInt(8.W)))
  val lfb_addr_entry_refill_way      = Seq.fill(LFB_ADDR_ENTRY)(Wire(UInt(8.W)))
  val lfb_addr_entry_pop_vld         = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_not_resp        = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_ld_da_hit_idx   = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_st_da_hit_idx   = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_rb_biu_req_hit_idx  = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_pfu_biu_req_hit_idx = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_wmb_write_req_hit_idx = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_wmb_read_req_hit_idx = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_vld = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_linefill_permit =  Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  val lfb_addr_entry_linefill_abort =  Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))

  lfb_addr_entry.zipWithIndex.foreach {
    case (entry, i) =>
      // input

      // output
      addr_entry_vld(i) := entry.io.out.addrEntryVld_x
      lfb_addr_entry_vb_pe_req(i) := entry.io.out.addrEntryVbPeReq_x
      lfb_addr_entry_addr_tto4(i) := entry.io.out.addrEntryAddrTto4_v
      lfb_addr_entry_ldamo(i) := entry.io.out.addrEntryLdamo_x
      lfb_addr_entry_pfu_dcache_hit(i)  := entry.io.out.addrEntryPfuDcacheHit_v
      lfb_addr_entry_pfu_dcache_miss(i) := entry.io.out.addrEntryPfuDcacheMiss_v
      lfb_addr_entry_depd(i)            := entry.io.out.addrEntryDepd_x
      lfb_addr_entry_discard_vld(i)     := entry.io.out.addrEntryDiscardVld_x
      lfb_addr_entry_refill_way(i)      := entry.io.out.addrEntryRefillWay_x
      lfb_addr_entry_pop_vld(i)         := entry.io.out.addrEntryPopVld_x
      lfb_addr_entry_not_resp(i)        := entry.io.out.addrEntryNotResp_x
      lfb_addr_entry_ld_da_hit_idx(i)         := entry.io.out.addrEntryLdDaHitIdx_x
      lfb_addr_entry_st_da_hit_idx(i)         := entry.io.out.addrEntryStDaHitIdx_x
      lfb_addr_entry_rb_biu_req_hit_idx(i)    := entry.io.out.addrEntryRbBiuReqHitIdx_x
      lfb_addr_entry_pfu_biu_req_hit_idx(i)   := entry.io.out.addrEntryPfuBiuReqHitIdx_x
      lfb_addr_entry_wmb_write_req_hit_idx(i) := entry.io.out.addrEntryWmbWriteReqHitIdx_x
      lfb_addr_entry_wmb_read_req_hit_idx(i)  := entry.io.out.addrEntryWmbWriteReqHitIdx_x
      lfb_addr_entry_vld(i)  := entry.io.out.addrEntryVld_x
      lfb_addr_entry_linefill_permit(i) := entry.io.out.addrEntryLinefillPermit_x
      lfb_addr_entry_linefill_abort(i) := entry.io.out.addrEntryLinefillAbort_x


  }
  //2 data entry
  val lfb_data_entry = Seq.fill(LFB_DATA_ENTRY)(Module(new LfbDataEntry))
  //wires define
  val lfb_data_entry_vld          = Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  val lfb_data_entry_wait_surplus = Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  val lfb_data_entry_lf_sm_req    = Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  val lfb_data_entry_addr_id      = Seq.fill(LFB_DATA_ENTRY)(Wire(UInt(LFB_ADDR_ENTRY.W)))
  val lfb_data_entry_dcache_share   = Seq.fill(LFB_DATA_ENTRY)(Wire(UInt(2.W)))
  val lfb_data_entry_data = Seq.fill(LFB_DATA_ENTRY)(Wire(UInt((XLEN*8).W)))
  val lfb_data_entry_addr_pop_req =  Seq.fill(LFB_DATA_ENTRY)(Wire(UInt(LFB_ADDR_ENTRY.W)))
  val lfb_data_entry_full =  Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  lfb_data_entry.zipWithIndex.foreach {
    case (entry, i) =>
      // input

      // output
      lfb_data_entry_vld(i)          := entry.io.out.vld_x
      lfb_data_entry_wait_surplus(i) := entry.io.out.waitSurplus_x
      lfb_data_entry_lf_sm_req(i)    := entry.io.out.lfSmReq_x
      lfb_data_entry_addr_id(i)      := entry.io.out.addrId_v
      lfb_data_entry_data(i)         := entry.io.out.data_v
      lfb_data_entry_addr_pop_req(i) := entry.io.out.addrPopReq_v
      lfb_data_entry_full(i)         := entry.io.out.full_x
      lfb_data_entry_vld(i)          := entry.io.out.vld_x
      lfb_data_entry_dcache_share(i) := entry.io.out.dcacheShare_x
  }
  //==========================================================
  //            Generate addr signal
  //==========================================================
  //------------------create ptr------------------------------
  val lfb_addr_create_ptr = RegInit(0.U(LFB_ADDR_ENTRY.W))
  lfb_addr_create_ptr := UIntToOH(PriorityEncoder(VecInit(addr_entry_vld).asUInt))
  //------------------grnt signal to lfb/pfu------------------
  val lfb_rb_create_grnt   = io.in.bus_arb_rb_ar_sel && io.in.rbIn.createReq
  val lfb_pfu_create_grnt  = io.in.bus_arb_pfu_ar_sel && io.in.pfuIn.createReq
  val lfb_create_id = Wire(UInt(LFB_ID_WIDTH.W))
  lfb_create_id := OHToUInt(lfb_addr_create_ptr) // equal to ct_rtu_encode_8 x_lsu_lfb_create_ptr_encode  @1276
  io.out.toRb.createId  := Cat(BIU_LFB_ID_T,lfb_create_id)
  io.out.toPfu.createId := Cat(BIU_LFB_ID_T,lfb_create_id)
  //------------------create signal---------------------------/
  val lfb_addr_rb_create_vld     = lfb_rb_create_grnt  && io.in.rbIn.createVld
  val lfb_addr_rb_create_dp_vld  = lfb_rb_create_grnt  && io.in.rbIn.createDpVld
  val lfb_addr_pfu_create_vld    = lfb_pfu_create_grnt && io.in.pfuIn.createVld
  val lfb_addr_pfu_create_dp_vld = lfb_pfu_create_grnt && io.in.pfuIn.createDpVld

  lfb_addr_entry.zipWithIndex.foreach {
    case (entry, i) =>
      // input
      entry.io.in.lfbIn.addrEntryRbCreateVld_x   := lfb_addr_rb_create_vld && (lfb_addr_create_ptr(i).asBool)
      entry.io.in.lfbIn.addrEntryRbCreateDpVld_x := lfb_addr_rb_create_dp_vld && lfb_addr_create_ptr(i).asBool
      entry.io.in.lfbIn.addrEntryPfuCreateVld_x   := lfb_addr_pfu_create_vld && lfb_addr_create_ptr(i).asBool
      entry.io.in.lfbIn.addrEntryPfuCreateDpVld_x := lfb_addr_pfu_create_dp_vld && lfb_addr_create_ptr(i).asBool
      entry.io.in.lfbIn.addrEntryCreateGateclkEn_x := io.in.rbIn.createGateclkEn || io.in.pfuIn.createGateclkEn && lfb_addr_create_ptr(i).asBool
  }
  //==========================================================
  //                Request vb addr entry
  //==========================================================
  //-------------------pop entry------------------------------
  val lfb_vb_pe_all_req = Wire(Bool())
  when(lfb_vb_pe_all_req){
    lfb_vb_req_unmask:= true.B
  }.elsewhen(lfb_create_vb_success || lfb_create_vb_cancel){
    lfb_vb_req_unmask:=false.B
  }
  val lfb_vb_addr_ptr  = RegInit(0.U(LFB_ADDR_ENTRY.W))
  val lfb_vb_addr_tto6 = RegInit(0.U((PA_WIDTH-6).W))
  val lfb_vb_pe_req_ptr       = RegInit(0.U(LFB_ADDR_ENTRY.W))
  val lfb_vb_pe_req_addr_tto6 = RegInit(0.U((PA_WIDTH-6).W))
  val lfb_vb_pe_req_permit = Wire(Bool())
  val lfb_vb_pe_rb_req  = Wire(Bool())
  val lfb_vb_pe_pfu_req = Wire(Bool())
  when(lfb_vb_pe_req_permit &&  lfb_vb_pe_req){
    lfb_vb_addr_ptr  := (lfb_vb_pe_req_ptr)
    lfb_vb_addr_tto6 := lfb_vb_pe_req_addr_tto6
  }.elsewhen(lfb_vb_pe_req_permit &&  lfb_vb_pe_rb_req){
    lfb_vb_addr_ptr  := lfb_addr_create_ptr
    lfb_vb_addr_tto6 := io.in.rbIn.addrTto4(PA_WIDTH-5,2)
  }.elsewhen(lfb_vb_pe_req_permit &&  lfb_vb_pe_pfu_req){
    lfb_vb_addr_ptr  := lfb_addr_create_ptr
    lfb_vb_addr_tto6 := io.in.pfuIn.reqAddr(PA_WIDTH-1,6)
  }
  io.out.toVb.addrTto6 := lfb_vb_addr_tto6
  //-----------------pop req signal---------------------------
  lfb_vb_pe_rb_req  := io.in.cp0In.lsuDcacheEn  &&  lfb_addr_rb_create_vld
  lfb_vb_pe_pfu_req := lfb_addr_pfu_create_vld
  lfb_vb_pe_req     := lfb_addr_entry_vb_pe_req.reduce(_ || _)
  lfb_vb_pe_all_req := lfb_vb_pe_req  ||  lfb_vb_pe_rb_req    ||  lfb_vb_pe_pfu_req
  //------------------permit signal---------------------------
  lfb_vb_pe_req_permit  := !lfb_vb_req_unmask || lfb_create_vb_cancel ||  lfb_create_vb_success
  val lfb_lf_sm_req_addr_tto6 = Wire(UInt((PA_WIDTH-6).W))
  val lfb_addr_entry_vb_pe_req_grnt = Wire(UInt(LFB_ADDR_ENTRY.W))
  lfb_vb_pe_req_ptr := UIntToOH(PriorityEncoder(VecInit(lfb_addr_entry_vb_pe_req).asUInt))
  for(i<- 0 until(LFB_ADDR_ENTRY)){
    when(lfb_vb_pe_req_ptr(i)){
      lfb_vb_pe_req_addr_tto6 := lfb_addr_entry_addr_tto4(i)(PA_WIDTH-5,2)
      //-------------------pop grnt signal------------------------
    }
  }
  lfb_addr_entry_vb_pe_req_grnt := Cat(Seq.fill(LFB_ADDR_ENTRY)(lfb_vb_pe_req_permit)) & lfb_vb_pe_req_ptr
  //-------------------request signal-------------------------
  val lfb_vb_req_ldamo = (lfb_vb_addr_ptr & VecInit(lfb_addr_entry_ldamo).asUInt).orR
  val lfb_vb_req_hit_idx = io.in.vbIn.vbReqHitIdx || io.in.snqIn.vbReqHitIdx  &&  !(lfb_vb_req_ldamo && io.in.lmIn.stateIsAmoLock)  ||  io.in.snqIn.vbReqHitIdx
  //req signal is for arbitration
  io.out.toVb.createReq := lfb_vb_req_unmask
  io.out.toVb.createVld       := lfb_vb_req_unmask &&  !lfb_vb_req_hit_idx
  io.out.toVb.createDpVld     := lfb_vb_req_unmask
  io.out.toVb.createGateclkEn := lfb_vb_req_unmask
  val lfb_vb_id = Wire(UInt(LFB_ID_WIDTH.W))
  lfb_vb_id := OHToUInt(lfb_vb_addr_ptr)
  io.out.toVb.id := lfb_vb_id
  lfb_create_vb_success := io.out.toVb.createVld && io.in.vbIn.createGrnt
  //when snq invalid lfb,should cancel vb req
  val lfb_vb_req_entry_vld = (lfb_vb_addr_ptr & VecInit(addr_entry_vld).asUInt).orR
  lfb_create_vb_cancel := lfb_vb_req_unmask  && !lfb_vb_req_entry_vld
  //==========================================================
  //            Reply dcache hit signal to pfu
  //==========================================================
  val lfb_pfu_dcache_hit  = lfb_addr_entry_pfu_dcache_hit.reduce(_ & _).orR
  val lfb_pfu_dcache_miss = lfb_addr_entry_pfu_dcache_miss.reduce(_ & _).orR
  io.out.toPfu.dcacheHit   := lfb_pfu_dcache_hit
  io.out.toPfu.dcacheMiss  := lfb_pfu_dcache_miss
  //==========================================================
  //                Pass data to data entry
  //==========================================================
  //------------------judge r signal--------------------------
  //----------r id------------------------
  val lfb_biu_r_id_hit = io.in.biuIn.rVld && (io.in.biuIn.rId(4,3) === BIU_LFB_ID_T)
  val lfb_biu_id_2to0 = io.in.biuIn.rId(2,0)
  val lfb_r_id_hit_addr_ptr = Seq.fill(LFB_ADDR_ENTRY)(RegInit(false.B))
  val lfb_data_not_full = Wire(Bool())
  val lfb_addr_entry_resp_set = Seq.fill(LFB_ADDR_ENTRY)(Wire(Bool()))
  for(i<- 0 until(LFB_ADDR_ENTRY)){
    lfb_addr_entry_resp_set(i) := lfb_biu_r_id_hit && lfb_data_not_full && lfb_r_id_hit_addr_ptr(i)
  }
  //----------r resp----------------------
  val lfb_r_resp_share = io.in.biuIn.rResp(3)
  val lfb_r_resp_err = io.in.biuIn.rResp(1,0) === DECERR ||  io.in.biuIn.rResp(1,0) === SLVERR
  //------------------create ptr------------------------------
  val lfb_data_create_ptr =  UIntToOH(PriorityEncoder(VecInit(lfb_data_entry_vld).asUInt))
  //------------------create signal---------------------------
  //if no vld, or only one vld and full, then create
  val lfb_data_wait_surplus = lfb_data_entry_wait_surplus.reduce(_ || _)
  val lfb_data_create_vld        = lfb_biu_r_id_hit  &&  !lfb_data_wait_surplus
  val lfb_data_create_dp_vld     = lfb_data_create_vld
  val lfb_data_create_gateclk_en = lfb_data_create_vld

  val lfb_data_entry_create_vld         = Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  val lfb_data_entry_create_dp_vld      = Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  val lfb_data_entry_create_gateclk_en  = Seq.fill(LFB_DATA_ENTRY)(Wire(Bool()))
  for(i<- 0 until(LFB_DATA_ENTRY)){
    lfb_data_entry_create_vld(i)        := lfb_data_create_vld && lfb_data_create_ptr(i).asBool
    lfb_data_entry_create_dp_vld(i)     := lfb_data_create_vld && lfb_data_create_ptr(i).asBool
    lfb_data_entry_create_gateclk_en(i) := lfb_data_create_vld && lfb_data_create_ptr(i).asBool
  }
  //------------------first pass ptr--------------------------
  val lfb_r_id_hit_addr_ptr_encode = Wire(UInt(LFB_ADDR_ENTRY.W))
  lfb_r_id_hit_addr_ptr_encode := UIntToOH(lfb_biu_id_2to0)
  for(i<- 0 to LFB_DATA_ENTRY){
    lfb_r_id_hit_addr_ptr(i) := lfb_r_id_hit_addr_ptr_encode(i).asBool
  }
  val lfb_pass_addr_5to4 = Wire(UInt(2.W))
  lfb_pass_addr_5to4 := lfb_r_id_hit_addr_ptr.zip(lfb_addr_entry_addr_tto4).map{
    case(ptr, tto4) =>
      Mux(ptr, tto4,0.U)
  }.reduce(_ | _)

  val lfb_first_pass_ptr = UIntToOH(lfb_pass_addr_5to4)
  //==========================================================
  //                Linefill state machine
  //==========================================================
  //----------------------registers---------------------------
  //+-----+
  //| vld |
  //+-----+
  val lfb_lf_sm_cnt = RegInit(false.B)
  when(lfb_lf_sm_req){
    lfb_lf_sm_vld := true.B
  }.elsewhen(lfb_lf_sm_cnt){
    lfb_lf_sm_vld := false.B
  }
  //+------------+---------+---------+------+
  //| refill way | addr_id | data_id | addr |
  //+------------+---------+---------+------+
  val lfb_lf_sm_dcache_share = RegInit(false.B)
  val lfb_lf_sm_refill_way   = RegInit(false.B)
  val lfb_lf_sm_addr_id = RegInit(0.U(LFB_ADDR_ENTRY.W))
  val lfb_lf_sm_data_id = RegInit(0.U(LFB_DATA_ENTRY.W))
  val lfb_lf_sm_addr_tto6 = RegInit(0.U((PA_WIDTH-6).W))
  val lfb_lf_sm_data_dcache_share = Wire(Bool())
  val lfb_lf_sm_req_refill_way    = Wire(Bool())
  val lfb_lf_sm_req_addr_ptr  = Wire(UInt(LFB_ADDR_ENTRY.W))
  val lfb_lf_sm_req_data_ptr  = Wire(UInt(LFB_DATA_ENTRY.W))

  when(lfb_lf_sm_req && lfb_lf_sm_permit){
    lfb_lf_sm_dcache_share :=  lfb_lf_sm_data_dcache_share
    lfb_lf_sm_refill_way   :=  lfb_lf_sm_req_refill_way
    lfb_lf_sm_addr_id      :=  (lfb_lf_sm_req_addr_ptr)
    lfb_lf_sm_data_id      :=  lfb_lf_sm_req_data_ptr
    lfb_lf_sm_addr_tto6    :=  lfb_lf_sm_req_addr_tto6
  }

  //+-----+------+
  //| cnt | bias |
  //+-----+------+
  //cnt is used for control path, bias is used for data path
  lfb_lf_sm_permit :=  !lfb_lf_sm_vld ||  lfb_lf_sm_cnt
  lfb_lf_sm_req := lfb_data_entry_lf_sm_req.reduce(_ || _)
  val lfb_lf_sm_create_vld = lfb_lf_sm_req && lfb_lf_sm_permit
  when(lfb_lf_sm_create_vld){
    lfb_lf_sm_cnt := false.B
  }.elsewhen(io.in.dcache_arb_lfb_ld_grnt){
    lfb_lf_sm_cnt := !lfb_lf_sm_cnt
  }
  lfb_lf_sm_req_data_ptr := UIntToOH(PriorityEncoder(VecInit(lfb_data_entry_lf_sm_req).asUInt))
  val lfb_lf_sm_data_grnt = Cat(Seq.fill(LFB_DATA_ENTRY)(lfb_lf_sm_create_vld))  & lfb_lf_sm_req_data_ptr

  lfb_lf_sm_req_addr_ptr := Cat(Seq.fill(LFB_ADDR_ENTRY)(lfb_lf_sm_req_data_ptr(0))) & lfb_data_entry_addr_id(0) |
    Cat(Seq.fill(LFB_ADDR_ENTRY)(lfb_lf_sm_req_data_ptr(1))) & lfb_data_entry_addr_id(1)
  for(i<- 0 until(LFB_ADDR_ENTRY)){
    when(lfb_lf_sm_req_addr_ptr(i).asBool){
      lfb_lf_sm_req_addr_tto6 := lfb_addr_entry_addr_tto4(i)
    }.otherwise{
      lfb_lf_sm_req_addr_tto6 := 0.U
    }
  }
  val lfb_lf_sm_req_depd       = (lfb_lf_sm_req_addr_ptr & VecInit(lfb_addr_entry_depd).asUInt).orR
  lfb_lf_sm_req_refill_way    := (lfb_lf_sm_req_addr_ptr & VecInit(lfb_addr_entry_refill_way).asUInt).orR
  lfb_lf_sm_data_dcache_share := (lfb_lf_sm_req_addr_ptr & VecInit(lfb_data_entry_dcache_share).asUInt).orR
  //------------------refill wakeup req-----------------------
  val lfb_lf_sm_refill_wakeup = lfb_lf_sm_req_depd  &&  lfb_lf_sm_create_vld
  //----------------------settle addr-------------------------
  //-----------------data-----------------
  val lfb_lf_sm_data512 = Cat(Seq.fill(XLEN*8)(lfb_lf_sm_data_id(0))) & lfb_data_entry_data(0) |
    Cat(Seq.fill(XLEN*8)(lfb_lf_sm_data_id(1))) & lfb_data_entry_data(1)
  val lfb_lf_sm_data256 = Mux(lfb_lf_sm_cnt ,lfb_lf_sm_data512(511,256) ,lfb_lf_sm_data512(255,0))
  val lfb_lf_sm_data_settle = Mux(lfb_lf_sm_refill_way ,Cat(lfb_lf_sm_data256(127,0),lfb_lf_sm_data256(255,128)) ,lfb_lf_sm_data256(255,0))

  //----------------------cache interface---------------------//----------------------cache interface---------------------
  io.out.toArb.ld_req:= lfb_lf_sm_vld
  io.out.toArb.st_req:= lfb_lf_sm_vld
  io.out.toArb.serial_req:= lfb_lf_sm_vld && !lfb_lf_sm_cnt
  //---------------tag array--------------
  io.out.toArb.ld_tag_req         := lfb_lf_sm_vld &&  lfb_lf_sm_cnt
  io.out.toArb.ld_tag_gateclk_en  := io.out.toArb.ld_tag_req
  io.out.toArb.ld_tag_idx         := lfb_lf_sm_addr_tto6(8,0)
  io.out.toArb.ld_tag_din         := Cat(1.U(1.W), lfb_lf_sm_addr_tto6(PA_WIDTH-7,8), 1.U(1.W), lfb_lf_sm_addr_tto6(PA_WIDTH-7,8))
  io.out.toArb.ld_tag_wen         := Mux(lfb_lf_sm_cnt,Cat(lfb_lf_sm_refill_way,!lfb_lf_sm_refill_way), 0.U(2.W))

  io.out.toArb.st_tag_req          := io.out.toArb.ld_tag_req
  io.out.toArb.st_tag_gateclk_en   := io.out.toArb.ld_tag_req
  io.out.toArb.st_tag_idx          := io.out.toArb.ld_tag_idx
  io.out.toArb.st_tag_din          := Cat( lfb_lf_sm_addr_tto6(PA_WIDTH-7,8), lfb_lf_sm_addr_tto6(PA_WIDTH-7,8))
  io.out.toArb.st_tag_wen          := io.out.toArb.ld_tag_wen
  //---------------dirty array------------
  io.out.toArb.st_dirty_req         := io.out.toArb.ld_tag_req
  io.out.toArb.st_dirty_gateclk_en  := io.out.toArb.ld_tag_req
  io.out.toArb.st_dirty_idx         := lfb_lf_sm_addr_tto6(8,0)
  io.out.toArb.st_dirty_din         := Cat(!lfb_lf_sm_refill_way,0.U(1.W), lfb_lf_sm_dcache_share, 1.U(1.W) , 0.U(1.W),lfb_lf_sm_dcache_share,1.U(1.W)  )
  io.out.toArb.st_dirty_wen         := Cat(1.U(1.W) , Cat(Seq.fill(3)(lfb_lf_sm_refill_way)), Cat(Seq.fill(3)(!lfb_lf_sm_refill_way)))
  //---------------data array-------------
  io.out.toArb.ld_data_gateclk_en := Cat(Seq.fill(8)(lfb_lf_sm_vld))
  io.out.toArb.ld_data_idx        := Cat(lfb_lf_sm_addr_tto6(8,0), lfb_lf_sm_cnt,lfb_lf_sm_refill_way)
  io.out.toArb.ld_data_low_din    := lfb_lf_sm_data_settle(127,0)
  io.out.toArb.ld_data_high_din   := lfb_lf_sm_data_settle(255,128)
  //----------------------pop signal--------------------------
  val lfb_lf_sm_addr_pop_req = lfb_lf_sm_addr_id & Cat(Seq.fill(LFB_ADDR_ENTRY)(lfb_lf_sm_vld &&  lfb_lf_sm_cnt))
  val lfb_lf_sm_data_pop_req = lfb_lf_sm_data_id & Cat(Seq.fill(LFB_DATA_ENTRY)(lfb_lf_sm_vld &&  lfb_lf_sm_cnt))
  val lfb_data_addr_pop_req = lfb_data_entry_addr_pop_req.reduce(_ | _)
  //==========================================================
  //                Maintain wakeup queue
  //==========================================================
  //----------------------registers---------------------------
  //+--------------+
  //| wakeup_queue |
  //+--------------+
  //the queue stores the instructions waiting for wakeup
  //the 12 bit of wakeup_queue is for mcic
  val lfb_wakeup_queue_bits = (RegInit((0.U(LSIQ_ENTRY.W))))
  val lfb_wakeup_queue_vld  = RegInit(false.B)
  val lfb_wakeup_queue_next_bits = Wire((UInt(LSIQ_ENTRY.W)))
  val lfb_wakeup_queue_next_vld  = Wire(Bool())
  when(io.in.rtuFlush){
    lfb_wakeup_queue_bits := 0.U(LSIQ_ENTRY.W)
    lfb_wakeup_queue_vld  := false.B
  }.elsewhen(io.in.ldDaIn.setWakeupQueue || io.out.popDepdFf){
    lfb_wakeup_queue_bits := lfb_wakeup_queue_next_bits
    lfb_wakeup_queue_vld  := lfb_wakeup_queue_next_vld
  }
  //+-------------+
  //| depd_pop_ff |
  //+-------------+
  //if depd pop, this will set to 1, and clear wakeup_queue next cycle
  val lfb_pop_depd_ff = RegInit(false.B)
  //----------------forward to depd_pop_ff------------------
  val lfb_addr_pop_depd        = (VecInit(lfb_addr_entry_pop_vld).asUInt & VecInit(lfb_addr_entry_depd       ).asUInt).orR
  val lfb_addr_pop_discard_vld = (VecInit(lfb_addr_entry_pop_vld).asUInt & VecInit(lfb_addr_entry_discard_vld).asUInt).orR
  when(lfb_addr_pop_depd || lfb_addr_pop_discard_vld  ||
    io.in.rbIn.boundaryDepdWakeup || lfb_lf_sm_refill_wakeup || io.in.lmIn.lfbDepdWakeup){

    lfb_pop_depd_ff := true.B
  }.otherwise{
    lfb_pop_depd_ff := false.B
  }
  io.out.popDepdFf := lfb_pop_depd_ff
  //------------------update wakeup queue---------------------
  val lfb_wakeup_queue_after_pop_bits = (RegInit((0.U(LSIQ_ENTRY.W))))
  val lfb_wakeup_queue_after_pop_vld  = RegInit(false.B)
  lfb_wakeup_queue_after_pop_bits := Mux(lfb_pop_depd_ff ,0.U(LSIQ_ENTRY.W) ,lfb_wakeup_queue_bits)
  lfb_wakeup_queue_after_pop_vld  := Mux(lfb_pop_depd_ff ,false.B ,lfb_wakeup_queue_vld)
  lfb_wakeup_queue_next_bits := lfb_wakeup_queue_after_pop_bits | io.in.ldDaIn.wakeupQueueNext.bits & Cat(Seq.fill(LSIQ_ENTRY)(io.in.ldDaIn.setWakeupQueue))
  lfb_wakeup_queue_next_vld :=  lfb_wakeup_queue_after_pop_vld || io.in.ldDaIn.setWakeupQueue && io.in.ldDaIn.wakeupQueueNext.valid
  //------------------------wakeup----------------------------
  io.out.lfbDepdWakeup := Mux(lfb_pop_depd_ff,lfb_wakeup_queue_bits , Cat(Seq.fill(LSIQ_ENTRY)(0.U)))
  io.out.mcicWakeup := Mux(lfb_pop_depd_ff||io.in.rtuFlush,  lfb_wakeup_queue_vld , false.B)
  //==========================================================
  //                for avoid deadlock with no rready
  //==========================================================
  val lfb_addr_create_vld = lfb_addr_rb_create_vld || lfb_addr_pfu_create_vld
  val lfb_no_rcl_cnt_create = Cat(0.U(3.W), lfb_addr_create_vld && io.in.cp0In.lsuDcacheEn)
  val lfb_no_rcl_cnt_pop = Cat(0.U(3.W), io.in.vbIn.rclDone) +  Cat(0.U(3.W), io.in.snqIn.bypassInvalid(0)) + Cat(0.U(3.W), io.in.snqIn.bypassInvalid(1))
  val lfb_no_rcl_cnt_updt_vld = lfb_addr_create_vld && io.in.cp0In.lsuDcacheEn || io.in.vbIn.rclDone || io.in.snqIn.bypassInvalid.orR
  val lfb_no_rcl_cnt = RegInit(0.U(4.W))
  val lfb_no_rcl_cnt_updt_val =  lfb_no_rcl_cnt +  lfb_no_rcl_cnt_create + lfb_no_rcl_cnt_pop
  when(lfb_no_rcl_cnt_updt_vld){
    lfb_no_rcl_cnt := lfb_no_rcl_cnt_updt_val
  }
  val lfb_nc_rready_grnt = lfb_no_rcl_cnt <= "b0001".U
  val lfb_ca_rready_grnt = lfb_no_rcl_cnt < "b0001".U
  io.out.toRb.ncRreadyGrnt := lfb_nc_rready_grnt
  io.out.toRb.caRreadyGrnt := lfb_ca_rready_grnt
  io.out.toPfu.rreadyGrnt := lfb_ca_rready_grnt
  //for rready,if all addr entry has resp,then not deassert rready
  val lfb_addr_all_resp = !(lfb_addr_entry_not_resp.reduce(_ || _))
  //==========================================================
  //              Interface to other module
  //==========================================================
  //---------------------hit idx------------------------------
  io.out.ldDaHitIdx := lfb_addr_entry_ld_da_hit_idx.reduce(_ || _)
  io.out.stDaHitIdx := lfb_addr_entry_st_da_hit_idx.reduce(_ || _)
  io.out.toRb.biuReqHitIdx := lfb_addr_entry_rb_biu_req_hit_idx.reduce(_ || _)
  io.out.toPfu.biuReqHitIdx := lfb_addr_entry_pfu_biu_req_hit_idx.reduce(_ || _)
  io.out.toWmb.writeReqHitIdx := lfb_addr_entry_wmb_write_req_hit_idx.reduce(_ || _)
  io.out.toWmb.readReqHitIdx := lfb_addr_entry_wmb_read_req_hit_idx.reduce(_ || _)


  //----------------interface to biu--------------------------
  lfb_data_not_full := !(lfb_data_entry_full.reduce(_ || _))
  val lsu_biu_r_linefill_ready = lfb_data_not_full || lfb_addr_all_resp
  io.out.lsuBiuRLinefillReady := lsu_biu_r_linefill_ready
  //------------------full/empty signal-----------------------
  val lfb_addr_empty = !(lfb_addr_entry_vld.reduce(_ || _))
  val lfb_data_empty = !(lfb_data_entry_vld.reduce(_ || _))
  io.out.lfbEmpty := lfb_addr_empty  &&  lfb_data_empty  &&  !lfb_vb_req_unmask
  io.out.lfbAddrFull := lfb_addr_entry_vld.reduce(_ && _)
  io.out.lfbAddrLess2 := (VecInit(lfb_addr_entry_vld).asUInt | lfb_addr_create_ptr).andR
  lfb_addr_entry.zipWithIndex.foreach {
    case (entry, i) =>
      // ptr signal broadcast to addr entries
      entry.io.in.lfbIn.dataAddrPopReq_x := lfb_data_addr_pop_req(i)
      entry.io.in.pfuIn.addr             := io.in.pfuIn.reqAddr
      entry.io.in.pfuIn.id               := io.in.pfuIn.id

      entry.io.in.rbIn.ldamo             := io.in.rbIn.ldamo
      entry.io.in.lsuSpecialClk := DontCare
      entry.io.in.vbIn.dcacheHit := io.in.vbIn.dcacheHit
      entry.io.in.vbIn.dcacheDirty := io.in.vbIn.dcacheDirty
      entry.io.in.vbIn.dcacheWay := io.in.vbIn.dcacheWay
      entry.io.in.vbIn.addrEntryRclDone_x := io.in.vbIn.addrEntryRcldone

      entry.io.in.cp0In.yyClkEn := io.in.cp0In.yyClkEn
      entry.io.in.cp0In.lsuDcacheEn := io.in.cp0In.lsuDcacheEn
      entry.io.in.cp0In.lsuIcgEn := io.in.cp0In.lsuIcgEn

      entry.io.in.wmbReq.readAddr := io.in.wmbIn.readAddr
      entry.io.in.wmbReq.writeAddr := io.in.wmbIn.writeAddr

      entry.io.in.lfbIn.addrEntryRespSet_x := lfb_addr_entry_resp_set(i)
      entry.io.in.lfbIn.addrEntryVbPeReqGrnt_x := lfb_addr_entry_vb_pe_req_grnt(i)
      entry.io.in.lfbIn.vbPeReqPermit := lfb_vb_pe_req_permit
      entry.io.in.lfbIn.vbPeReq       := lfb_vb_pe_req
      entry.io.in.lfbIn.lfSmAddrPopReq_x := lfb_lf_sm_addr_pop_req(0)




      entry.io.in.rbIn.addrTto4   := io.in.rbIn.addrTto4
      entry.io.in.rbIn.biuReqAddr := io.in.rbIn.biuReqAddr
      entry.io.in.rbIn.atomic     := io.in.rbIn.atomic
      entry.io.in.rbIn.depd       := io.in.rbIn.depd


      entry.io.in.snqBypassAddrTto6 := io.in.snqIn.bypassAddrTto6

      entry.io.in.lmAlreadySnoop := io.in.lmIn.alreadySnoop

      entry.io.in.ldDaIn.idx := io.in.ldDaIn.idx
      entry.io.in.ldDaIn.lfbDiscardGrnt := io.in.ldDaIn.discardGrnt

      entry.io.in.stDaAddr := io.in.stDaAddr






  }
  lfb_data_entry.zipWithIndex.foreach {
    case (entry, i) =>
      entry.io.in.cp0In.lsuDcacheEn  := io.in.cp0In.lsuDcacheEn
      entry.io.in.cp0In.lsuIcgEn     := io.in.cp0In.lsuIcgEn
      entry.io.in.cp0In.yyClkEn      := io.in.cp0In.yyClkEn
      entry.io.in.rRespShare := lfb_r_resp_share
      entry.io.in.rRespErr   := lfb_r_resp_err
      entry.io.in.addrEntryLinefillPermit := VecInit(lfb_addr_entry_linefill_permit).asUInt
      entry.io.in.addrEntryLinefillAbort  := VecInit(lfb_addr_entry_linefill_abort).asUInt

      entry.io.in.biuAxiR.rData := io.in.biuIn.rData
      entry.io.in.biuAxiR.rVld  := io.in.biuIn.rVld
      entry.io.in.biuAxiR.rLast := io.in.biuIn.rLast
      entry.io.in.biuRIdHit := lfb_biu_r_id_hit
      entry.io.in.firstPassPtr(0)  := lfb_first_pass_ptr(0)
      entry.io.in.firstPassPtr(1)  := lfb_first_pass_ptr(1)
      entry.io.in.firstPassPtr(2)  := lfb_first_pass_ptr(2)
      entry.io.in.firstPassPtr(3)  := lfb_first_pass_ptr(3)

      entry.io.in.snqIn.bypassInvalid_x := io.in.snqIn.bypassInvalid
      entry.io.in.snqIn.bypassChgTag_x := io.in.snqIn.bypassChgTag

      entry.io.in.lfSmDataPopReq_x := lfb_lf_sm_data_pop_req(i)
      entry.io.in.biuId2to0 := lfb_biu_id_2to0
      entry.io.in.dataEntryCreateDpVld_x := lfb_data_entry_create_dp_vld(i)
      entry.io.in.lfSmDataGrnt_x := lfb_lf_sm_data_grnt(i)
      entry.io.in.dataEntryCreateGateclkEn_x := DontCare
      entry.io.in.dataEntryCreateVld_x := lfb_data_entry_create_vld(i)

  }

  io.out.toSnq := DontCare
  io.out.ldHitPrefetch := DontCare
}
