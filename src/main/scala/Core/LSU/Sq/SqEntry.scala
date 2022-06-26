package Core.LSU.Sq
import Core.DCacheConfig.INDEX_WIDTH
import Core.IUConfig.MPPWidth
import Core.Config.XLEN
import Core.LSU.StoreExStage.{StDaToSq, StDcToSq, StDcToSqDa}
import Core.LsuConfig.BYTES_ACCESS_WIDTH
import Core.ROBConfig.{IidWidth, NumCommitEntry}
import Core.RTU.CompareIid
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToSqEntry extends Bundle with LsuConfig{
  val lsuIcgEn = Bool()
  val clkEn    = Bool()
  val privMode = UInt(MPPWidth.W)
}
class DcacheToSqEntry extends Bundle with DCacheConfig{
  val dirtyDin  = Flipped(ValidIO(Vec(WAYS, new DcacheDirtyDataEn)))
  val dirtyGwen = Bool()
  val dirtyWen  = Flipped(ValidIO(Vec(WAYS, new DcacheDirtyDataEn)))
  val idx       = UInt(INDEX_WIDTH.W)
  val tagDin    = Vec(WAYS,Flipped(ValidIO(UInt(TAG_WIDTH.W))))
  val tagGwen   = Bool()
  val tagWen    = Vec(WAYS,Bool())
}
class LdDcToSqEntry extends Bundle with LsuConfig {
 val addr0            = UInt(PA_WIDTH.W)
 val addr1_11to4      = UInt(8.W)
 val bytesVld         = UInt(BYTES_ACCESS_WIDTH.W)
 val bytesVld1        = UInt(BYTES_ACCESS_WIDTH.W)
 val chkAtomicInstVld = Bool()
 val chkLdAddr1Vld    = Bool()
 val chkLdBypassVld   = Bool()
 val chkLdInstVld     = Bool()
 val iid              = UInt(IidWidth.W)
}
class RtuToSqEntry extends Bundle with LsuConfig {
  val asyncFlush = Bool()
  val commitIidUpdt  = Vec(NumCommitEntry, UInt(IidWidth.W))
  val commit     = Vec(NumCommitEntry, Bool())
  val flush      = Bool()
}
//class SqIn extends Bundle with LsuConfig {
//  val asyncFlush = Bool()
//  val commitIid  = Vec(NumCommitEntry, UInt(IidWidth.W))
//  val commit     = Vec(NumCommitEntry, Bool())
//  val flush      = Bool()
//}
class SqToSqEntry extends Bundle with LsuConfig {
  val ageVecSet              = Bool()
  val createAgeVec           = UInt(LSIQ_ENTRY.W)
  val createSameAddrNewest   = Bool()
  val createSuccess          = Bool()
  val createVld              = UInt(LSIQ_ENTRY.W)
  val dataSettle             = UInt(XLEN.W)
  val createDpVldX           = Bool()
  val createGateclkEnX       = Bool()
  val createVldX             = Bool()
  val dataDiscardGrntX       = Bool()
  val fwdMultiDepdSetX       = Bool()
  val popToCeGrntB           = UInt(LSIQ_ENTRY.W)
  val popToCeGrntX           = Bool()
  val popPtrX                = Bool()
  //val create_pop_clk       = Bool()
}
class StDcToSqTotal extends Bundle{
  val sq    = new StDcToSq
  val sqda  = new StDcToSqDa
}
class StDaToSqTotal extends Bundle with LsuConfig {
  val stDaIn = new StDaToSq
  val addr      = UInt(PA_WIDTH.W)
  val bkptaData = Bool()
  val bkptbData = Bool()
  val instVld   = Bool()
  val iid       = UInt(IidWidth.W)
}
//----------------------------------------------------------
class SqEntryIn extends Bundle with LsuConfig{
  val cp0In    = new Cp0ToSqEntry
  val dcacheIn = new DcacheToSqEntry
  val ldDaLsid = UInt(LSIQ_ENTRY.W)
  val ldDcIn   = new LdDcToSqEntry
  val rtuIn    = new RtuToSqEntry
  val sdEx1In  = new SdEx1ToSqEntry
  val sqIn     = new SqToSqEntry
  val daIn     = new StDaToSqTotal
  val dcIn     = new StDcToSqTotal
  val wmbSqPopGrnt = Bool()
}
//==========================================================
//                        Output
//==========================================================
class SqEntryOut extends Bundle with LsuConfig{
  val addr0_v                    = UInt(PA_WIDTH.W)
  val addr1DepDiscard_x          = Bool()
  val ageVecSurplus1Ptr_x        = Bool()
  val ageVecZeroPtr_x            = Bool()
  val atomic_x                   = Bool()
  val bkptaData_x                = Bool()
  val bkptbData_x                = Bool()
  val bytesVld_v                 = UInt(BYTES_ACCESS_WIDTH.W)
  val cancelAccReq_x             = Bool()
  val cancelAheadWb_x            = Bool()
  val cmitData_vld_x             = Bool()
  val cmit_x                     = Bool()
  val dataDepdWakeup_v           = UInt(LSIQ_ENTRY.W)
  val dataDiscardReqShort_x      = Bool()
  val dataDiscardReq_x           = Bool()
  val data_v                     = UInt(XLEN.W)
  val dcacheDirty_x              = Bool()
  val dcacheInfo_vld_x           = Bool()
  val dcacheShare_x              = Bool()
  val dcacheValid_x              = Bool()
  val dcacheWay_x                = Bool()
  val depdSet_x                  = Bool()
  val depd_x                     = Bool()
  val discardReq_x               = Bool()
  val fenceMode_v                = UInt(FENCE_MODE_WIDTH.W)
  val fwdBypassReq_x             = Bool()
  val fwdReq_x                   = Bool()
  val icc_x                      = Bool()
  val iid_v                      = UInt(IidWidth.W)
  val instFlush_x                = Bool()
  val instHit_x                  = Bool()
  val instMode_v                 = UInt(INST_MODE_WIDTH.W)
  val instSize_v                 = UInt(INST_SIZE_WIDTH.W)
  val instType_v                 = UInt(INST_TYPE_WIDTH.W)
  val newestFwdReqDataVldShort_x = Bool()
  val newestFwdReqDataVld_x      = Bool()
  val pageBuf_x                  = Bool()
  val pageCa_x                   = Bool()
  val pageSec_x                  = Bool()
  val pageShare_x                = Bool()
  val pageSo_x                   = Bool()
  val pageWa_x                   = Bool()
  val popReq_x                   = Bool()
  val privMode_v                 = UInt(MPPWidth.W)
  val rotSel_v                   = UInt(ROT_SEL_WIDTH_8.W)
  val sameAddrNewest_x           = Bool()
  val settleDataHit_x            = Bool()
  val specFail_x                 = Bool()
  val stDcCreateAgeVec_x         = Bool()
  val stDcSameAddrNewer_x        = Bool()
  val syncFence_x                = Bool()
  val vld_x                      = Bool()
  val vstartVld_x                = Bool()
  val woSt_x                     = Bool()
}
//==========================================================
//                          IO
//==========================================================
class SqEntryIO extends Bundle with LsuConfig{
  val in = Input(new SqEntryIn)
  val out = Output(new SqEntryOut)
}

class SqEntry extends Module with LsuConfig {
  val io = IO(new SqEntryIO)
  val sq_entry_vld = RegInit(false.B)
  val sq_entry_pop_vld = Wire(Bool())
  val sq_entry_flush_pop_vld = Wire(Bool())
  val sq_entry_expt_pop_vld = Wire(Bool())
  //-----------create signal--------------
  val sq_entry_create_vld         = io.in.sqIn.createVldX
  val sq_entry_create_dp_vld      = io.in.sqIn.createDpVldX
  val sq_entry_create_gateclk_en  = io.in.sqIn.createGateclkEnX
  val sq_pop_ptr                  = io.in.sqIn.popPtrX
  dontTouch(sq_pop_ptr)
  //-----------grnt signal----------------
  val sq_entry_data_discard_grnt  = io.in.sqIn.dataDiscardGrntX
  val sq_entry_fwd_multi_depd_set = io.in.sqIn.fwdMultiDepdSetX
  val sq_entry_pop_to_ce_grnt     = io.in.sqIn.popToCeGrntX
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //----------entry gateclk---------------
  val sq_entry_clk_en  = sq_entry_vld  ||  sq_entry_create_gateclk_en
  //--------create update gateclk---------
  val sq_entry_create_clk_en  = sq_entry_create_gateclk_en
  val sq_entry_st_da_info_set = Wire(Bool())
  val sq_entry_create_da_clk_en  = sq_entry_create_gateclk_en  || sq_entry_st_da_info_set
  //----------data gateclk----------------
  val sq_entry_data_set = Wire(Bool())
  val sq_entry_data_clk_en = sq_entry_data_set
  //--------wakeup queue gateclk----------
  val sq_entry_data_set_ff = RegInit(false.B)
  val sq_entry_has_wait_restart = Wire(Bool())
  val sq_entry_wakeup_queue_clk_en = sq_entry_data_discard_grnt ||
    (sq_entry_data_set || sq_entry_data_set_ff || io.in.rtuIn.flush) &&
      sq_entry_has_wait_restart
  //==========================================================
  //                 Register
  //==========================================================
  //+-----------+
  //| entry_vld |
  //+-----------+
  when(sq_entry_pop_vld||sq_entry_flush_pop_vld||sq_entry_expt_pop_vld||io.in.rtuIn.asyncFlush){
    sq_entry_vld := false.B
  }.elsewhen(sq_entry_create_vld){
    sq_entry_vld := true.B
  }
  //+-----------+
  //| in wmb ce |
  //+-----------+
  val sq_entry_in_wmb_ce = RegInit(false.B)
  when(sq_entry_create_dp_vld){
    sq_entry_in_wmb_ce := false.B // if sq dp create, cant enable wmb
  }.elsewhen(sq_entry_pop_to_ce_grnt){
    sq_entry_in_wmb_ce := true.B
  }
  //+-----------+
  //| fwd_en    |
  //+-----------+
  // for most multi forward situation, use this bit for newest forward
  val sq_entry_same_addr_newest = RegInit(false.B)
  val sq_entry_same_addr_newest_clr = Wire(Bool())
  when(sq_entry_create_dp_vld){
    sq_entry_same_addr_newest := io.in.sqIn.createSameAddrNewest
  }.elsewhen(sq_entry_same_addr_newest_clr){
    sq_entry_same_addr_newest := false.B
  }
  //+-------------------------+
  //| instruction information |
  //+-------------------------+
  val sq_entry_sync_fence       = RegInit(false.B)
  val sq_entry_atomic           = RegInit(false.B)
  val sq_entry_icc              = RegInit(false.B)
  val sq_entry_inst_flush       = RegInit(false.B)
  val sq_entry_inst_type        = RegInit(0.U(INST_TYPE_WIDTH.W))
  val sq_entry_inst_size        = RegInit(0.U(INST_SIZE_WIDTH.W))
  val sq_entry_inst_mode        = RegInit(0.U(INST_MODE_WIDTH.W))
  val sq_entry_fence_mode       = RegInit(0.U(FENCE_MODE_WIDTH.W))
  val sq_entry_iid              = RegInit(0.U(IidWidth.W))
  val sq_entry_sdid             = RegInit(0.U(SDID_WIDTH.W))
  val sq_entry_page_share       = RegInit(false.B)
  val sq_entry_page_so          = RegInit(false.B)
  val sq_entry_page_ca          = RegInit(false.B)
  val sq_entry_page_wa          = RegInit(false.B)
  val sq_entry_page_buf         = RegInit(false.B)
  val sq_entry_page_sec         = RegInit(false.B)
  val sq_entry_wo_st            = RegInit(false.B)
  val sq_entry_boundary         = RegInit(false.B)
  val sq_entry_secd             = RegInit(false.B)
  val sq_entry_addr0            = RegInit(0.U(PA_WIDTH.W))
  val sq_entry_bytes_vld        = RegInit(0.U(BYTES_ACCESS_WIDTH.W))
  val sq_entry_priv_mode        = RegInit(0.U(MPPWidth.W))
  val sq_entry_rot_sel          = RegInit(0.U(ROT_SEL_WIDTH_8.W))
  when(sq_entry_create_dp_vld){
    sq_entry_sync_fence := io.in.dcIn.sqda.syncFence
    sq_entry_atomic     := io.in.dcIn.sqda.atomic
    sq_entry_icc        := io.in.dcIn.sqda.icc
    sq_entry_inst_flush := io.in.dcIn.sq.instFlush
    sq_entry_inst_type  := io.in.dcIn.sqda.instType
    sq_entry_inst_size  := io.in.dcIn.sqda.instSize
    sq_entry_inst_mode  := io.in.dcIn.sqda.instMode
    sq_entry_fence_mode := io.in.dcIn.sqda.fenceMode
    sq_entry_iid        := io.in.dcIn.sq.iid
    sq_entry_sdid       := io.in.dcIn.sq.sdid
    sq_entry_page_share := io.in.dcIn.sqda.pageShare
    sq_entry_page_so    := io.in.dcIn.sqda.pageSo
    sq_entry_page_ca    := io.in.dcIn.sqda.pageCa
    sq_entry_page_wa    := io.in.dcIn.sqda.pageWa
    sq_entry_page_buf   := io.in.dcIn.sqda.pageBuf
    sq_entry_page_sec   := io.in.dcIn.sqda.pageSec
    sq_entry_wo_st      := io.in.dcIn.sq.woStInst
    sq_entry_boundary   := io.in.dcIn.sqda.boundary
    sq_entry_secd       := io.in.dcIn.sqda.secd
    sq_entry_addr0      := io.in.dcIn.sq.addr0
    sq_entry_bytes_vld  := io.in.dcIn.sq.bytesVld
    sq_entry_priv_mode  := io.in.cp0In.privMode
    sq_entry_rot_sel    := io.in.dcIn.sq.rotSelRev
  }
  //+------+
  //| cmit |
  //+------+
  val sq_entry_cmit_set = Wire(Bool())
  val sq_entry_cmit = RegInit(false.B)
  when(sq_entry_create_dp_vld){
    sq_entry_cmit := false.B
  }.elsewhen(sq_entry_cmit_set){
    sq_entry_cmit := true.B
  }
  //+----------+
  //| data_vld |
  //+----------+
  val sq_entry_data_vld = RegInit(false.B)
  when(sq_entry_create_dp_vld){
    sq_entry_data_vld := io.in.dcIn.sq.sqDataVld
  }.elsewhen(sq_entry_data_set){
    sq_entry_data_vld := true.B
  }
  //+-------------+
  //| data_set_ff |
  //+-------------+
  when(sq_entry_create_dp_vld){
    sq_entry_data_set_ff := false.B
  }.elsewhen(sq_entry_data_set){
    sq_entry_data_set_ff := true.B
  }.otherwise{
    sq_entry_data_set_ff := false.B
  }
  //+--------------+
  //| wakeup_queue |
  //+--------------+
  val sq_entry_wakeup_queue = RegInit(0.U(LSIQ_ENTRY.W))
  when(sq_entry_create_dp_vld){
    sq_entry_wakeup_queue := 0.U(LSIQ_ENTRY.W)
  }.elsewhen(sq_entry_data_set){
    sq_entry_wakeup_queue := io.in.ldDaLsid
  }.elsewhen(sq_entry_data_discard_grnt){
    sq_entry_wakeup_queue := io.in.ldDaLsid | sq_entry_wakeup_queue
  }
  sq_entry_has_wait_restart := !(PopCount(sq_entry_wakeup_queue) === 0.U)
  //+------+
  //| data |
  //+------+
  val sq_entry_data = RegInit(0.U(XLEN.W))
  when(sq_entry_data_set){
    sq_entry_data := io.in.sqIn.dataSettle
  }
  //+-------------------+
  //| st_da information |
  //+-------------------+
  //include dcache info/no_restart info
  //include spec_fail/bkpt info for timing
  val sq_entry_spec_fail    = RegInit(false.B)
  val sq_entry_bkpta_data   = RegInit(false.B)
  val sq_entry_bkptb_data   = RegInit(false.B)
  val sq_entry_vstart_vld   = RegInit(false.B)
  when(sq_entry_st_da_info_set){
    sq_entry_spec_fail    := io.in.daIn.stDaIn.wb.specFail
    sq_entry_bkpta_data   := io.in.daIn.bkptaData
    sq_entry_bkptb_data   := io.in.daIn.bkptbData
    sq_entry_vstart_vld   := io.in.daIn.stDaIn.wbVstartVld
  }
  val sq_entry_dcache_info_vld  = RegInit(false.B)
  val sq_entry_no_restart       = RegInit(false.B)
  when(sq_entry_create_dp_vld){
    sq_entry_dcache_info_vld := false.B
    sq_entry_no_restart      := false.B
  }.elsewhen(sq_entry_st_da_info_set){
    sq_entry_dcache_info_vld := true.B
    sq_entry_no_restart      := io.in.daIn.stDaIn.sqNoRestart
  }
  //+-------------+
  //| dcache info |
  //+-------------+
  val sq_entry_dcache_update_vld = Wire(Bool())
  val sq_entry_dcache_mesi = RegInit(0.U.asTypeOf(new DcacheDirtyDataEn))
  val sq_entry_dcache_way   = RegInit(false.B)
  val sq_entry_update_dcache_mesi = WireInit(0.U.asTypeOf(new DcacheDirtyDataEn))
  val sq_entry_update_dcache_way   = Wire(Bool())
  when(sq_entry_dcache_update_vld){
    sq_entry_dcache_mesi := sq_entry_update_dcache_mesi
    sq_entry_dcache_way   := sq_entry_update_dcache_way
  }.elsewhen(sq_entry_st_da_info_set){
    sq_entry_dcache_mesi := io.in.daIn.stDaIn.sqDcacheMesi
    sq_entry_dcache_way   := io.in.daIn.stDaIn.sqDcacheWay
  }
  //+---------+
  //| age_vec |
  //+---------+
  val sq_entry_age_vec_next = Wire(UInt(LSIQ_ENTRY.W))
  val sq_entry_age_vec = RegInit(0.U(LSIQ_ENTRY.W))
  when(sq_entry_create_dp_vld){
    sq_entry_age_vec := io.in.sqIn.createAgeVec
  }.elsewhen(io.in.sqIn.ageVecSet && sq_entry_vld){
    sq_entry_age_vec := sq_entry_age_vec_next
  }
  //+------+
  //| depd |
  //+------+
  val sq_entry_depd_set = Wire(Bool())
  val sq_entry_depd = RegInit(false.B)
  when(sq_entry_create_dp_vld){
    sq_entry_depd := false.B
  }.elsewhen(sq_entry_depd_set){
    sq_entry_depd := true.B
  }
  val sq_entry_cmit_iid_hit = Seq.fill(NumCommitEntry)(RegInit(false.B))
  val sq_entry_cmit_iid_pre_hit = Seq.fill(NumCommitEntry)(Wire(Bool()))
  for(i <- 0 until NumCommitEntry){
    when(sq_entry_create_dp_vld){
      sq_entry_cmit_iid_hit(i) := io.in.dcIn.sq.cmitIidCrtHit(i)
    }.elsewhen(sq_entry_depd_set){
      sq_entry_cmit_iid_hit(i) := sq_entry_cmit_iid_pre_hit(i)
    }
  }
  val sq_entry_st_data_sdid_hit = RegInit(false.B)
  val sq_entry_sdid_hit = Wire(Bool())
  when(sq_entry_create_dp_vld){
    sq_entry_st_data_sdid_hit := io.in.dcIn.sq.sdidHit
  }.elsewhen(sq_entry_vld && !sq_entry_data_vld){
    sq_entry_st_data_sdid_hit := sq_entry_sdid_hit
  }
  val sq_entry_bond_first_only = RegInit(false.B)
  val sq_bond_secd_create_vld = Wire(Bool())
  when(sq_entry_create_dp_vld){
    sq_entry_bond_first_only := io.in.dcIn.sq.boundaryFirst
  }.elsewhen(sq_bond_secd_create_vld){
    sq_entry_bond_first_only := false.B
  }
  //==========================================================
  //        Generate inst type
  //==========================================================
  val sq_entry_dcache_inst     = !sq_entry_atomic  &&  sq_entry_icc      &&  (sq_entry_inst_type(1,0) === "b10".U)
  val sq_entry_st_inst         = !sq_entry_icc     &&  !sq_entry_atomic  &&  !sq_entry_sync_fence
  val sq_entry_wo_st_inst      = sq_entry_st_inst  &&  !sq_entry_page_so
  val sq_entry_dcache_sw_inst  = sq_entry_dcache_inst  &&  (sq_entry_inst_mode(1,0) === "b10".U)
  //==========================================================
  //      Generate cmit/st_da info/update signal
  //==========================================================
  //----------------------cmit signal-------------------------
  // dc iid equal commit iid can enable sq commit
  val sq_entry_cmit_hit = Seq.fill(NumCommitEntry)(Wire(Bool()))
  for(i <- 0 until NumCommitEntry){
    sq_entry_cmit_iid_pre_hit(i) := io.in.rtuIn.commitIidUpdt(i) === sq_entry_iid
    sq_entry_cmit_hit(i) := io.in.rtuIn.commit(i) && sq_entry_cmit_iid_pre_hit(i)
  }
  sq_entry_cmit_set := sq_entry_cmit_hit.reduce(_ || _) && sq_entry_vld
  //-----------------cmit data vld signal---------------------
  val sq_entry_cmit_data_not_vld = sq_entry_vld && (sq_entry_cmit || sq_entry_cmit_set)  &&
    !sq_entry_data_vld
  val sq_entry_cmit_data_vld = !sq_entry_cmit_data_not_vld
  //---------------------st_da info siganl--------------------
  sq_entry_st_da_info_set := sq_entry_vld && io.in.daIn.instVld &&
    !io.in.daIn.stDaIn.sqEccStall && !sq_entry_no_restart &&
    (io.in.daIn.stDaIn.secd === sq_entry_secd) && (io.in.daIn.iid === sq_entry_iid)
  //-------------------boundary secd signal-------------------
  sq_bond_secd_create_vld := sq_entry_vld && io.in.sqIn.createSuccess &&
    io.in.dcIn.sqda.secd && (io.in.dcIn.sq.iid === sq_entry_iid)
  //---------------------data update signal-------------------
  sq_entry_sdid_hit := sq_entry_sdid === io.in.sdEx1In.rfEx1Sdid
  val sq_entry_settle_data_hit = sq_entry_vld && !sq_entry_data_vld && sq_entry_st_data_sdid_hit
  sq_entry_data_set := sq_entry_vld && !sq_entry_data_vld && io.in.sdEx1In.ex1InstVld && sq_entry_st_data_sdid_hit
  //-----------------------fwd signal-------------------------
  //to decrease multi forward depd
  val sq_entry_newer_than_st_dc = Wire(Bool())
  val sq_entry_addr_11to4_hit_st_dc = sq_entry_addr0(11,4) === io.in.dcIn.sq.addr0(11,4)
  val sq_entry_st_dc_bv_do_hit = !(PopCount(io.in.dcIn.sq.bytesVld & sq_entry_bytes_vld) === 0.U)
  sq_entry_same_addr_newest_clr := sq_entry_vld && io.in.sqIn.createSuccess && !sq_entry_newer_than_st_dc &&
    sq_entry_addr_11to4_hit_st_dc && sq_entry_st_dc_bv_do_hit
  //to sq_create_fwd_newest
  val sq_entry_st_dc_same_addr_newer = sq_entry_vld && sq_entry_newer_than_st_dc &&
    sq_entry_addr_11to4_hit_st_dc && sq_entry_st_dc_bv_do_hit
  //==========================================================
  //                 sq iid check
  //==========================================================
  //check iid to judge whether to create sq
  val sq_entry_inst_hit = sq_entry_vld && !sq_entry_no_restart && (io.in.dcIn.sqda.secd === sq_entry_secd) && (io.in.dcIn.sq.iid === sq_entry_iid)
  //==========================================================
  //            Compare dcache write port(dcwp)
  //==========================================================
  val sq_entry_dcache_info_updata = Module(new LsuDcacheInfoUpdate)
  sq_entry_dcache_info_updata.io.in.compareDcwpAddr := sq_entry_addr0
  sq_entry_dcache_info_updata.io.in.compareDcwpSwInst := sq_entry_dcache_sw_inst
  sq_entry_dcache_info_updata.io.in.dcacheIn  := io.in.dcacheIn
  sq_entry_dcache_info_updata.io.in.originDcacheMesi := sq_entry_dcache_mesi
  sq_entry_dcache_info_updata.io.in.originDcacheWay  := sq_entry_dcache_way
  sq_entry_update_dcache_mesi := sq_entry_dcache_info_updata.io.out.updateDcacheMesi
  sq_entry_update_dcache_way := sq_entry_dcache_info_updata.io.out.updateDcacheWay
  val sq_entry_dcache_hit_idx = sq_entry_dcache_info_updata.io.out.compareDcwpHitIdx
  val sq_entry_dcache_update_vld_unmask = sq_entry_dcache_info_updata.io.out.compareDcwpUpdateVld
  sq_entry_dcache_update_vld := sq_entry_dcache_update_vld_unmask && sq_entry_vld && sq_entry_dcache_info_vld
  //==========================================================
  //                  Maintain Age Vector
  //==========================================================
  //if age_vec[n] = 1, it means sq_entry_n is older than this sq_entry
  //age_vec -> age_vec_create -> age_vec_next
  //-------------------age_vec after create-------------------
  //sq entry newer than st_dc
  val sqCompareStDc = Module(new CompareIid)
  sqCompareStDc.io.iid0 := io.in.dcIn.sq.iid
  sqCompareStDc.io.iid1 := sq_entry_iid
  val sq_entry_iid_newer_than_st_dc = sqCompareStDc.io.older
  val sq_entry_st_dc_create_age_vec = sq_entry_vld && !sq_entry_in_wmb_ce && !sq_entry_pop_to_ce_grnt && !sq_entry_newer_than_st_dc
  sq_entry_newer_than_st_dc := !sq_entry_cmit && sq_entry_iid_newer_than_st_dc
  /**
   * io.in.sqIn.createVld is the newest create ptr, which has form like 00001000, only one bit is choosed
   * and '|' with sq_entry_age_vec, which is old age_vec
   * this include 3 situations
   * 1. create a new ptr, old age_vec is all 0, only io.in.sqIn.createVld
   * 2. inherit old ptr, only old age_vec
   * 3. create new ptr but sq iid is old than dc iid
   */
  val sq_entry_age_vec_create = io.in.sqIn.createVld & Cat(Seq.fill(LSIQ_ENTRY)(sq_entry_iid_newer_than_st_dc)) | sq_entry_age_vec
  //-------------------age_vecafter pop-----------------------
  // consider pop ptr,add to age vec, then the ageVecNext is created
  sq_entry_age_vec_next := io.in.sqIn.popToCeGrntB & sq_entry_age_vec_create
  val sq_entry_age_vec_less2 = PopCount(sq_entry_age_vec) === 1.U // means this ptr will pop after another ptr
  // to judge sq_entry_age_vec is all zero && if not wmb in, this is the pop ptr
  io.out.ageVecZeroPtr_x := sq_entry_vld && (!sq_entry_in_wmb_ce) && (!sq_entry_age_vec.orR)
  io.out.ageVecSurplus1Ptr_x := sq_entry_vld && (!sq_entry_in_wmb_ce) && sq_entry_age_vec_less2
  //---------------------pop req------------------------------
  io.out.popReq_x := sq_pop_ptr && sq_entry_vld && sq_entry_cmit && sq_entry_data_vld && sq_entry_no_restart && !sq_entry_in_wmb_ce
  //==========================================================
  //                 Dependency check
  //==========================================================

  // No.    ld pipe         sq/wmb          addr  bytes_vld data_vld      manner
  // --------------------------------------------------------------------------
  // 1      ld              st              :4    part      x             discard
  // 2      ld atomic       any             x     x         x             discard
  // 3      ld              atomic          :4    do        x             discard
  // 4      ld bond(addr1)  st only boud 1  :4    do bv1    x             discard
  // 5      ld              st              :4    exact     0 bypass rf   forward bypass
  // 6      ld              st              :4    whole     0 exclude 5   data discard
  // 7      ld              st              :4    whole     1             forward
  // 8      ld(addr1)       st              :4    x         x             !acclr_en
  val sqCompareLdDc = Module(new CompareIid)
  sqCompareLdDc.io.iid0 := io.in.ldDcIn.iid
  sqCompareLdDc.io.iid1 := sq_entry_iid
  val sq_entry_iid_older_than_ld_dc = sqCompareLdDc.io.older
  val sq_entry_older_than_ld_dc = sq_entry_iid_older_than_ld_dc || sq_entry_cmit
  //-----------addr compare---------------
  // addr0 compare, sq addr compare with ld dc bypass
  val sq_entry_from_ld_dc_addr0 = io.in.ldDcIn.addr0
  // TODO 12 means?
  val sq_entry_depd_addr_tto12_hit = sq_entry_addr0(PA_WIDTH-1,12) === sq_entry_from_ld_dc_addr0(PA_WIDTH-1,12)
  val sq_entry_depd_addr0_11to4_hit = sq_entry_addr0(11,4)===sq_entry_from_ld_dc_addr0(11,4)
  val sq_entry_depd_addr1_11to4_hit = sq_entry_addr0(11,4)===io.in.ldDcIn.addr1_11to4(7,0)
  val sq_entry_depd_addr_tto4_hit = sq_entry_depd_addr_tto12_hit && sq_entry_depd_addr0_11to4_hit
  val sq_entry_depd_addr1_tto4_hit = sq_entry_depd_addr_tto12_hit && sq_entry_depd_addr1_11to4_hit

  //-----------bytes_vld compare---------
  val sq_entry_and_ld_dc_bytes_vld_hit      = (sq_entry_bytes_vld & io.in.ldDcIn.bytesVld).orR
  val sq_entry_not_and_ld_dc_bytes_vld_hit  = (io.in.ldDcIn.bytesVld & (~sq_entry_bytes_vld)).orR
  val sq_entry_and_ld_dc_bytes_vld1_hit     = (sq_entry_bytes_vld & io.in.ldDcIn.bytesVld1).orR
  //example:
  //depd_bytes_vld          ld_dc_bytes_vld     depd kinds
  //1111                    0011                do & whole
  //0011                    0011                do & whole
  //0110                    0011                do & part
  //0110                    1111                do & part
  //1100                    0011                /
  val sq_entry_depd_do_hit     = sq_entry_and_ld_dc_bytes_vld_hit
  val sq_entry_depd_whole_hit  = sq_entry_and_ld_dc_bytes_vld_hit && (!sq_entry_not_and_ld_dc_bytes_vld_hit)
  val sq_entry_depd_part_hit   = sq_entry_and_ld_dc_bytes_vld_hit && sq_entry_not_and_ld_dc_bytes_vld_hit
  val sq_entry_depd_exact_hit  = sq_entry_bytes_vld === io.in.ldDcIn.bytesVld
  val sq_entry_depd_bv1_do_hit = sq_entry_and_ld_dc_bytes_vld1_hit
  //-------------data vld----------------
  val sq_entry_data_vld_now = sq_entry_data_vld || sq_entry_data_set
  //------------------situation 1-----------------------------
  // load inst A depd on Sq inst B, and B older than A, and do & part
  val sq_entry_depd_hit1 = sq_entry_vld && sq_entry_wo_st_inst && io.in.ldDcIn.chkLdInstVld &&
    sq_entry_older_than_ld_dc && sq_entry_depd_addr_tto4_hit && sq_entry_depd_part_hit
  //------------------situation 2-----------------------------
  // load inst A is atmoic, and B older than A
  val sq_entry_depd_hit2 = sq_entry_vld && io.in.ldDcIn.chkAtomicInstVld && sq_entry_older_than_ld_dc
  //------------------situation 3-----------------------------
  // load inst A depd on Sq inst B, and B older than A, but B is atomic
  val sq_entry_depd_hit3 = sq_entry_vld && sq_entry_older_than_ld_dc && io.in.ldDcIn.chkLdInstVld &&
    sq_entry_depd_addr_tto4_hit && sq_entry_depd_do_hit
  //------------------situation 4-----------------------------
  // load inst A depd on Sq inst B, and B older than A, A and B are boundary
  val sq_entry_depd_hit4 = sq_entry_vld && sq_entry_wo_st_inst && io.in.ldDcIn.chkLdAddr1Vld &&
    sq_entry_older_than_ld_dc && sq_entry_bond_first_only && sq_entry_depd_addr1_tto4_hit && sq_entry_depd_bv1_do_hit
  //------------------situation 5-----------------------------
  // load inst A depd on Sq inst B, and B older than A, total hit, B can be forward to A
  val sq_entry_depd_hit5 = sq_entry_vld && sq_entry_wo_st_inst && io.in.sdEx1In.rfInstVldShort &&
    (!sq_entry_boundary) && io.in.ldDcIn.chkLdBypassVld && sq_entry_older_than_ld_dc &&
    sq_entry_depd_addr_tto4_hit && sq_entry_depd_exact_hit && (!sq_entry_data_vld) && sq_entry_sdid_hit
  //------------------situation 6-----------------------------
  // load inst A depd on Sq inst B, and B older than A, B data is not prepared, cant forward, need re-issue then wake up
  val sq_entry_depd_hit6 = sq_entry_vld && sq_entry_wo_st_inst && io.in.ldDcIn.chkLdInstVld &&
    sq_entry_older_than_ld_dc && sq_entry_depd_addr_tto4_hit && (!sq_entry_data_vld_now) && sq_entry_depd_whole_hit
  //------------------situation 7-----------------------------
  // load inst A depd on Sq inst B, and B older than A, need forward, from B to A
  val sq_entry_depd_hit7 = sq_entry_vld && sq_entry_wo_st_inst && io.in.ldDcIn.chkLdInstVld &&
    sq_entry_older_than_ld_dc && sq_entry_depd_addr_tto4_hit && (sq_entry_data_vld_now) && sq_entry_depd_whole_hit
  val sq_entry_newest_fwd_req_data_vld = sq_entry_vld && sq_entry_same_addr_newest && sq_entry_st_inst &&
    io.in.ldDcIn.chkLdInstVld && sq_entry_older_than_ld_dc && sq_entry_depd_addr_tto4_hit && sq_entry_data_vld && sq_entry_depd_whole_hit
  val sq_entry_newest_fwd_req_data_vld_short = sq_entry_vld && sq_entry_same_addr_newest && sq_entry_st_inst &&
    io.in.ldDcIn.chkLdInstVld && sq_entry_older_than_ld_dc  && sq_entry_depd_addr0_11to4_hit && sq_entry_data_vld
  //------------------situation 8-----------------------------
  // load inst A depd on Sq inst B, and B older than A, A is boundary, B is not, thus A should get data from Sq, bot dcache buffer
  val sq_entry_depd_hit8 = sq_entry_vld && (sq_entry_wo_st_inst || sq_entry_atomic) && sq_entry_older_than_ld_dc &&
    sq_entry_depd_addr1_tto4_hit && sq_entry_depd_bv1_do_hit
  //------------------cancel ahead wb-------------------------
  val sq_entry_cancel_ahead_wb = sq_entry_vld && (sq_entry_wo_st_inst || sq_entry_atomic) && sq_entry_older_than_ld_dc &&
    sq_entry_depd_addr_tto4_hit && sq_entry_depd_do_hit
  //------------------combine---------------------------------
  // situation 1-4 will drop A, need wake up
  val sq_entry_discard_req = sq_entry_depd_hit1  || sq_entry_depd_hit2 || sq_entry_depd_hit3 || sq_entry_depd_hit4
  val sq_entry_addr1_dep_discard = sq_entry_depd_hit4
  val sq_entry_fwd_bypass_req = sq_entry_depd_hit5
  val sq_entry_data_discard_req_short = sq_entry_depd_hit6
  val sq_entry_data_discard_req = sq_entry_depd_hit6 && !sq_entry_depd_hit5
  val sq_entry_fwd_req = sq_entry_depd_hit7
  val sq_entry_cancel_acc_req = sq_entry_depd_hit8
  //-------------------set depd signal------------------------
  sq_entry_depd_set := sq_entry_discard_req || sq_entry_fwd_multi_depd_set
  val sq_entry_data_depd_wakeup_vld = sq_entry_data_set || sq_entry_data_set_ff
  val sq_entry_data_depd_wakeup =  Cat(Seq. fill(LSIQ_ENTRY)(sq_entry_data_depd_wakeup_vld)) & sq_entry_wakeup_queue

  //==========================================================
  //                 Generate pop signal
  //==========================================================
  sq_entry_flush_pop_vld := io.in.rtuIn.flush && !sq_entry_cmit
  sq_entry_expt_pop_vld := sq_entry_st_da_info_set && io.in.daIn.stDaIn.wb.exptVld
  sq_entry_pop_vld := sq_entry_vld && sq_entry_in_wmb_ce && io.in.wmbSqPopGrnt
  //==========================================================
  //                 Generate interface
  //==========================================================
  //-----------------------output------------------------------
  io.out.vld_x := sq_entry_vld
  io.out.instHit_x := sq_entry_inst_hit
  io.out.syncFence_x := sq_entry_sync_fence
  io.out.atomic_x := sq_entry_atomic
  io.out.icc_x := sq_entry_icc
  io.out.instFlush_x := sq_entry_inst_flush
  io.out.instType_v := sq_entry_inst_type
  io.out.instSize_v := sq_entry_inst_size
  io.out.instMode_v := sq_entry_inst_mode
  io.out.fenceMode_v := sq_entry_fence_mode
  io.out.iid_v := sq_entry_iid
  io.out.pageShare_x := sq_entry_page_share
  io.out.pageSo_x := sq_entry_page_so
  io.out.pageCa_x := sq_entry_page_ca
  io.out.pageWa_x := sq_entry_page_wa
  io.out.pageBuf_x := sq_entry_page_buf
  io.out.pageSec_x := sq_entry_page_sec
  io.out.sameAddrNewest_x := sq_entry_same_addr_newest
  io.out.woSt_x := sq_entry_wo_st
  io.out.addr0_v := sq_entry_addr0
  io.out.bytesVld_v := sq_entry_bytes_vld
  io.out.specFail_x := sq_entry_spec_fail
  io.out.bkptaData_x := sq_entry_bkpta_data
  io.out.bkptbData_x := sq_entry_bkptb_data
  io.out.vstartVld_x := sq_entry_vstart_vld
  io.out.cmitData_vld_x := sq_entry_cmit_data_vld
  io.out.privMode_v := sq_entry_priv_mode
  io.out.data_v := sq_entry_data
  io.out.rotSel_v := sq_entry_rot_sel
  io.out.dcacheValid_x := sq_entry_update_dcache_mesi.valid
  io.out.dcacheShare_x := sq_entry_update_dcache_mesi.share
  io.out.dcacheDirty_x := sq_entry_update_dcache_mesi.dirty
  io.out.dcacheWay_x := sq_entry_dcache_way
  io.out.depd_x := sq_entry_depd
  io.out.dcacheInfo_vld_x := sq_entry_dcache_info_vld
  //-----------request--------------------
  io.out.dataDepdWakeup_v := sq_entry_data_depd_wakeup
  io.out.discardReq_x := sq_entry_discard_req
  io.out.cancelAheadWb_x := sq_entry_cancel_ahead_wb
  io.out.depdSet_x := sq_entry_depd_set
  io.out.cmit_x := sq_entry_cmit
  io.out.newestFwdReqDataVld_x := sq_entry_newest_fwd_req_data_vld
  io.out.newestFwdReqDataVldShort_x := sq_entry_newest_fwd_req_data_vld_short
  //-----------others---------------------
  io.out.stDcCreateAgeVec_x := sq_entry_age_vec
  io.out.settleDataHit_x := sq_entry_settle_data_hit
  io.out.stDcSameAddrNewer_x := sq_entry_st_dc_same_addr_newer
  io.out.addr1DepDiscard_x := sq_entry_addr1_dep_discard
  io.out.fwdBypassReq_x := sq_entry_fwd_bypass_req
  io.out.dataDiscardReq_x := sq_entry_data_discard_req
  io.out.dataDiscardReqShort_x := sq_entry_data_discard_req_short
  io.out.fwdReq_x := sq_entry_fwd_req
  io.out.cancelAccReq_x := sq_entry_cancel_acc_req
}
