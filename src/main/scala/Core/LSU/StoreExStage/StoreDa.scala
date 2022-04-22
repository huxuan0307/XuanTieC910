package Core.LSU.StoreExStage

import Core.ExceptionConfig.ExceptionVecWidth
import Core.LSU.Sq.{DcacheDirtyDataEn, DcacheToSqEntry, LsuDcacheInfoUpdate}
import Core.ROBConfig.{IidWidth, NumCommitEntry}
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToStDa extends Bundle with LsuConfig{
  val lsuDcacheEn   = Bool()
  val lsuIcgEn      = Bool()
  val lsuL2StPrefEn = Bool()
  val lsuNsfe       = Bool()
  val yyClkEn       = Bool()
}

class LdDaToStDa extends Bundle with LsuConfig{
  val hitIdx = Bool()
}
class RbToStDa extends Bundle with LsuConfig{
  val lsuHasFence = Bool()
  val full        = Bool()
  val hitIdx      = Bool()
}
class RtuToStDa extends Bundle with LsuConfig{
  val commitIid = Vec(NumCommitEntry, UInt(IidWidth.W))
  val commit    = Vec(NumCommitEntry, Bool())
  val flush     = Bool()
}

//----------------------------------------------------------
class StoreDaIn extends Bundle with LsuConfig{
  val cp0In    = new Cp0ToStDa
  val dcacheIn = new DcacheToSqEntry
  val dcIn     = new StDcToDa
  val ldDaIn   = new LdDaToStDa
  val rbIn     = new RbToStDa
  val rtuIn    = new RtuToStDa
  val lfbhitIdx       = Bool()            // from lfb
  val lmHitIdx        = Bool()            // from lm
  val mmuAccessFault1 = Bool()            // from mmu
  val padYyIcgScanEn  = Bool()            // from pad
  val pfuBiuReqAddr   = UInt(PA_WIDTH.W)  // from pfuBiu
  val amrWaCancel   = Bool()
}
//==========================================================
//                        Output
//==========================================================
class StDaToCtrl extends Bundle with LsuConfig {
  val  eccWakeup         = UInt(LSIQ_ENTRY.W)
  val  alreadyDa         = UInt(LSIQ_ENTRY.W)
  val  bkptaData         = UInt(LSIQ_ENTRY.W)
  val  bkptbData         = UInt(LSIQ_ENTRY.W)
  val  boundaryGateclkEn = UInt(LSIQ_ENTRY.W)
  val  popEntry          = UInt(LSIQ_ENTRY.W)
  val  popVld            = Bool()
  val  rbFull            = UInt(LSIQ_ENTRY.W)
  val  secd              = UInt(LSIQ_ENTRY.W)
  val  specFail          = UInt(LSIQ_ENTRY.W)
  val  waitFence         = UInt(LSIQ_ENTRY.W)
  val  rbFullGateclkEn   = Bool()
  val  borrowVld         = Bool()
}

class StDaToWb extends Bundle with LsuConfig{
  val cmpltReq          = Bool()
  val exptVec           = UInt(ExceptionVecWidth.W)
  val exptVld           = Bool()
  val mtValue           = UInt(PA_WIDTH.W)
  val noSpecHit         = Bool()
  val noSpecMispred     = Bool()
  val noSpecMiss        = Bool()
  val specFail          = Bool()
}
class StDaToSq extends Bundle with LsuConfig {
  val secd                = Bool()
  val sfAddrTto4          = UInt(32.W)
  val sfBytesVld          = UInt(BYTES_ACCESS_WIDTH.W)
  val sfIid               = UInt(IidWidth.W)
  val sfNoSpecMiss        = Bool()
  val sfNoSpecMissGate    = Bool()
  val sfSpecChk           = Bool()
  val sfSpecChkGate       = Bool()
  val snqBorrowSnq        = UInt(SNOOP_ID_WIDTH.W)
  val snqDcacheDirty      = Bool()
  val snqDcacheShare      = Bool()
  val snqDcacheValid      = Bool()
  val snqDcacheWay        = Bool()
  val snqEccErr           = Bool()
  val sqDcacheMesi       = new DcacheDirtyDataEn
  val sqDcacheWay         = Bool()
  val sqEccStall          = Bool()
  val sqNoRestart         = Bool()
  val syncFence           = Bool()
  val syncInst            = Bool()
  val vbEccErr            = Bool()
  val vbEccStall          = Bool()
  val vbFeedbackDddrTto14 = UInt(26.W)
  val vbTagReissue        = Bool()
  val waitFenceGateclkEn  = Bool()
  val wb = new StDaToWb
  val wbVstartVld         = Bool()
}
class StDaToRb extends Bundle with LsuConfig {
   val dcacheHit       = Bool() //to rb && vb
   val instSize        = UInt(INST_SIZE_WIDTH.W)
   val fenceInst       = Bool()
   val fenceMode       = UInt(FENCE_MODE_WIDTH.W)
   val old             = Bool()
   val pageBuf         = Bool()
   val pageCa          = Bool()
   val pageSec         = Bool()
   val pageSo          = Bool()
   val cmit            = Bool()
   val createDpVld     = Bool()
   val createGateclkEn = Bool()
   val createLfb       = Bool()
   val createVld       = Bool()
   val pageShare       = Bool()

}
class StDaToPfu extends Bundle with LsuConfig {
  val pageSecFf       = Bool()
  val pageShareFf     = Bool()
  val pc              = UInt(LSU_PC_WIDTH.W)
  val pfuActDpVld     = Bool()
  val pfuActVld       = Bool()
  val pfuBiuReqHitIdx = Bool()
  val pfuEvictCntVld  = Bool()
  val pfuPfInstVld    = Bool()
  val ppfuVa          = UInt(PA_WIDTH.W)
  val ppnFf           = UInt(VPN_WIDTH.W)
}
class StDaToVb extends Bundle with LsuConfig {
  val dirty         = Bool()
  val miss          = Bool()
  val replaceDirty  = Bool()
  val replaceValid  = Bool()
  val replaceWay    = Bool()
  val way           = Bool()
}
class StDaToIcc extends Bundle  with DCacheConfig {
  val borrowIccVld = Bool()
  val dirtyInfo = UInt(3.W)
  val tagInfo   = UInt(((TAG_WIDTH+1)*WAYS).W)
}
class StDaToRtu extends Bundle with LsuConfig {
  val splitSpecFailIid = UInt(IidWidth.W)
  val splitSpecFailVld = Bool()
}
//----------------------------------------------------------
class StoreDaOut extends Bundle with LsuConfig with DCacheConfig {
  val toCtrl    = new StDaToCtrl
  val toSq      = new StDaToSq
  val toRb      = new StDaToRb
  val toPfu     = new StDaToPfu
  val toVb      = new StDaToVb
  val toIcc     = new StDaToIcc
  val toRtu     = new StDaToRtu
  // these io is multi-used
  val addr      = UInt(PA_WIDTH.W)    // rb/lfb/lm/ldda
  val bkptaData = Bool()              // sq/wb
  val bkptbData = Bool()              // sq/wb
  val instVld   = Bool()              // sq/ctrl/wb/
  val iid       = UInt(IidWidth.W) // sq/rb/wb/pfu
}
//==========================================================
//                          IO
//==========================================================
class StoreDaIO extends Bundle with LsuConfig {
  val in  = Input(new StoreDaIn)
  val out = Output(new StoreDaOut)
}
class StoreDa extends Module with LsuConfig with DCacheConfig {
  val io = IO(new StoreDaIO)
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val st_da_clk_en = io.in.dcIn.instVld  || io.in.dcIn.toSqDa.borrowVld
  val st_da_inst_clk_en = io.in.dcIn.instVld
  val st_da_borrow_clk_en = io.in.dcIn.toSqDa.borrowVld
  val st_da_expt_clk_en  = io.in.dcIn.instVld && io.in.dcIn.exptVldGateEn
  //------------------dcache reg------------------------------
  val st_da_tag_ecc_stall           = false.B
  val st_da_tag_ecc_stall_gate      = false.B
  val st_da_ecc_stall               = false.B
  val st_da_vb_tag_reissue          = false.B
  val st_da_vb_ecc_stall            = false.B
  val st_da_vb_ecc_err              = false.B
  val st_da_snq_ecc_err             = false.B
  val st_da_ecc_stall_already       = false.B
  val st_da_ecc_stall_fatal         = false.B
  val st_da_ecc_stall_dcache_update = false.B
  val st_da_tag_dirty_clk_en = io.in.dcIn.getDcacheTagDirty || st_da_tag_ecc_stall_gate
  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+------------+
  //| inst_vld | borrow_vld |
  //+----------+------------+
  val st_da_inst_vld = RegInit(false.B)
  when(io.in.rtuIn.flush){
    st_da_inst_vld := false.B
  }.elsewhen(io.in.dcIn.instVld){
    st_da_inst_vld := true.B
  }.otherwise{
    st_da_inst_vld := false.B
  }
  val st_da_borrow_vld = RegInit(false.B)
  when(io.in.dcIn.toSqDa.borrowVld){
    st_da_borrow_vld := true.B
  }.otherwise{
    st_da_borrow_vld := false.B
  }
  io.out.instVld := st_da_inst_vld
  io.out.toCtrl.borrowVld := st_da_borrow_vld
  //------------------cache output part-----------------------
  //+-----+-------+
  //| tag | dirty |
  //+-----+-------+
  val st_da_dcache_tag_array   = RegInit(0.U(((TAG_WIDTH+1)*WAYS).W))
  val st_da_dcache_dirty_array = RegInit(0.U((OFFSET_WIDTH+1).W))
  val st_da_tag_hit            = Seq.fill(2)(RegInit(false.B))
  when(io.in.dcIn.getDcacheTagDirty){
    st_da_dcache_tag_array     := io.in.dcIn.dcacheTagArray
    st_da_dcache_dirty_array   := io.in.dcIn.dcacheDirtyArray
    st_da_tag_hit(0)           := io.in.dcIn.tagHit(0)
    st_da_tag_hit(1)           := io.in.dcIn.tagHit(1)
  }
  //------------------expt part-------------------------------
  //+----------+-----+-----------+
  //| expt_vec | vpn | dmmu_expt |
  //+----------+-----+-----------+
  val st_da_expt_vec = RegInit(0.U(ExceptionVecWidth.W))
  val st_da_mt_value = RegInit(0.U(PA_WIDTH.W))
  when(io.in.dcIn.instVld && io.in.dcIn.exptVldGateEn){
    st_da_expt_vec := io.in.dcIn.exptVec
    st_da_mt_value := io.in.dcIn.mtValue
  }
  //------------------borrow part-----------------------------
  //+-----+-----+
  //| rcl | snq |
  val st_da_borrow_dcache_replace = RegInit(false.B)
  val st_da_borrow_dcache_sw      = RegInit(false.B)
  val st_da_borrow_snq            = RegInit(false.B)
  val st_da_borrow_icc            = RegInit(false.B)
  val st_da_borrow_snq_id         = RegInit(0.U(SNOOP_ID_WIDTH.W))
  when(!st_da_ecc_stall && io.in.dcIn.toSqDa.borrowVld){
    st_da_borrow_dcache_replace := io.in.dcIn.borrowDcacheReplace
    st_da_borrow_dcache_sw      := io.in.dcIn.borrowDcacheSw
    st_da_borrow_snq            := io.in.dcIn.borrowSnq
    st_da_borrow_icc            := io.in.dcIn.borrowIcc
    st_da_borrow_snq_id         := io.in.dcIn.borrowSnqId
  }
  //------------------inst part----------------------------
  //+----------+-----+----+-----------+-----------+-----------+
  //| sync_fence | icc | ex | inst_type | inst_size | inst_mode |
  //+----------+-----+----+-----------+-----------+-----------+
  //+------+------------+-----------+-------+
  //| secd | already_da | spec_fail | split |
  //+------+------------+-----------+-------+
  //+----+-----+------+-----+
  //| ex | iid | lsid | old |
  //+----+-----+------+-----+
  //+----------+------+-------+-------+
  //| boundary | preg | bkpta | bkptb |
  //+----------+------+-------+-------+
  //+------------+------------+
  //| ldfifo_vld | ldfifo_idx |
  //+------------+------------+
  //+----+----+----+-----+-----+-------+
  //| so | ca | wa | buf | sec | share |
  //+----+----+----+-----+-----+-------+
  val st_da_mmu_req                    = RegInit(false.B)
  val st_da_expt_vld_except_access_err = RegInit(false.B)
  val st_da_expt_access_fault_mask     = RegInit(false.B)
  val st_da_expt_access_fault_extra    = RegInit(false.B)
  val st_da_expt_access_fault_mmu      = RegInit(false.B)
  val st_da_split                      = RegInit(false.B)
  val st_da_sync_fence                 = RegInit(false.B)
  val st_da_icc                        = RegInit(false.B)
  val st_da_inst_type                  = RegInit((0.U(INST_TYPE_WIDTH.W)))
  val st_da_inst_mode                  = RegInit((0.U(INST_MODE_WIDTH.W)))
  val st_da_inst_size                  = RegInit((0.U(INST_SIZE_WIDTH.W)))
  val st_da_fence_mode                 = RegInit((0.U(FENCE_MODE_WIDTH.W)))
  val st_da_st                         = RegInit(false.B)
  val st_da_secd                       = RegInit(false.B)
  val st_da_atomic                     = RegInit(false.B)
  val st_da_iid                        = RegInit((0.U(IidWidth.W)))
  val st_da_lsid                       = RegInit((0.U(LSIQ_ENTRY.W)))
  val st_da_old                        = RegInit(false.B)
  val st_da_boundary                   = RegInit(false.B)
  val st_da_spec_fail                  = RegInit(false.B)
  val st_da_bkpta_data                 = RegInit(false.B)
  val st_da_bkptb_data                 = RegInit(false.B)
  val st_da_page_so                    = RegInit(false.B)
  val st_da_page_ca                    = RegInit(false.B)
  val st_da_page_wa                    = RegInit(false.B)
  val st_da_page_buf                   = RegInit(false.B)
  val st_da_page_sec                   = RegInit(false.B)
  val st_da_page_share                 = RegInit(false.B)
  val st_da_already_da                 = RegInit(false.B)
  val st_da_no_spec                    = RegInit(false.B)
  val st_da_bytes_vld                  = RegInit(0.U(BYTES_ACCESS_WIDTH.W))
  val st_da_pc                         = RegInit(0.U(LSU_PC_WIDTH.W))
  val st_da_pfu_va                     = RegInit(0.U(PA_WIDTH.W))
  val st_da_pf_inst                    = RegInit(false.B)
  val st_da_vector_nop                 = RegInit(false.B)
  when(io.in.dcIn.instVld && !st_da_ecc_stall){
    st_da_mmu_req                    := io.in.dcIn.mmuReq
    st_da_expt_vld_except_access_err := io.in.dcIn.exptVldExceptAccessErr
    st_da_expt_access_fault_mask     := io.in.dcIn.exptAccessFaultMask
    st_da_expt_access_fault_extra    := io.in.dcIn.exptAccessFaultExtra
    st_da_expt_access_fault_mmu      := io.in.mmuAccessFault1
    st_da_split                      := io.in.dcIn.split
    st_da_sync_fence                 := io.in.dcIn.toSqDa.syncFence
    st_da_icc                        := io.in.dcIn.toSqDa.icc
    st_da_inst_type                  := io.in.dcIn.toSqDa.instType
    st_da_inst_mode                  := io.in.dcIn.toSqDa.instMode
    st_da_inst_size                  := io.in.dcIn.toSqDa.instSize
    st_da_fence_mode                 := io.in.dcIn.toSqDa.fenceMode
    st_da_st                         := io.in.dcIn.st
    st_da_secd                       := io.in.dcIn.toSqDa.secd
    st_da_atomic                     := io.in.dcIn.toSqDa.atomic
    st_da_iid                        := io.in.dcIn.toPwdDa.iid
    st_da_lsid                       := io.in.dcIn.lsid
    st_da_old                        := io.in.dcIn.toSqDa.old
    st_da_boundary                   := io.in.dcIn.toSqDa.boundary
    st_da_spec_fail                  := io.in.dcIn.specFail
    st_da_bkpta_data                 := io.in.dcIn.bkptaData
    st_da_bkptb_data                 := io.in.dcIn.bkptbData
    st_da_page_so                    := io.in.dcIn.toSqDa.pageSo
    st_da_page_ca                    := io.in.dcIn.toSqDa.pageCa
    st_da_page_wa                    := io.in.dcIn.toSqDa.pageWa
    st_da_page_buf                   := io.in.dcIn.toSqDa.pageBuf
    st_da_page_sec                   := io.in.dcIn.toSqDa.pageSec
    st_da_page_share                 := io.in.dcIn.toSqDa.pageShare
    st_da_already_da                 := io.in.dcIn.alreadyDa
    st_da_no_spec                    := io.in.dcIn.noSpec
    st_da_bytes_vld                  := io.in.dcIn.toPwdDa.bytesVld
    st_da_pc                         := io.in.dcIn.pc
    st_da_pfu_va                     := io.in.dcIn.pfuVa
    st_da_pf_inst                    := io.in.dcIn.pfInst
    st_da_vector_nop                 := io.in.dcIn.vectorNop
  }
  //------------------inst/borrow share part------------------
  //+-------+
  //| addr0 |
  //+-------+
  //+--------------+----------------+----------------+
  //| dcwp_hit_idx | dcwp_dirty_din | dcwp_dirty_wen |
  //+--------------+----------------+----------------+
  val st_da_addr0             = RegInit(0.U(PA_WIDTH.W))
  val st_da_dcwp_dc_hit_idx   = RegInit(false.B)
  val st_da_dcwp_dc_dirty_din = Seq.fill(WAYS)(RegInit(0.U.asTypeOf(new DcacheDirtyDataEn)))
  val st_da_dcwp_dc_dirty_wen = Seq.fill(WAYS)(RegInit(0.U.asTypeOf(new DcacheDirtyDataEn)))
  when(io.in.dcIn.instVld && io.in.dcIn.toSqDa.borrowVld){
    st_da_addr0             := io.in.dcIn.toPwdDa.addr0
    st_da_dcwp_dc_hit_idx   := io.in.dcIn.dcwpHitIdx
    for(i<- 0 until WAYS){
      st_da_dcwp_dc_dirty_din(i) := io.in.dcacheIn.dirtyDin.bits(i)
      st_da_dcwp_dc_dirty_wen(i) := io.in.dcacheIn.dirtyWen.bits(i)
    }

  }
  //==========================================================
  //        Generate expt info
  //==========================================================
  val st_da_expt_access_fault = (st_da_mmu_req && st_da_expt_access_fault_mmu || st_da_expt_access_fault_extra) && !st_da_expt_access_fault_mask
  val st_da_expt_vld          = (st_da_expt_vld_except_access_err || st_da_expt_access_fault || st_da_ecc_stall_fatal) && !(st_da_vector_nop)
  val st_da_wb_expt_vld       = (st_da_expt_vld_except_access_err || st_da_expt_access_fault) && !(st_da_vector_nop)
  when(st_da_expt_access_fault &&  !st_da_st){
    io.out.toSq.wb.exptVec := 5.U(ExceptionVecWidth.W)
    io.out.toSq.wb.exptVec := 0.U(PA_WIDTH.W)
  }.elsewhen(st_da_expt_access_fault &&  st_da_st){
    io.out.toSq.wb.exptVec := 7.U(ExceptionVecWidth.W)
    io.out.toSq.wb.exptVec := 0.U(PA_WIDTH.W)
  }.otherwise{
    io.out.toSq.wb.exptVec := st_da_expt_vec
    io.out.toSq.wb.exptVec := st_da_mt_value
  }
  val st_da_wb_vstart_vld = false.B
  //==========================================================
  //        Generate inst type
  //==========================================================
  //st/str/push/srs is treated as st inst
  val st_da_sync_inst    = st_da_sync_fence  &&  !st_da_atomic  &&  (st_da_inst_type(1,0) ===  "b00".U)
  val st_da_fence_inst   = st_da_sync_fence  &&  !st_da_atomic  &&  (st_da_inst_type(1,0) ===  "b01".U)
  val st_da_ctc_inst     = st_da_icc  &&  !st_da_atomic  &&  (st_da_inst_type(1,0) =/=  "b10".U)
  val st_da_dcache_inst  = st_da_icc  &&  !st_da_atomic  &&  (st_da_inst_type(1,0) ===  "b10".U)
  val st_da_l2cache_inst = st_da_icc  &&  !st_da_atomic  &&  (st_da_inst_type(1,0) ===  "b11".U)
  val st_da_st_inst      = !st_da_icc  &&  !st_da_atomic  &&  !st_da_sync_fence  &&  (st_da_inst_type(1,0) ===  "b00".U)
  val st_da_dcache_sw_inst     = st_da_dcache_inst  &&  (st_da_inst_mode(1,0) ===  "b10".U)
  val st_da_dcache_pa_inst     = st_da_dcache_inst  &&  (st_da_inst_mode(1,0) ===  "b11".U)
  val st_da_dcache_va_inst     = st_da_dcache_inst  &&  (st_da_inst_mode(1,0) ===  "b01".U)
  val st_da_dcache_1line_inst  = st_da_dcache_sw_inst  ||  st_da_dcache_pa_inst    ||  st_da_dcache_va_inst
  //==========================================================
  //              Compare tag and select data
  //==========================================================
  //------------------compare tag-----------------------------
  //------------------compare tag-----------------------------
  val st_da_dcache_sw_sel  = st_da_inst_vld  &&  st_da_dcache_sw_inst || st_da_borrow_vld && st_da_borrow_dcache_sw
  //if dcache sw inst, then hit_way is static as addr[31]
  val st_da_dcache_sw_way1  = st_da_addr0(31)
  val st_da_dcache_info_vld = st_da_inst_vld && st_da_page_ca || st_da_borrow_vld
  val st_da_dcache_valid = Seq.fill(2)(Wire(Bool()))
  val st_da_hit_way = Seq.fill(2)(Wire(Bool()))
  for(i <- 0 until 2){
    st_da_dcache_valid(i) := st_da_dcache_dirty_array(i*3) && io.in.cp0In.lsuDcacheEn && st_da_dcache_info_vld
    st_da_hit_way(i)      := st_da_dcache_valid(i) && (!(st_da_dcache_sw_sel) && st_da_tag_hit(i) || st_da_dcache_sw_sel && !(st_da_dcache_sw_way1))
  }
  io.out.toRb.dcacheHit := st_da_hit_way.reduce(_ || _)
  io.out.toVb.miss      := !io.out.toRb.dcacheHit
  //select cache hit info
  val st_da_dcache_dirty_hit_info =  Mux(st_da_hit_way.reduce(_ || _),Mux(st_da_hit_way(0),st_da_dcache_dirty_array(2,0),st_da_dcache_dirty_array(5,3)),0.U(3.W))
  //------output dcache info for inst/snq/vb rcl(inst/icc)-------
  io.out.toVb.dirty := st_da_dcache_dirty_hit_info(2)
  val st_da_dcache_way  = Mux(st_da_dcache_sw_sel, st_da_dcache_sw_way1, st_da_hit_way(1))
  io.out.toVb.way   := st_da_dcache_way
  //---------output dcache info for vb rcl line replace-------
  io.out.toVb.replaceWay   := Mux(io.out.toRb.dcacheHit, st_da_hit_way(1),st_da_dcache_dirty_array(6))
  io.out.toVb.replaceDirty := Mux(io.out.toVb.replaceWay, st_da_dcache_dirty_array(5), st_da_dcache_dirty_array(2))
  io.out.toVb.replaceValid := Mux(io.out.toVb.replaceWay, st_da_dcache_dirty_array(3), st_da_dcache_dirty_array(0))
  //------------------feedback addr to vb---------------------
  val st_da_feedback_sel_tag_way1  = io.out.toVb.replaceWay && st_da_borrow_dcache_replace || st_da_dcache_sw_way1  &&  st_da_borrow_dcache_sw
  val st_da_feedback_sel_tag       = st_da_borrow_dcache_replace  ||  st_da_borrow_dcache_sw
  val feedback_selecet = Cat(st_da_feedback_sel_tag,st_da_feedback_sel_tag_way1)

  when(feedback_selecet === "b10".U){
    io.out.toSq.vbFeedbackDddrTto14 := st_da_dcache_tag_array(25,0)
  }.elsewhen(feedback_selecet === "b11".U){
    io.out.toSq.vbFeedbackDddrTto14 := st_da_dcache_tag_array(((TAG_WIDTH+1)*WAYS-1),26)
  }.otherwise{
    io.out.toSq.vbFeedbackDddrTto14 := st_da_addr0(PA_WIDTH-1,14)
  }
  //---------------feedback dirty info to snq-----------------
  val st_da_snq_dcache_dirty_hit_info = Mux(st_da_tag_hit.reduce(_ || _),Mux(st_da_tag_hit(0),st_da_dcache_dirty_array(2,0),st_da_dcache_dirty_array(5,3)),0.U(3.W))
  io.out.toSq.snqDcacheValid := st_da_snq_dcache_dirty_hit_info(0) && io.in.cp0In.lsuDcacheEn
  io.out.toSq.snqDcacheShare  := st_da_snq_dcache_dirty_hit_info(1)
  io.out.toSq.snqDcacheDirty := st_da_snq_dcache_dirty_hit_info(2)
  io.out.toSq.snqDcacheWay   := st_da_dcache_dirty_array(3) && st_da_tag_hit(1)
  //==========================================================
  //          Dirty array update da stage for sq
  //==========================================================
  //when inst is in dc stage, then only dcache dirty array may be changed
  //when inst is in da stage, then tag & dirty array may be changed
  //-------update dirty info if index hit in dc stage---------

  val dcache_dirty_array = Seq.fill(WAYS)(RegInit(0.U.asTypeOf(new DcacheDirtyDataEn)))
  dcache_dirty_array(0).valid := st_da_dcache_dirty_array(0).asBool
  dcache_dirty_array(0).dirty := st_da_dcache_dirty_array(2).asBool
  dcache_dirty_array(0).share := st_da_dcache_dirty_array(1).asBool
  dcache_dirty_array(1).valid := st_da_dcache_dirty_array(3).asBool
  dcache_dirty_array(1).dirty := st_da_dcache_dirty_array(5).asBool
  dcache_dirty_array(1).share := st_da_dcache_dirty_array(4).asBool
  val st_da_dirty_dc_update      = io.in.dcacheIn.dirtyWen.bits
  val st_da_dirty_dc_update_dout = Seq.fill(WAYS)(RegInit(0.U.asTypeOf(new DcacheDirtyDataEn)))
  for(i<- 0 until WAYS){
    st_da_dirty_dc_update_dout(i).valid := (st_da_dirty_dc_update(i).valid && st_da_dcwp_dc_dirty_din(i).valid ) || (dcache_dirty_array(i).valid  && (!st_da_dirty_dc_update(i).valid ))
    st_da_dirty_dc_update_dout(i).dirty := (st_da_dirty_dc_update(i).dirty && st_da_dcwp_dc_dirty_din(i).dirty ) || (dcache_dirty_array(i).dirty  && (!st_da_dirty_dc_update(i).dirty ))
    st_da_dirty_dc_update_dout(i).share := (st_da_dirty_dc_update(i).share && st_da_dcwp_dc_dirty_din(i).share ) || (dcache_dirty_array(i).share  && (!st_da_dirty_dc_update(i).share ))
  }
   //select cache hit info
  val st_da_dcache_dirty_dc_up_hit_info = Mux(st_da_hit_way(0), st_da_dirty_dc_update_dout(0),st_da_dirty_dc_update_dout(1))
  val st_da_dcache_dc_up = WireInit(0.U.asTypeOf((new DcacheDirtyDataEn)))

  st_da_dcache_dc_up.dirty         := st_da_dcache_dirty_dc_up_hit_info.dirty && st_da_dcwp_dc_hit_idx
  st_da_dcache_dc_up.share         := st_da_dcache_dirty_dc_up_hit_info.share && st_da_dcwp_dc_hit_idx
  st_da_dcache_dc_up.valid         := st_da_dcache_dirty_dc_up_hit_info.valid && st_da_dcwp_dc_hit_idx
  val st_da_dcache_dc_up_way           = st_da_dcache_way
  //-------------update dcache info in da stage---------------
  val da_dcache_info_update = Module(new LsuDcacheInfoUpdate)
  da_dcache_info_update.io.in.compareDcwpAddr := st_da_addr0
  da_dcache_info_update.io.in.compareDcwpSwInst := st_da_dcache_sw_inst
  da_dcache_info_update.io.in.dcacheIn := io.in.dcacheIn
  da_dcache_info_update.io.in.originDcacheMesi := st_da_dcache_dc_up
  da_dcache_info_update.io.in.originDcacheWay:=st_da_dcache_dc_up_way
  io.out.toSq.sqDcacheMesi := da_dcache_info_update.io.out.updateDcacheMesi
  io.out.toSq.sqDcacheWay  := da_dcache_info_update.io.out.updateDcacheWay

  //==========================================================
  //        Generage commit signal
  //==========================================================
  val st_da_cmit_hit =Seq.fill(NumCommitEntry)(Wire(Bool()))
  for(i<- 0 until( NumCommitEntry)){
    st_da_cmit_hit(i) := Cat(io.in.rtuIn.commit(i),io.in.rtuIn.commitIid(i))  === Cat(true.B, st_da_iid(i))
  }
  io.out.toRb.cmit := st_da_cmit_hit.reduce(_ || _)
  //==========================================================
  //        Request read buffer & Compare index & discard
  //==========================================================
  //----------in mem copy mode, then it won't request rb------
  val st_da_rb_page_wa = st_da_page_wa && !io.in.amrWaCancel
  //------------------origin create read buffer---------------
  //-----------create 1-------------------
  //st/push/srs: cache miss, cacheable
  val st_da_page_ca_dcache_en = st_da_page_ca && io.in.cp0In.lsuDcacheEn
  val st_da_rb_create_vld_unmask  = st_da_inst_vld  &&
    !st_da_vector_nop &&
    !st_da_expt_vld &&
    !st_da_ecc_stall_already &&
    (st_da_st_inst && st_da_page_ca_dcache_en && st_da_rb_page_wa && io.out.toVb.miss ||
      st_da_sync_fence    &&  !st_da_atomic    &&  !io.in.rbIn.lsuHasFence)
  //------------------index hit/discard grnt signal-----------
  //addr is used to compare index, so addr0 is enough
  io.out.addr := st_da_addr0
  //------------------create read buffer info-----------------
  io.out.toSq.syncFence := st_da_sync_fence
  io.out.toRb.createVld := st_da_rb_create_vld_unmask  &&
    !st_da_ecc_stall  &&
    (!io.in.ldDaIn.hitIdx && !io.in.rbIn.hitIdx && !io.in.lfbhitIdx  && !io.in.lmHitIdx ||
    st_da_sync_fence && !st_da_atomic)
  io.out.toRb.createDpVld     := st_da_rb_create_vld_unmask
  io.out.toRb.createGateclkEn := st_da_rb_create_vld_unmask
  //-----------rb create signal-----------
  io.out.toRb.createLfb := st_da_st_inst
  //==========================================================
  //        Compare index
  //==========================================================
  //------------------compare pfu-----------------------------
  val st_da_cmp_pfu_biu_req_addr = io.in.pfuBiuReqAddr
  io.out.toPfu.pfuBiuReqHitIdx := st_da_rb_create_vld_unmask && (st_da_addr0(13,6) === st_da_cmp_pfu_biu_req_addr(13,6))
  //==========================================================
  //        Restart signal
  //==========================================================
  val st_da_rb_full_vld  = st_da_rb_create_vld_unmask &&  !st_da_ecc_stall  && io.in.rbIn.full
  io.out.toCtrl.rbFullGateclkEn := io.out.toRb.createGateclkEn &&  io.in.rbIn.full
  val st_da_wait_fence_vld       = st_da_inst_vld  &&  (st_da_fence_inst ||  st_da_sync_inst)  &&  io.in.rbIn.lsuHasFence
  io.out.toSq.waitFenceGateclkEn := st_da_wait_fence_vld
  val st_da_restart_vld = st_da_rb_full_vld || st_da_wait_fence_vld
  //==========================================================
  //        Generage to SQ signal
  //==========================================================
  io.out.toSq.sqEccStall  := st_da_ecc_stall || st_da_ecc_stall_dcache_update
  io.out.toSq.sqNoRestart := st_da_inst_vld  &&  !st_da_restart_vld
  //==========================================================
  //        Generage interface to prefetch buffer
  //==========================================================
  val st_da_split_miss_ff = RegInit(false.B)
  io.out.toPfu.pfuPfInstVld := st_da_inst_vld && st_da_pf_inst &&  !st_da_already_da  &&  !st_da_expt_vld
  val st_da_boundary_cross_2k = st_da_pfu_va(11) =/= st_da_addr0(11)
  io.out.toPfu.pfuActVld   := st_da_inst_vld && st_da_pf_inst && !st_da_expt_vld &&
    (io.out.toRb.createVld || st_da_split_miss_ff) && st_da_rb_page_wa && io.out.toVb.miss && !st_da_boundary_cross_2k
  io.out.toPfu.pfuActDpVld := st_da_inst_vld &&  st_da_pf_inst  && !st_da_expt_vld  && st_da_rb_page_wa  && io.out.toVb.miss    &&  !st_da_boundary_cross_2k
  //for evict count
  io.out.toPfu.pfuEvictCntVld := io.out.toPfu.pfuPfInstVld
  //st prefetch does not support gpfb
  io.out.toPfu.ppfuVa := st_da_pfu_va
  val st_da_ppn_ff = RegInit(0.U(PPN_WIDTH.W))
  val st_da_page_sec_ff = RegInit(false.B)
  val st_da_page_share_ff = RegInit(false.B)
  when(st_da_inst_vld && st_da_st_inst){
    st_da_ppn_ff        := st_da_addr0
    st_da_page_sec_ff   := st_da_page_sec
    st_da_page_share_ff := st_da_page_share

  }

  val st_da_split_miss = st_da_inst_vld  && st_da_st_inst && st_da_page_ca &&
    io.in.cp0In.lsuDcacheEn    && st_da_split &&
    !st_da_secd && !st_da_expt_vld  && io.out.toRb.createVld
  val st_da_split_last = st_da_inst_vld && st_da_st_inst && !st_da_split
  when(st_da_split_miss){
    st_da_split_miss_ff := true.B
  }.elsewhen(st_da_split_last){
    st_da_split_miss_ff := false.B
  }
  //==========================================================
  //        Generage to WB stage signal
  //==========================================================
  //------------------write back cmplt part request-----------
  val st_da_boundary_first = Wire(Bool())
  io.out.toSq.wb.cmpltReq :=  st_da_inst_vld  &&  !st_da_restart_vld  &&  !st_da_ecc_stall  &&
    !st_da_ecc_stall_dcache_update  &&  !st_da_boundary_first  &&
    (st_da_wb_expt_vld    ||  st_da_vector_nop    ||  (st_da_ctc_inst    ||  st_da_dcache_1line_inst    ||  st_da_l2cache_inst
    ||  st_da_st_inst &&  !st_da_page_so))
  //------------------other signal---------------------------
  io.out.toSq.wb.specFail := st_da_spec_fail  &&  !st_da_split
  //==========================================================
  //        Generate interface to borrow module
  //==========================================================
  val st_da_borrow_snq_vld = st_da_borrow_vld  &&  st_da_borrow_snq && !st_da_ecc_stall
  io.out.toSq.snqBorrowSnq := Mux(st_da_borrow_snq_vld,st_da_borrow_snq_id,0.U(LSIQ_ENTRY.W))
  io.out.toIcc.borrowIccVld := st_da_borrow_vld && st_da_borrow_icc
  io.out.toIcc.dirtyInfo := Mux(st_da_dcache_sw_sel,st_da_dcache_dirty_array(5,3),st_da_dcache_dirty_array(2,0))
  io.out.toIcc.tagInfo := Mux(st_da_dcache_sw_sel,st_da_dcache_tag_array(51,26),st_da_dcache_tag_array(25,0))
  //==========================================================
  //        Generate lsiq signal
  //==========================================================
  val st_da_mask_lsid = Mux(st_da_inst_vld, st_da_lsid, 0.U(LSIQ_ENTRY.W))
  st_da_boundary_first := st_da_boundary  &&  !st_da_expt_vld  &&  !st_da_secd
  //-----------lsiq signal----------------
  // &Force("output","st_da_ecc_wakeup"); @1045
  //for avoid dc vector nop from wakeup sdiq multiple times.use already_da as
  //symbol signal, here set already_da ahead for dc replay inst by da ecc stall
  //
  //note already_da is only used for performance in da stage,hence not accurate
  //here is fine
  io.out.toCtrl.alreadyDa := st_da_mask_lsid | 0.U(LSIQ_ENTRY.W)
  io.out.toCtrl.rbFull    := Mux(st_da_rb_full_vld,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  io.out.toCtrl.waitFence := Mux(st_da_wait_fence_vld,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  io.out.toCtrl.popVld    := st_da_inst_vld  &&  !st_da_boundary_first  &&  !st_da_tag_ecc_stall  &&
    !st_da_ecc_stall_dcache_update  &&  !st_da_restart_vld;
  io.out.toCtrl.popEntry  := Mux(io.out.toCtrl.popVld,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  io.out.toCtrl.specFail  := Mux(st_da_spec_fail  && st_da_boundary_first ,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  io.out.toCtrl.bkptaData := Mux(st_da_bkpta_data && st_da_boundary_first ,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  io.out.toCtrl.bkptbData := Mux(st_da_bkptb_data && st_da_boundary_first ,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  //---------boundary gateclk-------------
  val st_da_idu_boundary_gateclk_vld       = st_da_inst_vld  &&  st_da_boundary_first
  io.out.toCtrl.boundaryGateclkEn := Mux(st_da_idu_boundary_gateclk_vld,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  //-----------imme wakeup----------------
  val st_da_idu_secd_vld = st_da_inst_vld && st_da_boundary_first && !st_da_tag_ecc_stall && !st_da_ecc_stall_dcache_update && !st_da_restart_vld
  io.out.toCtrl.secd := Mux(st_da_idu_secd_vld,st_da_mask_lsid,0.U(LSIQ_ENTRY.W))
  //==========================================================
  //        interface for spec_fail prediction
  //==========================================================
  val st_da_no_spec_miss = st_da_inst_vld  && io.in.cp0In.lsuNsfe && !st_da_atomic  && st_da_spec_fail
  io.out.toSq.sfNoSpecMiss     := st_da_no_spec_miss && !st_da_restart_vld
  io.out.toSq.sfNoSpecMissGate := st_da_no_spec_miss
  io.out.toSq.sfIid            := st_da_iid
  io.out.toSq.sfAddrTto4 := st_da_addr0
  io.out.toSq.sfBytesVld := st_da_bytes_vld
  val st_da_no_spec_hit = st_da_inst_vld  && io.in.cp0In.lsuNsfe && st_da_no_spec && !st_da_spec_fail
  val st_da_no_spec_mispred = st_da_inst_vld  && io.in.cp0In.lsuNsfe  && st_da_no_spec && st_da_spec_fail
  io.out.toSq.wb.noSpecMiss    := st_da_no_spec_miss && !st_da_no_spec
  io.out.toSq.wb.noSpecHit     := st_da_no_spec_hit
  io.out.toSq.wb.noSpecMispred := st_da_no_spec_mispred
  //for spec mispred check
  io.out.toSq.sfSpecChk     :=  st_da_inst_vld && io.in.cp0In.lsuNsfe  && !st_da_restart_vld  && !st_da_spec_fail  && st_da_no_spec
  io.out.toSq.sfSpecChkGate := st_da_inst_vld && io.in.cp0In.lsuNsfe && st_da_no_spec
  //==========================================================
  //        Generate interface to rtu
  io.out.toRtu.splitSpecFailIid := st_da_inst_vld && !st_da_expt_vld && st_da_split && st_da_spec_fail
  io.out.toRtu.splitSpecFailVld := st_da_iid
  //==========================================================
  //        pipe regs out
  //==========================================================

  io.out.toPfu.pc                  := st_da_pc
  io.out.iid                       := st_da_iid

  io.out.toSq.vbTagReissue         := st_da_vb_tag_reissue
  io.out.toSq.vbEccStall           := st_da_vb_ecc_stall
  io.out.toSq.vbEccErr             := st_da_vb_ecc_err

  io.out.toSq.wbVstartVld          := st_da_wb_vstart_vld
  io.out.toSq.wb.exptVld           := st_da_wb_expt_vld
  io.out.toSq.wb.mtValue           := st_da_mt_value
  io.out.toSq.snqEccErr            := st_da_snq_ecc_err
  io.out.toSq.secd                 := st_da_secd
  io.out.toSq.syncInst             := st_da_sync_inst




  io.out.bkptaData                 := st_da_bkpta_data
  io.out.bkptbData                 := st_da_bkptb_data

  io.out.toPfu.ppnFf               := st_da_ppn_ff
  io.out.toPfu.pageSecFf           := st_da_page_sec_ff
  io.out.toPfu.pageShareFf         := st_da_page_share_ff


  io.out.toCtrl.eccWakeup          := 0.U(LSIQ_ENTRY)

  io.out.toRb.instSize             := st_da_inst_size
  io.out.toRb.fenceMode            := st_da_fence_mode
  io.out.toRb.fenceInst            := st_da_fence_inst


  io.out.toRb.pageCa               := st_da_page_ca
  io.out.toRb.pageSo               := st_da_page_so
  io.out.toRb.pageSec              := st_da_page_sec
  io.out.toRb.pageShare            := st_da_page_share
  io.out.toRb.pageBuf              := st_da_page_buf

  io.out.toRb.old                  := st_da_old



}











