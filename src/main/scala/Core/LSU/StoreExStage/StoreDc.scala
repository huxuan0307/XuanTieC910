package Core.LSU.StoreExStage

import Core.DCacheConfig.INDEX_WIDTH
import Core.ExceptionConfig.ExceptionVecWidth
import Core.ROBConfig.{IidWidth, NumCommitEntry}
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToStDc extends Bundle with LsuConfig{
  val dcacheEn      = Bool()
  val icgEn         = Bool()
  val l2StPrefEn    = Bool()
  val clkEn         = Bool()
}
class DcacheToStDc extends Bundle with LsuConfig{
  val arbBorrowIcc      = Bool()
  val arbBorrowSnq      = Bool()
  val arbBorrowSnqId    = UInt(SNOOP_ID_WIDTH.W)
  val arbBorrowVld      = Bool()
  val arbBorrowVldGate  = Bool()
  val arbBcacheReplace  = Bool()
  val arbBcacheSw       = Bool()
  val dirtyGwen         = Bool()
  val idx               = UInt(INDEX_WIDTH.W)
  val lsuStDirtyDout    = UInt(7.W)   // TODO
  val lsuStTagDout      = UInt(52.W)  // TODO
}
class LqToStDc extends Bundle with LsuConfig{
  val specFail = Bool()
}
class MmuToStDc extends Bundle with LsuConfig{
  val mmuEn   = Bool()
  val tlbBusy = Bool()
}
class RtuToStDc extends Bundle with LsuConfig{
  val commitIidUpdata = Vec(NumCommitEntry, UInt(IidWidth.W))
  val flush = Bool()
}
class SdEx1ToStDc extends Bundle with LsuConfig{
  val sdid = UInt(SDID_WIDTH.W)
}
class SqToStDc extends Bundle with LsuConfig{
  val full    = Bool()
  val instHit = Bool()
}
//----------------------------------------------------------
class StoreDcIn extends Bundle with LsuConfig{
  val cp0In    = new Cp0ToStDc
  val dcacheIn = new DcacheToStDc
  val lqIn     = new LqToStDc
  val mmuIn    = new MmuToStDc
  val rtuIn    = new RtuToStDc
  val sdEx1In  = new SdEx1ToStDc
  val sqIn     = new SqToStDc
  val agIn     = new StAgToDc
}
//==========================================================
//                        Output
//==========================================================
class StDcToIdu extends Bundle with LsuConfig {
  val sdiqEntry     = UInt(LSIQ_ENTRY.W)
  val staddr1Vld    = Bool()
  val staddrUnalign = Bool()
  val staddrVld     = Bool()
}
class StDcToMmu extends Bundle with LsuConfig {
  val vabuf1 = UInt(VPN_WIDTH.W)
}
class StDcToSq extends Bundle with LsuConfig{
  val cmitIidCrtHit        = Vec(NumCommitEntry,UInt(IidWidth.W))
  val boundaryFirst        = Bool()
  val instFlush            = Bool()
  val rotSelRev            = UInt(ROT_SEL_WIDTH.W)
  val sdid                 = UInt(SDID_WIDTH.W)
  val sdidHit              = Bool()
  val sqCreateDpVld        = Bool()
  val sqCreateGateclkEn    = Bool()
  val sqCreateVld          = Bool()
  val sqDataVld            = Bool()
  val sqFullGateclkEn      = Bool()
  val woStInst             = Bool()
  val iid                  = UInt(IidWidth.W)
  val addr0                = UInt(PA_WIDTH.W)
  val bytesVld             = UInt(BYTES_ACCESS_WIDTH.W)
}
class StDcToLq extends Bundle with LsuConfig{
  val addr0                = UInt(PA_WIDTH.W)
  val bytesVld             = UInt(BYTES_ACCESS_WIDTH.W)
  val chkStInstVld         = Bool()
  val chkStatomicInstVld   = Bool()
  val iid                  = UInt(IidWidth.W)
  val instVld              = Bool()
}
class StDcToSqDa extends Bundle with LsuConfig{
  val borrowVld     = Bool()
  val atomic        = Bool()
  val boundary      = Bool()
  val pageBuf       = Bool()
  val pageCa        = Bool()
  val pageSec       = Bool()
  val pageShare     = Bool()
  val pageSo        = Bool()
  val pageWa        = Bool()
  val fenceMode     = UInt(FENCE_MODE_WIDTH.W)
  val icc           = Bool()
  val old           = Bool()
  val secd          = Bool()
  val syncFence     = Bool()
  val instMode      = UInt(INST_MODE_WIDTH.W)
  val instSize      = UInt(INST_SIZE_WIDTH.W)
  val instType      = UInt(INST_TYPE_WIDTH.W)
}
class StDcToDa extends Bundle with LsuConfig with DCacheConfig {
  val toSqDa                       = new StDcToSqDa
  val toPwdDa                      = new StDcToLq
  val dcacheDirtyArray             = UInt((OFFSET_WIDTH+1).W)
  val dcacheTagArray               = UInt(((TAG_WIDTH+1)*WAYS).W)
  val exptVldGateEn                = Bool()
  val instVld                      = Bool()
  val tagHit                       = Vec(2, Bool())
  val dcwpHitIdx                   = Bool()
  val exptAccessFaultExtra         = Bool()
  val exptAccessFaultMask          = Bool()
  val exptVec                      = UInt(ExceptionVecWidth.W)
  val exptVldExceptAccessErr       = Bool()
  val getDcacheTagDirty            = Bool()
  val alreadyDa                    = Bool()
  val bkptaData                    = Bool()
  val bkptbData                    = Bool()
  val borrowDcacheReplace          = Bool()
  val borrowDcacheSw               = Bool()
  val borrowIcc                    = Bool()
  val borrowSnq                    = Bool()
  val borrowSnqId                  = UInt(SNOOP_ID_WIDTH.W)
  val iduSqFull                    = UInt(LSIQ_ENTRY.W)
  val iduTlbBusy                   = UInt(LSIQ_ENTRY.W)
  val lsid                         = UInt(LSIQ_ENTRY.W)
  val mmuReq                       = Bool()
  val mtValue                      = UInt(PA_WIDTH.W)
  val noSpec                       = Bool()
  val pc                           = UInt(LSU_PC_WIDTH.W)
  val pfInst                       = Bool()
  val pfuVa                        = UInt(PA_WIDTH.W)
  val specFail                     = Bool()
  val split                        = Bool()
  val st                           = Bool()
  val vectorNop                    = Bool()
}
class StDcToCtrl extends Bundle with LsuConfig with DCacheConfig {
 val tlbBusyGateclkEn   = Bool()
 val immeWakeup         = UInt(LSIQ_ENTRY.W)
}
//----------------------------------------------------------
class StoreDcOut extends Bundle with LsuConfig{
  val toIdu  = new StDcToIdu
  val toMmu  = new StDcToMmu
  val toSq   = new StDcToSq
  val toDa   = new StDcToDa
  val toCtrl = new StDcToCtrl
}
//==========================================================
//                          IO
//==========================================================
class StoreDcIO extends Bundle with LsuConfig{
  val in  = Input(new StoreDcIn)
  val out = Output(new StoreDcOut)
}
class StoreDc extends Module with LsuConfig with DCacheConfig{
  val io = IO(new StoreDcIO)
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  val st_dc_clk_en = io.in.agIn.instVld  ||  io.in.dcacheIn.arbBorrowVldGate
  val st_dc_inst_clk_en = io.in.agIn.instVld // TODO havent add gate cell
  //-----------------------borrow clk-------------------------
  val st_dc_borrow_clk_en =  io.in.dcacheIn.arbBorrowVldGate

  //-----------------------expt clk---------------------------
  //for saving register,misalign and illegal have been selected in ag stage
  val st_dc_expt_illegal_inst_clk_en = io.in.agIn.instVld && (io.in.agIn.exptIllegalInst || io.in.agIn.exptMisalignNoPage)
  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+------------+
  //| inst_vld | borrow_vld |
  //+----------+------------+
  //inst vld is used for sq_entry pop sel signal
  val st_dc_inst_vld = RegInit(false.B)
  when(io.in.rtuIn.flush){
    st_dc_inst_vld := false.B
  }.elsewhen(io.in.agIn.instVld){
    st_dc_inst_vld := true.B
  }.otherwise{
    st_dc_inst_vld := false.B
  }
  val st_dc_borrow_vld = RegInit(false.B)
  when(io.in.dcacheIn.arbBorrowVld){
    st_dc_borrow_vld := false.B
  }.otherwise{
    st_dc_borrow_vld := false.B
  }
  //------------------expt part-------------------------------
  //+--------------+-------+----------+--------+------+------+---------+-----+
  //| illegal_inst | tmiss | misalign | tfatal | tmod | deny | rd_tinv | vpn |
  //+--------------+-------+----------+--------+------+------+---------+-----+
  val st_dc_mmu_req                     = RegInit(false.B)
  val st_dc_expt_illegal_inst           = RegInit(false.B)
  val st_dc_expt_misalign_no_page       = RegInit(false.B)
  val st_dc_expt_page_fault             = RegInit(false.B)
  val st_dc_expt_misalign_with_page     = RegInit(false.B)
  val st_dc_expt_access_fault_with_page = RegInit(false.B)
  val st_dc_expt_stamo_not_ca           = RegInit(false.B)
  when(io.in.agIn.instVld){
    st_dc_mmu_req                     := io.in.agIn.dcMmuReq
    st_dc_expt_illegal_inst           := io.in.agIn.exptIllegalInst
    st_dc_expt_misalign_no_page       := io.in.agIn.exptMisalignNoPage
    st_dc_expt_page_fault             := io.in.agIn.exptPageFault
    st_dc_expt_misalign_with_page     := io.in.agIn.exptMisalignWithPage
    st_dc_expt_access_fault_with_page := io.in.agIn.exptAccessFaultWithPage
    st_dc_expt_stamo_not_ca           := io.in.agIn.exptStamoNotCa
  }
  val st_dc_mt_value_ori = RegInit(0.U(PA_WIDTH.W))
  when(io.in.agIn.instVld && ( io.in.agIn.exptIllegalInst||io.in.agIn.exptMisalignNoPage )){
    st_dc_mt_value_ori := io.in.agIn.mtValue
  }
  //------------------borrow part-----------------------------
  //+-----+------+-----+
  //| rcl | sndb | mmu |
  //+-----+------+-----+
  val st_dc_borrow_dcache_replace = RegInit(false.B)
  val st_dc_borrow_dcache_sw      = RegInit(false.B)
  val st_dc_borrow_snq            = RegInit(false.B)
  val st_dc_borrow_icc            = RegInit(false.B)
  val st_dc_borrow_snq_id         = RegInit(0.U(SNOOP_ID_WIDTH.W))
  when(io.in.dcacheIn.arbBorrowVld){
    st_dc_borrow_dcache_replace := io.in.dcacheIn.arbBcacheReplace
    st_dc_borrow_dcache_sw      := io.in.dcacheIn.arbBcacheSw
    st_dc_borrow_snq            := io.in.dcacheIn.arbBorrowSnq
    st_dc_borrow_icc            := io.in.dcacheIn.arbBorrowIcc
    st_dc_borrow_snq_id         := io.in.dcacheIn.arbBorrowSnqId
  }
  //------------------inst part----------------------------
  //+----------+-----+----+-----------+-----------+-----------+
  //| sync_fence | icc | ex | inst_type | inst_size | inst_mode |
  //+----------+-----+----+-----------+-----------+-----------+
  //+------+------------+----------+-------+-------+------------+
  //| secd | already_da | spec_fail| bkpta | bkptb | inst_flush |
  //+------+------------+----------+-------+-------+------------+
  //+----+-----+------+-----+------------+------------+
  //| ex | iid | lsid | old | bytes_vld0 | bytes_vld1 |
  //+----+-----+------+-----+------------+------------+
  //+--------+----------+-------+
  //| deform | boundary | split |
  //+--------+----------+-------+
  //+----+----+----+-----+-----+-------+
  //| so | ca | wa | buf | sec | share |
  //+----+----+----+-----+-----+-------+
  val st_dc_expt_vld_except_access_err = RegInit(false.B)
  val st_dc_vpn                        = RegInit(0.U(PA_WIDTH.W))
  val st_dc_split                      = RegInit(false.B)
  val st_dc_sync_fence                 = RegInit(false.B)
  val st_dc_fence_mode                 = RegInit(0.U(FENCE_MODE_WIDTH.W))
  val st_dc_icc                        = RegInit(false.B)
  val st_dc_inst_flush                 = RegInit(false.B)
  val st_dc_inst_type                  = RegInit(0.U(INST_TYPE_WIDTH.W))
  val st_dc_inst_size                  = RegInit(0.U(INST_SIZE_WIDTH.W))
  val st_dc_inst_mode                  = RegInit(0.U(INST_MODE_WIDTH.W))
  val st_dc_st                         = RegInit(false.B)
  val st_dc_secd                       = RegInit(false.B)
  val st_dc_already_da                 = RegInit(false.B)
  val st_dc_lsiq_spec_fail             = RegInit(false.B)
  val st_dc_lsiq_bkpta_data            = RegInit(false.B)
  val st_dc_lsiq_bkptb_data            = RegInit(false.B)
  val st_dc_atomic                     = RegInit(false.B)
  val st_dc_iid                        = RegInit(0.U(IidWidth.W))
  val st_dc_lsid                       = RegInit(0.U(LSIQ_ENTRY.W))
  val st_dc_sdid_oh                    = RegInit(0.U(LSIQ_ENTRY.W))
  val st_dc_old                        = RegInit(false.B)
  val st_dc_bytes_vld                  = RegInit(0.U(BYTES_ACCESS_WIDTH.W))
  val st_dc_rot_sel                    = RegInit(0.U(ROT_SEL_WIDTH.W))
  val st_dc_boundary                   = RegInit(false.B)
  val st_dc_page_so                    = RegInit(false.B)
  val st_dc_page_ca                    = RegInit(false.B)
  val st_dc_page_wa                    = RegInit(false.B)
  val st_dc_page_buf                   = RegInit(false.B)
  val st_dc_page_sec                   = RegInit(false.B)
  val st_dc_page_share                 = RegInit(false.B)
  val st_dc_utlb_miss                  = RegInit(false.B)
  val st_dc_tlb_busy                   = RegInit(false.B)
  val st_dc_no_spec                    = RegInit(false.B)
  val st_dc_staddr                     = RegInit(false.B)
  val st_dc_pc                         = RegInit(0.U(LSU_PC_WIDTH.W))
  val st_dc_lsfifo                     = RegInit(false.B)
  when(io.in.agIn.instVld){
    st_dc_expt_vld_except_access_err := io.in.agIn.exptVld
    st_dc_vpn                        := io.in.agIn.vpn
    st_dc_split                      := io.in.agIn.split
    st_dc_sync_fence                 := io.in.agIn.syncFence
    st_dc_fence_mode                 := io.in.agIn.fenceMode
    st_dc_icc                        := io.in.agIn.icc
    st_dc_inst_flush                 := io.in.agIn.instFlush
    st_dc_inst_type                  := io.in.agIn.instType
    st_dc_inst_size                  := io.in.agIn.dcAccessSize
    st_dc_inst_mode                  := io.in.agIn.instMode
    st_dc_st                         := io.in.agIn.st
    st_dc_secd                       := io.in.agIn.secd
    st_dc_already_da                 := io.in.agIn.alreadyDa
    st_dc_lsiq_spec_fail             := io.in.agIn.lsiqSpecFail
    st_dc_lsiq_bkpta_data            := io.in.agIn.lsiqBkptaData
    st_dc_lsiq_bkptb_data            := io.in.agIn.lsiqBkptbData
    st_dc_atomic                     := io.in.agIn.atomic
    st_dc_iid                        := io.in.agIn.iid
    st_dc_lsid                       := io.in.agIn.lsid
    st_dc_sdid_oh                    := io.in.agIn.sdidOh
    st_dc_old                        := io.in.agIn.old
    st_dc_bytes_vld                  := io.in.agIn.dcBytesVld
    st_dc_rot_sel                    := io.in.agIn.dcRotSel
    st_dc_boundary                   := io.in.agIn.boundary
    st_dc_page_so                    := io.in.agIn.pageSo
    st_dc_page_ca                    := io.in.agIn.pageCa
    st_dc_page_wa                    := io.in.agIn.pageWa
    st_dc_page_buf                   := io.in.agIn.pageBuf
    st_dc_page_sec                   := io.in.agIn.pageSec
    st_dc_page_share                 := io.in.agIn.dcPageShare
    st_dc_utlb_miss                  := io.in.agIn.utlbMiss
    st_dc_tlb_busy                   := io.in.mmuIn.tlbBusy
    st_dc_no_spec                    := io.in.agIn.noSpec
    st_dc_staddr                     := io.in.agIn.staddr
    st_dc_pc                         := io.in.agIn.pc
    st_dc_lsfifo                     := io.in.agIn.lsfifo
  }

  //------------------inst/borrow share part------------------
  //+-------+
  //| addr0 |
  //+-------+
  val st_dc_addr0 = RegInit(0.U(PA_WIDTH.W))
  when(io.in.agIn.instVld || io.in.dcacheIn.arbBorrowVld){
    st_dc_addr0 := io.in.agIn.dcAddr0
  }
  io.out.toDa.toPwdDa.addr0 := st_dc_addr0
  io.out.toSq.addr0         := st_dc_addr0
  //==========================================================
  //        Generate  va
  //==========================================================
  val st_dc_va = Cat(st_dc_vpn(PA_WIDTH-13,0), st_dc_addr0(OFFSET_WIDTH-1,4) ,Mux(st_dc_secd,0.U(4.W),st_dc_addr0(3,0)) )
  val st_dc_pfu_va_11to4  = st_dc_addr0(OFFSET_WIDTH-1,4)
  val st_dc_pfu_va  = Cat(st_dc_vpn(PA_WIDTH-13,0),st_dc_addr0(OFFSET_WIDTH-1,0))
  //Generage pfu signal
  val st_dc_st_inst = Wire(Bool())
  val st_dc_vector_nop = false.B
  val st_dc_pf_inst = st_dc_inst_vld && st_dc_st_inst && !st_dc_vector_nop && st_dc_lsfifo && st_dc_page_ca && io.in.cp0In.l2StPrefEn && !st_dc_split && !st_dc_secd
  //==========================================================
  //        Exception generate
  //==========================================================
  val st_dc_icc_inst = Wire(Bool())
  val st_dc_expt_access_fault_mask = st_dc_expt_misalign_no_page || st_dc_expt_page_fault || st_dc_expt_illegal_inst || st_dc_icc_inst
  val st_dc_expt_access_fault_extra = st_dc_mmu_req  &&  st_dc_expt_stamo_not_ca
  val st_dc_expt_access_fault_short = st_dc_mmu_req
  //if utlb_miss and dmmu expt,
  //then st_dc_expt_vld_except_access_err is 0,
  //but st_dc_da_expt_vld is not certain
  val st_dc_da_expt_vld_gate_en  = st_dc_expt_vld_except_access_err || st_dc_expt_access_fault_short
  when(st_dc_expt_illegal_inst){
    io.out.toDa.exptVec := 2.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := st_dc_mt_value_ori
  }.elsewhen(st_dc_expt_misalign_no_page){
    io.out.toDa.exptVec := 6.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := st_dc_mt_value_ori
  }.elsewhen(st_dc_expt_page_fault &&  !st_dc_st){
    io.out.toDa.exptVec := 13.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := st_dc_va
  }.elsewhen(st_dc_expt_page_fault &&  st_dc_st){
    io.out.toDa.exptVec := 15.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := st_dc_va
  }.elsewhen(st_dc_expt_misalign_with_page){
    io.out.toDa.exptVec := 6.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := st_dc_va
  }.elsewhen(st_dc_expt_access_fault_with_page){
    io.out.toDa.exptVec := 7.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := 0.U(PA_WIDTH.W)
  }.otherwise{
    io.out.toDa.exptVec := 0.U(ExceptionVecWidth.W)
    io.out.toDa.mtValue := 0.U(PA_WIDTH.W)
  }
  //==========================================================
  //                  get commit hit signal
  //==========================================================
  val st_dc_cmit_hit = Seq.fill(NumCommitEntry)(Wire(Bool()))
  for(i <- 0 until  NumCommitEntry){
    st_dc_cmit_hit(i) := io.in.rtuIn.commitIidUpdata(i) === st_dc_iid
  }
  io.out.toSq.cmitIidCrtHit := st_dc_cmit_hit
  //==========================================================
  //                      encode sdid
  //==========================================================
  //encode sdid to create lq signal
  io.out.toSq.sdid := OHToUInt(st_dc_sdid_oh)
  io.out.toSq.sdidHit := io.out.toSq.sdid === io.in.sdEx1In.sdid
  //==========================================================
  //        Generate inst type
  //==========================================================
  val st_dc_sync_inst          = st_dc_sync_fence  &&  !st_dc_atomic  &&  (st_dc_inst_type(1,0)  ===  "b00".U)
  val st_dc_fence_inst         = st_dc_sync_fence  &&  !st_dc_atomic  &&  (st_dc_inst_type(1,0)  ===  "b01".U)
  st_dc_icc_inst              := st_dc_icc  &&  !st_dc_atomic
  val st_dc_tlbi_inst          = st_dc_icc_inst  &&  (st_dc_inst_type(1,0)  ===  "b00".U)
  val st_dc_icache_inst        = st_dc_icc_inst  &&  (st_dc_inst_type(1,0)  ===  "b01".U)
  val st_dc_dcache_inst        = st_dc_icc_inst  &&  (st_dc_inst_type(1,0)  ===  "b10".U)
  val st_dc_l2cache_inst       = st_dc_icc_inst  &&  (st_dc_inst_type(1,0)  ===  "b11".U)
  st_dc_st_inst               := !st_dc_icc  &&  !st_dc_atomic  &&  !st_dc_sync_fence  &&  (st_dc_inst_type(1,0)  ===  "b00".U)
  val st_dc_wo_st_inst         = st_dc_st_inst  &&  !st_dc_page_so
  val st_dc_dcache_all_inst    = st_dc_dcache_inst  &&  (st_dc_inst_mode(1,0)  ===  "b00".U)
  val st_dc_dcache_1line_inst  = st_dc_dcache_inst  &&  (st_dc_inst_mode(1,0)  =/=  "b00".U)
  val st_dc_dcache_pa_sw_inst  = st_dc_dcache_inst  &&  st_dc_inst_mode(1)
  val st_dc_icache_all_inst    = st_dc_icache_inst  &&  (st_dc_inst_mode(1,0)  === "b00".U)
  //==========================================================
  //                 Create load queue
  //==========================================================
  //lq_create_vld is not accurate, comparing iid is a must precedure to create lq//lq_create_vld is not accurate, comparing iid is a must precedure to create lq

  //----------------create signal-----------------------------
  val st_dc_sq_create_vld        = st_dc_inst_vld && !st_dc_vector_nop && !io.in.sqIn.instHit && !st_dc_utlb_miss && !st_dc_expt_vld_except_access_err
  val st_dc_sq_create_dp_vld     = st_dc_sq_create_vld
  val st_dc_sq_create_gateclk_en = st_dc_sq_create_dp_vld
  val st_dc_sq_data_vld          = st_dc_inst_vld && !st_dc_staddr
  //----------------success signal----------------------------
  val st_dc_boundary_first  = st_dc_boundary && !st_dc_secd
  io.out.toSq.boundaryFirst := st_dc_boundary_first
  //==========================================================
  //        Generate check signal to lq/ld_dc stage
  //==========================================================
  io.out.toDa.toPwdDa.chkStInstVld := st_dc_inst_vld && st_dc_st_inst && !st_dc_vector_nop && !st_dc_expt_vld_except_access_err && !st_dc_utlb_miss && !st_dc_page_so
  io.out.toDa.toPwdDa.chkStatomicInstVld := st_dc_inst_vld && st_dc_atomic && !(st_dc_vector_nop) && !(st_dc_expt_vld_except_access_err) && !(st_dc_utlb_miss) && !(st_dc_page_so)
  //------------------data pre_select----------------
  io.out.toSq.rotSelRev := Cat(Reverse(UIntToOH(st_dc_rot_sel)(ROT_SEL_WIDTH-1,1)),st_dc_rot_sel(0))
  //==========================================================
  //        Compare cache write port
  //==========================================================
  io.out.toDa.dcwpHitIdx := io.in.dcacheIn.dirtyGwen && (st_dc_addr0(OFFSET_WIDTH+INDEX_WIDTH-1,OFFSET_WIDTH) === io.in.dcacheIn.idx(INDEX_WIDTH-1,0))
  //==========================================================
  //                 Restart signal
  //==========================================================
  //-----------arbiter----------------------------------------
  //prioritize:
  //1. utlb_miss(include tlb_busy)
  //2. sq_full
  val st_dc_utlb_miss_vld = st_dc_inst_vld && !st_dc_expt_vld_except_access_err && st_dc_utlb_miss
  val st_dc_sq_full_vld = !(st_dc_utlb_miss) && st_dc_sq_create_dp_vld && io.in.sqIn.full
  val st_dc_sq_full_gateclk_en = st_dc_sq_create_gateclk_en && io.in.sqIn.full
  val st_dc_restart_vld = st_dc_sq_full_vld || st_dc_utlb_miss_vld
  //---------------------restart kinds------------------------
  val st_dc_imme_restart_vld = st_dc_utlb_miss_vld && !(st_dc_tlb_busy)
  val st_dc_tlb_busy_restart_vld = st_dc_utlb_miss_vld && st_dc_tlb_busy
  val st_dc_tlb_busy_gateclk_en = st_dc_tlb_busy_restart_vld
  //==========================================================
  //        Combine signal of spec_fail/bkpt
  //==========================================================
  val st_dc_pipe_bkpta_data = (st_dc_st_inst || st_dc_atomic) && !st_dc_vector_nop  // TODO st_dc_bkpta_trap
  val st_dc_pipe_bkptb_data = (st_dc_st_inst || st_dc_atomic) && !st_dc_vector_nop  // TODO st_dc_bkpta_trap
  val st_dc_spec_fail = io.in.lqIn.specFail || st_dc_lsiq_spec_fail
  val st_dc_bkpta_data = st_dc_lsiq_bkpta_data || st_dc_pipe_bkpta_data
  val st_dc_bkptb_data = st_dc_lsiq_bkptb_data || st_dc_pipe_bkptb_data
  //==========================================================
  //            Generage get dcache signal
  //==========================================================
  io.out.toDa.getDcacheTagDirty := st_dc_inst_vld &&
    !st_dc_vector_nop &&
    !st_dc_utlb_miss &&
    (st_dc_st_inst || st_dc_atomic || st_dc_dcache_1line_inst) && (st_dc_page_ca || st_dc_dcache_pa_sw_inst) &&
    io.in.cp0In.dcacheEn &&
    !st_dc_expt_vld_except_access_err || st_dc_borrow_vld
  //==========================================================
  //                 Forward to st_data
  //==========================================================
  io.out.toIdu.staddrVld    := (st_dc_sq_create_dp_vld && !io.in.sqIn.full || st_dc_inst_vld && st_dc_vector_nop  && !st_dc_utlb_miss_vld) && !st_dc_already_da &&  st_dc_staddr
  io.out.toIdu.staddrUnalign := st_dc_boundary_first
  io.out.toIdu.staddr1Vld     := st_dc_secd
  io.out.toIdu.sdiqEntry     := st_dc_sdid_oh
  //==========================================================
  //        Generage to DA stage signal
  //==========================================================
  io.out.toDa.instVld           := st_dc_inst_vld  &&  !st_dc_restart_vld
  val st_dc_default_page = st_dc_sync_inst || st_dc_fence_inst || st_dc_dcache_all_inst || st_dc_icache_all_inst || st_dc_tlbi_inst || st_dc_st_inst && !st_dc_staddr || st_dc_l2cache_inst
  io.out.toDa.toSqDa.pageSo     := Mux(st_dc_default_page,false.B ,st_dc_page_so)
  val st_dc_dcache_pa_sw_page_ca = Mux(st_dc_dcache_pa_sw_inst,true.B ,st_dc_page_ca)
  io.out.toDa.toSqDa.pageCa     := Mux(st_dc_default_page,false.B ,st_dc_page_ca)
  io.out.toDa.toSqDa.pageWa     := Mux(st_dc_default_page,false.B ,st_dc_page_wa)
  io.out.toDa.toSqDa.pageBuf    := Mux(st_dc_default_page,true.B ,st_dc_page_buf)
  io.out.toDa.toSqDa.secd       := Mux(st_dc_default_page,false.B ,st_dc_page_sec)
  io.out.toDa.toSqDa.pageShare  := Mux(st_dc_default_page,true.B ,st_dc_page_share)
  //------------------dcache tag pre_compare----------------
  io.out.toDa.dcacheTagArray := io.in.dcacheIn.lsuStTagDout
  io.out.toDa.dcacheDirtyArray := io.in.dcacheIn.lsuStDirtyDout
  for(i <- 0 until 2){
    io.out.toDa.tagHit(i) := st_dc_addr0(PA_WIDTH-1,OFFSET_WIDTH+INDEX_WIDTH) === io.in.dcacheIn.lsuStTagDout(25+26*i,26*i)
  }
  //==========================================================
  //        Generage lsiq signal
  //==========================================================
  val st_dc_mask_lsid = Mux(st_dc_inst_vld,st_dc_lsid , 0.U(LSIQ_ENTRY))
  io.out.toDa.iduSqFull          := Mux(st_dc_sq_full_vld,         st_dc_mask_lsid , 0.U(LSIQ_ENTRY))
  io.out.toCtrl.immeWakeup       := Mux(st_dc_imme_restart_vld,    st_dc_mask_lsid , 0.U(LSIQ_ENTRY))
  io.out.toCtrl.tlbBusyGateclkEn := Mux(st_dc_tlb_busy_restart_vld,st_dc_mask_lsid , 0.U(LSIQ_ENTRY))
  //==========================================================
  //        for mmu power
  //==========================================================
  io.out.toMmu.vabuf1 := st_dc_vpn
  //==========================================================
  //        pipe regs out
  //==========================================================

  io.out.toSq.iid := st_dc_iid
  io.out.toSq.bytesVld := st_dc_bytes_vld
  io.out.toDa.st := st_dc_st
  io.out.toDa.toSqDa.instType := st_dc_inst_type
  io.out.toDa.specFail :=st_dc_spec_fail
  io.out.toSq.sqDataVld := st_dc_sq_data_vld
  io.out.toDa.noSpec := st_dc_no_spec
  io.out.toDa.pfuVa := st_dc_pfu_va
  io.out.toDa.toSqDa.icc := st_dc_icc
  io.out.toDa.toSqDa.borrowVld := st_dc_borrow_vld
  io.out.toDa.toSqDa.syncFence := st_dc_sync_fence

  io.out.toDa.exptAccessFaultExtra := st_dc_expt_access_fault_extra
  io.out.toDa.exptAccessFaultMask := st_dc_expt_access_fault_mask
  io.out.toDa.exptVldExceptAccessErr := st_dc_expt_vld_except_access_err
  io.out.toDa.exptVldGateEn := st_dc_da_expt_vld_gate_en

  io.out.toDa.borrowDcacheReplace := st_dc_borrow_dcache_replace
  io.out.toDa.borrowDcacheSw := st_dc_borrow_dcache_sw

  io.out.toDa.borrowSnq := st_dc_borrow_snq
  io.out.toDa.borrowSnqId := st_dc_borrow_snq_id

  io.out.toSq.woStInst := st_dc_wo_st_inst

  io.out.toSq.sqCreateDpVld := st_dc_sq_create_dp_vld
  io.out.toSq.sqCreateVld   := st_dc_sq_create_vld
  io.out.toSq.sqCreateGateclkEn := st_dc_sq_create_gateclk_en
  io.out.toSq.sqFullGateclkEn := st_dc_sq_full_gateclk_en

  io.out.toDa.toSqDa.fenceMode:= st_dc_fence_mode
  io.out.toDa.toSqDa.instSize := st_dc_inst_size
  io.out.toDa.toSqDa.instType := st_dc_inst_type
  io.out.toDa.toSqDa.instMode := st_dc_inst_mode
  io.out.toDa.toSqDa.boundary := st_dc_boundary
  io.out.toSq.instFlush := st_dc_inst_flush

  io.out.toDa.toPwdDa.iid := st_dc_iid
  io.out.toDa.toPwdDa.instVld := st_dc_inst_vld
  io.out.toDa.toPwdDa.bytesVld := st_dc_bytes_vld


  io.out.toDa.borrowIcc := st_dc_borrow_icc
  io.out.toDa.toSqDa.pageSec := st_dc_page_sec
  io.out.toDa.bkptbData := st_dc_bkptb_data
  io.out.toDa.bkptaData := st_dc_bkpta_data
  io.out.toDa.alreadyDa := st_dc_already_da
  io.out.toDa.lsid := st_dc_lsid

  io.out.toDa.vectorNop := st_dc_vector_nop
  io.out.toDa.pc  := st_dc_pc
  io.out.toDa.toSqDa.old := st_dc_old
  io.out.toDa.toSqDa.atomic := st_dc_atomic

  io.out.toDa.pfInst := st_dc_pf_inst
  io.out.toDa.mmuReq := st_dc_mmu_req
  io.out.toDa.split  := st_dc_split
  io.out.toDa.iduTlbBusy := st_dc_tlb_busy
}
