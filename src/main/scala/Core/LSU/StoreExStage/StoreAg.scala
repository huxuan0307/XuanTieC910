package Core.LSU.StoreExStage

import Core.IUConfig.{MPPWidth, XLEN}
import Core.LsuAccessSize._
import Core.ROBConfig.{IidWidth, NumCommitEntry}
import Core.RTU.CompareIid
import Core.{DCacheConfig, LsuConfig, ROBConfig}
import Utils.{LookupTree, sext}
import chisel3._
import chisel3.util._
//==========================================================
//                        Input
//==========================================================
class Cp0ToStAg extends Bundle with LsuConfig{
  val dcacheEn    = Bool()
  val icgEn       = Bool()
  val mm          = Bool()
  val tvm         = Bool()
  val ucme        = Bool()
  val wa          = Bool()
  val clkEn       = Bool()
  val privMode    = UInt(MPPWidth.W)
  val virtualMode = Bool()
}
class DcacheArbToStAg extends Bundle with LsuConfig{ // for acceleration
  val sel = Bool()
  val addr = UInt(PA_WIDTH.W)
  val borrowAddrVld = Bool()
}
class RfPipe4 extends Bundle with LsuConfig with DCacheConfig {
  val alreadyDa      = Bool()
  val atomic         = Bool()
  val bkptaData      = Bool()
  val bkptbData      = Bool()
  val fenceMode      = UInt(FENCE_MODE_WIDTH.W)
  val gateclkSel     = Bool()
  val icc            = Bool()
  val iid            = UInt(IidWidth.W)
  val instCode       = UInt(INST_CODE_WIDTH.W)
  val instFls        = Bool()
  val instFlush      = Bool()
  val instMode       = UInt(INST_MODE_WIDTH.W)
  val instShare      = Bool()
  val instSize       = UInt(INST_SIZE_WIDTH.W)
  val instStr        = Bool()
  val instType       = UInt(INST_TYPE_WIDTH.W)
  val lchEntry       = UInt(LSIQ_ENTRY.W)
  val lsfifo         = Bool()
  val mmuReq         = Bool()
  val noSpec         = Bool()
  val off0Extend     = Bool()
  val offset         = UInt(OFFSET_WIDTH.W)
  val offsetPlus     = UInt((OFFSET_WIDTH+1).W)
  val oldest         = Bool()
  val pc             = Bool()
  val sdiqEntry      = UInt(LSIQ_ENTRY.W)
  val sel            = Bool()
  val shift          = UInt(SHITF_WIDTH.W)
  val specFail       = Bool()
  val split          = Bool()
  val src0           = UInt(XLEN.W)
  val src1           = UInt(XLEN.W)
  val st             = Bool()
  val staddr         = Bool()
  val syncFence      = Bool()
  val unalign2nd     = Bool()
}
class LmToStAg extends Bundle with LsuConfig{
  val pa        = UInt(ADDR_PA_WIDTH.W)
  val pageBuf   = Bool()
  val pageCa    = Bool()
  val pageSec   = Bool()
  val pageShare = Bool()
  val pageSo    = Bool()
}
class MmuToStAg extends Bundle with LsuConfig{
  val buf1        = Bool()
  val ca1         = Bool()
  val pa1         = UInt(ADDR_PA_WIDTH.W)
  val pa1Vld     = Bool()
  val pageFault1 = Bool()
  val sec1        = Bool()
  val sh1         = Bool()
  val so1         = Bool()
  val stall1      = Bool()
}
class RtuToStAg extends Bundle with ROBConfig{
  val commit = Vec(NumCommitEntry,Bool())
  val iid    = Vec(NumCommitEntry, UInt(RobPtrWidth.W))
  val flush  = Bool()
}
//----------------------------------------------------------
class StoreAgIn extends Bundle with LsuConfig{
  val cp0In    = new Cp0ToStAg
  val dcacheIn = new DcacheArbToStAg
  val rfIn     = new RfPipe4
  val lmIn     = new LmToStAg
  val mmuIn    = new MmuToStAg
  val rtuIn    = new RtuToStAg
}
//==========================================================
//                        Output
//==========================================================
class StAgToDcacheArb extends Bundle with DCacheConfig{
  val dirtyGateclkEn    = Bool()
  val dirtyIdx          = UInt(INDEX_WIDTH.W)
  val dirtyReq          = Bool()
  val tagGateclkEn      = Bool()
  val tagIdx            = UInt(INDEX_WIDTH.W)
  val tagReq            = Bool()
}
class StAgToIdu extends Bundle with LsuConfig{
  val waitOld          = UInt(WAIT_OLD_WIDTH.W)
  val waitOldGateclkEn = Bool()
}
class StAgToMmu extends Bundle with LsuConfig{
 val abort1    = Bool()
 val id1       = UInt(IidWidth.W)
 val stInst1   = Bool()
 val stamoPa   = UInt(ADDR_PA_WIDTH.W)
 val stamoVld  = Bool()
 val va1       = UInt(64.W) // todo va? virtual addr? 39?
 val va1Vld    = Bool()
}
class StAgToDc extends Bundle with LsuConfig{
  val alreadyDa               = Bool()
  val atomic                  = Bool()
  val boundary                = Bool()
  val dcAccessSize            = UInt(ACCESS_SIZE_CHOOSE.W)
  val dcAddr0                 = UInt(PA_WIDTH.W)
  val dcBytesVld              = UInt(BYTES_ACCESS_WIDTH.W)
  val dcInstVld               = Bool()
  val dcMmuReq                = Bool()
  val dcPageShare             = Bool()
  val dcRotSel                = UInt(ROT_SEL_WIDTH.W)
  val exptAccessFaultWithPage = Bool()
  val exptIllegalInst         = Bool()
  val exptMisalignNoPage      = Bool()
  val exptMisalignWithPage    = Bool()
  val exptPageFault           = Bool()
  val exptStamoNotCa          = Bool()
  val exptVld                 = Bool()
  val fenceMode               = UInt(FENCE_MODE_WIDTH.W)
  val icc                     = Bool()
  val iid                     = UInt(IidWidth.W)
  val instFlush               = Bool()
  val instMode                = UInt(INST_MODE_WIDTH.W)
  val instType                = UInt(INST_TYPE_WIDTH.W)
  val instVld                 = Bool()
  val lsfifo                  = Bool()
  val lsid                    = UInt(LSIQ_ENTRY.W)
  val lsiqBkptaData           = Bool()
  val lsiqBkptbData           = Bool()
  val lsiqSpecFail            = Bool()
  val mtValue                 = UInt(PA_WIDTH.W) // preserve pc to mt
  val noSpec                  = Bool()
  val old                     = Bool()
  val pageBuf                 = Bool()
  val pageCa                  = Bool()
  val pageSec                 = Bool()
  val pageSo                  = Bool()
  val pageWa                  = Bool()
  val pc                      = UInt(LSU_PC_WIDTH.W)
  val sdidOh                  = UInt(LSIQ_ENTRY.W) // TODO ?
  val secd                    = Bool()
  val split                   = Bool()
  val st                      = Bool()
  val staddr                  = Bool()
  val stallOri                = Bool()
  val stallRestartEntry       = UInt(LSIQ_ENTRY.W)
  val syncFence               = Bool()
  val utlbMiss                = Bool()
  val vpn                     = UInt(VPN_WIDTH.W) // TODO ? why 28, not 3*9=27
}
//----------------------------------------------------------
class StoreAgOut extends Bundle with LsuConfig{
  val toDcacheArb = new StAgToDcacheArb
  val toIdu       = new StAgToIdu
  val toMmu       = new StAgToMmu
  val toDc        = new StAgToDc
  val rfVld       = Bool() // all done
}
//==========================================================
//                          IO
//==========================================================
class StoreAgIO extends Bundle with LsuConfig{
  val in  = Input(new StoreAgIn)
  val out = Output(new StoreAgOut)
}

class StoreAg extends Module with LsuConfig with DCacheConfig {
  val io = IO(new StoreAgIO)
  //==========================================================
  //                        RF signal
  //==========================================================
  val st_rf_inst_vld      = io.in.rfIn.gateclkSel
  val st_rf_inst_str      = io.in.rfIn.instStr
  val st_rf_off_0_extend  = io.in.rfIn.off0Extend
  //==========================================================
  //                 Pipeline Register
  //==========================================================
  //------------------control part----------------------------
  //+----------+
  //| inst_vld |
  //+----------+
  //if there is a stall in the AG stage ,the inst keep valid,
  //elseif there is inst in RF stage and no flush,
  //the inst goes to the AG stage next cycle
  val st_ag_inst_vld = RegInit(false.B)
  val st_ag_stall_vld = WireInit(false.B)
  when(io.in.rtuIn.flush){
    st_ag_inst_vld := false.B
  }.elsewhen(io.in.rfIn.sel || st_ag_stall_vld){
    st_ag_inst_vld := true.B
  }.otherwise{
    st_ag_inst_vld := false.B
  }
  //------------------data part-------------------------------
  //+-----------+-----------+-----------+------+------+------------+-----+-------+
  //| inst_type | inst_size | inst_mode | sdid | secd | sync_fence | icc | share |
  //+-----------+-----------+-----------+------+------+------------+-----+-------+
  //+----------------+----+-----+------+---------+-----+-----------+------------+
  //| unalign_permit | ex | iid | lsid | mmu_req | old | fdata_idx | already_da |
  //+----------------+----+-----+------+---------+-----+-----------+------------+
  //+----------------+------------+-------+
  //| lsiq_spec_fail | inst_flush | split |
  //+----------------+------------+-------+
  //+-------+-------+
  //| bkpta | bkptb |
  //+-------+-------+
  //if there is a stall in the AG stage ,the inst info keep unchanged,
  //elseif there is inst in RF stage, the inst goes to the AG stage next cycle
  val ag_pipe = RegInit(0.U.asTypeOf(new RfPipe4)) // if some signal unused, just ignore
  when(!st_ag_stall_vld && st_rf_inst_vld){
    ag_pipe := io.in.rfIn
  }
  //+------------------+
  //| already_cross_4k |
  //+------------------+
  //already cross 4k means addr1 is wrong, and mustn't merge from cache buffer
  val st_ag_already_cross_page_str_imme = RegInit(false.B)
  val st_ag_cross_page_str_imme_stall_req = WireInit(false.B)
  when(!st_ag_stall_vld){
    st_ag_already_cross_page_str_imme := false.B
  }.elsewhen(st_ag_stall_vld && st_ag_cross_page_str_imme_stall_req){
    st_ag_already_cross_page_str_imme := true.B
  }
  //+--------------+
  //| offset_shift |
  //+--------------+
  //if there is a stall in the AG stage ,offset_shift is reset to 0
  //cache stall will not change shift
  val st_ag_offset_shift = RegInit(1.U(SHITF_WIDTH.W))
  when(!st_ag_stall_vld){
    st_ag_offset_shift := io.in.rfIn.shift
  }.elsewhen(!st_ag_stall_vld && st_ag_cross_page_str_imme_stall_req){
    st_ag_offset_shift := 1.U(SHITF_WIDTH.W)
  }
  //+--------+
  //| offset |
  //+--------+
  //if the 1st time boundary 2nd instruction stall, the offset set 16 for bias, else
  //if stall, it set to 0, cache stall will not change offset
  val st_ag_offset = Wire(UInt(XLEN.W))
  val st_ag_offset_h = Reg(UInt((XLEN/2).W))
  val st_ag_offset_l = Reg(UInt((XLEN/2).W))
  val st_ag_cross_page_str_imme_stall_arb = Wire(Bool())
  val st_ag_str_imme_stall  = Wire(Bool())
  when(st_ag_cross_page_str_imme_stall_arb){
    st_ag_offset_h := 0.U(32.W)
  }.elsewhen(!st_ag_stall_vld && st_rf_inst_vld && !st_rf_inst_str){
    st_ag_offset_h := Cat(Fill(XLEN/2, io.in.rfIn.shift(SHITF_WIDTH-1)))
  }.elsewhen(!st_ag_stall_vld && st_rf_inst_vld && !st_rf_inst_str && st_rf_off_0_extend){
    st_ag_offset_h := 0.U(32.W)
  }.elsewhen(!st_ag_stall_vld && st_rf_inst_vld){
    st_ag_offset_h := io.in.rfIn.src1(XLEN-1,XLEN/2)
  }
  when(st_ag_cross_page_str_imme_stall_arb && st_ag_str_imme_stall){
    st_ag_offset_l := 16.U(32.W)
  }.elsewhen(st_ag_cross_page_str_imme_stall_arb){
    st_ag_offset_l := 0.U(32.W)
  }.elsewhen(!st_ag_stall_vld &&  st_rf_inst_vld  &&  !st_rf_inst_str){
    st_ag_offset_l := sext(XLEN/2,io.in.rfIn.shift)
  }.elsewhen(!st_ag_stall_vld && st_rf_inst_vld){
    st_ag_offset_l := io.in.rfIn.src1(XLEN/2-1,0)
  }
  st_ag_offset := Cat(st_ag_offset_h,st_ag_offset_l)
  //+-------------+
  //| offset_plus |
  //+-------------+
  //use this imm as offset when the ld/st inst need split and !secd
  val st_ag_offset_plus = RegInit(0.U((OFFSET_WIDTH+1).W))
  when(st_ag_cross_page_str_imme_stall_arb){
    st_ag_offset_plus := 0.U((OFFSET_WIDTH+1).W)
  }.elsewhen(!st_ag_stall_vld &&  st_rf_inst_vld){
    st_ag_offset_plus := io.in.rfIn.offsetPlus
  }
  //+------+
  //| base |
  //+------+
  val st_ag_base = RegInit(0.U(XLEN.W))
  when(st_ag_cross_page_str_imme_stall_arb){
    st_ag_base := 0.U(XLEN.W)
  }.elsewhen(!st_ag_stall_vld &&  st_rf_inst_vld){
    st_ag_base := io.in.rfIn.src0
  }
  //==========================================================
  //                      AG gateclk
  //==========================================================
  val st_ag_inst_stall_gateclk_en = st_ag_inst_vld
  //==========================================================
  //               Generate virtual address
  //==========================================================
  // for first boundary inst, use addr+offset+128 as va instead of addr+offset
  // for secd boundary,use addr+offset as va
  val st_ag_secd = RegInit(false.B)
  st_ag_secd := ag_pipe.unalign2nd
  val st_ag_offset_aftershift = st_ag_offset << OHToUInt(st_ag_offset)
  val st_ag_va_ori            = st_ag_base  + st_ag_offset_aftershift
  val st_ag_va_plus           = st_ag_base  + sext(XLEN,st_ag_offset_plus)
  val st_ag_va_plus_sel       = st_ag_secd  && !ag_pipe.instStr
  val st_ag_va                = Mux(st_ag_va_plus_sel, st_ag_va_plus,st_ag_va_ori)
  io.out.toDc.vpn            := st_ag_va(PA_WIDTH-1,OFFSET_WIDTH)
  //==========================================================
  //        Generate inst type
  //========================================================== TODO rename "bXX".U
  val st_ag_stamo_inst              = ag_pipe.atomic    && (ag_pipe.instType === "b00".U)
  val st_ag_icc_inst                = !ag_pipe.atomic   &&  ag_pipe.syncFence && ag_pipe.icc
  val st_ag_tlbi_inst               = st_ag_icc_inst    && (ag_pipe.instType === "b00".U)
  val st_ag_tlbi_all_inst           = st_ag_tlbi_inst   && (ag_pipe.instMode === "b00".U)
  val st_ag_dcache_inst             = st_ag_icc_inst    && (ag_pipe.instType === "b00".U)
  val st_ag_dcache_all_inst         = st_ag_dcache_inst && (ag_pipe.instMode === "b00".U)
  val st_ag_dcache_1line_inst       = st_ag_dcache_inst && (ag_pipe.instMode =/= "b00".U)
  val st_ag_dcache_pa_sw_inst       = st_ag_dcache_inst &&  ag_pipe.instMode(1)
  val st_ag_dcache_user_allow_inst  = st_ag_dcache_inst && (ag_pipe.instSize === byte(1,0)) && (ag_pipe.instMode === "b01".U)
  val st_ag_icache_inst             = ag_pipe.icc       && !ag_pipe.atomic && (ag_pipe.instType === "b01".U)
  val st_ag_icache_all_inst         = st_ag_icache_inst && (ag_pipe.instMode === "b00".U)
  val st_ag_icache_inv_va_inst      = st_ag_icache_inst && (ag_pipe.instMode === "b01".U)
  val st_ag_l2cache_inst            = ag_pipe.icc       && !ag_pipe.atomic && (ag_pipe.instType === "b11".U)
  val st_ag_fence_i_icache_all_inst = ag_pipe.split     &&  st_ag_icache_inst && (ag_pipe.instMode === "b00".U)
  val st_ag_st_inst                 = !ag_pipe.icc      && !ag_pipe.atomic && !ag_pipe.syncFence && (ag_pipe.instType === byte(1,0))
  val st_ag_statomic_inst           = st_ag_inst_vld    &&  ag_pipe.atomic
  //==========================================================
  //            Generate unalign, bytes_vld
  //==========================================================
  io.out.toDc.dcAccessSize := ag_pipe.instSize // TODO??? wtf, @ct_lsu_st_ag.v  865-889
  val st_ag_access_size_ori = LookupTree(ag_pipe.instSize,List(
    byte(1,0)  -> "b0000".U,
    half(1,0)  -> "b0001".U,
    word(1,0)  -> "b0011".U,
    dword(1,0) -> "b0111".U
  ))
  val st_ag_access_size = st_ag_access_size_ori
  //----------------generate uanlign--------------------------
  //-----------unalign--------------------
  val align_lookup = Cat(ag_pipe.instSize,st_ag_va_ori(2,0))
  val st_ag_align_list = ListLookup(align_lookup,List(false.B),Array(
    BitPat("b00???")  -> List(true.B),
    BitPat("b01??0")  -> List(true.B),
    BitPat("b10?00")  -> List(true.B),
    BitPat("b11000")  -> List(true.B)
  ))
  val st_ag_align :: Nil = st_ag_align_list
  val st_ag_unalign    = !st_ag_align
  val st_ag_unalign_so = !st_ag_align
  //---------------boundary---------------
  val st_ag_va_add_access_size = Cat("b0".U,st_ag_va_ori(3,0)) + Cat("b0".U,st_ag_access_size_ori)
  val st_ag_boundary_unmask = st_ag_va_add_access_size(4)
  val st_ag_boundary = (st_ag_boundary_unmask  ||  st_ag_secd)  &&  st_ag_st_inst
  val st_ag_le_bytes_vld_high_bits_full = LookupTree(st_ag_va_ori(3,0),List(
    "b0000".U -> 0xffff.U,
    "b0001".U -> 0xfffe.U,
    "b0010".U -> 0xfffc.U,
    "b0011".U -> 0xfff8.U,
    "b0100".U -> 0xfff0.U,
    "b0101".U -> 0xffe0.U,
    "b0110".U -> 0xffc0.U,
    "b0111".U -> 0xff80.U,
    "b1000".U -> 0xff00.U,
    "b1001".U -> 0xfe00.U,
    "b1010".U -> 0xfc00.U,
    "b1011".U -> 0xf800.U,
    "b1100".U -> 0xf000.U,
    "b1101".U -> 0xe000.U,
    "b1110".U -> 0xc000.U,
    "b1111".U -> 0x8000.U
  ))
  val st_ag_le_bytes_vld_low_bits_full = LookupTree(st_ag_va_add_access_size,List(
    "b0000".U -> 0x0001.U,
    "b0001".U -> 0x0003.U,
    "b0010".U -> 0x0007.U,
    "b0011".U -> 0x000f.U,
    "b0100".U -> 0x001f.U,
    "b0101".U -> 0x003f.U,
    "b0110".U -> 0x007f.U,
    "b0111".U -> 0x00ff.U,
    "b1000".U -> 0x01ff.U,
    "b1001".U -> 0x03ff.U,
    "b1010".U -> 0x07ff.U,
    "b1011".U -> 0x0fff.U,
    "b1100".U -> 0x1fff.U,
    "b1101".U -> 0x3fff.U,
    "b1110".U -> 0x7fff.U,
    "b1111".U -> 0xffff.U
  ))
  val st_ag_le_bytes_vld_cross = Wire(UInt(LINE_SIZE.W)) // this is use to shift, total is 16Bytes, a cache line
  // 0000 0000 0000 1111
  // 1111 0000 0000 0000
  // 1111 --------- 1111, thus named cross
  st_ag_le_bytes_vld_cross := st_ag_le_bytes_vld_high_bits_full & st_ag_le_bytes_vld_low_bits_full
  val st_ag_le_bytes_vld_high_cross_bits = Mux(st_ag_boundary_unmask,st_ag_le_bytes_vld_high_bits_full,st_ag_le_bytes_vld_cross)
  //-----------select bytes_vld-----------
  val st_ag_bytes_vld_low_bits = st_ag_le_bytes_vld_low_bits_full
  val st_ag_bytes_vld_high_cross_bits = st_ag_le_bytes_vld_high_cross_bits
  //used for st_dc_rot_sel when boundary split
  //bytes_vld1 is the bytes_vld of lower addr when there is a first(smaller) boundary st inst,
  //NOTE: it is different from ld pipe
  val st_ag_st_bytes_vld = Mux(st_ag_secd,st_ag_bytes_vld_low_bits,st_ag_bytes_vld_high_cross_bits) // low bits is second, which is over the boundary
  val st_ag_bytes_vld = Mux(st_ag_tlbi_inst,0xffff.U,st_ag_st_bytes_vld)
  val st_ag_dc_bytes_vld = st_ag_bytes_vld
  val st_ag_dc_rot_sel = Mux(st_ag_tlbi_inst,0.U(4.W),st_ag_va_ori(3,0))
  //==========================================================
  //        MMU interface
  //==========================================================
  //-----------mmu input--------------------------------------
  val st_ag_dcache_stall_req = Wire(Bool())
  val st_ag_expt_illegal_inst = Wire(Bool())
  val st_ag_expt_page_fault = Wire(Bool())
  io.out.toMmu.va1Vld   := ag_pipe.mmuReq && st_ag_inst_vld
  io.out.toMmu.stInst1  := ag_pipe.st
  io.out.toMmu.va1      := st_ag_base
  io.out.toMmu.abort1   := st_ag_cross_page_str_imme_stall_req || st_ag_dcache_stall_req || st_ag_expt_illegal_inst || io.in.rtuIn.flush
  io.out.toMmu.id1      := ag_pipe.iid
  io.out.toMmu.stamoVld := st_ag_inst_vld && st_ag_stamo_inst //atomic
  io.out.toMmu.stamoPa  := io.in.lmIn.pa
  //-----------mmu output-------------------------------------
  val st_ag_pn = io.in.mmuIn.pa1
  //0 means 4k
  //1 means 2M
  //2 means don't care
  val st_ag_page_so = Mux(st_ag_stamo_inst,io.in.lmIn.pageCa,(io.in.mmuIn.ca1 && io.in.mmuIn.pa1Vld))
  val st_ag_page_ca = Mux(st_ag_stamo_inst,io.in.lmIn.pageCa,io.in.mmuIn.ca1 && io.in.mmuIn.pa1Vld)  //TODO ca means can access?
  val st_ag_page_wa = st_ag_page_ca && io.in.cp0In.wa // TODO wa means wrong access?
  val st_ag_page_buf = Mux(st_ag_stamo_inst,io.in.lmIn.pageSec,io.in.mmuIn.sh1) // buffer store second page
  val st_ag_page_sec = Mux(st_ag_stamo_inst,io.in.lmIn.pageSec,io.in.mmuIn.sec1)
  val st_ag_page_share = Mux(st_ag_stamo_inst,io.in.lmIn.pageShare,io.in.mmuIn.sh1)
  val st_ag_utlb_miss = !io.in.mmuIn.pa1Vld && ag_pipe.mmuReq && !st_ag_expt_illegal_inst
  val st_ag_page_fault = io.in.mmuIn.pageFault1
  io.out.toDc.pageSo      := st_ag_page_so
  io.out.toDc.pageCa      := st_ag_page_ca
  io.out.toDc.pageWa      := st_ag_page_wa
  io.out.toDc.pageBuf     := st_ag_page_buf
  io.out.toDc.pageSec     := st_ag_page_sec
  io.out.toDc.dcPageShare := st_ag_page_share
  //==========================================================
  //        Generate physical address
  //==========================================================
  val st_ag_pa    = Cat(st_ag_pn(PA_WIDTH-13,0), st_ag_va(OFFSET_WIDTH-1,0))
  val st_ag_va_am = Mux(ag_pipe.syncFence||st_ag_tlbi_all_inst||st_ag_dcache_all_inst||st_ag_icache_all_inst||st_ag_l2cache_inst
    ,0.U(PA_WIDTH.W)
    ,st_ag_va  )
  val st_ag_addr =  Mux(ag_pipe.mmuReq || st_ag_stamo_inst
    ,st_ag_pn
    ,st_ag_va_am(PA_WIDTH-1,OFFSET_WIDTH))
  //==========================================================
  //        Generage dcache request information
  //==========================================================
  val ag_dcache_arb_st_gateclk_en = st_ag_inst_vld && io.in.cp0In.dcacheEn && (st_ag_st_inst || st_ag_statomic_inst || st_ag_dcache_1line_inst)
  val ag_dcache_arb_st_req = ag_dcache_arb_st_gateclk_en && (ag_pipe.mmuReq || st_ag_stamo_inst || st_ag_dcache_pa_sw_inst)
  //-----------tag array--------------------------------------
  io.out.toDcacheArb.tagGateclkEn   := ag_dcache_arb_st_gateclk_en
  io.out.toDcacheArb.tagReq         := ag_dcache_arb_st_req
  io.out.toDcacheArb.tagIdx         := st_ag_pa(PA_WIDTH-TAG_WIDTH-1,OFFSET_WIDTH) // (OFFSET_WIDTH+INDEX_WIDTH-1,OFFSET_WIDTH)
  //-----------dirty array------------------------------------
  io.out.toDcacheArb.dirtyGateclkEn := ag_dcache_arb_st_gateclk_en
  io.out.toDcacheArb.dirtyReq       := ag_dcache_arb_st_req
  io.out.toDcacheArb.dirtyIdx       := st_ag_pa(PA_WIDTH-TAG_WIDTH-1,OFFSET_WIDTH)
  //==========================================================
  //        Generage exception signal
  //==========================================================
  val tcore_vld = true.B //for tee
  val st_ag_prvlg_obey = (io.in.cp0In.privMode === "b00".U) &&
    (st_ag_dcache_inst && !(io.in.cp0In.ucme && st_ag_dcache_user_allow_inst) || st_ag_icache_inst && !(io.in.cp0In.ucme && st_ag_icache_inv_va_inst) && !st_ag_fence_i_icache_all_inst || st_ag_tlbi_inst||st_ag_l2cache_inst)||
    (io.in.cp0In.privMode  ===  "b01".U) && st_ag_tlbi_inst && (ag_pipe.instSize === "b00".U) && io.in.cp0In.tvm ||
    (io.in.cp0In.privMode  ===  "b01".U) && (io.in.cp0In.virtualMode && io.in.cp0In.tvm) && st_ag_tlbi_inst && (ag_pipe.instSize === "b01".U) ||
    !tcore_vld && st_ag_l2cache_inst
  //wtf
  st_ag_expt_illegal_inst := st_ag_prvlg_obey//&& !debugOn
  io.out.toDc.exptIllegalInst := st_ag_expt_illegal_inst
  when(st_ag_expt_illegal_inst){
    io.out.toDc.mtValue := sext(PA_WIDTH, ag_pipe.instCode)
  }.otherwise{
    io.out.toDc.mtValue := st_ag_va_ori //for misalign
  }
  val st_ag_inst_vls = false.B  //TODO  st_ag_inst_vls = 1'b0
  val st_ag_expt_misalign_no_page   = st_ag_unalign && (st_ag_st_inst && !io.in.cp0In.mm || ag_pipe.atomic || st_ag_inst_vls)
  io.out.toDc.exptMisalignNoPage   := st_ag_expt_misalign_no_page
  val st_ag_expt_misalign_with_page = st_ag_unalign_so && st_ag_page_so && io.in.mmuIn.pa1Vld && st_ag_st_inst
  io.out.toDc.exptMisalignWithPage := st_ag_expt_misalign_with_page
  st_ag_expt_page_fault            := ag_pipe.mmuReq && io.in.mmuIn.pa1Vld && st_ag_page_fault
  io.out.toDc.exptPageFault        := st_ag_expt_page_fault
  io.out.toDc.exptStamoNotCa       := io.in.mmuIn.pa1Vld && st_ag_stamo_inst && !st_ag_page_ca
  val st_ag_expt_access_fault_with_page = io.in.mmuIn.pa1Vld && st_ag_page_so && st_ag_st_inst && st_ag_inst_vls
  io.out.toDc.exptAccessFaultWithPage := st_ag_expt_access_fault_with_page
  io.out.toDc.exptVld := st_ag_expt_misalign_no_page || st_ag_expt_misalign_with_page || st_ag_expt_access_fault_with_page || st_ag_expt_illegal_inst || st_ag_expt_page_fault
  //==========================================================
  //        Generage stall/restart signal
  //==========================================================
  val st_ag_cmit_hit = Seq.fill(NumCommitEntry)(Wire(Bool()))
  for(i <- 0 until  NumCommitEntry){
    st_ag_cmit_hit(i) := Cat(io.in.rtuIn.commit(i),io.in.rtuIn.iid(i)) === Cat(1.U(1.W),ag_pipe.iid)
  }
  val st_ag_cmit = st_ag_cmit_hit.map{
    case hit => hit === true.B
  }.reduce(_ || _)
  val st_ag_atomic_no_cmit_restart_req = st_ag_inst_vld && st_ag_stamo_inst && !st_ag_cmit
  val st_ag_dcache_stall_unmask = !io.in.dcacheIn.sel
  st_ag_dcache_stall_req := st_ag_dcache_stall_unmask && st_ag_inst_vld
  //==========================================================
  //        Generage stall/restart signal
  //==========================================================
  //-----------stall------------------------------------------
  //get the stall signal if virtual address cross 4k address
  //for timing, if there is a carry adding last 12 bits, or there is '1' in high
  //bits, it will stall
  //---------------------cross 4k-----------------------------
  val st_ag_4k_sum_ori = Cat(0.U(1.W),st_ag_base(OFFSET_WIDTH,0)) + Cat(st_ag_offset(XLEN-1),st_ag_offset_aftershift(OFFSET_WIDTH,0))
  val st_ag_off_high_bits_all_0_ori = !(st_ag_offset_aftershift(XLEN-1,OFFSET_WIDTH).orR)
  val st_ag_off_high_bits_all_1_ori = st_ag_offset_aftershift(XLEN-1,OFFSET_WIDTH).andR
  val st_ag_off_high_bits_not_eq = !st_ag_off_high_bits_all_0_ori && !st_ag_off_high_bits_all_1_ori
  val st_ag_4k_sum_plus = Cat(0.U(1.W), st_ag_base(OFFSET_WIDTH-1,0)) + st_ag_offset_plus(OFFSET_WIDTH,0)
  val st_ag_4k_sum_12 = Mux(st_ag_va_plus_sel ,st_ag_4k_sum_plus(OFFSET_WIDTH) ,st_ag_4k_sum_ori(OFFSET_WIDTH) )
  val st_ag_cross_4k = st_ag_4k_sum_12 || st_ag_off_high_bits_not_eq
  //only str will trigger secd stall, and will stall at the first split
  val st_ag_boundary_stall = ag_pipe.instStr && st_ag_secd
  st_ag_str_imme_stall := st_ag_boundary_stall && !(st_ag_already_cross_page_str_imme)

  st_ag_cross_page_str_imme_stall_req := (st_ag_cross_4k || st_ag_str_imme_stall) && !st_ag_expt_misalign_no_page && st_ag_inst_vld && ag_pipe.mmuReq
  //-----------arbiter----------------------------------------
  //prioritize:
  //  1. cross_page_str_imme_stall    : cross_4k
  //  2. dcache_stall    : cache
  //  other restart flop to dc stage
  //  other_restart  : utlb_miss, tlb_busy
  val st_ag_stall_mask = Wire(Bool())
  st_ag_cross_page_str_imme_stall_arb := st_ag_atomic_no_cmit_restart_req && st_ag_cross_page_str_imme_stall_req && !st_ag_stall_mask
  val st_ag_atomic_no_cmit_restart_arb = st_ag_atomic_no_cmit_restart_req
  //-----------generate total siangl--------------------------
  // &Force("output","st_ag_stall_ori"); @1160
  val st_ag_stall_ori = (st_ag_cross_page_str_imme_stall_req || st_ag_dcache_stall_req || ag_pipe.mmuReq) && !st_ag_atomic_no_cmit_restart_req
  st_ag_stall_vld := st_ag_stall_ori && !st_ag_stall_mask
  val st_ag_stall_restart = st_ag_cross_page_str_imme_stall_req || st_ag_dcache_stall_req || ag_pipe.mmuReq || st_ag_atomic_no_cmit_restart_req
  val iid_is_old = Wire(Bool())// TODO add iid compare
  val st_ag_iid_compare = Module(new CompareIid)
  st_ag_iid_compare.io.iid0 := io.in.rfIn.iid
  st_ag_iid_compare.io.iid1 := ag_pipe.iid
  iid_is_old := st_ag_iid_compare.io.older

  st_ag_stall_mask := io.in.rfIn.sel && iid_is_old
  io.out.toDc.stallRestartEntry := Mux(st_ag_stall_mask,ag_pipe.lchEntry,io.in.rfIn.lchEntry)
  //==========================================================
  //        Generate restart/lsiq signal
  //==========================================================
  //-----------lsiq signal----------------
  val st_ag_mask_lsid = Mux(st_ag_inst_vld,ag_pipe.lchEntry,0.U(LSIQ_ENTRY.W))
  io.out.toIdu.waitOldGateclkEn := st_ag_atomic_no_cmit_restart_arb
  io.out.toIdu.waitOld          := Mux(st_ag_inst_vld,ag_pipe.lchEntry,0.U(LSIQ_ENTRY.W))
  //==========================================================
  //        Generage to DC stage signal
  //==========================================================
  io.out.toDc.dcInstVld   := st_ag_inst_vld && !st_ag_stall_restart
  io.out.toDc.dcMmuReq    := ag_pipe.mmuReq && !io.out.toMmu.abort1
  io.out.toDc.dcPageShare := ag_pipe.instShare || st_ag_page_share && (ag_pipe.mmuReq || st_ag_stamo_inst)
  io.out.toDc.dcAddr0     := Mux(io.in.dcacheIn.borrowAddrVld,io.in.dcacheIn.addr,st_ag_addr)
  io.out.toDc.dcBytesVld    := st_ag_dc_bytes_vld
  //==========================================================
  //         Pipe regs to out
  //==========================================================
  io.out.toDc.lsiqBkptaData := ag_pipe.bkptaData
  io.out.toDc.lsiqBkptbData := ag_pipe.bkptbData
  io.out.toDc.secd          := st_ag_secd
  io.out.toDc.st            := ag_pipe.st
  io.out.toDc.boundary      := st_ag_boundary
  io.out.toDc.noSpec        := ag_pipe.noSpec
  io.out.toDc.instFlush     := ag_pipe.instFlush
  io.out.toDc.instMode      := ag_pipe.instMode
  io.out.toDc.fenceMode     := ag_pipe.fenceMode
  io.out.toDc.instType      := ag_pipe.instType
  io.out.toDc.syncFence     := ag_pipe.syncFence
  io.out.toDc.instVld       := st_ag_inst_vld
  io.out.toDc.pc            := ag_pipe.pc
  io.out.toDc.lsid          := ag_pipe.lchEntry
  io.out.toDc.sdidOh        := ag_pipe.sdiqEntry
  io.out.toDc.iid           := ag_pipe.iid
  io.out.toDc.staddr        := ag_pipe.staddr
  io.out.toDc.icc           := ag_pipe.icc
  io.out.toDc.atomic        := ag_pipe.atomic
  io.out.toDc.lsiqSpecFail  := ag_pipe.specFail
  io.out.toDc.split         := ag_pipe.split
  io.out.toDc.old           := ag_pipe.oldest
  io.out.toDc.lsfifo        := ag_pipe.lsfifo
  io.out.toDc.alreadyDa     := ag_pipe.alreadyDa

  io.out.rfVld              := st_rf_inst_vld
  io.out.toDc.utlbMiss      := st_ag_utlb_miss
  io.out.toDc.dcRotSel      := st_ag_dc_rot_sel
  io.out.toDc.stallOri      := st_ag_stall_ori

}
