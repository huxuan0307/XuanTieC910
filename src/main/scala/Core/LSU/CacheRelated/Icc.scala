package Core.LSU.CacheRelated
/*
MCOR - Machine hardware operation register - config cache & BHT & BTB
Cache Invalid - MCOR[4]

Cache Clear   - MCOR[5]
CLR - 1, Cache lines marked as dirty will be written out of chip
CLR - 0, Cache lines marked as dirty will not be written out of chip
 */
import Core.IntConfig.XLEN
import Core.LSU.Sq.SqToIcc
import Core.LSU.StoreExStage.StDaToIcc
import Core.{DCacheConfig, LsuConfig}
import chisel3._
import chisel3.util._

trait IccStateConfig{
  def IDLE            = "b000".U
  def WAIT_FOR_READY  = "b001".U
  def INV_DCACHE_LINE = "b010".U
  def REQ_VB_WAY0     = "b100".U
  def REQ_VB_WAY1     = "b101".U
  def WAIT_VB_EMPTY   = "b110".U
  def READ_DCACHE     = "b011".U
  def WAIT_DATA       = "b111".U
}
//==========================================================
//                        Input
//==========================================================
class Cp0ToIcc extends Bundle with LsuConfig{
  val dcacheClr       = Bool()
  val dcacheInv       = Bool()
  val dcacheReadIndex = UInt(17.W)
  val dcacheReadLdTag = Bool()
  val dcacheReadReq   = Bool()
  val dcacheReadStTag = Bool()
  val dcacheReadWay   = Bool()
  val icgEn           = Bool()
}
class XxEmpty extends Bundle with LsuConfig{
  val lfb  = Bool()
  val rb   = Bool()
  val snq  = Bool()
  val sq   = Bool()
  val vb   = Bool()
  val wmb  = Bool()
}
//----------------------------------------------------------
class IccIn extends Bundle with LsuConfig{
  val cp0In = new Cp0ToIcc
  val dcacheArbIccLdGrnt = Bool()
  val ldDaIn = new Bundle() {
    val readData  = UInt((2*XLEN).W)
    val snqBorrow = Bool()
  }
  val empty  = new XxEmpty
  val sqIn   = new SqToIcc
  val stDaIn = new StDaToIcc
  val vbCreateGrnt = Bool()
  val pfuReady = Bool()

}
//==========================================================
//                        Output
//==========================================================
class IccToDcacheArb extends Bundle with LsuConfig with DCacheConfig{
  val dataWay = Bool()
  val way = Bool()
  val ld = new Bundle() {
    val borrowReq     = Bool()
    val dataGateclkEn = UInt(8.W)  // todo define 8
    val dataHighIdx   = UInt(11.W) // todo define 11
    val dataLowIdx    = UInt(11.W) // todo define 11
    val dataReq       = UInt(8.W)
    val req           = Bool()
    val tagGateclkEn  = Bool()
    val tagIdx        = UInt(INDEX_WIDTH.W)
    val tagRead       = Bool()
    val tagReq        = Bool()
  }
  val st = new Bundle() {
    val borrowReq      = Bool()
    val dirtyDin       = UInt(7.W)  // todo define 7
    val dirtyGateclkEn = Bool()
    val dirtyGwen      = Bool()
    val dirtyIdx       = UInt(INDEX_WIDTH.W)
    val dirtyReq       = Bool()
    val dirtyWen       = UInt(8.W)
    val req            = Bool()
    val tagReq         = Bool()
    val tagGateclkEn   = Bool()
    val tagIdx         = UInt(9.W)
  }
}
class IccToVb extends Bundle with LsuConfig{
  val way = Bool()
  val addrTto6        = UInt((PA_WIDTH-4).W)
  val createDpVld     = Bool()
  val createGateclkEn = Bool()
  val createReq       = Bool()
  val createVld       = Bool()
  val inv             = Bool()
}
class IccToCp0 extends Bundle with LsuConfig{
  val dcacheDone        = Bool()
  val dcacheReadData    = UInt((2*XLEN).W)
  val dcacheReadDataVld = Bool()
}
//----------------------------------------------------------
class IccOut extends Bundle with LsuConfig{
  val toArb = new IccToDcacheArb
  val iccIdle = Bool()
  val snqCreatePermit = Bool()
  val sqGrnt = Bool()
  val toVb = new IccToVb
  val wmbWriteImme = Bool()
  val toCp0 = new IccToCp0
}
//==========================================================
//                        IO
//==========================================================
class IccIO extends Bundle with LsuConfig{
  val in = Input(new IccIn)
  val out = Output(new IccOut)
}
class Icc extends Module with LsuConfig with IccStateConfig{
  val io = IO(new IccIO)
  val icc_state = RegInit(IDLE)
  val state_idle = icc_state === IDLE
  val icc_clk_en = io.in.sqIn.req ||
    io.in.cp0In.dcacheInv || io.in.cp0In.dcacheClr || io.in.cp0In.dcacheReadReq ||
    (!state_idle)
  //==========================================================
  //                      Registers
  //==========================================================
  //+-----+-----+
  //| clr | inv |
  //+-----+-----+
  val icc_clr = RegInit(false.B)
  val icc_inv = RegInit(false.B)
  val icc_set_clr = Wire(Bool())
  val icc_done = Wire(Bool())
  val icc_set_inv = Wire(Bool())
  when(icc_set_clr){
    icc_clr := true.B
  }.elsewhen(icc_done){
    icc_clr := false.B
  }
  when(icc_set_inv){
    icc_inv := true.B
  }.elsewhen(icc_done){
    icc_inv := false.B
  }
  //+-----+
  //| cnt |
  //+-----+
  val icc_cnt = RegInit(0.U(9.W))
  val icc_start = Wire(Bool())
  val icc_cnt_add_vld = io.in.dcacheArbIccLdGrnt || io.in.vbCreateGrnt &&
    (icc_state === REQ_VB_WAY1)
  when(icc_cnt_add_vld){
    icc_cnt :=( Cat("b0".U, icc_cnt) + 1.U(10.W))(8,0)
  }
  //+------------+
  //| cp0_create |
  //+------------+
  val icc_cp0_create = RegInit(false.B)
  when(state_idle && io.in.sqIn.req){
    icc_cp0_create := false.B
  }.elsewhen(state_idle && (io.in.cp0In.dcacheClr||io.in.cp0In.dcacheInv)){
    icc_cp0_create := true.B
  }
  //==========================================================
  //                 Generate next state
  //==========================================================
  val icc_ready = Wire(Bool())
  val icc_cnt_overflow = Wire(Bool())
  val dcache_read_data_vld = Wire(Bool())
  switch(icc_state){
    is(IDLE){
      when(icc_start){
        icc_state := WAIT_FOR_READY
      }
    }
    is(WAIT_FOR_READY){
      when(icc_ready && icc_clr){
        icc_state := REQ_VB_WAY0
      }.elsewhen(icc_ready && icc_inv){
        icc_state := INV_DCACHE_LINE
      }.elsewhen(icc_ready){
        icc_state := READ_DCACHE
      }
    }
    is(INV_DCACHE_LINE){
      when(io.in.dcacheArbIccLdGrnt && icc_cnt_overflow){
        icc_state := WAIT_VB_EMPTY
      }
    }
    is(REQ_VB_WAY0){
      when(io.in.vbCreateGrnt){
        icc_state := REQ_VB_WAY1
      }
    }
    is(REQ_VB_WAY1){
      when(io.in.vbCreateGrnt &&  icc_cnt_overflow){
        icc_state := WAIT_VB_EMPTY
      }.elsewhen(io.in.vbCreateGrnt ){
        icc_state := REQ_VB_WAY0
      }
    }
    is(WAIT_VB_EMPTY){
      when(io.in.empty.vb){
        icc_state := IDLE
      }
    }
    is(READ_DCACHE){
        icc_state := WAIT_DATA
    }
    is(WAIT_DATA){
      when(dcache_read_data_vld){
        icc_state := IDLE
      }
    }
  }
  //==========================================================
  //                  State 0 : idle
  //==========================================================
  io.out.iccIdle := state_idle
  io.out.sqGrnt := state_idle
  icc_start := state_idle && (io.in.sqIn.req || io.in.cp0In.dcacheInv || io.in.cp0In.dcacheClr || io.in.cp0In.dcacheReadReq )
  icc_set_clr := state_idle && (io.in.sqIn.req&&io.in.sqIn.clr || io.in.cp0In.dcacheClr)
  icc_set_inv := state_idle && (io.in.sqIn.req&&io.in.sqIn.inv || io.in.cp0In.dcacheInv)
  //==========================================================
  //                  State 1 : wait for ready
  //==========================================================
  //must wait for snq/wmb/icc/vb empty
  //if cp0 start icc state machine, it must wait for sq empty
  val icc_ready_expt_snq = io.in.empty.wmb && io.in.empty.rb && (io.in.empty.sq || (!icc_cp0_create)) && io.in.empty.vb && io.in.empty.lfb &&
    io.in.pfuReady
  icc_ready := icc_ready_expt_snq && io.in.empty.snq
  io.out.wmbWriteImme := icc_state === WAIT_FOR_READY
  //==========================================================
  //              State 2/3/4 : inv dcache line / req vb
  //==========================================================
  // @159
  // assign icc_cnt_add1[9:0]  = {1'b0,icc_cnt[8:0]}  + 10'b1;
  // assign icc_cnt_add_vld    = dcache_arb_icc_ld_grnt  ||  vb_icc_create_grnt  &&  (icc_state[2:0]  ==  REQ_VB_WAY1);
  //==========================================================
  //                  State 5 : wait vb empty
  //==========================================================
  icc_done := (icc_state === WAIT_VB_EMPTY) && io.in.empty.vb
  //==========================================================
  //                      Interface
  //==========================================================
  //----------------------cache interface---------------------
  val dcache_read_data_req = (icc_state === READ_DCACHE) &&
    (!io.in.cp0In.dcacheReadLdTag) && (!io.in.cp0In.dcacheReadStTag)
  val dcache_read_st_tag_req = (icc_state === READ_DCACHE)  && io.in.cp0In.dcacheReadStTag
  val dcache_read_ld_tag_req = (icc_state === READ_DCACHE)  && io.in.cp0In.dcacheReadLdTag
  val dcache_read_tag_req = dcache_read_ld_tag_req || dcache_read_st_tag_req
  io.out.toArb.ld.req := (icc_state === INV_DCACHE_LINE) || (icc_state === READ_DCACHE)
  io.out.toArb.st.req := (icc_state === INV_DCACHE_LINE) || (icc_state === READ_DCACHE)
  //---------------dirty array------------
  io.out.toArb.st.dirtyReq       := (icc_state === INV_DCACHE_LINE) || dcache_read_st_tag_req
  io.out.toArb.st.dirtyGateclkEn := io.out.toArb.st.dirtyReq
  io.out.toArb.st.dirtyIdx       := Mux( dcache_read_tag_req, io.in.cp0In.dcacheReadIndex(14,6), icc_cnt)
  io.out.toArb.st.dirtyDin       := 0.U(7.W)
  io.out.toArb.st.dirtyGwen      := !dcache_read_tag_req
  io.out.toArb.st.dirtyWen       := Mux(dcache_read_tag_req, 0.U(7.W), "h7f".U)
  //---------------tag array------------
  io.out.toArb.ld.tagReq        := (icc_state === INV_DCACHE_LINE) || dcache_read_ld_tag_req
  io.out.toArb.ld.tagGateclkEn  := io.out.toArb.ld.tagReq
  io.out.toArb.ld.tagIdx        := Mux(dcache_read_tag_req , io.in.cp0In.dcacheReadIndex(14,6), icc_cnt)
  io.out.toArb.ld.tagRead       := dcache_read_ld_tag_req

  io.out.toArb.st.tagReq        := dcache_read_st_tag_req
  io.out.toArb.st.tagGateclkEn  := io.out.toArb.st.tagReq
  io.out.toArb.st.tagIdx        := io.in.cp0In.dcacheReadIndex(14,6)
  io.out.toArb.st.borrowReq     := dcache_read_st_tag_req
  io.out.toArb.way        := io.in.cp0In.dcacheReadWay
  //---------------data array------------
  io.out.toArb.ld.borrowReq      := dcache_read_data_req || dcache_read_ld_tag_req
  io.out.toArb.ld.dataReq        := Cat(Seq.fill(8)(dcache_read_data_req.asUInt))
  io.out.toArb.ld.dataGateclkEn  := io.out.toArb.ld.dataReq
  io.out.toArb.ld.dataLowIdx     := io.in.cp0In.dcacheReadIndex(14,4)
  io.out.toArb.ld.dataHighIdx    := Cat(io.in.cp0In.dcacheReadIndex(14,5), ~io.in.cp0In.dcacheReadIndex(4))
  io.out.toArb.dataWay := (io.in.cp0In.dcacheReadIndex(4).asBool && (!io.in.cp0In.dcacheReadWay)) ||
    ((~io.in.cp0In.dcacheReadIndex(4)).asBool && (io.in.cp0In.dcacheReadWay))
  //----------------------vb interface------------------------
  io.out.toVb.way             := icc_state === REQ_VB_WAY1
  io.out.toVb.createReq       := (icc_state === REQ_VB_WAY1) || (icc_state === REQ_VB_WAY0)
  io.out.toVb.createVld       := io.out.toVb.createReq
  io.out.toVb.createDpVld     := io.out.toVb.createReq
  io.out.toVb.createGateclkEn := io.out.toVb.createReq
  io.out.toVb.addrTto6        := Cat(0.U((PA_WIDTH-32).W), io.out.toVb.way, 0.U(16.W),icc_cnt )
  io.out.toVb.inv             := icc_inv
  //----------------------cp0 interface-----------------------
  io.out.toCp0.dcacheDone := icc_done && icc_cp0_create
  dcache_read_data_vld := io.in.ldDaIn.snqBorrow || io.in.stDaIn.borrowIccVld
  io.out.toCp0.dcacheReadDataVld := dcache_read_data_vld
  val st_da_icc_read_data = Cat(0.U(88.W),
    io.in.stDaIn.tagInfo(PA_WIDTH-15,0),
    io.in.cp0In.dcacheReadIndex(13,13),
    0.U(9.W),
    io.in.stDaIn.dirtyInfo(2,0))
  io.out.toCp0.dcacheReadData    := Mux(io.in.ldDaIn.snqBorrow, io.in.ldDaIn.readData, st_da_icc_read_data)
  io.out.snqCreatePermit := (icc_state === IDLE) || (icc_state === WAIT_FOR_READY) && (!icc_ready_expt_snq)


}
