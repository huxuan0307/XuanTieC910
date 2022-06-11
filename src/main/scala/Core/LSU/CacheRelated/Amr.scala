package Core.LSU.CacheRelated

/*
AMR-L1 is enable bit of automatic adjustment of the Cache write allocate strategy
AMR is on the MHINT[3]
MHINT (what's the abbreviation for？)- Machine Mode Implicit Operation Registers
AMR - 0, The write allocation strategy is determined by the page attribute WA of the access address
AMR - 1, when multiple consecutive cache lines need to be stored,
subsequent storage operations of consecutive addresses are no longer written to L1Cache

WA is on the MHCR[2]
MHCR - config the core, includes performance and functionality
WA - 0, Dcache write non-allocate
WA - 1, Dcache write allocate
 */
import Core.LsuConfig
import chisel3._
import chisel3.util._

trait AmrStateConfig{
  def JUDGE     = "b000".U
  def MEM_SET_0 = "b001".U // 0 means write 1 dcache line
  def MEM_SET_1 = "b011".U // 1 add cancel write allocate
  def MEM_SET_2 = "b111".U // 2 add cancel l2 write allocate
}
//==========================================================
//                        Input
//==========================================================
class Cp0ToAmr extends Bundle with LsuConfig{
  val amr        = Bool() // from MHINT[3]
  val amr2       = Bool() // from MHINT[3]
  val icgEn      = Bool()
  val noOpReq    = Bool()
  val clkEn      = Bool()
}
class WmbCeToAmr extends Bundle with LsuConfig{
  val addr     = UInt(PA_WIDTH.W)
  val bytesVld = UInt(BYTES_ACCESS_WIDTH.W)
  val caStInst = Bool()
  val popVld   = Bool()
  val vld       = Bool()
}
//----------------------------------------------------------
class AmrIn extends Bundle with LsuConfig{
  val cp0In    = new Cp0ToAmr
  val wmbIn    = new WmbCeToAmr
  val iccIdle  = Bool()
}
//==========================================================
//                        Output
//==========================================================
class AmrOut extends Bundle with LsuConfig{
  val l2MemSet = Bool()
  val waCancel = Bool()
}
//==========================================================
//                          IO
//==========================================================
class AmrIO extends Bundle with LsuConfig{
  val in  = Input(new AmrIn)
  val out = Output(new AmrOut)
}
class Amr extends Module with LsuConfig with AmrStateConfig {
  val io = IO(new AmrIO)
  // val judge :: mem_set_0 :: mem_set_1 :: mem_set_2 :: Nil = Enum(4)
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //amr pop clk is for addr and bytes_vld when sq pops a entry
  val amr_cnt_met_set = Wire(Bool())
  val amr_not_idle = Wire(Bool())
  val amr_clk_en = Mux(io.in.cp0In.amr, (io.in.wmbIn.vld || amr_cnt_met_set || amr_not_idle), amr_not_idle)

  val amr_update_vld = Wire(Bool())
  val amr_update_clk_en  = amr_update_vld
  //==========================================================
  //                      Registers
  //==========================================================
  //+-------+
  //| state |
  //+-------+
  val amr_state = RegInit(JUDGE)
  io.out.waCancel := amr_state(0).asBool
  io.out.l2MemSet := amr_state(2).asBool && io.in.cp0In.amr2
  //+------+-----------+-----+-----+
  //| addr | bytes_vld | cnt | neg |
  //+------+-----------+-----+-----+
  val amr_cnt = RegInit(0.U(6.W)) // is a counter to count how many consecutive cache lines need to be stored
  val amr_judge_cancel   = Wire(Bool())
  val amr_judge_fail     = Wire(Bool())
  val amr_bytes_vld_full = Wire(Bool())
  val amr_judge_flush    = Wire(Bool())
  when(amr_judge_cancel){
    amr_cnt := 0.U(6.W)
  }.elsewhen(amr_judge_fail){
    amr_cnt := 0.U(6.W)
  }.elsewhen(amr_update_vld && amr_bytes_vld_full){
    amr_cnt := amr_cnt + 1.U(6.W)
  }.elsewhen(amr_update_vld){
    amr_cnt := amr_cnt // ? useless
  }
  val amr_from_wmb_ce_addr = io.in.wmbIn.addr
  val amr_bytes_vld = RegInit(0.U(BYTES_ACCESS_WIDTH.W))
  val amr_bytes_vld_next = Wire(UInt(BYTES_ACCESS_WIDTH.W))
  when(amr_judge_fail || amr_update_vld && amr_bytes_vld_full){
    amr_bytes_vld := io.in.wmbIn.bytesVld
  }.elsewhen(amr_update_vld){
    amr_bytes_vld := amr_bytes_vld_next
  }
  val amr_addr_tto4 = RegInit(0.U((PA_WIDTH-4).W))
  amr_addr_tto4 := amr_from_wmb_ce_addr((PA_WIDTH-1),4)
  //==========================================================
  //                 Generate next state
  //==========================================================
  switch(amr_state){
    is(JUDGE){
      when(amr_cnt_met_set){
        amr_state := MEM_SET_0
      }.otherwise{
        amr_state := JUDGE
      }
    }
    is(MEM_SET_0){
      when(amr_judge_flush){
        amr_state := JUDGE
      }.elsewhen( amr_cnt === "d16".U){
        amr_state := MEM_SET_1
      }
    }
    is(MEM_SET_1){
      when(amr_judge_flush){
        amr_state := MEM_SET_0
      }.elsewhen( amr_cnt === "d48".U){
        amr_state := MEM_SET_2
      }
    }
    is(MEM_SET_2){
      when(amr_judge_flush){
        amr_state := MEM_SET_1
      }
    }
  }
  //==========================================================
  //                        Wires
  //==========================================================
  //---------------------amr cnt state change-----------------
  amr_cnt_met_set := amr_cnt === "d8".U
  amr_update_vld := io.in.wmbIn.popVld
  val amr_addr_hit_normal_update = Wire(Bool())
  val amr_addr_hit_eq = Wire(Bool())
  val amr_addr_hit = Mux(amr_bytes_vld_full,amr_addr_hit_normal_update,amr_addr_hit_eq)
  val amr_addr_distance = Wire(UInt((PA_WIDTH-4).W))
  amr_addr_distance := amr_from_wmb_ce_addr(((PA_WIDTH-1)),4) - amr_addr_tto4
  // distance equal to 0
  amr_addr_hit_eq := !(amr_addr_distance.orR)
  amr_addr_hit_normal_update := amr_addr_distance(0) && ( (amr_addr_distance(PA_WIDTH-5,1).andR) || (!(amr_addr_distance(PA_WIDTH-5,1).orR)) )
  //---------------------next bytes_vld-----------------------
  amr_bytes_vld_next := amr_bytes_vld & io.in.wmbIn.bytesVld
  amr_bytes_vld_full := amr_bytes_vld.andR
  //cross hit means there is a common bit 1 both in
  //amr_bytes_vld and wmb_ce_bytes_vld
  val amr_bytes_vld_cross = (amr_bytes_vld & io.in.wmbIn.bytesVld).orR
  //---------------------judge flush--------------------------
  //cancel means it need to clear all the tags
  amr_judge_cancel := (!io.in.iccIdle) || (!io.in.cp0In.amr) || (amr_update_vld && io.in.wmbIn.caStInst) || io.in.cp0In.noOpReq
  //fail means it need to set all the tags to the pop st inst
  amr_judge_fail := amr_update_vld && ((!amr_addr_hit) || amr_bytes_vld_cross && (!amr_bytes_vld_full))
  amr_judge_flush := amr_judge_cancel || amr_judge_fail
  amr_not_idle := (amr_state =/= JUDGE) || (amr_cnt =/= 0.U(6.W))
}
