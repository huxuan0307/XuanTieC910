package Core.IFU
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._



class PCfifo extends Module with Config {
  val io = IO(new PCfifoIO)
  val cur_pc = Wire(Vec(8,UInt(VAddrBits.W)))
  //val over_mask = WireInit(0.U(8.W))
  cur_pc(0) := Mux(io.in.h0_vld,io.in.cur_pc(8),io.in.cur_pc(7))
  for(i <- 1 to 7){
    cur_pc(i) := io.in.cur_pc(7-i)
  }
  val pcoper = io.in.pc_oper
  val jal = io.in.jal
  val jalr = io.in.jalr
  val con_br = io.in.con_br
  val dst_vld = io.in.dst_vld
//  val index_0:UInt = PriorityMux(Seq(
//    pcoper(7) -> 0.U,
//    pcoper(6) -> 1.U,
//    pcoper(5) -> 2.U,
//    pcoper(4) -> 3.U,
//    pcoper(3) -> 4.U,
//    pcoper(2) -> 5.U,
//    pcoper(1) -> 6.U,
//    pcoper(0) -> 7.U,
//    true.B    -> 0.U
//  ))
//
//  val index_1 = Wire(UInt(4.W))
//  when(index_0 === 0.U) {
//    index_1 := PriorityMux(Seq(
//      pcoper(6) -> 1.U,
//      pcoper(5) -> 2.U,
//      pcoper(4) -> 3.U,
//      pcoper(3) -> 4.U,
//      pcoper(2) -> 5.U,
//      pcoper(1) -> 6.U,
//      pcoper(0) -> 7.U,
//      true.B    -> 7.U
//    ))
//  }.elsewhen(index_0 === 1.U) {
//    index_1 := PriorityMux(Seq(
//      pcoper(5) -> 2.U,
//      pcoper(4) -> 3.U,
//      pcoper(3) -> 4.U,
//      pcoper(2) -> 5.U,
//      pcoper(1) -> 6.U,
//      pcoper(0) -> 7.U,
//      true.B    -> 7.U
//    ))
//  }.elsewhen(index_0 === 2.U) {
//    index_1 := PriorityMux(Seq(
//      pcoper(4) -> 3.U,
//      pcoper(3) -> 4.U,
//      pcoper(2) -> 5.U,
//      pcoper(1) -> 6.U,
//      pcoper(0) -> 7.U,
//      true.B    -> 7.U
//    ))
//  }.elsewhen(index_0 === 3.U) {
//    index_1 := PriorityMux(Seq(
//      pcoper(3) -> 4.U,
//      pcoper(2) -> 5.U,
//      pcoper(1) -> 6.U,
//      pcoper(0) -> 7.U,
//      true.B    -> 7.U
//    ))
//  }.elsewhen(index_0 === 4.U) {
//    index_1 := PriorityMux(Seq(
//      pcoper(2) -> 5.U,
//      pcoper(1) -> 6.U,
//      pcoper(0) -> 7.U,
//      true.B    -> 7.U
//    ))
//  }.elsewhen(index_0 === 5.U) {
//    index_1 := PriorityMux(Seq(
//      pcoper(1) -> 6.U,
//      pcoper(0) -> 7.U,
//      true.B    -> 7.U
//    ))
//  }.otherwise {
//    index_1 := 7.U
//  }
  val index_0:UInt = PriorityMux(Seq(
    pcoper(0) -> 7.U,
    pcoper(1) -> 6.U,
    pcoper(2) -> 5.U,
    pcoper(3) -> 4.U,
    pcoper(4) -> 3.U,
    pcoper(5) -> 2.U,
    pcoper(6) -> 1.U,
    pcoper(7) -> 0.U,
    true.B    -> 7.U
  ))

  val index_1 = Wire(UInt(4.W))
  when(index_0 === 0.U) {
    index_1 := PriorityMux(Seq(
      pcoper(1) -> 6.U,
      pcoper(2) -> 5.U,
      pcoper(3) -> 4.U,
      pcoper(4) -> 3.U,
      pcoper(5) -> 2.U,
      pcoper(6) -> 1.U,
      pcoper(7) -> 0.U,
      true.B    -> 0.U
    ))
  }.elsewhen(index_0 === 1.U) {
    index_1 := PriorityMux(Seq(
      pcoper(2) -> 5.U,
      pcoper(3) -> 4.U,
      pcoper(4) -> 3.U,
      pcoper(5) -> 2.U,
      pcoper(6) -> 1.U,
      pcoper(7) -> 0.U,
      true.B    -> 0.U
    ))
  }.elsewhen(index_0 === 2.U) {
    index_1 := PriorityMux(Seq(
      pcoper(3) -> 4.U,
      pcoper(4) -> 3.U,
      pcoper(5) -> 2.U,
      pcoper(6) -> 1.U,
      pcoper(7) -> 0.U,
      true.B    -> 0.U
    ))
  }.elsewhen(index_0 === 3.U) {
    index_1 := PriorityMux(Seq(
      pcoper(4) -> 3.U,
      pcoper(5) -> 2.U,
      pcoper(6) -> 1.U,
      pcoper(7) -> 0.U,
      true.B    -> 0.U
    ))
  }.elsewhen(index_0 === 4.U) {
    index_1 := PriorityMux(Seq(
      pcoper(5) -> 2.U,
      pcoper(6) -> 1.U,
      pcoper(7) -> 0.U,
      true.B    -> 0.U
    ))
  }.elsewhen(index_0 === 5.U) {
    index_1 := PriorityMux(Seq(
      pcoper(6) -> 1.U,
      pcoper(7) -> 0.U,
      true.B    -> 0.U
    ))
  }.otherwise {
    index_1 := 0.U
  }

  val cur_pc_0  = cur_pc(index_0)
  val jal_0     = jal(index_0)
  val jalr_0    = jalr(index_0)
  val con_br_0  = con_br(index_0)
  val dst_vld_0 = dst_vld(index_0)

  val cur_pc_1  = cur_pc(index_1)
  val jal_1     = jal(index_1)
  val jalr_1    = jalr(index_1)
  val con_br_1  = con_br(index_1)
  val dst_vld_1 = dst_vld(index_1)

  val vld_0 = PriorityMux(Seq(
    pcoper(7) -> true.B,
    pcoper(6) -> true.B,
    pcoper(5) -> true.B,
    pcoper(4) -> true.B,
    pcoper(3) -> true.B,
    pcoper(2) -> true.B,
    pcoper(1) -> true.B,
    pcoper(0) -> true.B,
    true.B    -> false.B
  ))

  val vld_1 = PriorityMux(Seq(
    pcoper(7) -> Mux(pcoper(6,0).orR,true.B,false.B),
    pcoper(6) -> Mux(pcoper(5,0).orR,true.B,false.B),
    pcoper(5) -> Mux(pcoper(4,0).orR,true.B,false.B),
    pcoper(4) -> Mux(pcoper(3,0).orR,true.B,false.B),
    pcoper(3) -> Mux(pcoper(2,0).orR,true.B,false.B),
    pcoper(2) -> Mux(pcoper(1,0).orR,true.B,false.B),
    pcoper(1) -> Mux(pcoper(0),true.B,false.B),
    pcoper(0) -> false.B,
    true.B    -> false.B
  ))

  val over_mask = PriorityMux(Seq(
    (!vld_1) -> "b0000_0000".U,
    (index_1 === 7.U) -> "b0000_0000".U,
    (index_1 === 6.U) -> "b0000_0001".U,
    (index_1 === 5.U) -> "b0000_0011".U,
    (index_1 === 4.U) -> "b0000_0111".U,
    (index_1 === 3.U) -> "b0000_1111".U,
    (index_1 === 2.U) -> "b0001_1111".U,
    (index_1 === 1.U) -> "b0011_1111".U,
    true.B -> "b0000_0000".U
  ))

  //==========================================================
  //               Hn_target_pc generate
  //==========================================================
  //Only ind-br/preturn need target PC
  //IF preturn_vld, target_pc = ras_pc
  //ELSE            target_pc = ind_btb_pc-ind_br_offset
  //for timing consideration of BJU,we user target_pc-offset as target_pc
  //In EX1 of BJU ,it will compare target_pc with src1,instead using
  //src1+offset
  //pcfifo_ind_target_pc[PC_WIDTH-1:0]
  val pcfifo_ind_target_pc = Cat(io.ibctrl_pcfifo_if_ind_target_pc,0.U(1.W)) -
    Cat(Cat(Seq.fill(19)(io.ibdp_pcfifo_if_ind_br_offset(20))),io.ibdp_pcfifo_if_ind_br_offset(20,0))
  val hn_target_pc = Mux(io.ibctrl_pcfifo_if_ras_vld,Cat(io.ibctrl_pcfifo_if_ras_target_pc,0.U(1.W)),pcfifo_ind_target_pc)
  val inst_0_targetpc = hn_target_pc
  val inst_1_targetpc = hn_target_pc
  val bht_sel_result  = io.in.sel_res
  val bht_pre_result  = io.in.pre_res

  val inst0_vghr = io.in.vghr
  val inst1_vghr = Mux(con_br_0,Cat(io.in.vghr(20,0),0.U),io.in.vghr)

  val ind_btb_miss = io.in.ind_btb_miss

  //Inst 0
  //Inst 0 may from lbuf whenlbuf ACTIVE state
  //Otherwise it from IB data path
  io.out(0).en := io.fifo_create_vld && vld_0
  //Inst 1
  //Inst 1 can only from IB data path
  io.out(1).en := io.fifo_create_vld && vld_1

  io.pcfifo_if_ibctrl_more_than_two := PopCount(pcoper) > 2.U
  io.pcfifo_if_ibdp_over_mask := over_mask

  io.out(0).curPc  := cur_pc_0
  io.out(0).tarPc  := inst_0_targetpc
  io.out(0).dstVld := dst_vld_0
  io.out(0).jal    := jal_0
  io.out(0).jalr   := jalr_0
  //bht_pred_array_result
  //inst0 & inst1 has one con_br at most
  //thus share one bht_pred_array_result
  io.out(0).predStore.bhtPred := bht_pre_result(1)
  //bht_select_array_result
  //inst0 & inst1 has one con_br at most
  //thus share one bht_select_array_result
  io.out(0).predStore.chkIdx  := Cat(bht_pre_result(0),bht_sel_result,inst0_vghr)
  io.out(0).predStore.jmpMispred := ind_btb_miss

  io.out(1).curPc  := cur_pc_1
  io.out(1).tarPc  := inst_1_targetpc
  io.out(1).dstVld := dst_vld_1
  io.out(1).jal    := jal_1
  io.out(1).jalr   := jalr_1
  io.out(1).predStore.bhtPred := bht_pre_result(1)
  io.out(1).predStore.chkIdx  := Cat(bht_pre_result(0),bht_sel_result,inst1_vghr)
  io.out(1).predStore.jmpMispred := ind_btb_miss
}
