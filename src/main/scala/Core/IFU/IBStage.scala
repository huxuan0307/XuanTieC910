package Core.IFU
import Utils.UIntToMask
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

class IBStage extends Module with Config {
  val io = IO(new IBStageIO)

  val ib_data_valid = io.ip2ib.valid

  //check ubtb btb
  val ubtb_br_miss = io.ip2ib.bits.ubtb_miss
  val ubtb_br_mispred = io.ip2ib.bits.ubtb_mispred

  val ubtb_ras_pc_hit = io.ras_target_pc === io.ip2ib.bits.ubtb_resp.target_pc
  val ubtb_ras_mistaken = io.ip2ib.bits.ubtb_valid && io.ip2ib.bits.ubtb_resp.is_ret && !io.ip2ib.bits.pret   //is not return,but ubtb predict it is return,redirect in ip stage
  val ubtb_ras_miss     = !io.ip2ib.bits.ubtb_valid || !io.ip2ib.bits.ubtb_resp.is_ret && io.ip2ib.bits.pret
  val ubtb_ras_mispred  = io.ip2ib.bits.ubtb_valid && io.ip2ib.bits.ubtb_resp.is_ret && io.ip2ib.bits.pret && !ubtb_ras_pc_hit //is return,ubtb is return
  val ubtb_ras_hit      = io.ip2ib.bits.ubtb_valid && io.ip2ib.bits.ubtb_resp.is_ret && io.ip2ib.bits.pret && ubtb_ras_pc_hit

  val br_target1_8 = Cat(io.ip2ib.bits.pc(VAddrBits-1, 4), 0.U(4.W)) + io.ip2ib.bits.br_offset + (io.ip2ib.bits.br_position << 1.U)
  val br_target_0  = Cat(io.ip2ib.bits.pc(VAddrBits-1, 4), 0.U(4.W)) + io.ip2ib.bits.br_offset - 2.U
  val br_target    = Mux(io.ip2ib.bits.h0_vld && io.ip2ib.bits.br_position === 0.U, br_target_0, br_target1_8)
  val btb_miss    = io.ip2ib.bits.btb_miss
  val btb_mispred = io.ip2ib.bits.br_valid && io.ip2ib.bits.btb_valid && br_target(20,1) =/= io.ip2ib.bits.btb_target



  //redirect         icache predecode   ip predecode  ip predecode      ip predecode      ip predecode
  io.ib_redirect.valid := ib_data_valid && (btb_miss || btb_mispred || ubtb_ras_mispred || ubtb_ras_miss || io.ip2ib.bits.ind_vld)
  val ind_btb_pc = Cat(io.ip2ib.bits.pc(VAddrBits-1,21),io.ind_btb_target,0.U(1.W))
  //val br_target  = Cat(io.ip2ib.bits.pc(VAddrBits-1,21),io.ip2ib.bits.br_offset)
  io.ib_redirect.bits  := Mux(io.ip2ib.bits.ind_vld, ind_btb_pc, Mux(ubtb_ras_mispred || ubtb_ras_miss, io.ras_target_pc, br_target))

  //btb update
  io.btb_update.valid := (btb_miss || btb_mispred) && ib_data_valid
  io.btb_update.bits.btb_data  := br_target(20,1)
  io.btb_update.bits.btb_index := io.ip2ib.bits.pc(13,4)
  io.btb_update.bits.btb_tag   := Cat(io.ip2ib.bits.pc(20,14), io.ip2ib.bits.pc(3,1))

  //ubtb update
  val bht_res = io.ip2ib.bits.bht_res
  val ubtb_br_miss_update = ubtb_br_miss && !btb_miss && (io.ip2ib.bits.is_ab_br || bht_res === 3.U)
  io.ubtb_update_data.valid  := ib_data_valid && (ubtb_br_miss_update || ubtb_br_mispred || ubtb_ras_mistaken || ubtb_ras_miss || ubtb_ras_mispred)
  io.ubtb_update_data.bits.tag := io.ip2ib.bits.pc(15,1)
  io.ubtb_update_data.bits.target := io.ip2ib.bits.btb_target
  io.ubtb_update_data.bits.ras := io.ip2ib.bits.pret
  io.ubtb_update_data.bits.entry_valid := !ubtb_ras_mistaken
  io.ubtb_update_idx.valid   := ib_data_valid && (ubtb_br_mispred || ubtb_ras_mistaken || ubtb_ras_mispred)
  io.ubtb_update_idx.bits    := io.ip2ib.bits.ubtb_resp.hit_index

  io.ind_jmp_valid := ib_data_valid && io.ip2ib.bits.ind_vld   //todo

}
