package Core.IFU
import Core.{Config, CoreBundle}
import chisel3._
import chisel3.util._

class IFU extends Module with Config {
  val io = IO(new IFUIO)
  val ibuf = Module(new IBuffer)
  val ubtb = Module(new uBTB)
  val btb  = Module(new BTB)
  val bht  = Module(new BHT)
  val ipstage = Module(new IPStage)
  val ibstage = Module(new IBStage)
  val ras = Module(new RAS)
  val ind_btb = Module(new indBTB)
  val icache = Module(new ICache(cacheNum=0, is_sim=true))
  val pcfifo = Module(new PCfifo)


  val backend_redirect = io.bru_redirect.valid
  val bru_redirect_pc = RegInit(0.U(VAddrBits.W))
  val bru_redirect_valid = RegInit(false.B)
  when(backend_redirect && icache.io.refill_sm_busy){
    bru_redirect_pc := io.bru_redirect.bits
    bru_redirect_valid := true.B
  }.elsewhen(bru_redirect_valid && !icache.io.refill_sm_busy){
    bru_redirect_valid := false.B
  }



  // val reg_update       = !ibstage.io.ibctrl_ipctrl_stall && (!io.tlb.tlb_miss && icache.io.cache_req.ready) ||
  //   backend_redirect || ibstage.io.ib_redirect.valid || ipstage.io.ip_redirect.valid && !icache.io.refill_sm_busy //&& ibuf.io.allowEnq has been replace in ibstage

  val reg_update = !ibstage.io.ibctrl_ipctrl_stall && ((!io.tlb.tlb_miss && icache.io.cache_req.ready) || ibstage.io.ib_redirect.valid || ipstage.io.ip_redirect.valid) && !icache.io.refill_sm_busy ||
  !icache.io.refill_sm_busy && (backend_redirect || bru_redirect_valid)

  //pc select
  val pc_gen = Module(new PCGen)
  //todo:add ct_ifu_vector, had to ifu
  pc_gen.io.had_ifu_pcload := false.B
  pc_gen.io.had_ifu_pc := DontCare
  pc_gen.io.vector_pcgen_pcload := false.B
  pc_gen.io.vector_pcgen_pc := DontCare
  pc_gen.io.rtu_ifu_chgflw_vld := io.rtu_ifu_chgflw_vld
  pc_gen.io.rtu_ifu_chgflw_pc := io.rtu_ifu_chgflw_pc
  //
  pc_gen.io.redirect(0).valid := ubtb.io.ubtb_resp.valid
  pc_gen.io.redirect(0).bits  := ubtb.io.ubtb_resp.bits.target_pc
  pc_gen.io.redirect(1) := ipstage.io.ip_redirect
  pc_gen.io.redirect(2) := ibstage.io.ib_redirect
  pc_gen.io.redirect(3).valid := io.bru_redirect.valid || bru_redirect_valid
  pc_gen.io.redirect(3).bits  := Mux(backend_redirect, io.bru_redirect.bits, bru_redirect_pc)
  pc_gen.io.continue := reg_update
  pc_gen.io.IbufAllowEnq := ibuf.io.allowEnq

  //IF stage
  val if_pc = pc_gen.io.pc
  //  val if_pc = RegInit(PcStart.U)//RegEnable(pc_gen.io.pc, reg_update)
  //  //val if_pc = RegEnable(pc_gen.io.pc,RegNext(reg_update))
  //  when{reg_update}{
  //    if_pc := pc_gen.io.pc
  //  }
  io.pc := if_pc
  val if_data_valid = !(pc_gen.io.redirect(1).valid || pc_gen.io.redirect(2).valid || pc_gen.io.redirect(3).valid)

  ubtb.io.pc := if_pc
  btb.io.pc  := if_pc
  bht.io.pc  := if_pc
  //ubtb ras forward
  ubtb.io.ras_pc := PriorityMux(Seq(
    (ipstage.io.out.valid && ipstage.io.out.bits.pret) -> ipstage.io.out.bits.push_pc,
    ras.io.ifu_update.iscall -> ras.io.ifu_update.target,
    true.B                   -> ras.io.target
  ))

  io.tlb.vaddr.valid := if_data_valid
  io.tlb.vaddr.bits  := pc_gen.io.pc
  //io.cache_req.valid := if_data_valid && io.tlb.paddr.valid && !io.tlb.tlb_miss
  //io.cache_req.bits.vaddr := if_pc
  //io.cache_req.bits.paddr := io.tlb.paddr.bits

  icache.io.cache_req.valid := if_data_valid && io.tlb.paddr.valid && !io.tlb.tlb_miss
  icache.io.cache_req.bits.vaddr := if_pc
  icache.io.cache_req.bits.paddr := if_pc//io.tlb.paddr.bits //assume that paddr==vaddr

  icache.io.cohreq := DontCare
  icache.io.cohresp.valid := false.B
  icache.io.cohresp.bits := DontCare

  //IP stage
  val if_vld  = RegEnable(if_data_valid, false.B, reg_update)
  val ip_pc   = RegEnable(pc_gen.io.pc, PcStart.U(VAddrBits.W), reg_update)
  val ip_ubtb = RegEnable(ubtb.io.ubtb_resp, 0.U.asTypeOf(ubtb.io.ubtb_resp), reg_update)
//  val if_vld  = RegNext(if_data_valid)
//  val ip_pc   = RegNext(if_pc)
//  val ip_ubtb = RegNext(ubtb.io.ubtb_resp)
  val ip_cache_reg = RegInit(0.U.asTypeOf(icache.io.cache_resp))
  val ip_cache_reg_valid = RegInit(false.B)
  when(!reg_update && RegNext(reg_update) && ipstage.io.out.valid && !icache.io.refill_sm_busy){
    ip_cache_reg_valid := true.B
    ip_cache_reg := icache.io.cache_resp
  }

  when(icache.io.refill_sm_busy || reg_update){
    ip_cache_reg_valid := false.B
  }

  ipstage.io.if_vld      := if_vld
  ipstage.io.pc          := ip_pc
  ipstage.io.ip_flush    := pc_gen.io.redirect(2).valid || pc_gen.io.redirect(3).valid
  ipstage.io.ubtb_resp   := ip_ubtb
  ipstage.io.btb_resp    := btb.io.btb_target
  ipstage.io.bht_resp    := bht.io.bht_resp //TODO:封装bht输出
  ipstage.io.icache_resp := Mux(ip_cache_reg_valid, ip_cache_reg , icache.io.cache_resp)
  //ipstage.io.icache_resp := RegNext(io.cache_resp)

  bht.io.ip_bht_con_br_vld   := ipstage.io.ip_bht_con_br_vld
  bht.io.ip_bht_con_br_taken := ipstage.io.ip_bht_con_br_taken

  //IB stage
  val ip_vld = RegEnable(ipstage.io.out.valid, false.B, reg_update)
  val ip_out = RegEnable(ipstage.io.out.bits, 0.U.asTypeOf(ipstage.io.out.bits), reg_update) //ubtb,btb,bht
  val ib_pc  = ip_out.pc
  val ib_vld = ip_vld && !pc_gen.io.redirect(3).valid && !icache.io.refill_sm_busy



  ibstage.io.pc_oper_over_mask := pcfifo.io.pcfifo_if_ibdp_over_mask
  ibstage.io.pcgen_ibctrl_cancel := pc_gen.io.ibctrl_cancel
  ibstage.io.iu_ifu_mispred_stall := io.iu_ifu_mispred_stall
  ibstage.io.ibuf_ibctrl_stall := !ibuf.io.allowEnq
  ibstage.io.pcfifo_if_ibctrl_more_than_two := pcfifo.io.pcfifo_if_ibctrl_more_than_two
  ibstage.io.iu_ifu_pcfifo_full := io.iu_ifu_pcfifo_full
  ibstage.io.idu_ifu_id_stall := io.idu_ifu_id_stall
  ibstage.io.ip2ib.valid := ib_vld
  ibstage.io.ip2ib.bits  := RegEnable(ipstage.io.out.bits, 0.U.asTypeOf(ipstage.io.out.bits), reg_update)

  //ras
  ras.io.ifu_update.target := ip_out.push_pc
  ras.io.ifu_update.isret  := ip_out.pret && ib_vld//ip changeflow && pc mask && call return
  ras.io.ifu_update.iscall := ip_out.pcall && ib_vld
  ibstage.io.ras_target_pc := ras.io.target
  //ind_btb
  ind_btb.io.bht_ghr := bht.io.bht_ghr
  ind_btb.io.rtu_ghr := bht.io.rtu_ghr
  ind_btb.io.ind_btb_path := ip_out.pc(7,0)
  ind_btb.io.ib_jmp_valid := ibstage.io.ind_jmp_valid && ib_vld
  ibstage.io.ind_btb_target := ind_btb.io.ind_btb_target

  //BPU update signal
  //ubtb btb update
  ubtb.io.update_data   := ibstage.io.ubtb_update_data
  ubtb.io.update_idx    := ibstage.io.ubtb_update_idx
  btb.io.btb_update     := ibstage.io.btb_update
  btb.io.ib_btb_mispred := ibstage.io.ib_redirect.valid

  //backend bpu update
  bht.io.rtu_ifu_flush           := io.bpu_update.rtu_flush
  bht.io.rtu_retire_condbr       := io.bpu_update.rtu_retire_condbr
  bht.io.rtu_retire_condbr_taken := io.bpu_update.rtu_retire_condbr_taken
  bht.io.bht_update              := io.bpu_update.bht_update

  ras.io.rtu_update := io.bpu_update.rtu_ras_update
  ras.io.flush      := io.bpu_update.rtu_flush && !io.bpu_update.rtu_ras_update.isret
  ras.io.ras_flush  := io.bpu_update.rtu_flush && io.bpu_update.rtu_ras_update.isret

  ind_btb.io.commit_jmp_path := io.bpu_update.ind_btb_commit_jmp_path
  ind_btb.io.rtu_jmp_mispred := io.bpu_update.ind_btb_rtu_jmp_mispred
  ind_btb.io.rtu_jmp_pc      := io.bpu_update.ind_btb_rtu_jmp_pc
  ind_btb.io.rtu_flush       := io.bpu_update.rtu_flush

//  //addrgen
//  val addrgen = Module(new AddrGen)
//  addrgen.io.in := ibstage.io.ib2addrgen
//  ubtb.io.update_data := addrgen.io.ubtb_update
//  btb.io.btb_update := addrgen.io.btb_update

  //inst ibuf
  for(i <- 0 to 7){
    ibuf.io.in(i+1).bits.pc := Cat(ibstage.io.ip2ib.bits.pc(38,4), 0.U(4.W)) + (i.U << 1.U)
    ibuf.io.in(i+1).bits.data := ip_out.icache_resp.inst_data(i)
    ibuf.io.in(i+1).bits.is_inst32 := ip_out.inst_32_9(i+1)
    ibuf.io.in(i+1).valid := ip_out.chgflw_vld_mask(i+1) && ib_vld && reg_update
  }
  ibuf.io.in(0).bits.pc := Cat(ibstage.io.ip2ib.bits.pc(38,4), 0.U(4.W)) - 2.U
  ibuf.io.in(0).bits.data := ip_out.h0_data
  ibuf.io.in(0).bits.is_inst32 := ip_out.inst_32_9(0)
  ibuf.io.in(0).valid := ip_out.h0_vld && ib_vld && reg_update//ip_out.bits.chgflw_vld_mask(0)
  ibuf.io.idu_ifu_id_stall := io.idu_ifu_id_stall

  for(i <- 0 to 2){
    io.instData(i).vl_pred      := false.B
    io.instData(i).vl           := 0.U
    io.instData(i).pc           := ibuf.io.out(i).bits.pc(15,1)
    io.instData(i).vsew         := 0.U
    io.instData(i).vlmul        := 0.U
    io.instData(i).no_spec      := false.B
    io.instData(i).bkptb_inst   := false.B
    io.instData(i).bkpta_inst   := false.B
    val inst = ibuf.io.out(i).bits.inst
    io.instData(i).split_short  := inst(11,7) =/= 0.U &&
      (Cat(inst(15,12),inst(6,0)) === "b1001_0000010".U || inst(6,0) === "b1101111".U || Cat(inst(14,12),inst(6,0)) === "b000_1100111".U)
    io.instData(i).fence        := false.B
    io.instData(i).split_long   := false.B
    io.instData(i).high_hw_expt := false.B
    io.instData(i).expt_vec     := 0.U
    io.instData(i).expt_vld     := false.B
    io.instData(i).opcode       := ibuf.io.out(i).bits.inst
//    io.instData(i)             := ibuf.io.out(i).valid
//    ibuf.io.out(i).ready             := true.B
    io.instVld(i)                    := ibuf.io.out(i).valid
//    io.ifu_idu(i).ready             := ibuf.io.out(i).ready
  }
  io.toROB.curPcLoad := pc_gen.io.ifu_rtu_cur_pc_load
  io.toROB.curPc := pc_gen.io.ifu_rtu_cur_pc//0.U    //from had???
  ibuf.io.flush := backend_redirect

  pcfifo.io.fifo_create_vld := ibstage.io.fifo_create_vld
  pcfifo.io.ibdp_pcfifo_if_ind_br_offset := ibstage.io.ibdp_pcfifo_if_ind_br_offset
  pcfifo.io.ibctrl_pcfifo_if_ras_vld := ibstage.io.ibctrl_pcfifo_if_ras_vld
  pcfifo.io.ibctrl_pcfifo_if_ras_target_pc := ibstage.io.ibctrl_pcfifo_if_ras_target_pc
  pcfifo.io.ibctrl_pcfifo_if_ind_target_pc := ibstage.io.ibctrl_pcfifo_if_ind_target_pc
  pcfifo.io.in.h0_vld    := ip_out.h0_vld
  pcfifo.io.in.cur_pc(0) := ip_out.h0_pc ////todo: check it
  for(i <- 1 to 8) {
    pcfifo.io.in.cur_pc(i) := ip_out.cur_pc(i-1)
  }
  pcfifo.io.in.pc_oper   := ibstage.io.ibdp_pcfifo_if_hn_pc_oper //////Wire from ip_out, same cycle with ibstage
  pcfifo.io.in.jal       := ip_out.jal
  pcfifo.io.in.jalr      := ip_out.jalr
  pcfifo.io.in.con_br    := ip_out.con_br
  pcfifo.io.in.dst_vld   := ip_out.dst
  pcfifo.io.in.sel_res   := ip_out.bht_resp.pre_sel
  pcfifo.io.in.pre_res   := ip_out.bht_res
  pcfifo.io.in.vghr      := bht.io.bht_ghr //////has RegNext in bht, same cycle with ibstage
  pcfifo.io.in.ind_btb_miss := false.B //////todo: figure out

  for(i <- 0 to 1){
    io.ifuForward(i) := pcfifo.io.out(i)
  }
}
