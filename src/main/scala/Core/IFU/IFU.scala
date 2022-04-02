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



  val ifu_continue     = RegInit(true.B)
  ifu_continue := !io.tlb.tlb_miss && icache.io.cache_resp.valid && ibuf.io.allowEnq
  val backend_redirect = io.bru_redirect.valid
  val reg_update       = ifu_continue || backend_redirect

  //pc select
  val pc_gen = Module(new PCGen)
  pc_gen.io.redirect(0).valid := ubtb.io.ubtb_resp.valid
  pc_gen.io.redirect(0).bits  := ubtb.io.ubtb_resp.bits.target_pc
  pc_gen.io.redirect(1) := ipstage.io.ip_redirect
  pc_gen.io.redirect(2) := ibstage.io.ib_redirect
  pc_gen.io.redirect(3) := io.bru_redirect
  pc_gen.io.continue := !io.tlb.tlb_miss && icache.io.cache_resp.valid && ibuf.io.allowEnq

  //IF stage
  val if_pc = pc_gen.io.pc
  //  val if_pc = RegInit(PcStart.U)//RegEnable(pc_gen.io.pc, reg_update)
  //  //val if_pc = RegEnable(pc_gen.io.pc,RegNext(reg_update))
  //  when{reg_update}{
  //    if_pc := pc_gen.io.pc
  //  }
  io.pc := if_pc
  val if_data_valid = !pc_gen.io.redirect(1).valid || !pc_gen.io.redirect(2).valid || !pc_gen.io.redirect(3).valid


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
  val if_vld  = RegEnable(if_data_valid, reg_update)
  val ip_pc   = RegNext(RegEnable(pc_gen.io.pc, reg_update))
  val ip_ubtb = RegEnable(ubtb.io.ubtb_resp, reg_update)
//  val if_vld  = RegNext(if_data_valid)
//  val ip_pc   = RegNext(if_pc)
//  val ip_ubtb = RegNext(ubtb.io.ubtb_resp)

  ipstage.io.if_vld      := if_vld
  ipstage.io.pc          := ip_pc
  ipstage.io.ip_flush    := pc_gen.io.redirect(2).valid || pc_gen.io.redirect(3).valid
  ipstage.io.ubtb_resp   := ip_ubtb
  ipstage.io.btb_resp    := btb.io.btb_target
  ipstage.io.bht_resp    := bht.io.bht_resp //TODO:封装bht输出
  ipstage.io.icache_resp := RegEnable(icache.io.cache_resp,true.B)
  //ipstage.io.icache_resp := RegNext(io.cache_resp)

  bht.io.ip_bht_con_br_vld   := ipstage.io.ip_bht_con_br_vld
  bht.io.ip_bht_con_br_taken := ipstage.io.ip_bht_con_br_taken

  //IB stage
  val ip_vld = RegNext(ipstage.io.out.valid)
  val ip_out = RegEnable(ipstage.io.out.bits, reg_update) //ubtb,btb,bht
//  val ib_pc  = ip_out.pc
  val ib_vld = ip_vld && !pc_gen.io.redirect(3).valid



  ibstage.io.ip2ib.valid := ib_vld
  ibstage.io.ip2ib.bits  := RegNext(ipstage.io.out.bits)

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
    ibuf.io.in(i+1).valid := ip_out.chgflw_vld_mask(i+1) && ib_vld
  }
  ibuf.io.in(0).bits.pc := Cat(ibstage.io.ip2ib.bits.pc(38,4), 0.U(4.W)) - 2.U
  ibuf.io.in(0).bits.data := ip_out.h0_data
  ibuf.io.in(0).bits.is_inst32 := ip_out.inst_32_9(0)
  ibuf.io.in(0).valid := ip_out.h0_vld && ib_vld//ip_out.bits.chgflw_vld_mask(0)

  ibuf.io.out <> io.ifu_inst_out

  ibuf.io.flush := backend_redirect
}
