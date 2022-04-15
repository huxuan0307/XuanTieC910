package Core.IFU_IDU

import Core.AddrConfig.PcWidth
import Core.Config
import chisel3._
import chisel3.util._
import Core.IDU._
import Core.IFU._
import Core.RTU._

class ct_coreBundle extends Bundle {
  val RtutoIfu = new RetireToIfuBundle
}

class ct_core extends Module with Config{
  val ifu = Module(new IFU)
  val idu = Module(new IDStage)
  val rtu = Module(new RtuTop)

  //IFU
  ifu.io.bpu_update.rtu_flush := rtu.io.out.toIfu.flush
  ifu.io.bpu_update.rtu_ras_update.isret := rtu.io.out.toIfu.retire0.pReturn
  ifu.io.bpu_update.rtu_ras_update.iscall := rtu.io.out.toIfu.retire0.pCall
  ifu.io.bpu_update.rtu_ras_update.target := rtu.io.out.toIfu.retire0.incPc
  ifu.io.bpu_update.rtu_retire_condbr(0) := rtu.io.out.toIfu.retireVec(0).condBr
  ifu.io.bpu_update.rtu_retire_condbr(1) := rtu.io.out.toIfu.retireVec(1).condBr
  ifu.io.bpu_update.rtu_retire_condbr(2) := rtu.io.out.toIfu.retireVec(2).condBr
  ifu.io.bpu_update.rtu_retire_condbr_taken(0) := rtu.io.out.toIfu.retireVec(0).condBrTaken
  ifu.io.bpu_update.rtu_retire_condbr_taken(1) := rtu.io.out.toIfu.retireVec(1).condBrTaken
  ifu.io.bpu_update.rtu_retire_condbr_taken(2) := rtu.io.out.toIfu.retireVec(2).condBrTaken

  //IFU ignore other signals
  ifu.io.bpu_update.ind_btb_commit_jmp_path := DontCare
  ifu.io.bpu_update.ind_btb_rtu_jmp_pc := DontCare
  ifu.io.bpu_update.ind_btb_rtu_jmp_mispred := DontCare
  ifu.io.bpu_update.bht_update := DontCare //from BJU??
  ifu.io.ifuForward := DontCare // from BJU??
  ifu.io.bru_redirect.valid := false.B //from BJU
  ifu.io.bru_redirect.bits := DontCare
  ifu.io.tlb.tlb_miss := false.B
  ifu.io.tlb.paddr := ifu.io.tlb.vaddr // todo: add TLB, Now, suppose that paddr===vaddr
  ifu.io.instVld := Seq(true.B,true.B,true.B) //todo: ???
  dontTouch(ifu.io)

  //IDU
  idu.io.in.fromIFU.instData := ifu.io.instData
  idu.io.in.fromIFU.instVld  := Seq(true.B,true.B,true.B) //todo: ???
  idu.io.in.fromRTU.flushFe := rtu.io.out.toIdu.flushFe
  idu.io.in.fromIFU.pipedownGateclk := DontCare  //todo: figure out Gateclk

  //IDU ignore other signals
  idu.io.in.fromCp0 := DontCare
  idu.io.in.fromIU := DontCare
  idu.io.in.fromHad := DontCare
  idu.io.in.fromPad := DontCare
  idu.io.in.fromFence := DontCare
  idu.io.in.fromHpcp := DontCare
  idu.io.in.fromIR := DontCare



  //RTU
  rtu.io.in.fromIfu.curPcLoad := ifu.io.toROB.curPcLoad
  rtu.io.in.fromIfu.curPc := ifu.io.toROB.curPc
  rtu.io.in.fromIfu.xxSyncReset := false.B //todo: add xxSyncReset

  //RTU ignore other signals
  rtu.io.in.fromIdu := DontCare
  rtu.io.in.fromLsu := DontCare
  rtu.io.in.fromIu := DontCare
  rtu.io.in.fromCp0 := DontCare
  rtu.io.in.fromPad := DontCare
  rtu.io.in.fromHad := DontCare
  rtu.io.in.fromHpcp := DontCare
  rtu.io.in.fromMmu := DontCare
  rtu.io.in.fromVfpu := DontCare
}