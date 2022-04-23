package Core.IFU_IDU

import Core.AddrConfig.PcWidth
import Core.{Config, ROBConfig}
import chisel3._
import chisel3.util._
import Core.IDU._
import Core.IU._
import Core.IFU._
import Core.RTU._

class ct_coreBundle extends Bundle {
  val RtutoIfu = new RetireToIfuBundle
}

class ct_core extends Module with Config with ROBConfig {
  val ifu = Module(new IFU)
  val idu = Module(new IDU)
  val iu = Module(new IntegeUnit)
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
  idu.io.in.IDfromIFUIB.instData := ifu.io.instData
  idu.io.in.IDfromIFUIB.instVld := ifu.io.instVld
  idu.io.in.IDfromIFUIB.pipedownGateclk := DontCare  //todo: figure out Gateclk
  idu.io.in.fromIU.yyxxCancel := iu.io.bjuToIdu.cancel
  idu.io.in.fromIU.mispred_stall := iu.io.bjuToIdu.misPredStall
  idu.io.in.fromRTU.flush_fe := rtu.io.out.toIdu.flushFe
  idu.io.in.fromRTU.flush_is := rtu.io.out.toIdu.flushIs
  idu.io.in.fromRTU.flush_stall := rtu.io.out.toIdu.flushStall
  idu.io.in.fromRTU.freg := rtu.io.out.toIdu.fregAlloc.map(_.bits)
  idu.io.in.fromRTU.preg := rtu.io.out.toIdu.pregAlloc.map(_.bits)
  idu.io.in.fromRTU.ereg := rtu.io.out.toIdu.eregAlloc.map(_.bits)
  idu.io.in.fromRTU.vreg := rtu.io.out.toIdu.vregAlloc.map(_.bits)
  idu.io.in.fromRTU.freg_vld := rtu.io.out.toIdu.fregAlloc.map(_.valid)
  idu.io.in.fromRTU.preg_vld := rtu.io.out.toIdu.pregAlloc.map(_.valid)
  idu.io.in.fromRTU.ereg_vld := rtu.io.out.toIdu.eregAlloc.map(_.valid)
  idu.io.in.fromRTU.vreg_vld := rtu.io.out.toIdu.vregAlloc.map(_.valid)
  idu.io.in.fromRTU.srt_en := rtu.io.out.toIdu.srtEn
  idu.io.in.fromRTU.yy_xx_flush := rtu.io.out.yyXx.flush
  idu.io.in.RTfromRTUsub.rt_recover_preg := rtu.io.out.toIdu.rtRecoverPreg
  idu.io.in.ISfromRTUsub.rob_full := rtu.io.out.toIdu.robFull
  idu.io.in.ISfromRTUsub.rob_inst_idd := rtu.io.out.toIdu.robInstIidVec
  idu.io.in.ISfromRTUsub.retire_int_vld := rtu.io.out.toIdu.retire0InstValid

  //IDU ignore other signals
  idu.io.in.ifu_xx_sync_reset := false.B //////todo: ifu add signals
  idu.io.in.aiq0fromIU.div := DontCare //////todo: iu add signal
  idu.io.in.biqfromIU.div := DontCare //////todo: iu add signal
  idu.io.in.RTfromIU := DontCare //////todo: iu add signal
  idu.io.in.IDfromHad := DontCare
  idu.io.in.IDfromFence := DontCare
  idu.io.in.IRfromCp0Sub := DontCare
  idu.io.in.IQfromCp0sub := DontCare
  idu.io.in.RFfromIU.stall := DontCare //////todo: add signals
  idu.io.in.IQfromIUsub.wbPreg := DontCare //////todo: find out
  idu.io.in.RFfromHad := DontCare
  idu.io.in.RFfromVFPU := DontCare
  idu.io.in.RFfromCp0sub := DontCare
  idu.io.in.RTfromVFPU := DontCare
  idu.io.in.fromHpcp := DontCare
  idu.io.in.fromCp0 := DontCare
  idu.io.in.fromPad := DontCare
  idu.io.in.fromLSU := DontCare //////todo: add LSU
  idu.io.in.ISfromVFPU := DontCare
  idu.io.in.ISfromIUsub.pcfifo_dis_inst_pid := DontCare //////iu.io.bjuToIdu.allowPid todo: find out




  //IU
  for(i <- 0 to 1) {
    iu.io.ifuForward(i).curPc := ifu.io.ifuForward(i).curPc
    iu.io.ifuForward(i).tarPc := ifu.io.ifuForward(i).tarPc
    iu.io.ifuForward(i).dstVld := ifu.io.ifuForward(i).dstVld
    iu.io.ifuForward(i).predStore.bhtPred := ifu.io.ifuForward(i).predStore.bhtPred
    iu.io.ifuForward(i).predStore.chkIdx := ifu.io.ifuForward(i).predStore.chkIdx
    iu.io.ifuForward(i).predStore.jmpMispred := ifu.io.ifuForward(i).predStore.jmpMispred
    iu.io.ifuForward(i).predStore.pc := DontCare //////todo: find out
    iu.io.ifuForward(i).jalr := ifu.io.ifuForward(i).jalr
    iu.io.ifuForward(i).jal := ifu.io.ifuForward(i).jal
    iu.io.ifuForward(i).en := true.B //////todo: add signal
  }
  iu.io.rtuIn.flushFe := rtu.io.out.toIu.flushFe
  iu.io.rtuIn.flush := DontCare //////todo: find out
  iu.io.rtuIn.flushChgflwMask := rtu.io.out.toIu.flushChangeFlowMask
  iu.io.rtuIn.robReadPcfifovld := rtu.io.out.toIu.robReadPcFifoValid
  iu.io.rtuIn.robReadPcfifovldGateEn := rtu.io.out.toIu.robReadPcFifoGateClkValid
  iu.io.isIn.isuToFifo.instVld := idu.io.out.IStoIU.pcfifo_inst_vld
  iu.io.isIn.isuToFifo.instNum := idu.io.out.IStoIU.pcfifo_inst_num


  //IU ignore other signals
  iu.io.alu0Sel := DontCare //////todo: find out
  iu.io.alu1Sel := DontCare //////todo: find out
  iu.io.isIn.issue := DontCare //////todo: ISStage add idu_iu_is_div_issue
  iu.io.isIn.gateClkIssue := DontCare //////todo: idu_iu_is_div_gateclk_issue ???
  iu.io.specialPid := DontCare //////todo: check idu_iu_rf_pipe0_pid and idu_iu_rf_pipe1_pid ???
  iu.io.bjuSel := DontCare //////todo: from rf
  iu.io.mulSel := DontCare //////todo: from rf
  iu.io.specialSel := DontCare //////todo: from rf
  iu.io.divSel := DontCare //////todo: from rf
  iu.io.pipe0 := DontCare //////todo: from rf?
  iu.io.pipe1 := DontCare //////todo: from rf?
  iu.io.pipe2 := DontCare //////todo: from rf
  iu.io.cp0In := DontCare



  //RTU
  rtu.io.in.fromIfu.curPcLoad := ifu.io.toROB.curPcLoad
  rtu.io.in.fromIfu.curPc := ifu.io.toROB.curPc
  rtu.io.in.fromIfu.xxSyncReset := false.B //todo: add xxSyncReset

  rtu.io.in.fromIdu.fromIr.preg.allocValidVec := idu.io.out.IRtoRTU.preg_alloc_vld
  rtu.io.in.fromIdu.fromIr.preg.allocGateClkValid := idu.io.out.IRtoRTU.preg_alloc_gateclk_vld
  rtu.io.in.fromIdu.fromIr.freg.allocValidVec := idu.io.out.IRtoRTU.freg_alloc_vld
  rtu.io.in.fromIdu.fromIr.freg.allocGateClkValid := idu.io.out.IRtoRTU.freg_alloc_gateclk_vld
  rtu.io.in.fromIdu.fromIr.ereg.allocValidVec := idu.io.out.IRtoRTU.ereg_alloc_vld
  rtu.io.in.fromIdu.fromIr.ereg.allocGateClkValid := idu.io.out.IRtoRTU.ereg_alloc_gateclk_vld
  rtu.io.in.fromIdu.fromIr.vreg.allocValidVec := idu.io.out.IRtoRTU.vreg_alloc_vld
  rtu.io.in.fromIdu.fromIr.vreg.allocGateClkValid := idu.io.out.IRtoRTU.vreg_alloc_gateclk_vld
  for(i <- 0 to (NumCreateEntry-1)) {
    rtu.io.in.fromIdu.robCreate(i).en := idu.io.out.IStoRTU.rob_create(i).en
    rtu.io.in.fromIdu.robCreate(i).data.data.noSpec.hit := idu.io.out.IStoRTU.rob_create(i).data.NO_SPEC_HIT
    rtu.io.in.fromIdu.robCreate(i).data.data.noSpec.miss := idu.io.out.IStoRTU.rob_create(i).data.NO_SPEC_MISS
    rtu.io.in.fromIdu.robCreate(i).data.data.noSpec.mispred := idu.io.out.IStoRTU.rob_create(i).data.NO_SPEC_MISPRED
    rtu.io.in.fromIdu.robCreate(i).data.data.breakpointData.a := idu.io.out.IStoRTU.rob_create(i).data.BKPTA_DATA
    rtu.io.in.fromIdu.robCreate(i).data.data.breakpointData.b := idu.io.out.IStoRTU.rob_create(i).data.BKPTB_DATA
    rtu.io.in.fromIdu.robCreate(i).data.data.breakpointInst.a := idu.io.out.IStoRTU.rob_create(i).data.BKPTA_INST
    rtu.io.in.fromIdu.robCreate(i).data.data.breakpointInst.b := idu.io.out.IStoRTU.rob_create(i).data.BKPTB_INST
    rtu.io.in.fromIdu.robCreate(i).data.data.bju := idu.io.out.IStoRTU.rob_create(i).data.BJU
    rtu.io.in.fromIdu.robCreate(i).data.data.fpDirty := idu.io.out.IStoRTU.rob_create(i).data.FP_DIRTY
    rtu.io.in.fromIdu.robCreate(i).data.data.instNum := idu.io.out.IStoRTU.rob_create(i).data.INST_NUM
    rtu.io.in.fromIdu.robCreate(i).data.data.intMask := idu.io.out.IStoRTU.rob_create(i).data.INTMASK
    rtu.io.in.fromIdu.robCreate(i).data.data.load := idu.io.out.IStoRTU.rob_create(i).data.LOAD
    rtu.io.in.fromIdu.robCreate(i).data.data.pcFifo := idu.io.out.IStoRTU.rob_create(i).data.PCFIFO
    rtu.io.in.fromIdu.robCreate(i).data.data.pcOffset := idu.io.out.IStoRTU.rob_create(i).data.PC_OFFSET
    rtu.io.in.fromIdu.robCreate(i).data.data.ras := idu.io.out.IStoRTU.rob_create(i).data.RAS
    rtu.io.in.fromIdu.robCreate(i).data.data.split := idu.io.out.IStoRTU.rob_create(i).data.SPLIT
    rtu.io.in.fromIdu.robCreate(i).data.data.store := idu.io.out.IStoRTU.rob_create(i).data.STORE
    rtu.io.in.fromIdu.robCreate(i).data.data.vecDirty := idu.io.out.IStoRTU.rob_create(i).data.VEC_DIRTY
    rtu.io.in.fromIdu.robCreate(i).data.data.vl := idu.io.out.IStoRTU.rob_create(i).data.VL
    rtu.io.in.fromIdu.robCreate(i).data.data.vlmul := idu.io.out.IStoRTU.rob_create(i).data.VLMUL
    rtu.io.in.fromIdu.robCreate(i).data.data.vlPred := idu.io.out.IStoRTU.rob_create(i).data.VL_PRED
    rtu.io.in.fromIdu.robCreate(i).data.data.vsetvli := idu.io.out.IStoRTU.rob_create(i).data.VSETVLI
    rtu.io.in.fromIdu.robCreate(i).data.data.vsew := idu.io.out.IStoRTU.rob_create(i).data.VSEW
    rtu.io.in.fromIdu.robCreate(i).data.ctrl.valid := idu.io.out.IStoRTU.rob_create(i).data.VLD //////todo: check it
    rtu.io.in.fromIdu.robCreate(i).data.ctrl.cmpltCnt := idu.io.out.IStoRTU.rob_create(i).data.CMPLT_CNT
    rtu.io.in.fromIdu.robCreate(i).data.ctrl.cmpltValid := idu.io.out.IStoRTU.rob_create(i).data.CMPLT //////todo: check it
    rtu.io.in.fromIdu.robCreate(i).dpEn := idu.io.out.IStoRTU.rob_create(i).dp_en
    rtu.io.in.fromIdu.robCreate(i).gateClkEn := idu.io.out.IStoRTU.rob_create(i).gateclk_en
  }
  for(i <- 0 to (NumCreateEntry-1)) {
    //////todo:check it
    rtu.io.in.fromIdu.toPst.preg(i).preg := idu.io.out.IStoRTU.pst_dis(i).preg
    rtu.io.in.fromIdu.toPst.preg(i).pregValid := idu.io.out.IStoRTU.pst_dis(i).preg_vld
    rtu.io.in.fromIdu.toPst.preg(i).relPreg := idu.io.out.IStoRTU.pst_dis(i).rel_preg
    rtu.io.in.fromIdu.toPst.preg(i).iid := idu.io.out.IStoRTU.pst_dis(i).preg_iid
    rtu.io.in.fromIdu.toPst.preg(i).dstReg := idu.io.out.IStoRTU.pst_dis(i).dst_reg
    rtu.io.in.fromIdu.toPst.ereg(i).ereg := idu.io.out.IStoRTU.pst_dis(i).ereg
    rtu.io.in.fromIdu.toPst.ereg(i).eregValid := idu.io.out.IStoRTU.pst_dis(i).ereg_vld
    rtu.io.in.fromIdu.toPst.ereg(i).relEreg := idu.io.out.IStoRTU.pst_dis(i).rel_ereg
    rtu.io.in.fromIdu.toPst.ereg(i).iid := idu.io.out.IStoRTU.pst_dis(i).ereg_iid
    rtu.io.in.fromIdu.toPst.vfreg(i).preg := idu.io.out.IStoRTU.pst_dis(i).vreg //////todo: check it
    rtu.io.in.fromIdu.toPst.vfreg(i).vregValid := idu.io.out.IStoRTU.pst_dis(i).vreg_vld
    rtu.io.in.fromIdu.toPst.vfreg(i).fregValid := idu.io.out.IStoRTU.pst_dis(i).freg_vld
    rtu.io.in.fromIdu.toPst.vfreg(i).relPreg := idu.io.out.IStoRTU.pst_dis(i).rel_vreg
    rtu.io.in.fromIdu.toPst.vfreg(i).dstReg := idu.io.out.IStoRTU.pst_dis(i).dstv_reg
    rtu.io.in.fromIdu.toPst.vfreg(i).iid := idu.io.out.IStoRTU.pst_dis(i).vreg_iid
  }

  //RTU ignore other signals
  rtu.io.in.fromIdu.toPst.pregDeallocMaskOH := DontCare //////todo: find out, idu.io.out.sdiq.....
  rtu.io.in.fromIdu.toPst.fregDeallocMaskOH := DontCare
  rtu.io.in.fromIdu.toPst.vregDeallocMaskOH := DontCare
  rtu.io.in.fromIdu.fenceIdle := DontCare //////todo: find out
  rtu.io.in.fromLsu := DontCare
  rtu.io.in.fromIu := DontCare
  rtu.io.in.fromCp0 := DontCare
  rtu.io.in.fromPad := DontCare
  rtu.io.in.fromHad := DontCare
  rtu.io.in.fromHpcp := DontCare
  rtu.io.in.fromMmu := DontCare
  rtu.io.in.fromVfpu := DontCare
}