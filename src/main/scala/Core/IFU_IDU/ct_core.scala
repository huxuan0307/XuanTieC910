package Core.IFU_IDU

import Core.AddrConfig.PcWidth
import Core.{Config, ROBConfig}
import chisel3._
import chisel3.util._
import Core.IDU._
import Core.IU._
import Core.IFU._
import Core.RTU._
import difftest._
import firrtl.transforms.DontTouchAnnotation

class ct_coreBundle extends Bundle {
  val logCtrl = new LogCtrlIO
  val perfInfo = new PerfInfoIO
  val uart = new UARTIO
}

class SimTop extends Module with Config with ROBConfig {
  val io = IO(new ct_coreBundle)
  val ifu = Module(new IFU)
  val idu = Module(new IDU)
  val iu = Module(new IntegeUnit)
  val rtu = Module(new RtuTop)

  io.uart.in.valid  := false.B
  io.uart.out.valid := false.B
  io.uart.out.ch    := 0.U
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
  ifu.io.rtu_ifu_chgflw_vld := rtu.io.out.toIfu.changeFlowValid
  ifu.io.rtu_ifu_chgflw_pc := rtu.io.out.toIfu.changeFlowPc
  ifu.io.bru_redirect.valid := iu.io.bjuToIfu.chgflwVld
  ifu.io.bru_redirect.bits := iu.io.bjuToIfu.tarPc// Cat(iu.io.bjuToIfu.tarPc,0.U(1.W)) + 4.U //////todo: replace it with other way
  ifu.io.idu_ifu_id_stall := idu.io.out.IDtoIFU.stall
  ifu.io.iu_ifu_mispred_stall := iu.io.bjuToIfu.misPredStall
  ifu.io.iu_ifu_pcfifo_full := iu.io.bjuToIfu.pcFifoFull

  //IFU ignore other signals
  ifu.io.bpu_update.ind_btb_commit_jmp_path := DontCare
  ifu.io.bpu_update.ind_btb_rtu_jmp_pc := DontCare
  ifu.io.bpu_update.ind_btb_rtu_jmp_mispred := DontCare
  ifu.io.bpu_update.bht_update := DontCare //from BJU??
  ifu.io.tlb.tlb_miss := false.B
  ifu.io.tlb.paddr := ifu.io.tlb.vaddr // todo: add TLB, Now, suppose that paddr===vaddr
  //ifu.io.instVld := Seq(true.B,true.B,true.B).map(_) //todo: ???
  dontTouch(ifu.io)





  //IDU
  idu.io.in.IDfromIFUIB.instData := ifu.io.instData
  idu.io.in.IDfromIFUIB.instVld := ifu.io.instVld
  idu.io.in.IDfromIFUIB.pipedownGateclk := DontCare  //todo: figure out Gateclk
  idu.io.in.fromIU.yyxxCancel := iu.io.bjuToIdu.yyXxCancel
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
  idu.io.in.PRFfromIU.ex2_pipe0_wb_preg := iu.io.iuToRtu.rbusRslt(0).wbPreg //////todo: check it, seem that iu_rtu_ex2_pipe0_wb_preg_expand same with iu_idu_ex2_pipe0_wb_preg_expand
  idu.io.in.PRFfromIU.ex2_pipe0_wb_preg_vld := iu.io.iuToRtu.rbusRslt(0).wbPregVld
  idu.io.in.PRFfromIU.ex2_pipe0_wb_preg_data := iu.io.iuToRtu.rbusRslt(0).wbData
  idu.io.in.PRFfromIU.ex2_pipe1_wb_preg := iu.io.iuToRtu.rbusRslt(1).wbPreg
  idu.io.in.PRFfromIU.ex2_pipe1_wb_preg_vld := iu.io.iuToRtu.rbusRslt(1).wbPregVld
  idu.io.in.PRFfromIU.ex2_pipe1_wb_preg_data := iu.io.iuToRtu.rbusRslt(1).wbData
  idu.io.in.PRFfromIU.lsu_wb_pipe3_wb_preg := DontCare //////todo: from LSU??
  idu.io.in.PRFfromIU.lsu_wb_pipe3_wb_preg_vld := DontCare //////todo: from LSU??
  idu.io.in.PRFfromIU.lsu_wb_pipe3_wb_preg_data := DontCare //////todo: from LSU??
  idu.io.in.PRFfromRTUsub.yyXxDebugOn := rtu.io.out.yyXx.debugOn

  //IDU ignore other signals
  idu.io.in.ifu_xx_sync_reset := false.B //////todo: ifu add signals
  idu.io.in.aiq0fromIU.div := DontCare //////todo: iu add signal
  idu.io.in.biqfromIU.div := DontCare //////todo: iu add signal
  idu.io.in.RTfromIU := DontCare //////todo: iu add signal
  idu.io.in.RTfromIU.ex2_pipe0_wb_preg_dupx := iu.io.iuToRtu.rbusRslt(0).wbPreg //////todo: check it, and compare with PRFfromIU
  idu.io.in.RTfromIU.ex2_pipe0_wb_preg_vld_dupx := iu.io.iuToRtu.rbusRslt(0).wbPregVld
  idu.io.in.RTfromIU.ex2_pipe1_wb_preg_dupx := iu.io.iuToRtu.rbusRslt(1).wbPreg
  idu.io.in.RTfromIU.ex2_pipe1_wb_preg_vld_dupx := iu.io.iuToRtu.rbusRslt(1).wbPregVld
  idu.io.in.IDfromHad := DontCare
  idu.io.in.IDfromFence := DontCare
  idu.io.in.IRfromCp0Sub := DontCare
  idu.io.in.IQfromCp0sub := DontCare
  idu.io.in.RFfromIU.stall := DontCare //////todo: add signals
  idu.io.in.IQfromIUsub.wbPreg(0).bits := iu.io.iuToRtu.rbusRslt(0).wbPreg//DontCare //////todo: check it, and compare with PRFfromIU
  idu.io.in.IQfromIUsub.wbPreg(1).bits := iu.io.iuToRtu.rbusRslt(1).wbPreg
  idu.io.in.IQfromIUsub.wbPreg(2).bits := DontCare //////todo: check it, from lsu?
  idu.io.in.IQfromIUsub.wbPreg(0).valid := iu.io.iuToRtu.rbusRslt(0).wbPregVld
  idu.io.in.IQfromIUsub.wbPreg(1).valid := iu.io.iuToRtu.rbusRslt(1).wbPregVld
  idu.io.in.IQfromIUsub.wbPreg(2).valid := DontCare //////todo: check it, from lsu?
  idu.io.in.RFfromHad := DontCare
  idu.io.in.RFfromVFPU := DontCare
  idu.io.in.RFfromCp0sub := DontCare
  idu.io.in.RTfromVFPU := DontCare
  idu.io.in.fromHpcp := DontCare
  idu.io.in.fromCp0 := DontCare
  idu.io.in.fromPad := DontCare
  idu.io.in.fromLSU := DontCare //////todo: add LSU
  idu.io.in.ISfromVFPU := DontCare
  idu.io.in.ISfromIUsub.pcfifo_dis_inst_pid := iu.io.bjuToIdu.alloPid




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
    iu.io.ifuForward(i).en := ifu.io.ifuForward(i).en //////todo: add signal
  }
  iu.io.rtuIn.rtuFlush.fe := rtu.io.out.toIu.flushFe
  iu.io.rtuIn.rtuFlush.flush := rtu.io.out.yyXx.flush //////todo: find out
  iu.io.rtuIn.flushChgflwMask := rtu.io.out.toIu.flushChangeFlowMask
  iu.io.rtuIn.robReadPcfifovld := rtu.io.out.toIu.robReadPcFifoValid
  iu.io.rtuIn.robReadPcfifovldGateEn := rtu.io.out.toIu.robReadPcFifoGateClkValid
  iu.io.isIn.isuToFifo.instVld := idu.io.out.IStoIU.pcfifo_inst_vld
  iu.io.isIn.isuToFifo.instNum := idu.io.out.IStoIU.pcfifo_inst_num


  //IU ignore other signals
  iu.io.alu0Sel.sel := idu.io.out.RFCtrl.toAlu0.sel
  iu.io.alu0Sel.gateSel := idu.io.out.RFCtrl.toAlu0.gateClkSel
  iu.io.alu1Sel.sel := idu.io.out.RFCtrl.toAlu1.sel
  iu.io.alu1Sel.gateSel := idu.io.out.RFCtrl.toAlu1.gateClkSel
  iu.io.isIn.issue := false.B //////todo: ISStage add idu_iu_is_div_issue
  iu.io.isIn.gateClkIssue := false.B //////todo: idu_iu_is_div_gateclk_issue ???
  iu.io.specialPid := DontCare //////todo: check idu_iu_rf_pipe0_pid and idu_iu_rf_pipe1_pid ???
  iu.io.bjuSel.sel := idu.io.out.RFCtrl.toBju.sel
  iu.io.bjuSel.gateSel := idu.io.out.RFCtrl.toBju.gateClkSel
  iu.io.mulSel.sel := idu.io.out.RFCtrl.toMul.sel
  iu.io.mulSel.gateSel := idu.io.out.RFCtrl.toMul.gateClkSel
  iu.io.specialSel.sel := idu.io.out.RFCtrl.toSpecial.sel
  iu.io.specialSel.gateSel := idu.io.out.RFCtrl.toSpecial.gateClkSel
  iu.io.divSel.sel := idu.io.out.RFCtrl.toDiv.sel
  iu.io.divSel.gateSel := idu.io.out.RFCtrl.toDiv.gateClkSel
  iu.io.pipe0.iid := idu.io.out.RFData.toIu0.iid
  iu.io.pipe0.dstVld := idu.io.out.RFData.toIu0.dstPreg.valid
  iu.io.pipe0.dstPreg := idu.io.out.RFData.toIu0.dstPreg.bits
  iu.io.pipe0.opcode := idu.io.out.RFData.toIu0.opcode
  iu.io.pipe0.exptVec := idu.io.out.RFData.toIu0.exceptVec.bits
  iu.io.pipe0.exptVld := idu.io.out.RFData.toIu0.exceptVec.valid
  iu.io.pipe0.highHwExpt := idu.io.out.RFData.toIu0.highHwExpt
  iu.io.pipe0.specialImm := idu.io.out.RFData.toIu0.specialImm
  iu.io.pipe0.aluShort := idu.io.out.RFData.toIu0.aluShort
  iu.io.pipe0.imm := idu.io.out.RFData.toIu0.imm
  iu.io.pipe0.src0 := idu.io.out.RFData.toIu0.src0
  iu.io.pipe0.src1 := idu.io.out.RFData.toIu0.src1
  iu.io.pipe0.src2 := idu.io.out.RFData.toIu0.src2
  iu.io.pipe0.src1NoImm := idu.io.out.RFData.toIu0.src1NoImm
  iu.io.pipe0.opcode := idu.io.out.RFData.toIu0.opcode //////todo: check it, use opcode(7.W) to replace func(7.W)
  iu.io.pipe1.iid := idu.io.out.RFData.toIu1.iid
  iu.io.pipe1.dstVld := idu.io.out.RFData.toIu1.dstPreg.valid
  iu.io.pipe1.dstPreg := idu.io.out.RFData.toIu1.dstPreg.bits
  iu.io.pipe1.multFunc := DontCare //////todo: find it, (8.W) different with toIu1.opcode(7.W)
  iu.io.pipe1.mlaSrc2Preg := DontCare //////todo: find it
  iu.io.pipe1.mlaSrc2Vld := DontCare //////todo: find it
  iu.io.pipe1.mulSel := DontCare //////todo: find it
  iu.io.pipe1.aluShort := idu.io.out.RFData.toIu1.aluShort
  iu.io.pipe1.imm := idu.io.out.RFData.toIu1.imm
  iu.io.pipe1.src0 := idu.io.out.RFData.toIu1.src0
  iu.io.pipe1.src1 := idu.io.out.RFData.toIu1.src1
  iu.io.pipe1.src2 := idu.io.out.RFData.toIu1.src2
  iu.io.pipe1.src1NoImm := idu.io.out.RFData.toIu1.src1NoImm
  iu.io.pipe1.opcode := idu.io.out.RFData.toIu1.opcode
  iu.io.pipe2.iid := idu.io.out.RFData.toBju.iid
  iu.io.pipe2.src0 := idu.io.out.RFData.toBju.src0
  iu.io.pipe2.src1 := idu.io.out.RFData.toBju.src1
  iu.io.pipe2.opcode := idu.io.out.RFData.toBju.opcode
  iu.io.pipe2.pid := idu.io.out.RFData.toBju.pid
  iu.io.pipe2.length := DontCare //////todo: complete in rf
  iu.io.pipe2.offset := DontCare //////todo: complete in rf
  iu.io.pipe2.pCall := DontCare //////todo: complete in rf
  iu.io.pipe2.rts := DontCare //////todo: complete in rf
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
    rtu.io.in.fromIdu.robCreate(i).data.data.instr := idu.io.out.IStoRTU.rob_create(i).data.INSTR
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

  rtu match {
    case rtu: RtuTop =>
      val in0 = rtu.io.in.fromIu.pipeCtrlVec(0)
      val in1 = rtu.io.in.fromIu.pipeCtrlVec(1)
      val in2 = rtu.io.in.fromIu.pipeCtrlVec(2)
      val wbdata = rtu.io.in.fromIu.wbData
      val pcFifoPop0 = rtu.io.in.fromIu.pcFifoPopDataVec(0)
      val pcFifoPop1 = rtu.io.in.fromIu.pcFifoPopDataVec(1)
      val pcFifoPop2 = rtu.io.in.fromIu.pcFifoPopDataVec(2)
      in0.iid := iu.io.iuToRtu.cbusRslt.pipe0Iid
      in1.iid := iu.io.iuToRtu.cbusRslt.pipe1Iid
      in2.iid := iu.io.iuToRtu.cbusRslt.pipe2Iid
      in0.flush := iu.io.iuToRtu.cbusRslt.pipe0Flush
      in1.flush := false.B //////todo: find it
      in2.flush := false.B //////todo: find it
      in0.noSpec := 0.U.asTypeOf(in0.noSpec) //////todo: find it
      in1.noSpec := 0.U.asTypeOf(in1.noSpec) //////todo: find it
      in2.noSpec := 0.U.asTypeOf(in2.noSpec) //////todo: find it
      in0.mtval := iu.io.iuToRtu.cbusRslt.pipe0Mtval
      in1.mtval := DontCare //////todo: find it
      in2.mtval := DontCare //////todo: find it
      in0.instMmuException := DontCare //////todo: find it
      in1.instMmuException := DontCare //////todo: find it
      in2.instMmuException := DontCare //////todo: find it
      in0.abnormal := iu.io.iuToRtu.cbusRslt.pipe0Abnormal
      in1.abnormal := DontCare //////todo: find it
      in2.abnormal := iu.io.iuToRtu.cbusRslt.pipe2Abnormal //////todo: check it
      in0.bhtMispred := DontCare
      in1.bhtMispred := DontCare
      in2.bhtMispred := iu.io.iuToRtu.cbusRslt.pipe2BhtMispred
      in0.breakPoint := iu.io.iuToRtu.cbusRslt.pipe0Bkpt
      in1.breakPoint := DontCare
      in2.breakPoint := DontCare
      in0.breakpointData := DontCare //////todo: find it
      in1.breakpointData := DontCare //////todo: find it
      in2.breakpointData := DontCare //////todo: find it
      in0.cmplt := iu.io.iuToRtu.cbusRslt.pipe0Cmplt
      in1.cmplt := iu.io.iuToRtu.cbusRslt.pipe1Cmplt
      in2.cmplt := iu.io.iuToRtu.cbusRslt.pipe2Cmplt
      in0.efPc.bits := iu.io.iuToRtu.cbusRslt.pipe0Efpc
      in0.efPc.valid := iu.io.iuToRtu.cbusRslt.pipe0EfpcVld
      in1.efPc := DontCare
      in2.efPc := DontCare
      in0.exceptionVec.bits := iu.io.iuToRtu.cbusRslt.pipe0ExptVec //////todo: check it, exceptionVec and exceptVec
      in0.exceptionVec.valid := iu.io.iuToRtu.cbusRslt.pipe0ExptVld
      in1.exceptionVec := DontCare
      in2.exceptionVec := DontCare
      in0.exceptVec.bits := iu.io.iuToRtu.cbusRslt.pipe0ExptVec
      in0.exceptVec.valid := iu.io.iuToRtu.cbusRslt.pipe0ExptVld //////todo: check it, exceptionVec and exceptVec
      in1.exceptVec := DontCare
      in2.exceptVec := DontCare
      in0.highHwException := iu.io.iuToRtu.cbusRslt.pipe0HighHwExpt
      in1.highHwException := DontCare
      in2.highHwException := DontCare
      in0.jmpMispred := DontCare //////todo: check it
      in1.jmpMispred := DontCare
      in2.jmpMispred := iu.io.iuToRtu.cbusRslt.pipe2JmpMispred
      in0.specFail := DontCare //////todo: find it
      in1.specFail := DontCare
      in2.specFail := DontCare
      in0.splitSpecFailIid := DontCare //////todo: find it
      in1.splitSpecFailIid := DontCare
      in2.splitSpecFailIid := DontCare
      in0.vsetvl := DontCare
      in1.vsetvl := DontCare
      in2.vsetvl := DontCare
      in0.vstart := DontCare
      in1.vstart := DontCare
      in2.vstart := DontCare
      wbdata(0).bits := UIntToOH(iu.io.iuToRtu.rbusRslt(0).wbPreg)(95,0).asBools //////todo: check it
      wbdata(1).bits := UIntToOH(iu.io.iuToRtu.rbusRslt(1).wbPreg)(95,0).asBools
      wbdata(0).valid := iu.io.iuToRtu.rbusRslt(0).wbPregVld
      wbdata(1).valid := iu.io.iuToRtu.rbusRslt(1).wbPregVld
      pcFifoPop0.length := false.B //////todo: find it
      pcFifoPop0.bhtPred := iu.io.bjuToRtu(0).bhtPred //////todo: popNum = 3
      pcFifoPop0.bhtMispred := iu.io.bjuToRtu(0).bhtMispred
      pcFifoPop0.jmp := iu.io.bjuToRtu(0).jmp
      pcFifoPop0.pret := iu.io.bjuToRtu(0).pret
      pcFifoPop0.pcall := iu.io.bjuToRtu(0).pcall
      pcFifoPop0.condBranch := iu.io.bjuToRtu(0).condBranch
      pcFifoPop0.pcNext := iu.io.bjuToRtu(0).pcNext
      pcFifoPop0.lsb := false.B //////todo: add it
      pcFifoPop1 := 0.U.asTypeOf(pcFifoPop1)
      pcFifoPop2 := 0.U.asTypeOf(pcFifoPop2)
  }


  //RTU ignore other signals
  rtu.io.in.fromIdu.toPst.pregDeallocMaskOH := 0.U.asTypeOf(rtu.io.in.fromIdu.toPst.pregDeallocMaskOH) //////todo: add sdiq, idu.io.out.sdiq.....
  rtu.io.in.fromIdu.toPst.fregDeallocMaskOH := 0.U.asTypeOf(rtu.io.in.fromIdu.toPst.fregDeallocMaskOH) //////todo: add sdiq, idu.io.out.sdiq.....
  rtu.io.in.fromIdu.toPst.vregDeallocMaskOH := 0.U.asTypeOf(rtu.io.in.fromIdu.toPst.vregDeallocMaskOH) //////todo: add sdiq, idu.io.out.sdiq.....
  rtu.io.in.fromIdu.fenceIdle := 0.U.asTypeOf(rtu.io.in.fromIdu.fenceIdle) //////todo: find out
  rtu.io.in.fromLsu := 0.U.asTypeOf(rtu.io.in.fromLsu)
//  rtu.io.in.fromLsu.pipeCtrlVec := 0.U.asTypeOf(rtu.io.in.fromLsu.pipeCtrlVec)
//  rtu.io.in.fromLsu.wbPregData := 0.U.asTypeOf(rtu.io.in.fromLsu.wbPregData)
//  rtu.io.in.fromLsu.wbVFregData := 0.U.asTypeOf(rtu.io.in.fromLsu.wbVFregData)
//  rtu.io.in.fromLsu.asyncExceptAddr := 0.U.asTypeOf(rtu.io.in.fromLsu.asyncExceptAddr)
//  rtu.io.in.fromLsu.asyncExceptValid := 0.U.asTypeOf(rtu.io.in.fromLsu.asyncExceptValid)
  rtu.io.in.fromLsu.allCommitDataValid := true.B
//  rtu.io.in.fromLsu.ctcFlushValid := 0.U.asTypeOf(rtu.io.in.fromLsu.ctcFlushValid)
  //rtu.io.in.fromIu := DontCare //////todo: pcFifoPopDataVec: iu_rtu_pcfifo_pop0_data... wbData: iu_rtu_ex2_pipe0_wb_preg_expand?  Ctrl: ...
  rtu.io.in.fromCp0.xxIntB := true.B //////_b
  rtu.io.in.fromCp0.xxVec := 0.U.asTypeOf(rtu.io.in.fromCp0.xxVec)
  rtu.io.in.fromCp0.icgEn := false.B
  rtu.io.in.fromCp0.yyClkEn := false.B
  rtu.io.in.fromCp0.srtEn := false.B
  rtu.io.in.fromPad := 0.U.asTypeOf(rtu.io.in.fromPad)
  rtu.io.in.fromHad := 0.U.asTypeOf(rtu.io.in.fromHad)
  rtu.io.in.fromHpcp := 0.U.asTypeOf(rtu.io.in.fromHpcp)
  rtu.io.in.fromMmu := 0.U.asTypeOf(rtu.io.in.fromMmu)
  rtu.io.in.fromVfpu := 0.U.asTypeOf(rtu.io.in.fromVfpu)


  dontTouch(ifu.io)
  dontTouch(idu.io)
  dontTouch(iu.io)
  dontTouch(rtu.io)

  io.uart.in.valid  := false.B
  io.uart.out.valid := false.B
  io.uart.out.ch    := 0.U

  //  val instrCommit = Module(new DifftestInstrCommit)
  //  instrCommit.io.clock := clock
  //  instrCommit.io.coreid := 0.U
  //  instrCommit.io.index := 0.U
  //  instrCommit.io.skip := false.B
  //  instrCommit.io.isRVC := false.B
  //  instrCommit.io.scFailed := false.B
  //
  //  instrCommit.io.valid := true.B
  //  instrCommit.io.pc    := 0.U
  //
  //  instrCommit.io.instr := 0.U
  //
  //  instrCommit.io.wen   := false.B
  //  instrCommit.io.wdata := 0.U
  //  instrCommit.io.wdest := 0.U


  val csrCommit = Module(new DifftestCSRState)
  csrCommit.io.clock          := clock
  csrCommit.io.priviledgeMode := 0.U
  csrCommit.io.mstatus        := 0.U
  csrCommit.io.sstatus        := 0.U
  csrCommit.io.mepc           := 0.U
  csrCommit.io.sepc           := 0.U
  csrCommit.io.mtval          := 0.U
  csrCommit.io.stval          := 0.U
  csrCommit.io.mtvec          := 0.U
  csrCommit.io.stvec          := 0.U
  csrCommit.io.mcause         := 0.U
  csrCommit.io.scause         := 0.U
  csrCommit.io.satp           := 0.U
  csrCommit.io.mip            := 0.U
  csrCommit.io.mie            := 0.U
  csrCommit.io.mscratch       := 0.U
  csrCommit.io.sscratch       := 0.U
  csrCommit.io.mideleg        := 0.U
  csrCommit.io.medeleg        := 0.U


}