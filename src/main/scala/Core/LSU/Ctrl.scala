package Core.LSU
import Core.Config.XLEN
import Core.LSU.LoadExStage.LoadDA2Ctrl
import Core.LSU.Sq.SqToCtrl
import Core.LSU.StoreExStage.{StDaToCtrl, StDcToCtrl}
import Core.LsuConfig
import chisel3._
import chisel3.util._


class Cp0ToCtrl extends Bundle with LsuConfig{
  val dcachePrefDist = UInt(CACHE_DIST_SELECT.W) // TODO 2?
  val icgEn          = Bool()
  val l2PrefDist     = UInt(CACHE_DIST_SELECT.W) // TODO 2?
  val yyClkEn        = Bool()
}
class DcacheArbIn0ToCtrl extends Bundle with LsuConfig{
  val ldDcBorrowGate  = Bool()
  val stDcBorrowGate  = Bool()
}
class IduRfSelToCtrl extends Bundle with LsuConfig{
  val ldPipeSel    = Bool() // pipe 3
  val ldPipGateSel = Bool() // pipe 3
  val stPipeAddrSel    = Bool() // pipe 4
  val stPipeAddrGateSel = Bool()// pipe 4
  val stPipeDataGateSel = Bool()// pipe 5
}
class lsExStageWaitOld extends Bundle with LsuConfig {
  val old       = UInt(LSIQ_ENTRY.W)
  val oldGateEn = Bool()
}
class RbtoCtrl extends Bundle{
  val empty        = Bool()
  val ldWbCmpltReq = Bool()
  val ldWbDataReq  = Bool()
}
class PfuToCtrl extends Bundle with LsuConfig {
  val lfbCreateGateclkEn = Bool()
  val partEmpty          = Bool()
}
class ExAgToCtrl extends Bundle with LsuConfig {
  val instVld           = Bool()
  val stallOri          = Bool()
  val stallRestartEntry = UInt(LSIQ_ENTRY.W)
}
class StDcToCtrlExt extends StDcToCtrl{
  val borrowVld       = Bool()
  val iduSqFull       = UInt(LSIQ_ENTRY.W)
  val iduTlbBusy      = UInt(LSIQ_ENTRY.W)
  val instVld         = Bool()
  val sqFullGateclkEn = Bool()
}

class VmbToCtrl extends Bundle with LsuConfig {
  val empty        = Bool()
  val ldWbDataReq  = Bool()
}
class WmbToCtrl extends Bundle with LsuConfig {
  val depdWakeup   = UInt(LSIQ_ENTRY.W)
  val empty        = Bool()
  val ldWbDataReq  = Bool()
  val noOp         = Bool()
  val stWbCmpltReq = Bool()
  val writeReqIcc  = Bool()
}

class StDaToCtrlExt extends StDaToCtrl {
  // some bundle did not bind in StDaToCtrl
  // thus in this bundle extend,
  val instVld = Bool() // in bundle 'StoreDaOut'
  val rbcreateGateclkEn = Bool() // in bundle 'StDaToRb'
  val waitFenceGateclkEn = Bool() // in bundle 'StDaToSq'
}
class LoadDA2CtrlExt extends LoadDA2Ctrl{
  val instVld = Bool()
}
//----------------------------------------------------------
class CtrlIn extends Bundle with LsuConfig{
  val cp0In       = new Cp0ToCtrl
  val dcacheArbIn = new DcacheArbIn0ToCtrl
  val iccVbCreateGateEn = Bool()
  val rfPipeSel   = new IduRfSelToCtrl
  val vmbCreateGateEn = Vec(2, Bool())
  val ldAgIn  = new ExAgToCtrl
  val ldDaIn  = new LoadDA2CtrlExt
  val ldDcIn  = new Bundle{
    val borrowVld         = Bool()
    val iduLqFull         = UInt(LSIQ_ENTRY.W)
    val iduTlbBusy        = UInt(LSIQ_ENTRY.W)
    val immeWakeup        = UInt(LSIQ_ENTRY.W)
    val instVld           = Bool()
    val lqFullGateclkEn   = Bool()
    val tlbBusyGateclkEn  = Bool()
  }
  val ldWbIn = new Bundle() {
    val dataVld = Bool()
    val instVld = Bool()
  }
  val wbIn = new Bundle() {
    val dataVld = Bool()
    val instVld = Bool()
  }
  val lfbIn = new Bundle() {
    val depdWakeUp = UInt(LSIQ_ENTRY.W)
    val empty      =Bool()
    val popDepdFf  =Bool()
  }
  val lbfDepdWakeUp = Bool()
  val lsuHasFence = Bool()
  val lsuExWait = new Bundle() {
    val ldAg = new lsExStageWaitOld
    val ldDa = new lsExStageWaitOld
    val stAg = new lsExStageWaitOld
  }
  val rbIn = new RbtoCtrl
  val pfuIn = new PfuToCtrl
  val stEx1InstVld = Bool()
  val sqIn = new SqToCtrl
  val stAgIn = new ExAgToCtrl
  val stDaIn = new StDaToCtrlExt
  val stDcIn = new StDcToCtrlExt
  val stWbInstVld = Bool()
  val vbEmpty = Bool()
  val vmbIn = new VmbToCtrl
  val wmbIn = new WmbToCtrl
  val mmuTlbWakep = UInt(LSIQ_ENTRY.W)
}
//==========================================================
//                        Output
//==========================================================
class CtrltoIdu extends Bundle with LsuConfig{
  val alreadyDa           = UInt(LSIQ_ENTRY.W)
  val bkptaData           = UInt(LSIQ_ENTRY.W)
  val bkptbData           = UInt(LSIQ_ENTRY.W)
  val lqFull              = UInt(LSIQ_ENTRY.W)
  val lqFullGateclkEn     = Bool()
  val lsiqPopVld          = ValidIO(Vec(2,Bool())) // pop vld + pop1(load) + pop2(store)
  val lsiqPopEntry        = UInt(LSIQ_ENTRY.W)
  val rbFull              = UInt(LSIQ_ENTRY.W)
  val rbFullGateclkEn     = Bool()
  val secd                = UInt(LSIQ_ENTRY.W)
  val specFail            = UInt(LSIQ_ENTRY.W)
  val sqFull              = UInt(LSIQ_ENTRY.W)
  val sqFullGateclkEn     = Bool()
  val tlbBusy             = UInt(LSIQ_ENTRY.W)
  val tlbBusyGateclkEn    = Bool()
  val tlbWakeup           = UInt(LSIQ_ENTRY.W)
  val unalignGateclkEn    = UInt(LSIQ_ENTRY.W)
  val waitFence           = UInt(LSIQ_ENTRY.W)
  val waitFenceGateclkEn  = Bool()
  val waitOld             = UInt(LSIQ_ENTRY.W)
  val waitOldGateclkEn    = Bool()
  val wakeup              = UInt(LSIQ_ENTRY.W)
}
//----------------------------------------------------------
class CtrlOut extends Bundle with LsuConfig{
  val ctrlClk = new Bundle() {
    val ldClk = Bool()
    val stClk = Bool()
  }
  val toIdu = new CtrltoIdu
  val toPfu = new Bundle() {
    val l1DistSel = UInt(CACHE_DIST_SELECT_ADDR.W) // l1/2 cache number?
    val l2DistSel = UInt(CACHE_DIST_SELECT_ADDR.W) // l1/2 cache number?
  }
  val lsuSpecialClk = Bool()
  val lsuyyxxNoOp = Bool()
}
//==========================================================
//                          IO
//==========================================================
class CtrlIO extends Bundle with LsuConfig{
  val in  = Input(new CtrlIn)
  val out = Output(new CtrlOut)
}
class Ctrl extends Module with LsuConfig {
  val io = IO(new CtrlIO)
  //==========================================================
  //              Instance of Global Gated Cell
  //==========================================================
  val lsu_special_clk_en = io.in.ldDaIn.special_gateclk_en || io.in.stDaIn.rbcreateGateclkEn ||
    io.in.wmbIn.writeReqIcc || io.in.iccVbCreateGateEn || io.in.pfuIn.lfbCreateGateclkEn ||
    (!io.in.rbIn.empty) || (!io.in.vbEmpty) || (!io.in.lfbIn.empty) || (!io.in.vmbIn.empty) || io.in.lfbIn.popDepdFf ||
    (!io.in.pfuIn.partEmpty) || io.in.lsuHasFence || io.in.lbfDepdWakeUp|| io.in.vmbCreateGateEn.reduce(_ || _)
  //==========================================================
  //                 Instance of Gated Cell
  //==========================================================
  //ctrl_ld_clk is used for ld pipe
  val ctrl_ld_clk_en = io.in.rfPipeSel.ldPipGateSel ||
    io.in.ldAgIn.instVld || io.in.ldDcIn.instVld || io.in.ldDaIn.instVld ||
    io.in.ldWbIn.instVld  || io.in.ldWbIn.dataVld
    io.in.rbIn.ldWbCmpltReq ||
    io.in.wmbIn.ldWbDataReq || io.in.vmbIn.ldWbDataReq || io.in.rbIn.ldWbDataReq ||
    io.in.dcacheArbIn.ldDcBorrowGate || io.in.ldDcIn.borrowVld || io.in.ldDaIn.borrow_vld

  val ctrl_st_clk_en = io.in.rfPipeSel.stPipeAddrGateSel || io.in.rfPipeSel.stPipeDataGateSel ||
    io.in.stAgIn.instVld || io.in.stEx1InstVld || io.in.stDcIn.instVld || io.in.stDaIn.instVld || io.in.wbIn.instVld ||
    io.in.wmbIn.stWbCmpltReq || io.in.dcacheArbIn.stDcBorrowGate ||
    io.in.stDcIn.borrowVld || io.in.stDaIn.borrowVld
  val cp0_lsu_up_vld = Wire(Bool())
  val cp0_lsu_l1_dist_sel = UIntToOH(io.in.cp0In.dcachePrefDist)
  val cp0_lsu_l2_dist_sel = UIntToOH(io.in.cp0In.l2PrefDist)
  val lsu_pref_dist_up = (io.out.toPfu.l1DistSel =/= cp0_lsu_l1_dist_sel) ||
    (io.out.toPfu.l2DistSel =/= cp0_lsu_l2_dist_sel)
  cp0_lsu_up_vld := lsu_pref_dist_up
  val cp0_lsu_clk_en = cp0_lsu_up_vld
// TODO val lsu_hpcp_clk_en: Nothing = lsu_hpcp_up_vld
  //==========================================================
  //                Generate cp0 signal
  //==========================================================
  // for timing, flop some cp0_lsu signals
  // whats this ?


  val lsu_pfu_l1_dist_sel = RegInit(0.U(CACHE_DIST_SELECT_ADDR.W))
  val lsu_pfu_l2_dist_sel = RegInit(0.U(CACHE_DIST_SELECT_ADDR.W))
  when(cp0_lsu_up_vld){
    lsu_pfu_l1_dist_sel  := cp0_lsu_l1_dist_sel
    lsu_pfu_l2_dist_sel  := cp0_lsu_l2_dist_sel
  }
  io.out.toPfu.l1DistSel  := lsu_pfu_l1_dist_sel
  io.out.toPfu.l2DistSel  := lsu_pfu_l2_dist_sel
  //==========================================================
  //        Pipeline signal
  //==========================================================
  //------------------rf stage--------------------------------
  val ld_rf_restart_vld = io.in.rfPipeSel.ldPipeSel && io.in.ldAgIn.stallOri
  val ld_rf_imme_wakeup = io.in.ldAgIn.stallRestartEntry & Cat(Seq.fill(LSIQ_ENTRY)(ld_rf_restart_vld))
  val st_rf_restart_vld = io.in.rfPipeSel.stPipeAddrSel && io.in.ldAgIn.stallOri
  val st_rf_imme_wakeup = io.in.stAgIn.stallRestartEntry & Cat(Seq.fill(LSIQ_ENTRY)(st_rf_restart_vld))
  //------------------ag stage--------------------------------
  io.out.toIdu.tlbBusy := io.in.ldDcIn.iduTlbBusy | io.in.stDcIn.iduTlbBusy
  //------------------dc stage--------------------------------
  io.out.toIdu.lqFull := io.in.ldDcIn.iduLqFull
  io.out.toIdu.sqFull := io.in.stDcIn.iduSqFull
  //------------------da stage--------------------------------
  io.out.toIdu.waitFence := io.in.ldDaIn.idu_wait_fence.asUInt | io.in.stDaIn.waitFence
  io.out.toIdu.rbFull    := io.in.ldDaIn.idu_rb_full.asUInt    | io.in.stDaIn.rbFull
  io.out.toIdu.alreadyDa := io.in.ldDaIn.idu_already_da.asUInt | io.in.stDaIn.alreadyDa
  //---------------boundary signal----------------------------
  io.out.toIdu.unalignGateclkEn := io.in.ldDaIn.idu_boundary_gateclk_en.asUInt | io.in.stDaIn.boundaryGateclkEn
  io.out.toIdu.secd := io.in.ldDaIn.idu_secd.asUInt | io.in.stDaIn.secd
  io.out.toIdu.specFail := io.in.ldDaIn.idu_spec_fail.asUInt | io.in.stDaIn.specFail
  io.out.toIdu.bkptaData := io.in.ldDaIn.idu_bkpta_data.asUInt | io.in.stDaIn.bkptaData
  io.out.toIdu.bkptbData := io.in.ldDaIn.idu_bkptb_data.asUInt | io.in.stDaIn.bkptbData
  //------------------pop signals-----------------------------
  io.out.toIdu.lsiqPopVld.valid := io.in.ldDaIn.idu_pop_vld || io.in.stDaIn.popVld
  io.out.toIdu.lsiqPopEntry := io.in.ldDaIn.idu_pop_entry.asUInt | io.in.stDaIn.popEntry
  //--------------pop num-----------------
  io.out.toIdu.lsiqPopVld.bits(0) := io.in.ldDaIn.idu_pop_vld
  io.out.toIdu.lsiqPopVld.bits(1) := io.in.stDaIn.popVld
  //--------------gateclk-----------------
  io.out.toIdu.lqFullGateclkEn    := io.in.ldDcIn.lqFullGateclkEn
  io.out.toIdu.sqFullGateclkEn    := io.in.stDcIn.sqFullGateclkEn
  io.out.toIdu.tlbBusyGateclkEn   := io.in.ldDcIn.tlbBusyGateclkEn || io.in.stDcIn.tlbBusyGateclkEn
  io.out.toIdu.rbFullGateclkEn    := io.in.ldDaIn.rb_full_gateclk_en || io.in.stDaIn.rbFullGateclkEn
  io.out.toIdu.waitFenceGateclkEn := io.in.ldDaIn.wait_fence_gateclk_en || io.in.stDaIn.waitFenceGateclkEn
  //==========================================================
  //        Imme & Buffer maintain restart
  //==========================================================
  io.out.toIdu.wakeup := ld_rf_imme_wakeup | st_rf_imme_wakeup |
    io.in.ldDcIn.immeWakeup.asUInt | io.in.stDcIn.immeWakeup |
    io.in.ldDaIn.idu_secd.asUInt   | io.in.ldDaIn.ecc_wakeup |
    io.in.stDaIn.secd              | io.in.stDaIn.eccWakeup  |
    io.in.sqIn.globalDepdWakeup    | io.in.sqIn.dataDepdWakeup |
    io.in.wmbIn.depdWakeup         | io.in.lfbIn.depdWakeUp

  io.out.toIdu.tlbWakeup := io.in.mmuTlbWakep
  //merge idu wakeup
  io.out.toIdu.waitOld          := io.in.lsuExWait.ldAg.old       | io.in.lsuExWait.stAg.old       | io.in.lsuExWait.ldDa.old
  io.out.toIdu.waitOldGateclkEn := io.in.lsuExWait.ldAg.oldGateEn | io.in.lsuExWait.stAg.oldGateEn | io.in.lsuExWait.ldDa.oldGateEn
  // no operation yyxx
  io.out.lsuyyxxNoOp := io.in.wmbIn.empty && io.in.sqIn.empty && io.in.rbIn.empty && io.in.lfbIn.empty && io.in.vmbIn.empty && io.in.vbEmpty


  // clk
  io.out.lsuSpecialClk := DontCare
  io.out.ctrlClk := DontCare



}
