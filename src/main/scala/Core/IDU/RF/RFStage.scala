package Core.IDU.RF

import Core.IDU.IS.AiqConfig.NumSrcArith
import Core.IDU.IS.BiqConfig.NumSrcBr
import Core.IDU.IS.LsiqConfig.NumSrcLs
import Core.IDU.IS.SdiqConfig.NumSrcSd
import Core.IDU.IS.VfiqConfig.NumSrcVf
import Core.IDU.IS._
import Core.IntConfig._
import chisel3._

trait RFStageConfig {
  // aiq + biq + lsiq + sdiq + viq
  def NumIssueAiq = 2
  def NumIssueBiq = 1
  def NumIssueLsiq = 2
  def NumIssueSdiq = 1
  def NumIssueVfiq = 2
  def NumIssue : Int = NumIssueAiq + NumIssueBiq + NumIssueLsiq +
    NumIssueSdiq + NumIssueVfiq
  def NumAiqEntry = 8
  def NumBiqEntry = 12
  def NumLsiqEntry = 12
  def NumSdiqEntry = 12
  def NumVfiqEntry = 8
  // Todo
  def StorePipeNumSet = Seq(5)
  def LsuPipeNumSet = Seq(3, 4, 5)
  def BjuPipeNumSet = Seq(2)
}

class IssueEnBundle extends Bundle {
  val gateClkIssueEn = Bool()
  val issueEn = Bool()
}

class RFStageFromCp0Bundle extends Bundle {
  val iduIcgEn : Bool = Bool()
  val yyClkEn : Bool = Bool()
  val lsuFenceIBroadDis = Bool()
  val lsuFenceRwBroadDis = Bool()
  val lsuTlbBroadDis = Bool()
}

class RFStageFromHpcp extends Bundle {
  val iduCntEn = Bool()
}

class RFStageFromIuBundle extends Bundle {
  val stall = new Bundle {
    val divWb   : Bool = Bool()
    val mulEx1  : Bool = Bool()
  }
}

class RFStageFromVfpuBundle extends Bundle {
  val stall = new Bundle {
    val vdivWb = Bool()
  }
}

class RFStageFromRtuBundle extends Bundle {
  val flush = new Bundle {
    val fe = Bool()
    val is = Bool()
  }
  // Todo: check if need be merged into flush bundle
  val yyXxFlush = Bool()
}

class RFStageCtrlInput extends Bundle with RFStageConfig {
  val fromCp0 = new RFStageFromCp0Bundle
  val fromIu = new RFStageFromIuBundle
  val issueEnVec = Vec(NumIssue, new IssueEnBundle)
  val fromPad = new PrfFromPadBundle
  val fromRtu = new RFStageFromRtuBundle
  val fromVfpu = new RFStageFromVfpuBundle
}

class RFStageToIqBundle extends Bundle with RFStageConfig {
  val launchFailValid = Bool()
  val popValid = Bool()
  // Todo: add forward signal
  //  e.g. ctrl_aiq0_rf_pipe0_alu_reg_fwd_vld ctrl_viq1_rf_pipe7_vmla_vreg_fwd_vld
  val popDlbValid = Bool()
  val stall = Bool()
  /**
   *   Only for sdiq
   */
  val stAddrReadySet = Bool()
  val otherLaunchFail = Bool()
  // Todo: Add launch valid dup
  //  e.g. ctrl_xx_rf_pipe0_preg_lch_vld_dup0
}

class RFStageToCp0Bundle extends Bundle {
  // ctrl
  // Todo: figure out
  val gateClkSel = Bool()
  val sel = Bool()
  // Todo: imm
  // data
  val func = UInt(5.W)
  val iid = UInt(InstructionIdWidth.W)
  val opcode = UInt(OpcodeBits.W)
  val preg = UInt(NumPhysicRegsBits.W)
  val src0 = UInt(XLEN.W)
}

class RFStageToHpcpBundle extends Bundle with RFStageConfig {
  val instValid = Bool()
  class PipeSignal extends Bundle {
    val instValid = Bool()
    val launchFailValid = Bool()
    val regLaunchFailValid = Bool()
  }
  val pipeVec = Vec(NumIssue, new PipeSignal)
}

class RFStageToExuGateClkBundle extends Bundle {
  // gateClkIssue
  // issue
  // gateClkSel
  // sel
  // cbusGateClkSel
}

class RFStageToFuBundle extends Bundle {
  val sel = Bool()
  val gateClkSel = Bool()
}

class RFStageCtrlOutput extends Bundle with RFStageConfig {
  val toIq = Vec(NumIssue, new RFStageToIqBundle)
  val toCp0 = new RFStageToCp0Bundle
  val toHpcp = new RFStageToHpcpBundle
  val toExu = new RFStageToExuGateClkBundle
  val toBju = new RFStageToFuBundle
  val toDiv = new RFStageToFuBundle
  val toMul = new RFStageToFuBundle
  val toSpecial = new RFStageToFuBundle
  val toAlu0 = new RFStageToFuBundle // pipe0
  val toAlu1 = new RFStageToFuBundle // pipe1
  val toLu = new RFStageToFuBundle // pipe3
  val toSt = new RFStageToFuBundle // pipe4
  val toSd = new RFStageToFuBundle // pipe5
  val toVfpu0 = new RFStageToFuBundle // pipe6
  val toVfpu1 = new RFStageToFuBundle // pipe6
}

class RFStageCtrlBundle extends Bundle {
  val in = Flipped(Output(new RFStageCtrlInput))
  val out = Output(new RFStageCtrlOutput)
}

class RFStageFromHadBundle extends Bundle {
  val iduWbBrData = UInt(XLEN.W)
  val iduWbBrValid = Bool()
}

class RFStageFromPadBundle extends Bundle {
  val yyIcgScanEn = Bool()
}

class RFStageDataInput extends Bundle with RFStageConfig {
  class AiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH = UInt(NumAiqEntry.W)
    val issueReadData = new AiqEntryData
    val issueEn = Bool()
    val issueGateClkEn = Bool()
  }
  class BiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH = UInt(NumBiqEntry.W)
    val issueReadData = new BiqEntryData
    val issueEn = Bool()
    val issueGateClkEn = Bool()
  }
  class LsiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH = UInt(NumLsiqEntry.W)
    val issueReadData = new LsiqEntryData
    val issueEn = Bool()
    val issueGateClkEn = Bool()
  }
  class SdiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH = UInt(NumSdiqEntry.W)
    val issueReadData = new SdiqEntryData
    val issueEn = Bool()
    val issueGateClkEn = Bool()
  }
  class VfiqIssueBundle extends Bundle with RFStageConfig {
    val issueEntryOH = UInt(NumVfiqEntry.W)
    val issueReadData = new VfiqEntryData
    val issueEn = Bool()
    val issueGateClkEn = Bool()
  }

  val aiq0 = new AiqIssueBundle
  val aiq1 = new AiqIssueBundle
  val biq = new BiqIssueBundle
  val lsiq0 = new LsiqIssueBundle
  val lsiq1 = new LsiqIssueBundle
  val sdiq = new SdiqIssueBundle
  val vfiq0 = new VfiqIssueBundle
  val vfiq1 = new VfiqIssueBundle
  val fromHad = new RFStageFromHadBundle
  val fromPad = new RFStageFromPadBundle
}

class RFStageDataOutput extends Bundle with RFStageConfig {
  class RFStageToIqBundle(numEntry: Int, numSrc: Int) extends Bundle {
    val launchEntryOH = UInt(numEntry.W)
    val readyClr = UInt(numSrc.W)

  }
  val toAiq0  = new RFStageToIqBundle(NumAiqEntry, NumSrcArith)
  val toAiq1  = new RFStageToIqBundle(NumAiqEntry, NumSrcArith)
  val toBiq   = new RFStageToIqBundle(NumBiqEntry, NumSrcBr)
  val toLsiq0 = new RFStageToIqBundle(NumLsiqEntry, NumSrcLs)
  val toLsiq1 = new RFStageToIqBundle(NumLsiqEntry, NumSrcLs)
  val toSdiq  = new RFStageToIqBundle(NumSdiqEntry, NumSrcSd)
  val toVfiq0 = new RFStageToIqBundle(NumVfiqEntry, NumSrcVf)
  val toVfiq1 = new RFStageToIqBundle(NumVfiqEntry, NumSrcVf)

}

class RFStageDataBundle extends Bundle {
  val in = Flipped(Output(new RFStageDataInput))
  val out = Output(new RFStageDataOutput)
}

class RFStageIO extends Bundle {
  val data = new RFStageDataBundle
  val ctrl = new RFStageCtrlBundle
}

class RFStage extends Module with RFStageConfig {
  val io = IO(new RFStageIO)

  val rtu = io.ctrl.in.fromRtu

  /**
   * Regs
   */

  // performance monitor
  val pipeInstValidVec = RegInit(VecInit(Seq.fill(NumIssue)(false.B)))
  val lsuLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssueLsiq + NumIssueSdiq)(false.B)))

  //==========================================================
  //                 RF Inst Valid registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Instruction Valid
  //----------------------------------------------------------
  //alu and special should set ready for pipe0 ex2 fwd
  //pipe0 ex1 fwd is set by alu reg fwd vld

  private val aiq0IssueAluRegValid = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.launchPreg
  private val aiq0IssueSpecialValid = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.special
  // Todo: div gateclk
  private val iuIsDivIssue = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.div

  private val pipe0PregLaunchValid = RegInit(false.B)
  private val pipe0SpecialValid = RegInit(false.B)

  //----------------------------------------------------------
  //                Pipe1 Instruction Valid
  //----------------------------------------------------------

  private val pipe1PregLaunchValid = RegInit(false.B)

  private val aiq1IssueAluRegValid = io.ctrl.in.issueEnVec(1).issueEn &&
    io.data.in.aiq1.issueReadData.launchPreg

  //----------------------------------------------------------
  //                Pipe2 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe3 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe4 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe5 Instruction Valid
  //----------------------------------------------------------

  //----------------------------------------------------------
  //                Pipe6 Instruction Valid
  //----------------------------------------------------------

  val pipe6vmlaPregLaunchValid = RegInit(false.B)
  val viq0IssueVmlaRfValid = io.ctrl.in.issueEnVec(6).issueEn &&
    io.data.in.vfiq0.issueReadData.dstVregValid &&
    io.data.in.vfiq0.issueReadData.vmla

  //----------------------------------------------------------
  //                Pipe7 Instruction Valid
  //----------------------------------------------------------

  val pipe7vmlaPregLaunchValid = RegInit(false.B)
  val viq1IssueVmlaRfValid = io.ctrl.in.issueEnVec(7).issueEn &&
    io.data.in.vfiq1.issueReadData.dstVregValid &&
    io.data.in.vfiq1.issueReadData.vmla
  pipeInstValidVec.zipWithIndex.foreach {
    case (instValid, i) =>
      // pipe 5 rf stage is flush by flush_be
      // st pipe
      if (StorePipeNumSet.contains(i)) {
        when(rtu.yyXxFlush) {
          instValid := false.B
        }.otherwise {
          instValid := io.ctrl.in.issueEnVec(i)
        }
      }
      else {
        when(rtu.flush.fe || rtu.flush.is) {
          instValid := false.B
        }.otherwise {
          instValid := io.ctrl.in.issueEnVec(i)
        }
      }
  }

  when(rtu.flush.fe || rtu.flush.is) {
    pipe0PregLaunchValid  := false.B
    pipe0SpecialValid     := false.B
    pipe1PregLaunchValid  := false.B
    pipe6vmlaPregLaunchValid := false.B
    pipe7vmlaPregLaunchValid := false.B
  }.otherwise {
    pipe0PregLaunchValid  := aiq0IssueAluRegValid
    pipe0SpecialValid     := aiq0IssueSpecialValid
    pipe1PregLaunchValid  := aiq1IssueAluRegValid
    pipe6vmlaPregLaunchValid := viq0IssueVmlaRfValid
    pipe7vmlaPregLaunchValid := viq1IssueVmlaRfValid
  }

  //==========================================================
  //                RF Launch Ready registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd
  //  val pipe0AluRegFwdValidVec = Vec()

  val aiq0IssueAluFwdInst = io.ctrl.in.issueEnVec(0).issueEn &&
    io.data.in.aiq0.issueReadData.dstValid &&
    io.data.in.aiq0.issueReadData.aluShort

  //----------------------------------------------------------
  //                Pipe1 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  val aiq1IssueAluFwdInst = io.ctrl.in.issueEnVec(1).issueEn &&
    io.data.in.aiq1.issueReadData.dstValid &&
    io.data.in.aiq1.issueReadData.aluShort
  // Todo:
//  val aiq1IssueMlaFwdInst = io.ctrl.in.issueEnVec(1).issueEn &&
//    io.data.in.aiq1.issueReadData.mlaValid

  //----------------------------------------------------------
  //                Pipe6 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  val viq0IssueVmlaFwdInst = io.ctrl.in.issueEnVec(6).issueEn &&
    io.data.in.vfiq0.issueReadData.vmlaShort

  //----------------------------------------------------------
  //                Pipe7 Launch Ready valid
  //----------------------------------------------------------

  // Todo: fwd

  val viq1IssueVmlaFwdInst = io.ctrl.in.issueEnVec(7).issueEn &&
    io.data.in.vfiq1.issueReadData.vmlaShort

  //==========================================================
  //                     RF Stall Signals
  //==========================================================
  //----------------------------------------------------------
  //                     VR/GPR move stall
  //----------------------------------------------------------
  //ex2 pipe0/1 mtvr will share ex1 pipe6/7
  //stall viq all inst issue at pipe0/1 rf

  val pipe0MtvrValid = pipeInstValidVec(0) && io.data.in.aiq0.issueReadData.mtvr
  val pipe1MtvrValid = pipeInstValidVec(1) && io.data.in.aiq1.issueReadData.mtvr

  val pipe6MfvrValid = pipeInstValidVec(6) && io.data.in.vfiq0.issueReadData.mfvr
  val pipe7MfvrValid = pipeInstValidVec(7) && io.data.in.vfiq1.issueReadData.mfvr

  //----------------------------------------------------------
  //                     div/vdiv stall
  //----------------------------------------------------------
  //when div need to write back, it will stall issue at wb pipe
  val pipe0DivStall = io.ctrl.in.fromIu.stall.divWb
  val pipe6VdivStall = io.ctrl.in.fromVfpu.stall.vdivWb

  //----------------------------------------------------------
  //                        mul stall
  //----------------------------------------------------------
  val pipe1MulStall = io.ctrl.in.fromIu.stall.mulEx1

  //----------------------------------------------------------
  //                 IU special cmplt stall
  //----------------------------------------------------------
  //when IU special need cmplt at EX1, it will share pipe0
  //RF cmplt port. it will not be conflict with wb port share,
  //so do not need lch fail
  val pipe0SpecialStall = pipe0SpecialValid

  //----------------------------------------------------------
  //              Output Stall Signals to IQ
  //----------------------------------------------------------
  // Note: aiq0 --> viq0, aiq1 --> viq1
  // aiq0
  io.ctrl.out.toIq(0).stall := pipe6MfvrValid || pipe0DivStall || pipe0SpecialStall
  io.ctrl.out.toIq(1).stall := pipe7MfvrValid
  // viq0
  io.ctrl.out.toIq(6).stall := pipe0MtvrValid || pipe6VdivStall
  io.ctrl.out.toIq(7).stall := pipe1MtvrValid

  //==========================================================
  //                   Launch fail Signals
  //==========================================================
  val pipeLaunchFailVec = Wire(Vec(NumIssue, Bool()))
  // Todo: figure this CAUTION
  //CAUTION: avoid dead lock: when inst1 lch fail inst2, inst2
  //         may lch fail inst1 through src no ready

  //----------------------------------------------------------
  //                  div/vdiv wb lch fail
  //----------------------------------------------------------
  //pipe6 vdiv stall may be conflict with pipe0 mtvr stall
  //pipe0 div stall may be conflict with pipe6 mfvr stall
  //so launch fail mfvr/mtvr at this situation
  val pipe0VdivMtvrLaunchFail = pipe0MtvrValid && pipe6VdivStall
  val pipe6DivMfvrLaunchFail = pipe6MfvrValid && pipe0DivStall

  //----------------------------------------------------------
  //                mult mfvr share lch fail
  //----------------------------------------------------------
  //pipe1 ex2 mult will share pipe1 rf
  //pipe7 ex2 mfvr will share pipe1 ex1,
  //  (which is pipe7 ex1 share pipe1 rf)
  //pipe1 ex2 mult will be conflict with pipe7 ex1 mfvr
  //so launch fail pipe7 rf mfvr when pipe1 ex1 is mult
  val pipe7mulMfvrLaunchFail = pipe7MfvrValid && pipe1MulStall

  //----------------------------------------------------------
  //               preg read port share lch fail
  //----------------------------------------------------------
  //prepare src vld signals
  val pipe0src2Valid = pipeInstValidVec(0) && io.data.in.aiq0.issueReadData.srcValid(2)
  val pipe1src2Valid = pipeInstValidVec(1) && io.data.in.aiq1.issueReadData.srcValid(2)
  val pipe3src1Valid = pipeInstValidVec(3) && io.data.in.lsiq0.issueReadData.srcValid(1)
  val pipe5src0Valid = pipeInstValidVec(5) && io.data.in.sdiq.issueReadData.src0Valid
  //preg read port share priority:
  //pipe0 src2 > pipe3 src1, pipe1 src1 > pipe5 src0
  //so launch fail pipe3/5 when pipe0/1 shares preg read port
  val pipe3PregLaunchFail = pipe3src1Valid && pipe0src2Valid
  val pipe5PregLaunchFail = pipe5src0Valid && pipe1src2Valid

  //----------------------------------------------------------
  //               vreg read port share lch fail
  //----------------------------------------------------------
  val pipe3srcVmValid = pipeInstValidVec(3) && io.data.in.lsiq0.issueReadData.srcVmValid
  val pipe4srcVmValid = pipeInstValidVec(4) && io.data.in.lsiq1.issueReadData.srcVmValid
  val pipe6srcV2Valid = pipeInstValidVec(6) && io.data.in.vfiq0.issueReadData.srcValidVec(2)
  val pipe7srcV2Valid = pipeInstValidVec(7) && io.data.in.vfiq1.issueReadData.srcValidVec(2)

  //vreg read port share priority:
  //pipe6 srcv2 > pipe3 srcvm, pipe7 srcv2 > pipe4 srcvm
  //so launch fail pipe3/4 when pipe6/7 vreg read port need to shared
  val pipe3VregLaunchFail = pipe3srcVmValid && pipe6srcV2Valid
  val pipe4VregLaunchFail = pipe4srcVmValid && pipe7srcV2Valid

  //----------------------------------------------------------
  //               unsplit vmlu share lch fail
  //----------------------------------------------------------
  // Todo: figure out
  //need to consider pipe7 lch fail, avoiding dead lock:
  //if pipe7 vmul unsplit lch fail pipe6 older inst x, older inst x
  //may lch fail pipe7 vmul unsplit through src no ready

  val pipe7VmulUnsplitValid = pipeInstValidVec(7) &&
    !pipeLaunchFailVec(7) &&
    io.data.in.vfiq1.issueReadData.vmulUnsplit
  val pipe6VmulValid = pipeInstValidVec(7) &&
    io.data.in.vfiq0.issueReadData.vmul
  //pipe7 vmul unsplit inst will share pipe6 vmul
  //so lch fail pipe6 vmul at this situation
  val pipe6vmulUnsplitLaunchFail = pipe7VmulUnsplitValid && pipe6VmulValid

  //----------------------------------------------------------
  //                 Source Not Ready Signal
  //----------------------------------------------------------
  val pipeSrcNotReadyVec = Wire(Vec(NumIssue, Bool()))
  // Todo:
  //----------------------------------------------------------
  //                 Launch Fail Signals
  //----------------------------------------------------------
  //lch fail without src no rdy
  val pipeOtherLaunchFailVec = WireInit(VecInit(Seq.fill(NumIssue)(false.B)))
  pipeOtherLaunchFailVec(0) := pipe0VdivMtvrLaunchFail
  pipeOtherLaunchFailVec(3) := pipe3PregLaunchFail || pipe3VregLaunchFail
  pipeOtherLaunchFailVec(4) := pipe4VregLaunchFail
  pipeOtherLaunchFailVec(5) := pipe5PregLaunchFail
  pipeOtherLaunchFailVec(6) := pipe6DivMfvrLaunchFail || pipe6vmulUnsplitLaunchFail
  pipeOtherLaunchFailVec(7) := pipe7mulMfvrLaunchFail

  //should consider src no ready lch fail
  pipeLaunchFailVec.zipWithIndex.foreach {
    case (launchFail, i) =>
      launchFail := pipeSrcNotReadyVec(i) || pipeOtherLaunchFailVec(i)
  }

  //==========================================================
  //                    RF Control Signals
  //==========================================================
  //----------------------------------------------------------
  //                    RF pipedown valid
  //----------------------------------------------------------
  val pipeDownValidVec = Wire(Vec(NumIssue, Bool()))

  pipeDownValidVec.zipWithIndex.foreach {
    case (pipeDownValid, i) =>
      pipeDownValid := pipeInstValidVec(i) && !pipeLaunchFailVec(i)
  }

  //----------------------------------------------------------
  //                 lch fail to clear iq frz
  //----------------------------------------------------------
  io.ctrl.out.toIq.zipWithIndex.foreach {
    case (iq, i) =>
      iq.launchFailValid := pipeInstValidVec(i) && pipeLaunchFailVec(i)
  }

  //----------------------------------------------------------
  //               no lch fail to pop iq entry
  //----------------------------------------------------------
  io.ctrl.out.toIq.zipWithIndex.foreach {
    case (iq, i) =>
      //pipe3/4/5 pop by LSU
      if (!LsuPipeNumSet.contains(i))
        iq.popValid := pipeDownValidVec(i)

      //pop singals for IR dlb, optimized for timing
      if (!LsuPipeNumSet.contains(i) || !BjuPipeNumSet.contains(i))
        iq.popDlbValid := pipeInstValidVec(i)
  }

  //----------------------------------------------------------
  //                    lch fail for hpcp
  //----------------------------------------------------------
  //lch fail by src no rdy
  io.ctrl.out.toHpcp.pipeVec.zipWithIndex.foreach {
    case (pipe, i) =>
      pipe.launchFailValid := pipeInstValidVec(i) && pipeSrcNotReadyVec(i)
  }
  //lch fail by preg share
  io.ctrl.out.toHpcp.pipeVec.foreach(_.regLaunchFailValid := false.B)
  io.ctrl.out.toHpcp.pipeVec(3).regLaunchFailValid := pipeInstValidVec(3) &&
    (pipe3PregLaunchFail || pipe3VregLaunchFail)
  io.ctrl.out.toHpcp.pipeVec(4).regLaunchFailValid := pipeInstValidVec(4) &&
    pipe4VregLaunchFail
  io.ctrl.out.toHpcp.pipeVec(5).regLaunchFailValid := pipeInstValidVec(5) &&
    pipe5PregLaunchFail

  //----------------------------------------------------------
  //                staddr rdy set for SDIQ
  //----------------------------------------------------------
  //staddr replayed after DC stage should not set stdata ready, because
  //its stdata already poped after staddr signal valid at DC stage
  io.ctrl.out.toIq.foreach(_.stAddrReadySet := false.B)
  io.ctrl.out.toIq(5).stAddrReadySet := pipeInstValidVec(4) &&
    !pipeLaunchFailVec(4) && io.data.in.lsiq1.issueReadData.stAddr

  //==========================================================
  //                RF Execution Unit Selection
  //==========================================================
  // Todo:
  //----------------------------------------------------------
  //               Pipe0 Exectuion Unit Selection
  //----------------------------------------------------------

  //----------------------------------------------------------
  //               Pipe1 Exectuion Unit Selection
  //----------------------------------------------------------

  //----------------------------------------------------------
  //               Pipe2 Exectuion Unit Selection
  //----------------------------------------------------------
  io.ctrl.out.toIq(2).


}
