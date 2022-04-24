package Core.IDU.RF

import Core.IS.AiqConfig.NumSrcArith
import Core.IS.BiqConfig.NumSrcBr
import Core.IS.LsiqConfig.NumSrcLs
import Core.IS.SdiqConfig.NumSrcSd
import Core.IS.VfiqConfig.NumSrcVf
import Core.IS.{AiqEntryData, BiqEntryData, LsiqEntryData, SdiqEntryData, VfiqEntryData}
import chisel3._
import chisel3.util._
import Core.IntConfig._

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
  // Todo: add forward signal
  //  e.g. ctrl_aiq0_rf_pipe0_alu_reg_fwd_vld ctrl_viq1_rf_pipe7_vmla_vreg_fwd_vld
  val popDlbValid = Bool()
  val popValid = Bool()
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
  // Todo: figure out
  val gateClkSel = Bool()
  val sel = Bool()
  // Todo: imm
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

class RFStageCtrlOutput extends Bundle with RFStageConfig {
  val toIq = Vec(NumIssue, new RFStageToIqBundle)
  val toCp0 = new RFStageToCp0Bundle
  val toHpcp = new RFStageToHpcpBundle
  val toExu = new RFStageToExuGateClkBundle
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
  val pipeLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssue)(false.B)))
  val lsuLaunchFailValidVec = RegInit(VecInit(Seq.fill(NumIssueLsiq + NumIssueSdiq)(false.B)))

  //==========================================================
  //                 RF Inst Valid registers
  //==========================================================
  //----------------------------------------------------------
  //                Pipe0 Instruction Valid
  //----------------------------------------------------------
  private val aiq0IssueLaunchPreg = Wire(Bool())
  private val aiq0IssueSpecial = Wire(Bool())
  private val aiq0IssueDiv = Wire(Bool())

  private val aiq0IssueAluRegValid = io.ctrl.in.issueEnVec(0).issueEn &&
                                      aiq0IssueLaunchPreg
  private val aiq0IssueSpecialValid = io.ctrl.in.issueEnVec(0).issueEn &&
                                      aiq0IssueSpecial
  private val iuIsDivIssue = io.ctrl.in.issueEnVec(0).issueEn &&
                                      aiq0IssueDiv

  private val pipe0InstValid = RegInit(false.B)
  private val pipe0PregLaunchValid = RegInit(false.B)
  private val pipe0SpecialValid = RegInit(false.B)

  when(rtu.flush.fe || rtu.flush.is) {
    pipe0InstValid        := false.B
    pipe0PregLaunchValid  := false.B
    pipe0SpecialValid     := false.B
  }.otherwise {
    pipe0InstValid        := io.ctrl.in.issueEnVec(0).issueEn
    pipe0PregLaunchValid  := aiq0IssueAluRegValid
    pipe0SpecialValid     := pipe0SpecialValid
  }
}
